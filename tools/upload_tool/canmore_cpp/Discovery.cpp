#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

#include "Discovery.hpp"

using namespace Canmore;

#define STALE_DISCOVERY_MS 2000

Discovery::Discovery(): socketFd(-1), threadEventFd(-1) {}

Discovery::~Discovery() {
    if (thread.joinable()) {
        // eventFd must exist if the thread was created
        threadStopFlag.clear();
        notifyEventFd();

        thread.join();
        close(threadEventFd);
        threadEventFd = -1;
    }

    if (socketFd >= 0) {
        close(socketFd);
        socketFd = -1;
    }
}

void Discovery::startSocketThread(int socketFd) {
    assert(!thread.joinable());
    this->socketFd = socketFd;

    // Set thread to start, or else it will think it should stop
    threadStopFlag.test_and_set();
    // Using eventfd since it can be polled along with the socket fd
    threadEventFd = eventfd(0, EFD_NONBLOCK);
    if (threadEventFd < 0) {
        throw std::system_error(errno, std::generic_category(), "eventfd");
    }
    thread = std::thread(&Discovery::socketThread, this);
}

void Discovery::socketThread() {
    while (threadStopFlag.test_and_set()) {
        struct pollfd fds[2];

        // Watch for wakeup events
        fds[0].fd = threadEventFd;
        fds[0].events = POLLIN;

        // Watch for socket events
        fds[1].fd = socketFd;
        fds[1].events = POLLIN;

        if (poll(fds, 2, -1) == -1) {
            throw std::system_error(errno, std::generic_category(), "poll");
        }

        // eventFd ready
        if (fds[0].revents & POLLIN) {
            uint64_t data;
            if (read(threadEventFd, &data, sizeof(data)) != sizeof(data)) {
                throw std::system_error(errno, std::generic_category(), "eventFd read");
            }
            // Discard data, it's just to wake up the thread
        }

        if (fds[1].revents & POLLIN) {
            handleSocketData(socketFd);
        }
    }
}

void Discovery::notifyEventFd() {
    uint64_t data = 1;
    if (write(threadEventFd, &data, sizeof(data)) != sizeof(data)) {
        throw std::system_error(errno, std::generic_category(), "eventfd write");
    }
}

void Discovery::reportDiscoveredDevice(const std::shared_ptr<Device> &device) {
    const std::lock_guard<std::mutex> lock(discoveredDevicesMutex);

    // Remove older version of Device if it exists
    for (auto it = discoveredDevices.begin(); it != discoveredDevices.end(); ) {
        if (**it == *device) {
            discoveredDevices.erase(it++);
        }
        else {
            ++it;
        }
    }

    // Insert new version
    discoveredDevices.insert(device);

    // Take chance to prune any not discovered devices
    pruneDiscoveredDevices();

    // Notify
    discoveredNotify.notify_one();
}

void Discovery::pruneDiscoveredDevices() {
    // NOTE: The caller MUST lock the mutex before calling this function

    // Get range of valid devices
    auto lastValidTime = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch())
        - std::chrono::milliseconds(STALE_DISCOVERY_MS);
    auto searchDev = std::make_shared<Device>(lastValidTime);
    auto it = discoveredDevices.lower_bound(searchDev);

    // Remove devices not in this range
    discoveredDevices.erase(discoveredDevices.begin(), it);
}

// ========================================
// Public Functions
// ========================================

void Discovery::discoverDevices(std::vector<std::shared_ptr<UploadTool::RP2040Device>> &devicesOut) {
    const std::lock_guard<std::mutex> lock(discoveredDevicesMutex);
    pruneDiscoveredDevices();

    devicesOut.clear();
    for (auto device : discoveredDevices) {
        devicesOut.push_back(std::static_pointer_cast<UploadTool::RP2040Device>(device));
    }
}

void Discovery::discoverCanmoreDevices(std::vector<std::shared_ptr<Device>> &devicesOut) {
    const std::lock_guard<std::mutex> lock(discoveredDevicesMutex);
    pruneDiscoveredDevices();

    devicesOut.clear();
    for (auto device : discoveredDevices) {
        devicesOut.push_back(device);
    }
}

std::shared_ptr<BootloaderClient> Discovery::catchDeviceInBootDelay(uint8_t clientId) {
    auto device = waitForDevice<BootDelayDevice>(clientId, -1);

    if (device == nullptr) {
        throw BootloaderError("Failed to catch device in boot delay!");
    }

    return device->enterBootloader();
}
