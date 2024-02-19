#pragma once

#include <limits.h>
#include <memory>
#include <poll.h>
#include <system_error>
#include <vector>

namespace Canmore {

class PollFDHandler;

class PollFDDescriptor {
    friend class PollGroup;

public:
    /**
     * @brief Creates a new PollFD Descriptor.
     *
     * @attention Because PollFDDescriptor contains a reference to the handler, the handler class must keep a owning
     * reference to that object.
     *
     * @param handler The given PollFDHandler for this fd
     * @param fd The fd to monitor
     * @param events Events to monitor (see pollfd struct man page)
     * @param enabled If this given PollFDDescriptor starts enabled. Defaults to true
     * @return std::shared_ptr<PollFDDescriptor> A shared pointer to the new object. This must be kept by the Handler
     * class
     */
    static std::shared_ptr<PollFDDescriptor> create(PollFDHandler &handler, int fd, short events, bool enabled = true) {
        return std::shared_ptr<PollFDDescriptor>(new PollFDDescriptor(handler, fd, events, enabled));
    }

    // Disable all copying, we want this to be purely managed via shared ptrs
    PollFDDescriptor(const PollFDDescriptor &) = delete;
    PollFDDescriptor &operator=(PollFDDescriptor const &) = delete;

    /**
     * @brief Enables/disables the requested PollFD descriptor. This will apply the next time the poll syscall is ran.
     *
     * @param enabled True to enable the given pollfd, false to disable
     */
    void setEnabled(bool enabled) { enabled_ = enabled; }

protected:
    PollFDHandler &handler_;
    const int fd_;
    const short events_;
    bool enabled_;

private:
    PollFDDescriptor(PollFDHandler &handler, int fd, short events, bool enabled):
        handler_(handler), fd_(fd), events_(events), enabled_(enabled) {}
};

/**
 * @brief Class which can be added to a PollGroup.
 *
 * This object can either on its own be a PollFDHandler, or have members which are PollFDHandlers.
 *
 * For an example of the latter, Think of the case where a class has both a member object which handles reading stdin,
 * but also an object which manages a UDP socket. Although the owner class does not have a fd of its own (so it won't
 * implement PollFDHandler), it will still implement PollFD so it can be added to a PollGroup, so that in populateFds it
 * will call the two member populateFd functions.
 */
class PollFD {
public:
    /**
     * @brief Populates the vector with the pollfd descriptors to monitor for.
     *
     * These can be created with PollFDDescriptor::create(). See more details on that method
     *
     * @param fds A vector of weak pointers to PollFDDescriptors
     */
    virtual void populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) = 0;
};

/**
 * @brief Class which has fds which will be polled in the PollGroup.
 *
 * Whenever a fd has an event, handleEvent is called. This class, by default, is also a PollFD as it must be able to
 * provide the fds it owns to the PollGroup.
 */
class PollFDHandler : public PollFD {
    friend class PollGroup;

protected:
    /**
     * @brief Called when poll returns that an event occurred
     *
     * @param fd The pollfd that fired
     */
    virtual void handleEvent(const pollfd &fd) = 0;

    /**
     * @brief Called if poll reports a POLLNVAL event. By default this throws a runtime error.
     */
    virtual void handleInvalidFd() { throw std::runtime_error("Invalid FD during poll (POLLNVAL)"); }
};

/**
 * @brief Collection of pollfd structs and their corresponding PollFDHandlers. After adding all of the PollFDs to this
 * object, calling handleEvent runs poll until an event occurs.
 *
 */
class PollGroup {
public:
    /**
     * @brief Construct a new Poll Group which will handle polling and calling of PollFD objects
     */
    PollGroup() {}

    /**
     * @brief Add a new PollFD to the PollGroup
     *
     * @param fd The PollFD object to poll for
     */
    void addFd(PollFD &fd) {
        // Get the fds that are to be registered
        std::vector<std::weak_ptr<PollFDDescriptor>> descriptors;
        fd.populateFds(descriptors);

        // Copy all the fds into the local
        for (auto &descrWeak : descriptors) {
            auto descr = descrWeak.lock();
            if (descr) {
                pollfd fd = { .fd = descr->fd_, .events = descr->events_, .revents = 0 };
                fds_.push_back(fd);
                fdDescriptors_.push_back(descrWeak);
            }
        }
    };

    /**
     * @brief Processes a single event, waiting up to timeoutMs for an event to occur.
     * Note this will return after a single event occurs.
     *
     * This is equivalent to if you called poll() with timeout, then processed all the events accordingly.
     *
     * @attention This function returns if interrupted by a signal. In that case, no PollFDHandlers will be called,
     * however timeoutMs not will have yet elapsed.
     *
     * @param timeoutMs The maximum timeout to wait. 0 will return immediately, a negative number blocks indefinitely.
     */
    void processEvent(int timeoutMs) {
        // Refresh the fds_ (enabling/disabling as requested) before calling poll
        refreshFdConfig();

        int rc = poll(fds_.data(), fds_.size(), timeoutMs);
        if (rc == 0) {
            return;
        }
        if (rc < 0) {
            if (errno == EINTR) {
                // Signal handle fired while waiting, just return, almost like an event fired
                return;
            }
            else {
                throw std::system_error(errno, std::generic_category(), "poll");
            }
        }

        // If we get here, then we have events to process
        // Iterate through the events, checking which ones fired
        // Also keep backend itr so we can pair call the appropriate backend
        auto descrItr = fdDescriptors_.begin();
        auto itr = fds_.begin();
        while (itr != fds_.end()) {
            // Skip if no event pending
            if (!itr->revents) {
                itr++;
                descrItr++;
                continue;
            }

            // Try lock, erasing the reference if it's been invalidated
            // This shouldn't happen since we check it above, but just in case
            auto descr = descrItr->lock();
            if (!descr) {
                itr = fds_.erase(itr);
                descrItr = fdDescriptors_.erase(descrItr);
                continue;
            }

            if (itr->revents & POLLNVAL) {
                // If we get an invalid FD, report it to the PollFD object, and then remove it from the poll group
                itr = fds_.erase(itr);
                descrItr = fdDescriptors_.erase(descrItr);
                descr->handler_.handleInvalidFd();
            }
            else {
                // We have a normal event, handle it
                descr->handler_.handleEvent(*itr);
                itr++;
                descrItr++;
            }
        }
    }

private:
    std::vector<struct pollfd> fds_;
    std::vector<std::weak_ptr<PollFDDescriptor>> fdDescriptors_;

    /**
     * @brief Reconfigures the pollfd struct depending on if a given descriptor is enabled/disabled
     * Also garbage collects any invalid PollFDDescriptor objects
     *
     * This should be called before running poll
     */
    void refreshFdConfig() {
        auto fdItr = fds_.begin();
        auto itr = fdDescriptors_.begin();
        while (itr != fdDescriptors_.end()) {
            if (auto descr = itr->lock()) {
                configureFd(*fdItr, descr->enabled_);
                itr++;
                fdItr++;
            }
            else {
                itr = fdDescriptors_.erase(itr);
                fdItr = fds_.erase(fdItr);
            }
        }
    }

    /**
     * @brief Enables/disables the requested pollfd
     *
     * Disabling an fd causes poll to not alert on events for that specific pollfd until it is re-enabled
     *
     * @param fd Pollfd reference to configure
     * @param enable If true enables the fd for polling, if false disables the fd for polling
     */
    void configureFd(pollfd &fd, bool enable) {
        if (enable) {
            // Special check for INT_MIN since this is used to represent 0 (since there's no such thing as negative 0)
            if (fd.fd == INT_MIN)
                fd.fd = 0;
            else if (fd.fd < 0)
                fd.fd = -fd.fd;
        }
        else {
            // Disable the fd
            // Since negative 0 is still 0, we need special handling
            if (fd.fd == 0)
                fd.fd = INT_MIN;  // We can use the fact that INT_MIN does not have a corresponding 2s complemnet value
                                  // Instead, INT_MIN will encode negative 0
            else if (fd.fd > 0)
                fd.fd = -fd.fd;
        }
    }
};

};  // namespace Canmore
