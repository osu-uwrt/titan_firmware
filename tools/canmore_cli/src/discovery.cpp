#include "CanmoreCLI.hpp"
#include "TerminalDraw.hpp"

#include <algorithm>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <termios.h>
#include <unistd.h>
#include <unordered_map>

using namespace Canmore;

// ========================================
// Terminal Control
// ========================================

volatile bool receivedInt = false;
static void intHandler(int sig) {
    // Restore default signal handler
    // The keyboard input processor should handle this past one, if it doesn't let the user hit control c again and get
    // out
    signal(sig, SIG_DFL);
    receivedInt = true;
}

class TerminalOverride {
private:
    static bool initialized;
    struct termios oldt;
    sighandler_t oldHandler;

public:
    TerminalOverride() {
        if (initialized)
            throw std::logic_error("Cannot call initialize terminal in nested configuration");
        initialized = true;

        // Set signal handler to allow terminal to capture it and respond accordingly
        oldHandler = signal(SIGINT, intHandler);
        if (oldHandler == SIG_ERR) {
            throw std::system_error(errno, std::generic_category(), "signal");
        }

        // Backup old flags
        if (tcgetattr(STDIN_FILENO, &oldt) < 0) {
            throw std::system_error(errno, std::generic_category(), "tcgetattr");
        }

        // Set new flags disabling echo and canonical mode
        struct termios newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO);
        newt.c_cc[VMIN] = 1;
        newt.c_cc[VTIME] = 0;
        newt.c_iflag &= ~(IXON | IXOFF | IXANY);

        if (tcsetattr(STDIN_FILENO, TCSANOW, &newt) < 0) {
            throw std::system_error(errno, std::generic_category(), "tcsetattr");
        }

        // Draw terminal onto aux buffer
        std::cout << CURSOR_DISABLE BUFFER_ALT_ENABLE CLEAR_TERMINAL COLOR_RESET CURSOR_GOTO_START << std::flush;
    }

    ~TerminalOverride() {
        std::cout << COLOR_RESET BUFFER_ALT_DISABLE CURSOR_ENABLE << std::flush;

        // Restore default terminal behavior
        // No use checking for error since it'll just terminate
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        signal(SIGINT, oldHandler);

        initialized = false;
    }
};
bool TerminalOverride::initialized = false;

struct canmore_device_fixed_sort {
    inline bool operator()(const std::shared_ptr<Device> &dev1, const std::shared_ptr<Device> &dev2) {
        return dev1->interfaceName == dev2->interfaceName ? (dev1->clientId < dev2->clientId) :
                                                            (dev1->interfaceName < dev2->interfaceName);
    }
};

struct canmore_device_lookup {
    explicit canmore_device_lookup(std::shared_ptr<Device> &target): target(target) {}
    inline bool operator()(const std::shared_ptr<Device> &dev) {
        if (target == dev)
            return true;
        if (!target || !dev)
            return false;
        return *target == *dev;
    }
    std::shared_ptr<Device> target;
};

struct canmore_device_hasher {
    std::size_t operator()(const Device &k) const {
        using std::hash;

        // Taken from stack overflow on how to combine hashes
        std::size_t computed = hash<uint8_t>()(k.clientId);
        computed ^= hash<std::string>()(k.interfaceName) + 0x9e3779b9 * (computed << 6) + (computed >> 2);
        return computed;
    }
};

enum Keypress { KEY_NONE = 0, KEY_UP, KEY_DOWN, KEY_ENTER, KEY_CTRL_C };

bool keypressAvailable(unsigned int timeoutMs) {
    struct pollfd pfd = { .fd = STDIN_FILENO, .events = POLLIN, .revents = 0 };
    if (poll(&pfd, 1, timeoutMs) < 0) {
        if (errno == EINTR && receivedInt)
            return false;
        else
            throw std::system_error(errno, std::generic_category(), "poll");
    }
    return pfd.revents != 0;
}

static Keypress getKeypress(unsigned int timeoutMs = 500) {
    if (!keypressAvailable(timeoutMs))
        return (receivedInt ? KEY_CTRL_C : KEY_NONE);

    int c = getchar();
    if (c == '\033') {
        // Terminal input sequence
        // Note can't poll for whatever reason, just going to blindly poll and trust we're getting a valid sequence
        c = getchar();
        if (c != '[')
            return KEY_NONE;  // Unknown terminal input sequence, they should all start with [

        std::string modifiers;
        while (true) {
            c = getchar();
            if (c == ';' || (c >= '0' && c <= '9'))
                modifiers += (char) c;
            else
                break;
        }

        if (c == 'A' && modifiers.size() == 0) {
            return KEY_UP;
        }
        else if (c == 'B' && modifiers.size() == 0) {
            return KEY_DOWN;
        }
    }
    else if (c == '\n') {
        return KEY_ENTER;
    }

    // Any other keys, just return none and let it redraw
    return KEY_NONE;
}

// ========================================
// UI Rendering
// ========================================

static size_t remapDeviceIndex(std::vector<std::shared_ptr<Device>> &devicesNew, std::shared_ptr<Device> lastSelected) {
    // If null pointer, then that means we haven't found one yet or Quit was selected. Return quit pointer
    if (!lastSelected || devicesNew.size() == 0) {
        return 0;
    }

    // Try to find device in new array
    auto result = std::find_if(devicesNew.begin(), devicesNew.end(), canmore_device_lookup(lastSelected));
    if (result != devicesNew.end()) {
        return result - devicesNew.begin() + 1;  // Return index + 1 (as quit is index 0)
    }

    // If we couldn't find it, try to find the next closest
    auto closest = lower_bound(devicesNew.begin(), devicesNew.end(), lastSelected);
    if (closest == devicesNew.begin()) {
        return 0;
    }
    else {
        // Need to get previous from closest, as the lower bound function will first item not less than the
        // last selected. Since we couldn't find last selected, then that will need to be the element after
        // it. So we will get the previous element, which will be the item right before the selected item.
        // Note we had to check that we weren't at the beginning to avoid the case where the first element is greater.
        return std::prev(closest) - devicesNew.begin() + 1;
    }
}

void renderHeader(std::string const &title, int titleWidth) {
    int requiredPadding = titleWidth - (int) title.size();
    int paddingBefore = requiredPadding > 0 ? requiredPadding / 2 : 0;
    int paddingAfter = requiredPadding > 0 ? (requiredPadding / 2) + (requiredPadding % 1) : 0;
    std::cout << COLOR_TITLE;
    for (int i = 0; i < paddingBefore; i++) {
        std::cout << "=";
    }
    std::cout << title;
    for (int i = 0; i < paddingAfter; i++) {
        std::cout << "=";
    }
    std::cout << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
}

void renderName(std::string const &name, bool selected) {
    if (selected)
        std::cout << COLOR_NAME_SELECTED " > ";
    else
        std::cout << COLOR_NAME "   ";
    std::cout << name << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
}

void renderField(std::string const &name, std::string const &value, int nameWidth) {
    std::cout << COLOR_HEADER "        " << name << ':';
    for (int i = 0; i < (nameWidth - ((int) name.size() + 1)); i++) {
        std::cout << ' ';
    }
    std::cout << COLOR_BODY << value << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
}

static void showDiscovered(const DeviceMap &devMap,
                           std::unordered_map<Device, uint64_t, canmore_device_hasher> &flashIdCache,
                           std::vector<std::shared_ptr<Device>> &devices, size_t selectedIndex) {
    std::cout << CURSOR_GOTO_START;
    renderHeader("Select Device");

    for (size_t i = 0; i < (devices.size() + 1); i++) {
        if (i == 0) {
            renderName("Quit", selectedIndex == i);
        }
        else {
            auto dev = devices.at(i - 1);
            auto itr = flashIdCache.find(*dev);
            uint64_t flashId;
            if (itr != flashIdCache.end()) {
                flashId = itr->second;
            }
            else {
                flashId = dev->getFlashId();
                if (flashId != 0) {
                    flashIdCache.emplace(*dev, flashId);
                }
            }
            auto devDescr = devMap.lookupSerial(flashId);

            renderName(devDescr.name, selectedIndex == i);
            if (devDescr.boardType != "unknown")
                renderField("Board Type", devDescr.boardType);
            else if (flashId != 0)
                renderField("Unique ID", devDescr.hexSerialNum());
            renderField("Interface", dev->getInterface());
            renderField("Mode", dev->getMode());
            renderField("Error State", dev->inErrorState ? COLOR_ERROR "Fault Present" : "Normal");
            if (dev->termValid)
                renderField("Term Resistor", dev->termEnabled ? "On" : "Off");
        }
        std::cout << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
    }
    if (devices.size() == 0) {
        std::cout << COLOR_RESET "No devices found. Discovering..." CLEAR_LINE_AFTER << std::endl;
    }
    std::cout << CLEAR_TERMINAL_AFTER << std::flush;
    // TODO: Gracefully handle scrolling
}

// ========================================
// Main Discovery Program
// ========================================

std::shared_ptr<Device> getTargetDevice(const DeviceMap &devMap,
                                        std::vector<std::shared_ptr<Discovery>> discoverySources) {
    TerminalOverride termOver;

    int cnt = 0;

    std::unordered_map<Device, uint64_t, canmore_device_hasher> flashIdCache;
    std::shared_ptr<Device> lastSelection = nullptr;
    while (true) {
        // Check all sources for devices
        std::vector<std::shared_ptr<Device>> discovered;
        for (auto discovery : discoverySources) {
            std::vector<std::shared_ptr<Device>> interfaceDiscovered;
            discovery->discoverCanmoreDevices(interfaceDiscovered);
            discovered.insert(std::end(discovered), std::begin(interfaceDiscovered), std::end(interfaceDiscovered));
        }

        // First sort discovered devices into fixed ordering
        std::sort(discovered.begin(), discovered.end(), canmore_device_fixed_sort());

        // Then look up the next best index for the previous loop's device in the new table
        size_t selectionIndex = remapDeviceIndex(discovered, lastSelection);

        // Only redraw if keypress available
        // Prevent slowdowns from constant redrawing of the terminal to stop processing of stdin
        if (!keypressAvailable()) {
            cnt++;
            showDiscovered(devMap, flashIdCache, discovered, selectionIndex);
        }

        // Wait for user input, within timeout to allow constant redrawing
        auto key = getKeypress();

        // Break if control c hit
        if (key == KEY_CTRL_C) {
            lastSelection = nullptr;
            break;
        }

        // Handle up/down keys before recomputing lastSelection
        if (key == KEY_UP) {
            if (selectionIndex > 0)
                selectionIndex--;
        }
        if (key == KEY_DOWN) {
            if (selectionIndex < discovered.size())
                selectionIndex++;
        }

        // Recompute lastSelection
        if (selectionIndex > 0)
            lastSelection = discovered.at(selectionIndex - 1);
        else
            lastSelection = nullptr;

        // If enter hit, then use the last selection
        if (key == KEY_ENTER) {
            break;
        }
    }
    return lastSelection;
}
