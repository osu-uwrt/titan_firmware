#include "CanmoreCLI.hpp"
#include "TerminalDraw.hpp"

#include <algorithm>
#include <iostream>
#include <poll.h>
#include <signal.h>
#include <sstream>
#include <sys/ioctl.h>
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

static void winchHandler(int sig) {
    (void) sig;
    // Do nothing, we just want the interrupt to break the keybaord character poll so we redraw faster
}

class TerminalOverride {
private:
    static bool initialized;
    struct termios oldt;
    sighandler_t oldIntHandler;
    sighandler_t oldWinchHandler;

public:
    TerminalOverride() {
        if (initialized)
            throw std::logic_error("Cannot call initialize terminal in nested configuration");
        initialized = true;

        // Set signal handler to allow terminal to capture it and respond accordingly
        oldIntHandler = signal(SIGINT, intHandler);
        if (oldIntHandler == SIG_ERR) {
            throw std::system_error(errno, std::generic_category(), "signal");
        }

        oldWinchHandler = signal(SIGWINCH, winchHandler);
        if (oldWinchHandler == SIG_ERR) {
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
        // No use checking for error since it'll just terminate if we throw an exception
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
        signal(SIGINT, oldIntHandler);
        signal(SIGWINCH, oldWinchHandler);

        initialized = false;
    }
};
bool TerminalOverride::initialized = false;

struct canmore_device_fixed_sort {
    inline bool operator()(const std::shared_ptr<Device> &dev1, const std::shared_ptr<Device> &dev2) const {
        return dev1->interfaceName == dev2->interfaceName ? (dev1->clientId < dev2->clientId) :
                                                            (dev1->interfaceName < dev2->interfaceName);
    }
};

template <typename T> struct shared_ptr_equality {
    explicit shared_ptr_equality(std::shared_ptr<T> &lhs): lhs(lhs) {}
    inline bool operator()(const std::shared_ptr<T> &rhs) {
        if (lhs == rhs)
            return true;
        if (!lhs || !rhs)
            return false;
        return *lhs == *rhs;
    }
    std::shared_ptr<T> lhs;
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
        if (errno == EINTR)
            return false;
        else
            throw std::system_error(errno, std::generic_category(), "poll");
    }
    return pfd.revents != 0;
}

static Keypress getKeypress(unsigned int timeoutMs = 500) {
    if (!keypressAvailable(timeoutMs)) {
        if (receivedInt) {
            receivedInt = false;
            return KEY_CTRL_C;
        }
        else {
            return KEY_NONE;
        }
    }

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

int getTerminalHeight() {
    struct winsize w;
    if (ioctl(STDOUT_FILENO, TIOCGWINSZ, &w)) {
        return 0;
    }

    return w.ws_row;
}

// ========================================
// UI Rendering
// ========================================

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

class MenuItem {
public:
    virtual void render(bool selected, int numLines = 0) const = 0;
    virtual int getRenderedHeight() const = 0;
    virtual bool hasPaddingAfter() const = 0;
    virtual bool canSelect() const = 0;
    virtual bool operator==(const MenuItem &rhs) const = 0;
};

class MenuHeader : public MenuItem {
public:
    MenuHeader(const std::string &title): title(title) {}

    void render(bool selected, int numLines) const override {
        (void) selected;
        (void) numLines;  // We're only 1 line, if they ask us to render, we can fit on the screen
        renderHeader(title);
    }
    int getRenderedHeight() const override { return 1; }

    bool hasPaddingAfter() const override final { return false; }
    bool canSelect() const override final { return false; }

    bool operator==(const MenuItem &rhs) const override {
        auto pRhs = dynamic_cast<const MenuHeader *>(&rhs);
        if (pRhs == nullptr) {
            return false;  // Not a derived.  Cannot be equal.
        }
        if (this == pRhs) {
            return true;
        }
        return title == pRhs->title;
    }

private:
    std::string title;
};

class MenuText : public MenuItem {
public:
    MenuText(const std::string &text, bool paddingAfter = true): text(text), paddingAfter(paddingAfter) {}

    void render(bool selected, int numLines) const override {
        (void) selected;
        (void) numLines;  // We're only 1 line, if they ask us to render, we can fit on the screen
        std::cout << COLOR_RESET << text << CLEAR_LINE_AFTER << std::endl;
    }
    int getRenderedHeight() const override { return 1; }

    bool hasPaddingAfter() const override final { return paddingAfter; }
    bool canSelect() const override final { return false; }

    bool operator==(const MenuItem &rhs) const override {
        auto pRhs = dynamic_cast<const MenuText *>(&rhs);
        if (pRhs == nullptr) {
            return false;  // Not a derived.  Cannot be equal.
        }
        if (this == pRhs) {
            return true;
        }
        return text == pRhs->text;
    }

private:
    std::string text;
    bool paddingAfter;
};

class MenuButton : public MenuItem {
public:
    MenuButton(const std::string &label): label(label) {}

    void render(bool selected, int numLines) const override {
        (void) numLines;  // We're only 1 line, if they ask us to render, we can fit on the screen
        renderName(label, selected);
    }
    int getRenderedHeight() const override { return 1; }
    bool hasPaddingAfter() const override final { return true; }
    bool canSelect() const override final { return true; }

    bool operator==(const MenuItem &rhs) const override {
        auto pRhs = dynamic_cast<const MenuButton *>(&rhs);
        if (pRhs == nullptr) {
            return false;  // Not a derived.  Cannot be equal.
        }
        if (this == pRhs) {
            return true;
        }
        return label == pRhs->label;
    }

private:
    std::string label;
};

class MenuDeviceEntry : public MenuItem {
public:
    MenuDeviceEntry(const DeviceMap &devMap, std::unordered_map<Device, uint64_t, canmore_device_hasher> &flashIdCache,
                    std::shared_ptr<Device> dev):
        dev(dev) {
        // Lookup device attributes
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

        // Fill out the attributes based on the description
        // This can be rendered by calling render
        name = devDescr.name;
        if (!devDescr.isUnknown)
            fields.emplace("Board Type", devDescr.boardType);
        else if (flashId != 0)
            fields.emplace("Unique ID", devDescr.hexSerialNum());
        fields.emplace("Interface", dev->getInterface());
        fields.emplace("Mode", dev->getMode());
        fields.emplace("Error State", dev->inErrorState ? COLOR_ERROR "Fault Present" : "Normal");
        if (dev->termValid)
            fields.emplace("Term Resistor", dev->termEnabled ? "On" : "Off");
    }

    void render(bool selected, int numLines) const override {
        renderName(name, selected);
        int i = 1;
        for (const auto &entry : fields) {
            // Stop printing after we reach the number of lines requested
            if (numLines != 0 && numLines <= i)
                break;
            i++;
            renderField(entry.first, entry.second);
        }
    }

    int getRenderedHeight() const override {
        // Add 1 extra for the name
        return 1 + fields.size();
    }

    bool hasPaddingAfter() const override final { return true; }
    bool canSelect() const override final { return true; }

    bool operator==(const MenuItem &rhs) const override {
        auto pRhs = dynamic_cast<const MenuDeviceEntry *>(&rhs);
        if (pRhs == nullptr) {
            return false;  // Not a derived.  Cannot be equal.
        }
        if (this == pRhs) {
            return true;
        }
        if (dev == nullptr || pRhs->dev == nullptr) {
            return false;
        }
        return *dev == *(pRhs->dev);
    }

    const std::shared_ptr<Device> dev;

private:
    std::string name;
    std::map<std::string, std::string> fields;
};

typedef std::vector<std::shared_ptr<MenuItem>>::const_iterator MenuItemConstIterator;

static MenuItemConstIterator showDiscoveredMenu(const std::vector<std::shared_ptr<MenuItem>> &menuItems,
                                                MenuItemConstIterator lastRenderTop,
                                                MenuItemConstIterator selectedItem) {
    if (menuItems.empty()) {
        // Nothing to do if screen is empty
        // The code below will crash otherwise (since we assume selectedItem is valid, which can't be true if menuItems
        // is empty)
        std::cout << CURSOR_GOTO_START COLOR_HEADER "<No Menu Items to Display>" COLOR_RESET CLEAR_TERMINAL_AFTER
                  << std::endl;
        return menuItems.begin();
    }
    // Need to compute which parts of the screen to render, in the event the terminal gets cut off
    // We know how tall every element is, and so we need to determine what will be allowed to be rendered.
    // Additionally, we want scrolling to be reasonable, so when you scroll down, the selected item appears at the
    // bottom, but when we scroll up, the selected items will appear at the top

    // Computation for how much stuff we can fit on screen
    // Note this can return 0, but that just means that we'll assume the worst and scroll as much as possible
    int remainingScreen = getTerminalHeight() - 2;  // Subtract top and bottom lines

    // We are always going to render the selected item
    remainingScreen -= (*selectedItem)->getRenderedHeight();
    auto renderTop = selectedItem;
    auto renderEnd = selectedItem + 1;

    // Begin solving how much else we can render onto the screen, using priority stated above
    // Note if we have 0 height left, there might be some stuff we can do to squeeze a 1 tall title into it
    if (remainingScreen >= 0) {
        // First try to allocate space scanning backwards until we hit the last render top (makes scrolling feel more
        // natural)
        // Only need to do this if the currently selected item is after the last render top
        if (selectedItem > lastRenderTop) {
            auto itr = selectedItem;

            // Loop until we hit the lastRenderTop, or we run out of space on screen
            do {
                itr--;

                // Compute height (including padding)
                int height = (*itr)->getRenderedHeight();
                if ((*itr)->hasPaddingAfter()) {
                    height++;
                }

                // If we are the first line, we get an extra line of space due to not needing to add the scroll marker
                // Compute the effective screen height if we're at the top
                int remainingScreenCompensated = remainingScreen;
                if (itr == menuItems.begin()) {
                    remainingScreenCompensated++;
                }

                // See if we can fit it in
                if (height <= remainingScreenCompensated) {
                    remainingScreen -= height;
                    renderTop = itr;

                    // If we ended up scrolling to the top, add back the extra line of space
                    if (itr == menuItems.begin()) {
                        remainingScreen++;
                    }
                }
                else {
                    break;
                }
            } while (itr != lastRenderTop);
        }

        // Then after we hit the last top, fill in remaining space below selected item
        bool lastHasPadding = (*selectedItem)->hasPaddingAfter();
        for (auto itr = selectedItem + 1; itr != menuItems.end(); itr++) {
            // Compute height (including padding)
            int height = (*itr)->getRenderedHeight();
            if (lastHasPadding) {
                height++;
            }
            lastHasPadding = (*itr)->hasPaddingAfter();

            // See if we can fit it in
            if (height <= remainingScreen) {
                remainingScreen -= height;
                renderEnd = itr + 1;
            }
            else {
                break;
            }
        }

        // Finally, if we still have space after we hit the bottom, we can begin rendering back up past the last
        // render top (prevents cases where we scroll to the bottom, expand the window, and we get blank space if the
        // window is made larger again)
        if (renderTop != menuItems.begin()) {
            auto itr = renderTop;
            do {
                itr--;

                // Compute height (including padding)
                int height = (*itr)->getRenderedHeight();
                if ((*itr)->hasPaddingAfter()) {
                    height++;
                }

                // If we are the first line, we get an extra line of space due to not needing to add the scroll marker
                // Compute the effective screen height if we're at the top
                int remainingScreenCompensated = remainingScreen;
                if (itr == menuItems.begin()) {
                    remainingScreenCompensated++;
                }

                // See if we can fit it in
                if (height <= remainingScreenCompensated) {
                    remainingScreen -= height;
                    renderTop = itr;

                    // If we ended up scrolling to the top, add back the extra line of space
                    if (itr == menuItems.begin()) {
                        remainingScreen++;
                    }
                }
                else {
                    break;
                }
            } while (itr != menuItems.begin());
        }
    }

    // Reset terminal to start
    std::cout << CURSOR_GOTO_START;
    // Add scroll message if we aren't at the top of the screen
    if (renderTop != menuItems.begin()) {
        std::cout << COLOR_HEADER_HIGHLIGHTED "   Scroll ^   " COLOR_RESET CLEAR_LINE_AFTER << std::endl;
    }

    // Render the core content
    assert(renderTop < renderEnd);  // We should render at least one item, and start should be before the end
    bool needsPaddingAfter = false;
    for (auto itr = renderTop; itr != renderEnd; itr++) {
        // Render each item, adding padding as needed
        // Padding is separate so we can get an extra line in the terminal (as we don't pad if we hit the bottom)
        if (needsPaddingAfter) {
            std::cout << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
        }
        (*itr)->render(itr == selectedItem);
        needsPaddingAfter = (*itr)->hasPaddingAfter();
    }

    // Partially render the last line if we have some remaining screen left
    if (renderEnd != menuItems.end() && remainingScreen > (needsPaddingAfter ? 1 : 0)) {
        if (needsPaddingAfter) {
            std::cout << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
            remainingScreen--;
        }
        (*renderEnd)->render(renderEnd == selectedItem, remainingScreen);
        remainingScreen = 0;
    }
    else {
        // Render final line (and clear the rest of the terminal)
        for (int i = 0; i < remainingScreen; i++) {
            std::cout << COLOR_RESET CLEAR_LINE_AFTER << std::endl;
        }
    }
    if (renderEnd != menuItems.end()) {
        std::cout << COLOR_HEADER_HIGHLIGHTED "   Scroll v   ";
    }
    std::cout << COLOR_RESET CLEAR_TERMINAL_AFTER << std::flush;

    return renderTop;
}

// ========================================
// Main Discovery Program
// ========================================

static MenuItemConstIterator mapDeviceItr(const std::vector<std::shared_ptr<MenuItem>> &menuItems,
                                          std::shared_ptr<MenuItem> lastSelected) {
    // Only search if lastSelected is valid and menuItems
    if (lastSelected && !menuItems.empty()) {
        // Try to find device in new array
        auto result = std::find_if(menuItems.begin(), menuItems.end(), shared_ptr_equality<MenuItem>(lastSelected));
        if (result != menuItems.end()) {
            return result;
        }

        // Couldn't find it, if its a device, try to find the next closest device in the list
        auto pLastSelectedDev = std::dynamic_pointer_cast<MenuDeviceEntry>(lastSelected);
        if (pLastSelectedDev != nullptr) {
            // Create map of devices to iterators so we can search through it
            std::map<std::shared_ptr<Device>, MenuItemConstIterator, canmore_device_fixed_sort> deviceMap;
            for (auto itr = menuItems.begin(); itr != menuItems.end(); itr++) {
                auto pDevEntry = std::dynamic_pointer_cast<MenuDeviceEntry>(*itr);
                if (pDevEntry) {
                    deviceMap.emplace(pDevEntry->dev, itr);
                }
            }

            // Try to find the closest match before the previous last selected device on the list
            auto closest = deviceMap.lower_bound(pLastSelectedDev->dev);

            if (closest != deviceMap.begin()) {
                // Need to get previous from closest, as the lower bound function will first item not less than the
                // last selected. Since we couldn't find last selected, then that will need to be the element after
                // it. So we will get the previous element, which will be the item right before the selected item.
                // Note we had to check that we weren't at the beginning to avoid the case where the first element is
                // greater.

                return std::prev(closest)->second;
            }
        }

        // Couldn't find a good match, just fall through to first selection
    }

    // If we fell through, then just try to find the first selectable item
    for (auto itr = menuItems.begin(); itr != menuItems.end(); itr++) {
        if ((*itr)->canSelect()) {
            return itr;
        }
    }

    // If we can't find anything, just return the beginning, the UI will just have to deal with the fact that
    // nothing on screen can be selected right now
    return menuItems.begin();
}

std::shared_ptr<Device> getTargetDevice(const DeviceMap &devMap,
                                        std::vector<std::shared_ptr<Discovery>> discoverySources) {
    TerminalOverride termOver;

    std::unordered_map<Device, uint64_t, canmore_device_hasher> flashIdCache;
    std::shared_ptr<MenuItem> lastTop = nullptr;
    std::shared_ptr<MenuItem> lastSelection = nullptr;
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

        // Create the menu entries
        std::vector<std::shared_ptr<MenuItem>> menuItems;
        menuItems.push_back(std::make_shared<MenuHeader>("Select Device"));
        menuItems.push_back(std::make_shared<MenuButton>("Quit"));
        for (const auto &dev : discovered) {
            menuItems.push_back(std::make_shared<MenuDeviceEntry>(devMap, flashIdCache, dev));
        }

        if (discovered.empty()) {
            menuItems.push_back(std::make_shared<MenuText>("No devices found. Discovering..."));
        }

        // Select the device in the menu
        // We need to look up the next best index for the previous loop's device in the new table
        // since devices can appear and disappear, and we want to try to keep the cursor from jumping randomly
        // around
        auto selectedItemItr = mapDeviceItr(menuItems, lastSelection);

        // Only redraw if keypress isn't available
        // Prevent slowdowns from constant redrawing of the terminal if someone holds down a key, filling up stdin
        // and it'll be redrawing as it tries to eat up the backlog
        if (!keypressAvailable()) {
            auto lastTopItr = mapDeviceItr(menuItems, lastTop);
            auto renderTop = showDiscoveredMenu(menuItems, lastTopItr, selectedItemItr);
            lastTop = *renderTop;
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
            // Try to go up to find the next item (skipping unselectable items)
            auto itr = selectedItemItr;
            while (itr != menuItems.begin()) {
                itr--;
                if ((*itr)->canSelect()) {
                    selectedItemItr = itr;
                    break;
                }
            }
        }
        if (key == KEY_DOWN) {
            // Try to go up to find the next item (skipping unselectable items)
            auto itr = selectedItemItr;
            while (++itr != menuItems.end()) {
                if ((*itr)->canSelect()) {
                    selectedItemItr = itr;
                    break;
                }
            }
        }

        // If enter hit, then use the last selection
        if (key == KEY_ENTER) {
            break;
        }

        // Assign the last selection to the iterator so we can find it when we rescan
        lastSelection = *selectedItemItr;
    }

    if (lastSelection != nullptr) {
        auto ptr = std::dynamic_pointer_cast<MenuDeviceEntry>(lastSelection);
        if (ptr) {
            return ptr->dev;
        }
    }
    return nullptr;
}
