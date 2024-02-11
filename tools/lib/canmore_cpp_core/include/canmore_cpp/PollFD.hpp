#pragma once

#include <poll.h>
#include <system_error>
#include <vector>

namespace Canmore {

class PollFDHandler;

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
     * @brief Populates the vector with a pair of pollfd structs
     *
     * The PollGroup will call the provided PollFDHandler if the given pollfd matches
     *
     * @param fds A vector of pollfd structs and the corresponding handler for that struct
     */
    virtual void populateFds(std::vector<std::pair<PollFDHandler *, pollfd>> &fds) = 0;
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
    void addFd(PollFD *fd) {
        // Get the fds that are to be registered
        std::vector<std::pair<PollFDHandler *, pollfd>> pollFds;
        fd->populateFds(pollFds);

        // Copy all the fds into the local
        for (auto &fd : pollFds) {
            fds.push_back(fd.second);
            fdBackends.push_back(fd.first);
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
        int rc = poll(fds.data(), fds.size(), timeoutMs);
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
        auto backendItr = fdBackends.begin();
        auto itr = fds.begin();
        while (itr != fds.end()) {
            int revents = itr->revents;
            auto backend = *backendItr;

            if (revents & POLLNVAL) {
                // If we get an invalid FD, report it to the PollFD object, and then remove it from the poll group
                itr = fds.erase(itr);
                backendItr = fdBackends.erase(backendItr);
                backend->handleInvalidFd();
            }
            else {
                if (revents) {
                    backend->handleEvent(*itr);
                }
                itr++;
                backendItr++;
            }
        }
    }

private:
    std::vector<struct pollfd> fds;
    std::vector<PollFDHandler *> fdBackends;
};

};  // namespace Canmore
