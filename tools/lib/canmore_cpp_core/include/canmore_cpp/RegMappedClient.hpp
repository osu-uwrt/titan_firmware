#pragma once

#include "CANSocket.hpp"
#include "Canmore.hpp"
#include "SocketSingleton.hpp"

#include "titan/canmore.h"

#include <arpa/inet.h>
#include <map>
#include <memory>
#include <span>
#include <string>
#include <unordered_map>
#include <vector>

// The W25Q16JV datasheet specifies max sector erase time to be 400ms
#define REG_MAPPED_TIMEOUT_MS 1000
#define REG_MAPPED_MAX_IN_FLIGHT_PACKETS_CAN 32  // Matches config stored in CAN bus (32 frame long FIFO)
#define REG_MAPPED_MAX_IN_FLIGHT_PACKETS_ETH 8

namespace Canmore {

class RegMappedClientError : public CanmoreError {
public:
    RegMappedClientError(int errorCode, uint8_t mode, uint8_t page, uint8_t offset, bool isWrite):
        CanmoreError(std::string("Register Mapped Access Error (Register Direct ") + (isWrite ? "Write" : "Read") +
                     " - mode: " + lookupMode(mode) + ", page: " + std::to_string(page) +
                     ", offset: " + std::to_string(offset) + "): " + lookupError(errorCode)),
        errorCode(errorCode) {}

    RegMappedClientError(int errorCode, uint8_t mode, uint8_t page, uint8_t offset, uint8_t length, bool isWrite):
        CanmoreError(std::string("Register Mapped Access Error (Array ") + (isWrite ? "Write" : "Read") + " - mode: " +
                     lookupMode(mode) + ", page: " + std::to_string(page) + ", offset: " + std::to_string(offset) +
                     ", length: " + std::to_string(length) + "): " + lookupError(errorCode)),
        errorCode(errorCode) {}

    RegMappedClientError(int errorCode, uint8_t mode, uint8_t page):
        CanmoreError("Register Mapped Access Error (String Access - mode: " + lookupMode(mode) +
                     ", page: " + std::to_string(page) + "): " + lookupError(errorCode)),
        errorCode(errorCode) {}

    RegMappedClientError(int errorCode, size_t length):
        CanmoreError("Raw Access Error (length: " + std::to_string(length) + "): " + lookupError(errorCode)),
        errorCode(errorCode) {}

    const int errorCode;
    static const std::map<int, std::string> errorMap;

private:
    static const std::string lookupError(int errorCode) {
#define REG_MAPPED_ERR_DEF(name)                                                                                       \
    if (errorCode == name)                                                                                             \
        return #name;                                                                                                  \
    else

        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_SUCCESSFUL)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_MALFORMED_REQUEST)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_BULK_REQUEST_SEQ_ERROR)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_INVALID_REGISTER_ADDRESS)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_INVALID_REGISTER_MODE)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_INVALID_DATA)
        REG_MAPPED_ERR_DEF(REG_MAPPED_RESULT_INVALID_MODE)

        REG_MAPPED_ERR_DEF(REG_MAPPED_CLIENT_RESULT_TX_FAIL)
        REG_MAPPED_ERR_DEF(REG_MAPPED_CLIENT_RESULT_RX_FAIL)
        REG_MAPPED_ERR_DEF(REG_MAPPED_CLIENT_RESULT_RX_CLEAR_FAIL)
        REG_MAPPED_ERR_DEF(REG_MAPPED_CLIENT_RESULT_INVALID_ARG)
        REG_MAPPED_ERR_DEF(REG_MAPPED_CLIENT_RESULT_INVALID_BULK_COUNT)

        // Fallthrough to this error if none of the other if statements catch it
        return "Unknown Error " + std::to_string(errorCode);
    }

    static const std::string lookupMode(uint8_t mode) {
        if (mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_NORMAL)
            return "Normal";
        else if (mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOT_DELAY)
            return "Boot Delay";
        else if (mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_BOOTLOADER)
            return "Bootloader";
        else if (mode == CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX)
            return "Linux";
        else
            return "Unknown Mode " + std::to_string(mode);
    }
};

class RegMappedClient {
public:
    virtual ~RegMappedClient() = 0;

    uint32_t readRegister(uint8_t mode, uint8_t page, uint8_t offset);
    void writeRegister(uint8_t mode, uint8_t page, uint8_t offset, uint32_t data);
    void readArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &dst, uint8_t numWords);
    void writeArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &data);
    // TODO: Upgrade to using std::span
    void writeStringPage(uint8_t mode, uint8_t page, const std::string &data);
    std::string readStringPage(uint8_t mode, uint8_t page);

protected:
    reg_mapped_client_cfg_t clientCfg;
};

class RegisterPage {
public:
    RegisterPage(std::shared_ptr<RegMappedClient> client, uint8_t mode, uint8_t page):
        client(client), mode(mode), page(page) {}

    uint32_t readRegister(uint8_t offset) { return client->readRegister(mode, page, offset); }

    void writeRegister(uint8_t offset, uint32_t data) { client->writeRegister(mode, page, offset, data); }

protected:
    std::shared_ptr<RegMappedClient> client;
    uint8_t mode;
    uint8_t page;
};

// ========================================
// CAN Bus
// ========================================

struct CANSocketKey {
    int ifIndex;
    uint8_t clientId;
    uint8_t channel;

    CANSocketKey(int ifIndex, uint8_t clientId, uint8_t channel):
        ifIndex(ifIndex), clientId(clientId), channel(channel) {}

    bool operator==(const CANSocketKey &other) const {
        return (ifIndex == other.ifIndex && channel == other.channel && clientId == other.clientId);
    }
};

struct CANSocketKeyHasher {
    std::size_t operator()(const CANSocketKey &k) const {
        using std::hash;

        // Since client_id and channel are 8-bit integers
        // Just hash the combined 16-bit integer
        return hash<uint64_t>()((((uint64_t) k.clientId) << 40) | (((uint64_t) k.channel) << 32) |
                                ((uint64_t) k.ifIndex));
    }
};

class RegMappedCANClient :
    public CANSocket,
    public RegMappedClient,
    public SocketSingleton<RegMappedCANClient, CANSocketKey, CANSocketKeyHasher> {
public:
    friend class SocketSingleton<RegMappedCANClient, CANSocketKey, CANSocketKeyHasher>;
    // To prevent creating of duplicate sockets (which would break things), there will only be one socket per channel
    // per client per interface per application The factory will need to be used to check if one exists already, and if
    // so, return it static std::shared_ptr<RegMappedCANClient> create(int ifIndex, uint8_t clientId, uint8_t channel);
    // However if all references are removed, then we can destroy it and recreate it next time its needed

    void sendRaw(const std::span<const uint8_t> &data);

    // Configured clientId and channel
    const uint8_t clientId;
    const uint8_t channel;

private:
    // Instance handles
    // static std::unordered_map<CANSocketKey,std::weak_ptr<RegMappedCANClient>,CANSocketKeyHasher> clients;
    RegMappedCANClient(int ifIndex, uint8_t clientId, uint8_t channel);

    // This will hold the received id and data from the frame during the CANSocket frame callback
    std::shared_ptr<std::pair<canid_t, std::vector<uint8_t>>> frameMailbox;
    void handleFrame(canid_t can_id, const std::span<const uint8_t> &data) {
        if (frameMailbox) {
            // If this has data, then we somehow got handleFrame twice before the mailbox was flushed
            // This shouldn't be possible as the poll is only ran for one event loop
            throw std::logic_error("Packet mailbox already has a frame in it");
        }
        frameMailbox = std::make_shared<std::pair<canid_t, std::vector<uint8_t>>>();
        frameMailbox->first = can_id;
        frameMailbox->second.assign(data.begin(), data.end());
    }

    // Function Callbacks
    bool clientRx(const std::span<uint8_t> &buf, unsigned int timeoutMs);

    static bool clientRxCB(uint8_t *buf, size_t len, unsigned int timeout, void *arg) {
        auto inst = (RegMappedCANClient *) arg;
        return inst->clientRx(std::span<uint8_t>(buf, len), timeout);
    }

    static bool clearRxCB(void *arg) {
        auto inst = (RegMappedCANClient *) arg;
        inst->clearRxBuffer();
        return true;
    }

    static bool clientTxCB(const uint8_t *buf, size_t len, void *arg) {
        auto inst = (RegMappedCANClient *) arg;
        inst->sendRaw(std::span<const uint8_t>(buf, len));
        return true;
    }

public:
    RegMappedCANClient(RegMappedCANClient const &) = delete;
    RegMappedCANClient &operator=(RegMappedCANClient const &) = delete;
};

// ========================================
// Ethernet
// ========================================

struct EthernetSocketKey {
    struct in_addr ipAddr;
    uint16_t port;

    EthernetSocketKey(struct in_addr ipAddr, uint16_t port): ipAddr(ipAddr), port(port) {}

    bool operator==(const EthernetSocketKey &other) const {
        return (ipAddr.s_addr == other.ipAddr.s_addr && port == other.port);
    }
};

struct EthernetSocketKeyHasher {
    std::size_t operator()(const EthernetSocketKey &k) const {
        using std::hash;

        // Since client_id and channel are 8-bit integers
        // Just hash the combined 16-bit integer
        return hash<uint64_t>()((((uint64_t) k.port) << 32) | ((uint64_t) k.ipAddr.s_addr));
    }
};

class RegMappedEthernetClient :
    public RegMappedClient,
    public SocketSingleton<RegMappedEthernetClient, EthernetSocketKey, EthernetSocketKeyHasher> {
public:
    friend class SocketSingleton<RegMappedEthernetClient, EthernetSocketKey, EthernetSocketKeyHasher>;
    // To prevent creating of duplicate sockets (which would break things), there will only be one socket per port per
    // IP per application The factory will need to be used to check if one exists already, and if so, return it

    // However if all references are removed, then we can destroy it and recreate it next time its needed
    ~RegMappedEthernetClient();

    void sendRaw(const std::vector<uint8_t> data);

private:
    // Instance handles
    RegMappedEthernetClient(struct in_addr ipAddr, uint16_t port);

    // State Variables
    struct sockaddr_in destaddr;
    int socketFd;

    // Function Callbacks
    bool clientTx(const uint8_t *buf, size_t len);
    bool clientRx(uint8_t *buf, size_t len, unsigned int timeoutMs);
    bool clearRx(void);

    static bool clientRxCB(uint8_t *buf, size_t len, unsigned int timeout, void *arg) {
        auto inst = (RegMappedEthernetClient *) arg;
        return inst->clientRx(buf, len, timeout);
    }

    static bool clearRxCB(void *arg) {
        auto inst = (RegMappedEthernetClient *) arg;
        return inst->clearRx();
    }

    static bool clientTxCB(const uint8_t *buf, size_t len, void *arg) {
        auto inst = (RegMappedEthernetClient *) arg;
        return inst->clientTx(buf, len);
    }

public:
    RegMappedEthernetClient(RegMappedEthernetClient const &) = delete;
    RegMappedEthernetClient &operator=(RegMappedEthernetClient const &) = delete;
};

}  // namespace Canmore
