#pragma once

#include <string>
#include <memory>
#include <vector>
#include <stdexcept>
#include <unordered_map>

#include "SocketSingleton.hpp"

namespace Canmore {

#include "canmore_titan/reg_mapped_client.h"

class RegMappedClientError : public std::runtime_error
{
    public:
        RegMappedClientError(int error_code): runtime_error("Register Mapped Client Error: " + std::to_string(error_code)) {}
};

class RegMappedClient {
    public:
        virtual ~RegMappedClient()=0;

        uint32_t readRegister(uint8_t mode, uint8_t page, uint8_t offset);
        void writeRegister(uint8_t mode, uint8_t page, uint8_t offset, uint32_t data);
        void readArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &dst, uint8_t numWords);
        void writeArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &data);
        void readStringPage(uint8_t mode, uint8_t page, std::string &strOut);

    protected:
        reg_mapped_client_cfg_t clientCfg;
};

class RegisterPage {
    public:
        RegisterPage(std::shared_ptr<RegMappedClient> client, uint8_t mode, uint8_t page): client(client), mode(mode), page(page) {}

        uint32_t readRegister(uint8_t offset)
            {return client->readRegister(mode, page, offset);}

        void writeRegister(uint8_t offset, uint32_t data)
            {client->writeRegister(mode, page, offset, data);}

    protected:
        std::shared_ptr<RegMappedClient> client;
        uint8_t mode;
        uint8_t page;
};

// ========================================
// CAN Bus
// ========================================

struct CANSocketKey
{
    int ifIndex;
    uint8_t clientId;
    uint8_t channel;

    CANSocketKey(int ifIndex, uint8_t clientId, uint8_t channel): ifIndex(ifIndex), clientId(clientId), channel(channel) {}

    bool operator==(const CANSocketKey &other) const {
        return (ifIndex == other.ifIndex
                && channel == other.channel
                && clientId == other.clientId);
    }
};

struct CANSocketKeyHasher
{
    std::size_t operator()(const CANSocketKey& k) const
    {
        using std::hash;

        // Since client_id and channel are 8-bit integers
        // Just hash the combined 16-bit integer
        return hash<uint64_t>()((((uint64_t)k.clientId) << 40) |
                                (((uint64_t)k.channel) << 32) |
                                ((uint64_t)k.ifIndex));
    }
};

class RegMappedCANClient: public RegMappedClient, public SocketSingleton<RegMappedCANClient, CANSocketKey, CANSocketKeyHasher>  {
    public:
        friend class SocketSingleton<RegMappedCANClient, CANSocketKey, CANSocketKeyHasher>;
        // To prevent creating of duplicate sockets (which would break things), there will only be one socket per channel per client per interface per application
        // The factory will need to be used to check if one exists already, and if so, return it
        // static std::shared_ptr<RegMappedCANClient> create(int ifIndex, uint8_t clientId, uint8_t channel);

        // However if all references are removed, then we can destroy it and recreate it next time its needed
        ~RegMappedCANClient();

        void sendRaw(const std::vector<uint8_t> data);

    private:
        // Instance handles
        // static std::unordered_map<CANSocketKey,std::weak_ptr<RegMappedCANClient>,CANSocketKeyHasher> clients;
        RegMappedCANClient(int ifIndex, uint8_t clientId, uint8_t channel);

        // State Variables
        int ifIndex;
        uint8_t clientId;
        uint8_t channel;
        int socketFd;

        // Function Callbacks
        bool clientTx(const uint8_t* buf, size_t len);
        bool clientRx(uint8_t *buf, size_t len, unsigned int timeoutMs);
        bool clearRx(void);

        static bool clientRxCB(uint8_t *buf, size_t len, unsigned int timeout, void* arg) {
            auto inst = (RegMappedCANClient*) arg;
            return inst->clientRx(buf, len, timeout);
        }

        static bool clearRxCB(void* arg) {
            auto inst = (RegMappedCANClient*) arg;
            return inst->clearRx();
        }

        static bool clientTxCB(const uint8_t *buf, size_t len, void* arg) {
            auto inst = (RegMappedCANClient*) arg;
            return inst->clientTx(buf, len);
        }

    public:
        RegMappedCANClient(RegMappedCANClient const &) = delete;
        RegMappedCANClient& operator=(RegMappedCANClient const &) = delete;
};

}