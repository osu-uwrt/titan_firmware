#pragma once

#include <string>
#include <memory>
#include <vector>
#include <stdexcept>

namespace Canmore {

#include "canmore_titan/reg_mapped_client.h"

class RegMappedClientError : public std::runtime_error
{
    public:
        RegMappedClientError(int error_code): runtime_error("Register Mapped Client Error: " + std::to_string(error_code)) {}
};

class RegMappedClient {
    public:
        RegMappedClient(): interfaceModeSet(false) {};
        virtual ~RegMappedClient()=0;

        uint32_t readRegister(uint8_t page, uint8_t offset);
        void writeRegister(uint8_t page, uint8_t offset, uint32_t data);
        void readArray(uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &dst, uint8_t numWords);
        void writeArray(uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &data);
        void readStringPage(uint8_t page, std::string &strOut);

        void configureInterfaceMode(uint8_t mode);

    protected:
        reg_mapped_client_cfg_t clientCfg;

    private:
        bool interfaceModeSet;
};

class RegMappedCANClient: public RegMappedClient {
    public:
        RegMappedCANClient(int ifIndex, uint8_t clientId, uint8_t channel, uint32_t timeoutMs);
        ~RegMappedCANClient();

    private:
        int ifIndex;
        uint8_t clientId;
        uint8_t channel;
        int socketFd;

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
};

}