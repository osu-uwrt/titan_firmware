#include <algorithm>

#include "RegMappedClient.hpp"

using namespace Canmore;

RegMappedClient::~RegMappedClient() {}

uint32_t RegMappedClient::readRegister(uint8_t mode, uint8_t page, uint8_t offset) {
    clientCfg.control_interface_mode = mode;

    uint32_t data;
    int ret = reg_mapped_client_read_register(&clientCfg, page, offset, &data);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret);
    }

    return data;
}

void RegMappedClient::writeRegister(uint8_t mode, uint8_t page, uint8_t offset, uint32_t data) {
    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_write_register(&clientCfg, page, offset, data);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret);
    }
}

void RegMappedClient::readArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &dst, uint8_t numWords) {
    clientCfg.control_interface_mode = mode;

    auto buf = new uint32_t[numWords];
    int ret = reg_mapped_client_read_array(&clientCfg, page, offsetStart, buf, numWords);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        delete buf;
        throw RegMappedClientError(ret);
    }

    dst.assign(buf, buf + numWords);
    delete buf;
}

void RegMappedClient::writeArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &data) {
    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_write_array(&clientCfg, page, offsetStart, data.data(), data.size());
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret);
    }
}

void RegMappedClient::readStringPage(uint8_t mode, uint8_t page, std::string &strOut) {
    const size_t strMaxSize = REG_MAPPED_PAGE_SIZE + 1;
    auto strArray = new char[strMaxSize];

    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_read_string_page(&clientCfg, page, strArray, strMaxSize);
    if (ret < 0) {
        delete strArray;
        throw RegMappedClientError(-ret);
    }

    strOut.assign(strArray, ret);
    delete strArray;
}
