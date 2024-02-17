#include "canmore_cpp/RegMappedClient.hpp"

#include <algorithm>

using namespace Canmore;

RegMappedClient::~RegMappedClient() {}

uint32_t RegMappedClient::readRegister(uint8_t mode, uint8_t page, uint8_t offset) {
    clientCfg.control_interface_mode = mode;

    uint32_t data;
    int ret = reg_mapped_client_read_register(&clientCfg, page, offset, &data);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret, mode, page, offset, false);
    }

    return data;
}

void RegMappedClient::writeRegister(uint8_t mode, uint8_t page, uint8_t offset, uint32_t data) {
    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_write_register(&clientCfg, page, offset, data);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret, mode, page, offset, true);
    }
}

void RegMappedClient::readArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &dst,
                                uint8_t numWords) {
    clientCfg.control_interface_mode = mode;

    auto buf = new uint32_t[numWords];
    int ret = reg_mapped_client_read_array(&clientCfg, page, offsetStart, buf, numWords);
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        delete buf;
        throw RegMappedClientError(ret, mode, page, offsetStart, numWords, false);
    }

    dst.assign(buf, buf + numWords);
    delete buf;
}

void RegMappedClient::writeArray(uint8_t mode, uint8_t page, uint8_t offsetStart, std::vector<uint32_t> &data) {
    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_write_array(&clientCfg, page, offsetStart, data.data(), data.size());
    if (ret != REG_MAPPED_RESULT_SUCCESSFUL) {
        throw RegMappedClientError(ret, mode, page, offsetStart, data.size(), true);
    }
}

void RegMappedClient::writeStringPage(uint8_t mode, uint8_t page, const std::string &data) {
    // Convert the 8-bit string into a 32-bit array that can be written
    std::vector<uint32_t> dataArray((data.length() + 3) / 4, 0);
    size_t bytesSize = data.size();
    for (size_t word = 0; (word * 4) < bytesSize; word++) {
        uint32_t value = 0;
        for (int byteOff = 0; byteOff + (word * 4) < bytesSize && byteOff < 4; byteOff++) {
            value |= data.at(byteOff + (word * 4)) << (8 * byteOff);
        }

        dataArray.at(word) = value;
    }

    // Finally write the args converted to a word array
    writeArray(mode, page, 0, dataArray);
}

std::string RegMappedClient::readStringPage(uint8_t mode, uint8_t page) {
    const size_t strMaxSize = REG_MAPPED_PAGE_SIZE + 1;
    auto strArray = new char[strMaxSize];

    clientCfg.control_interface_mode = mode;
    int ret = reg_mapped_client_read_string_page(&clientCfg, page, strArray, strMaxSize);
    if (ret < 0) {
        delete[] strArray;
        throw RegMappedClientError(-ret, mode, page);
    }

    auto readStr = std::string(strArray);
    delete[] strArray;

    return readStr;
}
