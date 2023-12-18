#include "GDBServer.hpp"

#include "TerminalDraw.hpp"

#include <arpa/inet.h>
#include <cstring>
#include <functional>
#include <iostream>
#include <memory>
#include <poll.h>
#include <sstream>
#include <sys/socket.h>
#include <unistd.h>

static_assert(EINVAL == 0x16, "EINVAL does not match expected value");
#define ERR_STR_EINVAL "E16"

const static std::string target_xml = "<target version=\"1.0\">"
                                      "<architecture>armv6s-m</architecture>"
                                      "<feature name=\"org.gnu.gdb.arm.m-profile\">"
                                      "<reg name=\"r0\" bitsize=\"32\"/>"
                                      "<reg name=\"r1\" bitsize=\"32\"/>"
                                      "<reg name=\"r2\" bitsize=\"32\"/>"
                                      "<reg name=\"r3\" bitsize=\"32\"/>"
                                      "<reg name=\"r4\" bitsize=\"32\"/>"
                                      "<reg name=\"r5\" bitsize=\"32\"/>"
                                      "<reg name=\"r6\" bitsize=\"32\"/>"
                                      "<reg name=\"r7\" bitsize=\"32\"/>"
                                      "<reg name=\"r8\" bitsize=\"32\"/>"
                                      "<reg name=\"r9\" bitsize=\"32\"/>"
                                      "<reg name=\"r10\" bitsize=\"32\"/>"
                                      "<reg name=\"r11\" bitsize=\"32\"/>"
                                      "<reg name=\"r12\" bitsize=\"32\"/>"
                                      "<reg name=\"sp\" bitsize=\"32\" type=\"data_ptr\"/>"
                                      "<reg name=\"lr\" bitsize=\"32\"/>"
                                      "<reg name=\"pc\" bitsize=\"32\" type=\"code_ptr\"/>"
                                      "<reg name=\"xpsr\" bitsize=\"32\"/>"
                                      "</feature>"
                                      "</target>";

std::string byteToHex(uint8_t byte) {
    const std::string hexLookup = "0123456789ABCDEF";
    return std::string(1, hexLookup.at(byte >> 4)) + hexLookup.at(byte & 0xF);
}

class GDBServerError : public std::runtime_error {
public:
    explicit GDBServerError(const std::string &msg): std::runtime_error(msg) {}
    explicit GDBServerError(const char *msg): std::runtime_error(msg) {}
};

static bool waitForEventOrInterrupt(int socketFd, int &reventsOut, std::function<bool(void)> periodicCheck) {
    // Waits for a new connection (and returns true) or enter is pressed (and returns false)
    struct pollfd fds[2];
    fds[0].fd = socketFd;
    fds[0].events = POLLIN;
    fds[1].fd = STDIN_FILENO;
    fds[1].events = POLLIN;

    while (true) {
        // Poll every second
        if (poll(fds, 2, 1000) < 0) {
            throw std::system_error(errno, std::generic_category(), "poll");
        }

        // Check if we got a socket connection
        if (fds[0].revents) {
            reventsOut = fds[0].revents;
            return true;
        }

        // Check for keypress instead
        if (fds[1].revents & POLLIN) {
            char c = getchar();
            // If enter (or ctrl+c), then break
            if (c == '\n' || c == '\x03') {
                return false;
            }
            // If a different character, poll again
        }

        // Send periodic check function if we don't get a command for a while to make sure we're still alive
        if (periodicCheck) {
            if (!periodicCheck()) {
                return false;
            }
        }
    }
}

static int lookupHexChar(char c) {
    int val;
    if (c >= '0' && c <= '9') {
        val = c - '0';
    }
    else if (c >= 'A' && c <= 'F') {
        val = c - 'A' + 0xA;
    }
    else if (c >= 'a' && c <= 'f') {
        val = c - 'a' + 0xA;
    }
    else {
        val = -1;
    }
    return val;
}

static bool convertHexStr(const std::string_view &data, uint64_t &valOut) {
    // Overflow and Even Check
    if (data.length() > UINT64_WIDTH * 2 || data.length() == 0) {
        return false;
    }

    uint64_t decoded = 0;
    for (char c : data) {
        int nibbleVal = lookupHexChar(c);
        if (nibbleVal < 0) {
            return false;
        }
        decoded <<= 4;
        decoded |= nibbleVal;
    }

    valOut = decoded;
    return true;
}

class GDBPacketHandler {
public:
    GDBPacketHandler(std::shared_ptr<Canmore::DebugClient> client): client(client) {}

    std::string processPacket(const std::string_view &req) {
        if (req.at(0) == '?') {
            // Report signal none
            return "S00";
        }
        else if (req.at(0) == 'c') {
            // Report signal none for stop
            return "S00";
        }
        else if (req.at(0) == 's') {
            // Report signal trap
            return "T05";
        }
        else if (req.at(0) == 'D') {
            // Accept detach and notify to stop server
            detachReceived = true;
            return "OK";
        }
        else if (req.at(0) == 'k') {
            // Accept kill and notify to stop server
            detachReceived = true;
            return "OK";
        }
        else if (req.at(0) == 'H') {
            return "OK";  // Accept any thread set requests (we only report 1, so we should be fine to ack any)
        }
        else if (req.at(0) == 'q') {
            return processQ(req.substr(1));
        }
        else if (req.at(0) == 'Q') {
            return processQUpper(req.substr(1));
        }

        else if (req.at(0) == 'g') {
            std::stringstream regs;
            regs
                << "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
                   "xxxxx";  // Populate regs r0-r12 with unknown

            try {
                uint32_t pc = client->getGDBStubPC();
                uint32_t sp = client->getGDBStubSP();
                uint32_t lr = client->getGDBStubLR();

                // Put in SP register
                for (int i = 0; i < 4; i++) {
                    regs << byteToHex((sp >> (i * 8)) & 0xFF);
                }
                // Put in LR register
                for (int i = 0; i < 4; i++) {
                    regs << byteToHex((lr >> (i * 8)) & 0xFF);
                }

                // Put in PC register
                for (int i = 0; i < 4; i++) {
                    regs << byteToHex((pc >> (i * 8)) & 0xFF);
                }

                // Put in XPSR register
                regs << "00000000";

                return regs.str();
            } catch (Canmore::CanmoreError &e) {
                std::cout << COLOR_ERROR "Error reading registers: " << e.what() << COLOR_RESET << std::endl;
                return "E00";
            }
        }

        else if (req.at(0) == 'G') {
            return "E00";  // Can't set registers
        }

        else if (req.at(0) == 'M') {
            return memoryWrite(req.substr(1));
        }

        else if (req.at(0) == 'm') {
            return memoryRead(req.substr(1));
        }

        else {
            // We must return an empty response on unknown packets
            return "";
        }
    }

    bool ping() {
        try {
            client->ping();
            return true;
        } catch (Canmore::CanmoreError &e) {
            std::cout << COLOR_ERROR "Failed to ping device: " << e.what() << COLOR_RESET << std::endl;
            return false;
        }
    }

    void resetState() {
        detachReceived = false;
        sendAck = true;
    }
    bool shouldDisconnect() { return detachReceived; }
    bool shouldSendAck() { return sendAck; }

private:
    std::shared_ptr<Canmore::DebugClient> client;
    bool detachReceived = false;
    bool sendAck = true;

    std::string processQ(const std::string_view &subcmd) {
        if (subcmd.rfind("Supported", 0) == 0) {
            // subcmd Supported: https://sourceware.org/gdb/onlinedocs/gdb/General-Query-Packets.html#qSupported
            // Report the features supported by the RSP server. As a minimum, just the packet size can be reported.
            // Reporting 4096 packet length, we can probably receive more, but its a good max value
            return "PacketSize=4000;qXfer:features:read+;QStartNoAckMode+";
        }
        else if (subcmd == "Attached") {
            return "1";  // Process already running
        }
        else if (subcmd == "fThreadInfo") {
            return "m 1";  // Only 1 thread - thread id 1
        }
        else if (subcmd == "sThreadInfo") {
            return "l";  // No other threads in the system
        }
        else if (subcmd == "C") {
            return "T01;";  // Report thread id 1
        }
        else if (subcmd.rfind("Xfer", 0) == 0) {
            return processXfer(subcmd);  // Process XFER request, additional logic
        }
        else {
            return "";
        }
    }

    std::string processQUpper(const std::string_view &subcmd) {
        if (subcmd == "StartNoAckMode") {
            sendAck = false;
            return "OK";
        }
        else {
            return "";
        }
    }

    std::string processXfer(const std::string_view &subcmd) {
        // Split up the command
        size_t hdr_obj_sep = subcmd.find(':', 0);
        if (hdr_obj_sep == std::string::npos) {
            return "E00";
        }
        hdr_obj_sep++;
        size_t obj_op_sep = subcmd.find(':', hdr_obj_sep);
        if (obj_op_sep == std::string::npos) {
            return "E00";
        }
        obj_op_sep++;
        size_t op_annex_sep = subcmd.find(':', obj_op_sep);
        if (op_annex_sep == std::string::npos) {
            return "E00";
        }
        op_annex_sep++;
        size_t annex_offset_sep = subcmd.find(':', op_annex_sep);
        if (annex_offset_sep == std::string::npos) {
            return "E00";
        }
        annex_offset_sep++;
        if (subcmd.find(':', annex_offset_sep + 1) != std::string::npos) {
            // Should be last colon, error if not
            return "E00";
        }
        size_t offset_len_sep = subcmd.find(',', annex_offset_sep);
        if (offset_len_sep == std::string::npos) {
            return "E00";
        }
        offset_len_sep++;

        // Extract the strings from the separator
        auto obj = subcmd.substr(hdr_obj_sep, obj_op_sep - hdr_obj_sep - 1);
        auto op = subcmd.substr(obj_op_sep, op_annex_sep - obj_op_sep - 1);
        auto annex = subcmd.substr(op_annex_sep, annex_offset_sep - op_annex_sep - 1);

        // Decode offset and length hex strings
        uint64_t offset, len;
        if (!convertHexStr(subcmd.substr(annex_offset_sep, offset_len_sep - annex_offset_sep - 1), offset)) {
            return "E00";
        }
        if (!convertHexStr(subcmd.substr(offset_len_sep), len)) {
            return "E00";
        }

        if (op != "read") {
            return "";  // Invalid request, we can only read
        }

        if (obj != "features") {
            return "";  // Unrecognized object, return empty string according to docs
        }

        if (annex != "target.xml") {
            return "E00";  // Invalid annex, return error code
        }

        // Grab the requested portion of target_xml
        if (offset > target_xml.size()) {
            return "l";
        }
        else if (offset + len > target_xml.size()) {
            return "l" + target_xml.substr(offset);
        }
        else {
            return "m" + target_xml.substr(offset, len);
        }
    }

    std::string memoryRead(const std::string_view &subcmd) {
        size_t offset_len_sep = subcmd.find(',');
        if (offset_len_sep == std::string::npos) {
            return ERR_STR_EINVAL;
        }
        uint64_t offset, len;
        if (!convertHexStr(subcmd.substr(0, offset_len_sep), offset)) {
            return ERR_STR_EINVAL;
        }
        if (!convertHexStr(subcmd.substr(offset_len_sep + 1), len)) {
            return ERR_STR_EINVAL;
        }
        // Make sure all address parts are in bounds
        if (offset > UINT32_MAX || len > UINT32_MAX || offset + len > UINT32_MAX) {
            return ERR_STR_EINVAL;
        }

        // Configure output stream
        std::stringstream result;

        // Begin reading in the memory
        bool read_some = false;
        try {
            // Read, re-aligning as required
            // Sort of hack to prevent constant readings of PC by gdb
            // We'll say we've already read address 0 (and also check for addr 4 inside)
            bool addr_valid = false;
            uint32_t addr = 0;
            uint32_t last_read = 0;
            for (uint64_t i = 0; i < len; i++) {
                // Refresh from memory if the word changed
                uint32_t cur_idx = (uint32_t) ((offset + i) % 4);
                uint32_t cur_word = (uint32_t) ((offset + i) - cur_idx);

                if (addr != cur_word || !addr_valid) {
                    addr_valid = true;
                    addr = cur_word;
                    last_read = client->readMemory(addr);
                }

                // Now take the part of the word we're on and add it to the stringstream
                result << byteToHex((last_read >> (cur_idx * 8)) & 0xFF);
            }
        } catch (Canmore::CanmoreError &e) {
            if (read_some) {
                return result.str();
            }
            else {
                // Couldn't read memory at requested address
                return ERR_STR_EINVAL;
            }
        }

        return result.str();
    }

    std::string memoryWrite(const std::string_view &subcmd) {
        size_t offset_len_sep = subcmd.find(',');
        size_t len_data_sep = subcmd.find(':');
        if (len_data_sep == std::string::npos || offset_len_sep == std::string::npos ||
            offset_len_sep >= len_data_sep) {
            return ERR_STR_EINVAL;
        }
        uint64_t offset, len;
        if (!convertHexStr(subcmd.substr(0, offset_len_sep), offset)) {
            return ERR_STR_EINVAL;
        }
        if (!convertHexStr(subcmd.substr(offset_len_sep + 1, len_data_sep - offset_len_sep - 1), len)) {
            return ERR_STR_EINVAL;
        }
        // Make sure all address parts are in bounds
        if (offset > UINT32_MAX || len > UINT32_MAX || offset + len > UINT32_MAX) {
            return ERR_STR_EINVAL;
        }

        try {
            // Write, aligning as required
            uint32_t val = 0;
            uint32_t first_idx = offset % 4;
            size_t hex_offset = len_data_sep + 1;
            for (uint64_t i = 0; i < len; i++) {
                // Make sure the string is actually long enough to hold the hex data
                if (subcmd.length() - hex_offset < 2) {
                    return ERR_STR_EINVAL;
                }

                // Decode the next byte to send
                uint64_t byte_val;
                if (!convertHexStr(subcmd.substr(hex_offset, 2), byte_val)) {
                    return ERR_STR_EINVAL;
                }
                hex_offset += 2;

                // Refresh from memory if the word changed
                uint32_t cur_idx = (uint32_t) ((offset + i) % 4);
                uint32_t cur_word = (uint32_t) ((offset + i) - cur_idx);

                val |= (byte_val << (8 * cur_idx));

                // Write if it's the last byte in the word, or we are out of data to write
                if (cur_idx == 3 || i + 1 == len) {
                    // If we are doing a partial write, we'll need to read the previous value
                    // Compute it real quick
                    uint32_t write_mask = ((1llu << ((cur_idx - first_idx + 1) * 8)) - 1) << (first_idx * 8);
                    uint32_t keep_mask = ~write_mask;
                    if (keep_mask) {
                        // Read back previous value if we need to keep any bits
                        uint32_t prev_val = client->readMemory(cur_word);
                        val = (prev_val & keep_mask) | (val & write_mask);
                    }

                    // Write the new value
                    client->writeMemory(cur_word, val);

                    // Reset variables for the next word
                    val = 0;
                    first_idx = 0;
                }
            }

            // Write complete
            return "OK";
        } catch (Canmore::CanmoreError &e) {
            // Some sort of error occurred while writing
            return ERR_STR_EINVAL;
        }
    }
};

class GDBPacketParser {
public:
    GDBPacketParser(std::shared_ptr<GDBPacketHandler> handler, std::function<void(std::string)> sendRespCb):
        handler(handler), sendRespCb(sendRespCb) {
        handler->resetState();
    }

    void processData(const std::string_view &data) {
        for (char c : data) {
            bool parseFail = false;
            bool processData = false;

            // Character Decoding
            // State Machines... My Favorite :')

            // Handle any critical control characters
            // Note in PARSE_SOP, this is the only way to reset it
            if (c == '$') {
                state = PARSE_DATA;
                cmdBuf.clear();
                calc_checksum = 0;
            }

            else if (c == '#') {
                if (state == PARSE_DATA) {
                    state = PARSE_CHECKSUM1;
                }
                else {
                    parseFail = true;
                }
            }

            else if (c == '}') {
                if (state == PARSE_DATA) {
                    calc_checksum += c;
                    state = PARSE_ESCAPED_DATA;
                }
                else {
                    parseFail = true;
                }
            }

            // Handle remaining state data
            else if (state == PARSE_DATA) {
                calc_checksum += c;
                cmdBuf.push_back(c);
            }
            else if (state == PARSE_ESCAPED_DATA) {
                calc_checksum += c;
                cmdBuf.push_back(c ^ 0x20);
                state = PARSE_DATA;
            }

            else if (state == PARSE_CHECKSUM1) {
                int val = lookupHexChar(c);
                if (val < 0) {
                    parseFail = true;
                }
                else {
                    exp_checksum = val << 4;
                    state = PARSE_CHECKSUM2;
                }
            }
            else if (state == PARSE_CHECKSUM2) {
                int val = lookupHexChar(c);
                if (val < 0) {
                    parseFail = true;
                }
                else {
                    exp_checksum += val;
                    if (exp_checksum == calc_checksum) {
                        processData = true;
                    }
                    else {
                        parseFail = true;
                    }
                }
            }

            // Handle data processing
            if (parseFail) {
                // Only send NAK if we aren't at SOP, if so, don't do anything until we get the start of another packet
                // Prevents garbage from breaking everything
                if (state != PARSE_SOP) {
                    state = PARSE_SOP;
                    if (handler->shouldSendAck()) {
                        sendRespCb("-");
                    }
                }
            }
            else if (processData) {
                if (handler->shouldSendAck()) {
                    sendRespCb("+");
                }
                auto packetView = std::string_view(cmdBuf.data(), cmdBuf.size());
                auto resp = handler->processPacket(packetView);
                transmitResponse(resp);
                state = PARSE_SOP;
            }
        }
    }

private:
    enum PraseState {
        PARSE_SOP,
        PARSE_DATA,
        PARSE_ESCAPED_DATA,

        PARSE_CHECKSUM1,
        PARSE_CHECKSUM2,
    } state = PARSE_SOP;
    uint8_t calc_checksum = 0;
    uint8_t exp_checksum = 0;

    std::shared_ptr<GDBPacketHandler> handler;
    std::function<void(std::string)> sendRespCb;
    std::vector<char> cmdBuf;

    void transmitResponse(const std::string_view &data) {
        // Compute the string response
        std::stringstream packet;

        // Encode start of packet
        packet << "$";

        // Encode contents, escaping characters as required
        size_t last_loc = 0;
        do {
            size_t loc = data.find_first_of("$#*}", last_loc);
            if (loc == std::string::npos) {
                if (last_loc == 0) {
                    packet << data;
                }
                else {
                    packet << data.substr(last_loc);
                }
            }
            else {
                packet << data.substr(last_loc, loc - last_loc);
                packet << "}" << (char) (data.at(loc) ^ 0x20);
                loc++;
            }
            last_loc = loc;
        } while (last_loc < data.size());

        // Compute checksum
        uint8_t checksum = 0;
        for (char c : data) {
            checksum += c;
        }

        // Add end of packet + checksum
        packet << "#" << byteToHex(checksum);

        // Send the encoded packet
        sendRespCb(packet.str());
    }
};

class GDBServerSocket {
public:
    GDBServerSocket(std::shared_ptr<GDBPacketHandler> handler, int socketFd, sockaddr socketAddr):
        socketAddr(socketAddr), handler(handler), socketFd(socketFd),
        packetParser(handler, std::bind(&GDBServerSocket::send, this, std::placeholders::_1)) {}

    ~GDBServerSocket() {
        if (socketFd >= 0) {
            close(socketFd);
        }
    }

    bool serve() {
        try {
            // Returns true if another connection can be accepted, false if the server should quit
            int event = 0;
            while (waitForEventOrInterrupt(socketFd, event, std::bind(&GDBPacketHandler::ping, handler.get()))) {
                if (event & POLLIN) {
                    // New data to receive
                    int recvlen = read(socketFd, rxbuf, sizeof(rxbuf));
                    if (recvlen > 0) {
                        auto rxview = std::string_view(rxbuf, recvlen);
                        packetParser.processData(rxview);
                        if (handler->shouldDisconnect()) {
                            shutdown(socketFd, SHUT_RDWR);
                            return true;
                        }
                    }
                    else if (recvlen == 0) {
                        shutdown(socketFd, SHUT_RDWR);
                        return true;
                    }
                    else {
                        throw GDBServerError(std::string("Error During Read: ") + std::strerror(errno));
                    }
                }
                else {
                    throw GDBServerError("Unexpected Poll Event: " + std::to_string(event));
                }
            }

            // Interrupted by user, shut down server
            shutdown(socketFd, SHUT_RDWR);
            return false;
        } catch (GDBServerError &e) {
            std::cout << COLOR_ERROR "Connection Error: " << e.what() << COLOR_RESET << std::endl;
            shutdown(socketFd, SHUT_RDWR);
            return true;  // It's okay to start another connection after connection error
        }
    }

    std::string formatAddr() {
        char s[32];
        uint16_t port = 0;

        switch (socketAddr.sa_family) {
        case AF_INET: {
            struct sockaddr_in *sa = (struct sockaddr_in *) &socketAddr;
            inet_ntop(AF_INET, &sa->sin_addr, s, sizeof(s));
            port = sa->sin_port;
            break;
        }

        case AF_INET6: {
            struct sockaddr_in6 *sa = (struct sockaddr_in6 *) &socketAddr;
            inet_ntop(AF_INET6, &sa->sin6_addr, s, sizeof(s));
            port = sa->sin6_port;
            break;
        }

        default:
            return "Unknown AF " + std::to_string(socketAddr.sa_family);
        }

        return s + (":" + std::to_string(port));
    }

private:
    sockaddr socketAddr;
    std::shared_ptr<GDBPacketHandler> handler;
    int socketFd;
    GDBPacketParser packetParser;
    char rxbuf[1024];

    void send(const std::string &data) {
        int len = write(socketFd, data.data(), data.size());
        if (len < 0) {
            throw GDBServerError(std::string("Error During Write: ") + std::strerror(errno));
        }
        if ((size_t) len != data.size()) {
            throw GDBServerError("Failed to write all data to socket");
        }
    }
};

class GDBServerListener {
public:
    GDBServerListener(uint16_t port, std::shared_ptr<GDBPacketHandler> handler): handler(handler), socketFd(-1) {
        // Open socket
        if ((socketFd = socket(AF_INET, SOCK_STREAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0)) < 0) {
            throw std::system_error(errno, std::generic_category(), "TCP Socket Create");
        }

        // Bind to port for GDB server
        struct sockaddr_in addr;
        addr.sin_family = AF_INET;
        addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        addr.sin_port = htons(port);

        if (bind(socketFd, (struct sockaddr *) &addr, sizeof(addr)) < 0) {
            close(socketFd);
            socketFd = -1;
            throw std::system_error(errno, std::generic_category(), "TCP Socket Bind");
        }

        // Begin listening for TCP connections
        if (listen(socketFd, 1) < 0) {
            close(socketFd);
            socketFd = -1;
            throw std::system_error(errno, std::generic_category(), "TCP Socket Listen");
        }
    }

    ~GDBServerListener() {
        if (socketFd >= 0) {
            close(socketFd);
        }
    }

    std::unique_ptr<GDBServerSocket> acceptConnection() {
        // Returns nullptr if interrupted by enter/ctrl+c
        int event = 0;
        if (waitForEventOrInterrupt(socketFd, event, std::bind(&GDBPacketHandler::ping, handler.get()))) {
            if (event & POLLIN) {
                int newFd;
                sockaddr addr;
                socklen_t addrlen = sizeof(addr);
                if ((newFd = accept4(socketFd, &addr, &addrlen, SOCK_CLOEXEC | SOCK_NONBLOCK)) < 0) {
                    throw std::system_error(errno, std::generic_category(), "TCP Socket Accept4");
                }
                return std::make_unique<GDBServerSocket>(handler, newFd, addr);
            }
            else {
                throw std::runtime_error("Unexpected error event on server: " + std::to_string(event));
            }
        }
        else {
            return nullptr;
        }
    }

private:
    std::shared_ptr<GDBPacketHandler> handler;
    int socketFd;
};

void runGdbServer(uint16_t port, std::shared_ptr<Canmore::DebugClient> client) {
    auto handler = std::make_shared<GDBPacketHandler>(client);

    try {
        std::cout << COLOR_NAME "Listening for connections on localhost:" << port
                  << " (Press Enter or Ctrl+C to exit)" COLOR_RESET << std::endl;
        GDBServerListener listener(port, handler);
        while (true) {
            std::string connAddr;
            auto conn = listener.acceptConnection();
            if (!conn) {
                std::cout << COLOR_NOTICE "Break" << COLOR_RESET << std::endl;
                break;
            }
            std::cout << COLOR_HEADER "Accepted new connection from " << conn->formatAddr() << COLOR_RESET << std::endl;
            if (!conn->serve()) {
                std::cout << COLOR_NOTICE "Break" COLOR_RESET << std::endl;
                break;
            }
            std::cout << COLOR_HEADER "Connection closed" COLOR_RESET << std::endl;
            conn.reset();
        }
    } catch (std::exception &e) {
        std::cout << COLOR_ERROR "Exception caught while running GDB Server:" COLOR_RESET << std::endl;
        std::cout << COLOR_ERROR "  what(): " << e.what() << COLOR_RESET << std::endl;
    }
}
