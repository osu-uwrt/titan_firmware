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

static bool waitForEventOrInterrupt(int socketFd, int &reventsOut) {
    // Waits for a new connection (and returns true) or enter is pressed (and returns false)
    struct pollfd fds[2];
    fds[0].fd = socketFd;
    fds[0].events = POLLIN;
    fds[1].fd = STDIN_FILENO;
    fds[1].events = POLLIN;

    while (true) {
        if (poll(fds, 2, -1) < 0) {
            throw std::system_error(errno, std::generic_category(), "poll");
        }

        // Check if we got a socket connection
        if (fds[0].revents) {
            reventsOut = fds[0].revents;
            return true;
        }

        // Check for keypress instead
        else if (fds[1].revents & POLLIN) {
            char c = getchar();
            // If enter (or ctrl+c), then break
            if (c == '\n' || c == '\x03') {
                return false;
            }
            // If a different character, poll again
        }
        else {
            throw std::runtime_error("Poll unexpectedly returned");
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

        else if (req.at(0) == 'g') {
            // TODO: Make this better
            return "xxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxxx"
                   "xxxxxxxxxxxxx000000000400000000000000";
        }

        // TODO: Memory write

        else if (req.at(0) == 'm') {
            return memoryRead(req.substr(1));
        }

        else {
            // std::cout << "Unknown Command '" << req << "'" << std::endl;
            return "";
        }
    }

    void resetState() { detachReceived = false; }
    bool shouldDisconnect() { return detachReceived; }

private:
    std::shared_ptr<Canmore::DebugClient> client;
    bool detachReceived = false;

    std::string processQ(const std::string_view &subcmd) {
        if (subcmd.rfind("Supported", 0) == 0) {
            // subcmd Supported: https://sourceware.org/gdb/onlinedocs/gdb/General-Query-Packets.html#qSupported
            // Report the features supported by the RSP server. As a minimum, just the packet size can be reported.
            // Reporting 4096 packet length, we can probably receive more, but its a good max value
            return "PacketSize=4096;qXfer:features:read+";
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
            uint32_t addr = 0;  // Bounds checked above
            uint32_t last_read = 0x20041ff0;
            for (uint64_t i = 0; i < len; i++) {
                // Refresh from memory if the word changed
                uint32_t cur_idx = (uint32_t) ((offset + i) % 4);
                uint32_t cur_word = (uint32_t) ((offset + i) - cur_idx);

                if (addr != cur_word) {
                    addr = cur_word;
                    if (addr != 0 && addr != 4) {
                        last_read = client->readMemory(addr);
                    }
                    else {
                        last_read = (addr == 0 ? 0x20041ff0 : 0xef);
                    }
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
};

class GDBPacketParser {
public:
    GDBPacketParser(std::shared_ptr<GDBPacketHandler> handler, std::function<void(std::string)> sendRespCb):
        state(PARSE_SOP), handler(handler), sendRespCb(sendRespCb) {
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
                    sendRespCb("-");
                }
            }
            else if (processData) {
                sendRespCb("+");
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
    } state;
    uint8_t calc_checksum;
    uint8_t exp_checksum;

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
    GDBServerSocket(std::shared_ptr<GDBPacketHandler> handler, int socketFd):
        handler(handler), socketFd(socketFd),
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
            while (waitForEventOrInterrupt(socketFd, event)) {
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

private:
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
        if (waitForEventOrInterrupt(socketFd, event)) {
            if (event & POLLIN) {
                int newFd;
                if ((newFd = accept4(socketFd, NULL, NULL, SOCK_CLOEXEC | SOCK_NONBLOCK)) < 0) {
                    throw std::system_error(errno, std::generic_category(), "TCP Socket Accept4");
                }
                return std::make_unique<GDBServerSocket>(handler, newFd);
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
            auto conn = listener.acceptConnection();
            if (!conn) {
                std::cout << COLOR_NOTICE "Break" << COLOR_RESET << std::endl;
                break;
            }
            std::cout << COLOR_HEADER "Accepted new connection" COLOR_RESET << std::endl;
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
