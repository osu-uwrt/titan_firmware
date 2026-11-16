#include "canmore_cpp/CANRegistry.hpp"
#include "canmore_cpp/CANTransport.hpp"
#include "canmore_cpp/Cannelloni.hpp"
#include "canmore_cpp/FdCompat.hpp"
#include "canmore_cpp/PollFD.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <fcntl.h>
#include <functional>
#include <glob.h>
#include <iomanip>
#include <iostream>
#include <libusb.h>
#include <map>
#include <netdb.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>

using namespace Canmore;

// ========================================
// Helpers
// ========================================

// A transport that hands every frame to a callback
class Tap final : public CANTransport {
public:
    Tap(const std::string &spec, std::function<void(const CANFrame &)> callback):
        CANTransport(spec), callback(callback) {}

    using CANTransport::transmitFrameNoexcept;
    using CANTransport::usingCanFd;

protected:
    void handleFrame(const CANFrame &frame) override { callback(frame); }

private:
    std::function<void(const CANFrame &)> callback;
};

// Wakes the main loop on Ctrl-C / SIGTERM
static int stopFd = -1;
static volatile sig_atomic_t stopRequested = 0;

static void stopHandler(int) {
    stopRequested = 1;
    FdCompat::notifyEvent(stopFd);
}

class StopWatcher final : public PollFDHandler {
public:
    StopWatcher() {
        stopFd = FdCompat::createEvent();
        descriptor = PollFDDescriptor::create(*this, stopFd, POLLIN);
        signal(SIGINT, stopHandler);
        signal(SIGTERM, stopHandler);
        signal(SIGPIPE, SIG_IGN);
    }

    void populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) override {
        descriptors.push_back(descriptor);
    }

protected:
    void handleEvent(const pollfd &fd) override { (void) fd; }

private:
    std::shared_ptr<PollFDDescriptor> descriptor;
};

static void runUntilStopped(PollGroup &group, const std::function<void()> &periodic = nullptr) {
    while (!stopRequested) {
        group.processEvent(1000);
        if (periodic)
            periodic();
    }
}

static double now() {
    return std::chrono::duration<double>(std::chrono::system_clock::now().time_since_epoch()).count();
}

static std::string formatFrame(const std::string &name, const CANFrame &frame) {
    std::stringstream ss;
    ss << "  " << name << "  " << std::uppercase << std::hex << std::setfill('0');
    if (frame.id & CAN_EFF_FLAG)
        ss << std::setw(8) << (frame.id & CAN_EFF_MASK);
    else
        ss << "     " << std::setw(3) << (frame.id & CAN_SFF_MASK);

    ss << std::dec << std::setfill(' ');
    if (frame.fd)
        ss << "  [" << std::setw(2) << std::setfill('0') << (int) frame.len << "]" << (frame.brs ? " B" : "  ");
    else
        ss << "   [" << (int) frame.len << "]";
    if (frame.id & CAN_RTR_FLAG) {
        ss << "  remote request";
        return ss.str();
    }

    ss << std::hex << std::uppercase << std::setfill('0');
    for (int i = 0; i < frame.len; i++) {
        ss << " " << std::setw(2) << (int) frame.data[i];
    }
    return ss.str();
}

// cansend's syntax: <id>#<data>, <id>#R, <id>##<flags><data> for CAN FD (flags 1 = BRS). 8 digit ids are extended
static CANFrame parseFrame(const std::string &str) {
    CANFrame frame;
    size_t hash = str.find('#');
    if (hash == std::string::npos || hash == 0)
        throw std::invalid_argument("Frames look like 123#DEADBEEF, 12345678#00, 123#R or 123##1DEADBEEF");

    std::string id = str.substr(0, hash);
    frame.id = std::stoul(id, nullptr, 16);
    if (id.size() > 3)
        frame.id |= CAN_EFF_FLAG;

    std::string data = str.substr(hash + 1);
    if (!data.empty() && data[0] == '#') {
        if (data.size() < 2)
            throw std::invalid_argument("CAN FD frames need a flags digit: 123##1DEADBEEF");
        frame.fd = true;
        frame.brs = (data[1] - '0') & 1;
        data = data.substr(2);
    }
    else if (data == "R" || data == "r") {
        frame.id |= CAN_RTR_FLAG;
        return frame;
    }

    std::string hex;
    for (char c : data) {
        if (c != '.')
            hex += c;
    }
    if (hex.size() % 2 != 0 || hex.size() / 2 > (size_t) (frame.fd ? CANFD_MAX_DLEN : CAN_MAX_DLEN))
        throw std::invalid_argument("Bad frame data: " + data);
    frame.len = hex.size() / 2;
    for (size_t i = 0; i < frame.len; i++) {
        frame.data[i] = std::stoul(hex.substr(2 * i, 2), nullptr, 16);
    }
    return frame;
}

// ========================================
// pcap output
// ========================================

// LINKTYPE_CAN_SOCKETCAN: each packet is the Linux can(fd)_frame, id in network byte order
class PcapWriter {
public:
    PcapWriter(const std::string &path) {
        file = (path == "-" ? stdout : fopen(path.c_str(), "wb"));
        if (!file)
            throw std::system_error(errno, std::generic_category(), path);

        uint32_t header[6] = { 0xa1b2c3d4, 0x00040002, 0, 0, 72, 227 };
        fwrite(header, sizeof(header), 1, file);
        fflush(file);
    }

    ~PcapWriter() {
        if (file != stdout)
            fclose(file);
    }

    void write(const CANFrame &frame) {
        double t = now();
        uint8_t pkt[8 + CANFD_MAX_DLEN] = {};
        uint32_t id = htonl(frame.id);
        memcpy(pkt, &id, 4);
        pkt[4] = frame.len;
        // CANFD_BRS, and CANFD_FDF to mark CAN FD
        pkt[5] = (frame.fd ? 0x04 | (frame.brs ? 0x01 : 0) : 0);
        memcpy(pkt + 8, frame.data, frame.len);

        uint32_t len = 8 + frame.len;
        uint32_t rec[4] = { (uint32_t) t, (uint32_t) ((t - (uint32_t) t) * 1000000), len, len };
        fwrite(rec, sizeof(rec), 1, file);
        fwrite(pkt, len, 1, file);
        fflush(file);
    }

private:
    FILE *file;
};

// ========================================
// cannelloni hub
// ========================================

// Serves a bus to cannelloni peers like firmware_simulator's hub does: it learns peers from what they send (an empty
// datagram is a keepalive), sends each one the bus's frames and every other peer's, and forgets peers silent for 10s
class CannelloniHub final : public PollFDHandler {
public:
    CannelloniHub(Tap &bus, const std::string &listen): bus(bus), seq(0) {
        size_t colon = listen.rfind(':');
        if (colon == std::string::npos)
            throw std::invalid_argument("--listen takes HOST:PORT");
        std::string host = listen.substr(0, colon);
        std::string port = listen.substr(colon + 1);

        struct addrinfo hints = {};
        hints.ai_family = AF_UNSPEC;
        hints.ai_socktype = SOCK_DGRAM;
        hints.ai_flags = AI_PASSIVE;
        struct addrinfo *res;
        int rc = getaddrinfo(host.c_str(), port.c_str(), &hints, &res);
        if (rc != 0)
            throw std::runtime_error("--listen " + listen + ": " + gai_strerror(rc));

        fd = FdCompat::openSocket(res->ai_family, SOCK_DGRAM, 0, true);
        if (fd < 0 || bind(fd, res->ai_addr, res->ai_addrlen) < 0) {
            int err = errno;
            freeaddrinfo(res);
            throw std::system_error(err, std::generic_category(), "bind " + listen);
        }
        freeaddrinfo(res);

        // Read back the port, in case it was 0
        struct sockaddr_storage addr;
        socklen_t addrLen = sizeof(addr);
        getsockname(fd, (struct sockaddr *) &addr, &addrLen);
        char hostBuf[NI_MAXHOST], portBuf[NI_MAXSERV];
        getnameinfo((struct sockaddr *) &addr, addrLen, hostBuf, sizeof(hostBuf), portBuf, sizeof(portBuf),
                    NI_NUMERICHOST | NI_NUMERICSERV);
        std::string boundHost = hostBuf;
        if (boundHost == "0.0.0.0" || boundHost == "::")
            boundHost = "127.0.0.1";
        address = boundHost + ":" + portBuf;

        descriptor = PollFDDescriptor::create(*this, fd, POLLIN);
    }

    ~CannelloniHub() { close(fd); }

    // Where local clients reach it
    std::string address;

    void sendToPeers(const CANFrame &frame) {
        auto packet = Cannelloni::encode(&frame, 1, seq++);
        sendExcept(packet.data(), packet.size(), "");
    }

    void populateFds(std::vector<std::weak_ptr<PollFDDescriptor>> &descriptors) override {
        descriptors.push_back(descriptor);
    }

protected:
    void handleEvent(const pollfd &pfd) override {
        if (!(pfd.revents & POLLIN))
            return;

        uint8_t buf[65536];
        struct sockaddr_storage addr;
        socklen_t addrLen = sizeof(addr);
        ssize_t len = recvfrom(fd, buf, sizeof(buf), 0, (struct sockaddr *) &addr, &addrLen);
        if (len < 0)
            return;

        std::string key((const char *) &addr, addrLen);
        auto &peer = peers[key];
        peer.addr = addr;
        peer.addrLen = addrLen;
        peer.lastSeen = std::chrono::steady_clock::now();

        std::vector<CANFrame> frames;
        if (!Cannelloni::decode(buf, len, frames) || frames.empty())
            return;

        // Onto the bus, and to the other peers as on a shared vcan
        for (auto &frame : frames) {
            if (!bus.transmitFrameNoexcept(frame))
                dropped++;
        }
        sendExcept(buf, len, key);
    }

private:
    struct Peer {
        struct sockaddr_storage addr;
        socklen_t addrLen;
        std::chrono::steady_clock::time_point lastSeen;
    };

    Tap &bus;
    int fd;
    uint8_t seq;
    std::map<std::string, Peer> peers;
    std::shared_ptr<PollFDDescriptor> descriptor;
    size_t dropped = 0;

    void sendExcept(const uint8_t *data, size_t len, const std::string &except) {
        auto cutoff = std::chrono::steady_clock::now() - std::chrono::seconds(10);
        auto itr = peers.begin();
        while (itr != peers.end()) {
            if (itr->second.lastSeen < cutoff) {
                itr = peers.erase(itr);
                continue;
            }
            if (itr->first != except)
                sendto(fd, data, len, 0, (struct sockaddr *) &itr->second.addr, itr->second.addrLen);
            itr++;
        }
    }
};

// ========================================
// Commands
// ========================================

static const uint16_t gsUsbIds[][2] = {
    { 0x1d50, 0x606f }, { 0x1209, 0x2323 }, { 0x1cd2, 0x606f },
    { 0x16d0, 0x10b8 }, { 0x16d0, 0x0f30 }, { 0x1209, 0xca01 },
};
static const uint16_t slcanIds[][2] = { { 0x16d0, 0x117e } };

static int cmdList() {
    std::cout << "CAN buses (what upload_tool, canmore_cli and the agent can use):" << std::endl;
    auto buses = enumerateCANDevices(true);
    if (buses.empty())
        std::cout << "  (none)" << std::endl;
    for (auto &bus : buses) {
        std::cout << "  " << std::left << std::setw(12) << bus.name << " " << bus.description << std::endl;
    }

    std::cout << std::endl << "USB CAN adapters (bring one up with canmore-can up <spec>):" << std::endl;
    libusb_context *ctx;
    bool any = false;
    if (libusb_init(&ctx) == 0) {
        libusb_device **list;
        ssize_t count = libusb_get_device_list(ctx, &list);
        for (ssize_t i = 0; i < count; i++) {
            struct libusb_device_descriptor desc;
            if (libusb_get_device_descriptor(list[i], &desc) != 0)
                continue;

            std::string kind;
            for (auto &id : gsUsbIds) {
                if (desc.idVendor == id[0] && desc.idProduct == id[1])
                    kind = "gsusb";
            }
            for (auto &id : slcanIds) {
                if (desc.idVendor == id[0] && desc.idProduct == id[1])
                    kind = "slcan";
            }
            if (kind.empty())
                continue;

            char serial[128] = "?";
            libusb_device_handle *h;
            if (desc.iSerialNumber && libusb_open(list[i], &h) == 0) {
                libusb_get_string_descriptor_ascii(h, desc.iSerialNumber, (unsigned char *) serial, sizeof(serial));
                libusb_close(h);
            }

            any = true;
            std::stringstream ids;
            ids << std::hex << std::setfill('0') << std::setw(4) << desc.idVendor << ":" << std::setw(4)
                << desc.idProduct;
            if (kind == "gsusb")
                std::cout << "  gsusb:" << serial << "  (gs_usb/candleLight " << ids.str() << ")" << std::endl;
            else
                std::cout << "  slcan:<serial port>  (CANable slcan " << ids.str() << ", serial " << serial << ")"
                          << std::endl;
        }
        if (count >= 0)
            libusb_free_device_list(list, 1);
        libusb_exit(ctx);
    }
    if (!any)
        std::cout << "  (none)" << std::endl;

    glob_t g;
    if (glob("/dev/cu.usbmodem*", 0, NULL, &g) == 0 || glob("/dev/ttyACM*", 0, NULL, &g) == 0) {
        std::cout << std::endl << "Serial ports (for slcan:<port>):";
        for (size_t i = 0; i < g.gl_pathc; i++)
            std::cout << " " << g.gl_pathv[i];
        std::cout << std::endl;
        globfree(&g);
    }
    return 0;
}

static std::string namePrefix(const std::string &spec) {
    if (spec.rfind("gsusb", 0) == 0)
        return "gsusb";
    if (spec.rfind("slcan", 0) == 0)
        return "slcan";
    return "hub";
}

static int cmdUp(const std::string &spec, std::string name, const std::string &listen, bool foreground) {
    if (name.empty())
        name = CANRegistry::freeName(namePrefix(spec));

    // Bring-up runs in the child, which reports back before going into the background, so errors (and how long the
    // bitrate search takes) show up here
    int reportPipe[2] = { -1, -1 };
    if (!foreground) {
        if (pipe(reportPipe) < 0)
            throw std::system_error(errno, std::generic_category(), "pipe");

        pid_t pid = fork();
        if (pid < 0)
            throw std::system_error(errno, std::generic_category(), "fork");
        if (pid > 0) {
            close(reportPipe[1]);
            std::string report;
            char buf[512];
            ssize_t len;
            while ((len = read(reportPipe[0], buf, sizeof(buf))) > 0)
                report.append(buf, len);
            close(reportPipe[0]);

            if (report.rfind("OK\n", 0) == 0) {
                std::cout << report.substr(3);
                return 0;
            }
            std::cerr << (report.empty() ? "canmore-can up exited without reporting\n" : report);
            return 1;
        }

        close(reportPipe[0]);
        setsid();
    }

    auto report = [&](bool ok, const std::string &msg) {
        if (foreground) {
            (ok ? std::cout : std::cerr) << msg;
            return;
        }
        std::string out = (ok ? "OK\n" : "") + msg;
        if (write(reportPipe[1], out.data(), out.size()) < 0) {
        }
        close(reportPipe[1]);

        // Nothing to print to from here on
        int devNull = open("/dev/null", O_RDWR);
        dup2(devNull, STDIN_FILENO);
        dup2(devNull, STDOUT_FILENO);
        dup2(devNull, STDERR_FILENO);
        close(devNull);
    };

    try {
        StopWatcher stop;
        std::unique_ptr<CannelloniHub> hub;
        Tap bus(spec, [&hub](const CANFrame &frame) {
            if (hub)
                hub->sendToPeers(frame);
        });
        hub = std::make_unique<CannelloniHub>(bus, listen);

        CANRegistry::Entry entry;
        entry.name = name;
        entry.spec = "cannelloni:" + hub->address + bus.getRateSpec();
        entry.description = bus.describe() + ", pid " + std::to_string(getpid());
        entry.pid = getpid();
        CANRegistry::add(entry);

        report(true, name + " is up: " + bus.describe() + "\n  served as " + entry.spec + " (pid " +
                         std::to_string(getpid()) + ")\n  use it as " + name +
                         " (upload_tool and canmore_cli find it, micro_ros_agent can -D " + name +
                         "); take it down with canmore-can down " + name + "\n");

        PollGroup group;
        group.addFd(stop);
        group.addFd(bus);
        group.addFd(*hub);
        runUntilStopped(group);

        CANRegistry::remove(name);
    } catch (std::exception &e) {
        CANRegistry::remove(name);
        report(false, std::string("canmore-can up ") + spec + ": " + e.what() + "\n");
        return 1;
    }
    return 0;
}

static int cmdDown(const std::string &name) {
    std::vector<CANRegistry::Entry> entries;
    if (name == "all") {
        entries = CANRegistry::list();
    }
    else {
        CANRegistry::Entry entry;
        if (!CANRegistry::lookup(name, entry)) {
            std::cerr << name << " isn't up (see canmore-can list)" << std::endl;
            return 1;
        }
        entries.push_back(entry);
    }

    for (auto &entry : entries) {
        kill(entry.pid, SIGTERM);
        CANRegistry::Entry check;
        for (int i = 0; i < 40 && CANRegistry::lookup(entry.name, check); i++)
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        std::cout << entry.name << " is down" << std::endl;
    }
    return 0;
}

static int cmdDump(const std::string &spec, const std::string &pcapPath) {
    std::unique_ptr<PcapWriter> pcap;
    if (!pcapPath.empty())
        pcap = std::make_unique<PcapWriter>(pcapPath);
    bool quiet = (pcapPath == "-");

    StopWatcher stop;
    Tap bus(spec, [&](const CANFrame &frame) {
        if (pcap)
            pcap->write(frame);
        if (!quiet)
            std::cout << formatFrame(spec, frame) << std::endl;
    });
    if (!quiet)
        std::cerr << "Listening on " << bus.describe() << std::endl;

    PollGroup group;
    group.addFd(stop);
    group.addFd(bus);
    runUntilStopped(group);
    return 0;
}

static int cmdSend(const std::string &spec, const std::vector<std::string> &frames) {
    Tap bus(spec, [](const CANFrame &) {});
    for (auto &str : frames) {
        CANFrame frame = parseFrame(str);
        if (!bus.transmitFrameNoexcept(frame)) {
            std::cerr << "Failed to send " << str << (frame.fd && !bus.usingCanFd() ? " (not a CAN FD bus)" : "")
                      << std::endl;
            return 1;
        }
    }

    // Let it get onto the bus before the adapter closes
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    return 0;
}

static int cmdBridge(const std::string &specA, const std::string &specB) {
    StopWatcher stop;
    std::unique_ptr<Tap> a, b;
    a = std::make_unique<Tap>(specA, [&b](const CANFrame &frame) {
        if (b)
            b->transmitFrameNoexcept(frame);
    });
    b = std::make_unique<Tap>(specB, [&a](const CANFrame &frame) { a->transmitFrameNoexcept(frame); });
    std::cerr << "Bridging " << a->describe() << " <-> " << b->describe() << std::endl;

    PollGroup group;
    group.addFd(stop);
    group.addFd(*a);
    group.addFd(*b);
    runUntilStopped(group);
    return 0;
}

static void usage(const char *prog) {
    std::cerr << "Usage:\n"
              << "  " << prog << " list                  CAN buses, and USB CAN adapters to bring up\n"
              << "  " << prog << " up <spec> [--name NAME] [--listen HOST:PORT] [--foreground]\n"
              << "      Takes an adapter and serves its bus to other programs, like ip link set up on Linux.\n"
              << "      Without a rate it listens for the bus's rate (1M/5M CAN FD or 250k) first.\n"
              << "      e.g. up gsusb, up gsusb@250k, up slcan:/dev/cu.usbmodem1234@1M/5M\n"
              << "  " << prog << " down <name>|all\n"
              << "  " << prog << " dump <spec> [--pcap FILE|-]   (- writes pcap to stdout for wireshark -k -i -)\n"
              << "  " << prog << " send <spec> <frame>...        (cansend syntax: 123#DEADBEEF, 123##1<data> for FD)\n"
              << "  " << prog << " bridge <spec> <spec>\n"
              << "\nSpecs: can0, gsusb[:serial][#ch][@rate], slcan:<port>[@rate], cannelloni:<host>:<port>[@rate],\n"
              << "or a name from up (gsusb0). Rates: @1M/5M for CAN FD, @250k for classic.\n"
              << "python-can works too: its slcan and gs_usb interfaces, or udp_multicast/socketcan on Linux."
              << std::endl;
}

int main(int argc, char **argv) {
    if (argc < 2) {
        usage(argv[0]);
        return 1;
    }

    std::string cmd = argv[1];
    std::vector<std::string> args(argv + 2, argv + argc);

    try {
        if (cmd == "list") {
            return cmdList();
        }
        else if (cmd == "up" && !args.empty()) {
            std::string name, listen = "127.0.0.1:0";
            bool foreground = false;
            for (size_t i = 1; i < args.size(); i++) {
                if (args[i] == "--name" && i + 1 < args.size())
                    name = args[++i];
                else if (args[i] == "--listen" && i + 1 < args.size())
                    listen = args[++i];
                else if (args[i] == "--foreground" || args[i] == "-f")
                    foreground = true;
                else {
                    usage(argv[0]);
                    return 1;
                }
            }
            return cmdUp(args[0], name, listen, foreground);
        }
        else if (cmd == "down" && args.size() == 1) {
            return cmdDown(args[0]);
        }
        else if (cmd == "dump" && (args.size() == 1 || (args.size() == 3 && args[1] == "--pcap"))) {
            return cmdDump(args[0], args.size() == 3 ? args[2] : "");
        }
        else if (cmd == "send" && args.size() >= 2) {
            return cmdSend(args[0], std::vector<std::string>(args.begin() + 1, args.end()));
        }
        else if (cmd == "bridge" && args.size() == 2) {
            return cmdBridge(args[0], args[1]);
        }
    } catch (std::exception &e) {
        std::cerr << "canmore-can " << cmd << ": " << e.what() << std::endl;
        return 1;
    }

    usage(argv[0]);
    return 1;
}
