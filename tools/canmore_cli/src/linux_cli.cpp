#include "CLIBackends.hpp"
#include "CLIInterface.hpp"
#include "DeviceMap.hpp"
#include "RemoteTTYClientTask.hpp"
#include "canmore_cpp/LinuxClient.hpp"

#include "canmore/crc32.h"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string.h>
#include <sys/stat.h>
#include <thread>

using namespace std::chrono_literals;

class LinuxKeepaliveTask : public CLIBackgroundTask<Canmore::LinuxClient> {
public:
    void callback(CLIInterface<Canmore::LinuxClient> &interface) override { interface.handle->ping(); }
};

class LinuxVersionCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxVersionCommand(): CLICommandHandler("version") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Returns the current version."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        renderField("Version", interface.handle->getVersion() + COLOR_RESET);
    }
};

class LinuxRebootCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxRebootCommand(): CLICommandHandler("reboot") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Performs a full system reboot"; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Sending reboot command...");
        interface.handle->reboot();
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        interface.close("Closed due to Reboot Request");
    }
};

class LinuxDaemonRestartCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxDaemonRestartCommand(): CLICommandHandler("daemon_restart") {}

    std::string getArgList() const override { return ""; }
    std::string getHelp() const override { return "Restarts the daemon service (if bash manages to break)"; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        interface.writeLine("Resetting CanmoreCLI Daemon...");
        interface.handle->restartDaemon();
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        interface.close("Closed due to daemon stopping");
    }
};

// file upload batch size is conveniently also the same size as the buffer for strerror
// this batch size must be a multiple of 4
#define FILE_UPLOAD_BATCH_SIZE 1024

#if FILE_UPLOAD_BATCH_SIZE % 4 != 0
#error FILE_UPLOAD_BATCH_SIZE must be a multiple of 4!
#endif

class LinuxFileUploadCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxFileUploadCommand(): CLICommandHandler("upload") {}

    std::string getArgList() const override { return "[src file], [dst file]"; }
    std::string getHelp() const override { return "Uploads a file to the remote client."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        bool first_write = true;

        // find file to upload on local system
        if (args.size() != 2) {
            std::string err = COLOR_ERROR "Expected 2 arguments but got ";
            err += std::to_string(args.size());
            err += COLOR_RESET;
            interface.writeLine(err);
            return;
        }

        std::string src_file = args.at(0), dst_file = args.at(1);

        // open and check source file
        std::ifstream file(src_file.c_str());
        if (!file.good()) {
            std::string err = COLOR_ERROR "Failed to open file ";
            err += src_file;
            err += ": " + std::string(strerror(errno));
            err += COLOR_RESET;
            interface.writeLine(err);
            return;
        }

        // now begin file upload loop
        memset(file_buffer_contents, 0, sizeof(file_buffer_contents));
        int file_size = std::filesystem::file_size(src_file);
        do {
            int offset_after_filename = write_string_into_buffer_page(interface, dst_file, 0);
            file.read(file_buffer_contents, sizeof(file_buffer_contents));
            int bytes_just_read = file.gcount();

            write_string_into_buffer_page(interface, std::string(file_buffer_contents), offset_after_filename);

            // now issue server write. need to fill the proper information first

            // filename length
            interface.handle->client->writeRegister(
                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                CANMORE_LINUX_UPLOAD_CONTROL_FILENAME_LENGTH_OFFSET, (uint32_t) dst_file.length());

            // data length
            int data_len = (offset_after_filename * 4) + bytes_just_read;
            interface.handle->client->writeRegister(
                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                CANMORE_LINUX_UPLOAD_CONTROL_DATA_LENGTH_OFFSET, (uint32_t) data_len);

            // checksum
            uint32_t crc32 = crc32_compute((uint8_t *) file_buffer_contents, data_len * 4);
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_CRC_OFFSET, crc32);

            // clear file
            interface.handle->client->writeRegister(
                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                CANMORE_LINUX_UPLOAD_CONTROL_CLEAR_FILE_OFFSET, (first_write ? 1 : 0));

            // file inode
            uint32_t inode = 0;
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_FILE_INODE_OFFSET, inode);

            // get ready for file write. In order to do this, the "write" register must be set to 0 and the "write
            // status" register must have a value of 0 indicating "ready"
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_WRITE_OFFSET, 0);

            uint32_t write_status = -1;
            auto start_time = std::chrono::system_clock::now();
            while (write_status != CANMORE_LINUX_UPLOAD_WRITE_STATUS_READY) {
                write_status = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                                      CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                                      CANMORE_LINUX_UPLOAD_CONTROL_WRITE_STATUS_OFFSET);

                if (std::chrono::system_clock::now() - start_time > 1s) {
                    interface.writeLine(COLOR_ERROR "Timed out waiting for write status to become READY" COLOR_RESET);

                    break;
                }
            }

            // command write
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_UPLOAD_CONTROL_WRITE_OFFSET, 1);

            // wait for write to complete or fail
            write_status = -1;
            start_time = std::chrono::system_clock::now();
            while (write_status != CANMORE_LINUX_UPLOAD_WRITE_STATUS_READY) {
                write_status = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                                      CANMORE_LINUX_UPLOAD_CONTROL_PAGE_NUM,
                                                                      CANMORE_LINUX_UPLOAD_CONTROL_WRITE_STATUS_OFFSET);

                if (write_status == CANMORE_LINUX_UPLOAD_WRITE_STATUS_FAIL_DEVICE_ERROR) {
                    // read the actual error message, which has been stored in the file buffer
                    read_file_page(interface, file_buffer_contents, sizeof(file_buffer_contents) / 4);

                    // assemble and send error message
                    std::string errMsg = COLOR_ERROR "File upload failed because of a server error: ";
                    errMsg += COLOR_RESET;
                    interface.writeLine(errMsg);
                    return;
                }

                if (std::chrono::system_clock::now() - start_time > 1s) {
                    interface.writeLine(COLOR_ERROR "Timed out waiting for write status to become READY" COLOR_RESET);

                    return;
                }
            }

            first_write = false;
        } while (file && !file.eof());

        interface.writeLine("Upload complete.");
    }

private:
    /**
     * @brief Writes a string into the server buffer page.
     * @param interface used to perform the writes.
     * @param str the string to write
     * @param offset the offset to write the string at.
     * @param buf the buffer to use to group the string into individual registers. Should be no less than 4 characters
     * long
     * @return The number of registers occupied by the previously written string.
     */
    size_t write_string_into_buffer_page(CLIInterface<Canmore::LinuxClient> &interface, const std::string &str,
                                         int offset) {
        char buf[4];
        int position_in_string = 0, position_in_buf = 0;
        memset(buf, 0, sizeof(buf));
        while (position_in_string < str.size()) {
            buf[position_in_buf] = str.at(position_in_string);

            position_in_string++;
            position_in_buf++;
            if (position_in_buf >= sizeof(buf) || position_in_string >= str.size()) {
                // populate reg val
                uint32_t reg_val = 0;
                reg_val |= static_cast<uint32_t>(buf[0]);
                reg_val |= static_cast<uint32_t>(buf[1]) << 8;
                reg_val |= static_cast<uint32_t>(buf[2]) << 16;
                reg_val |= static_cast<uint32_t>(buf[3]) << 24;

                // reg val populated, now write register

                interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                        CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, offset, reg_val);

                memset(buf, 0, sizeof(buf));
                offset++;
                position_in_buf = 0;
            }
        }

        return offset;
    }

    void read_file_page(CLIInterface<Canmore::LinuxClient> &interface, char *contents, size_t num_regs) {
        uint32_t reg_value;
        for (size_t reg = 0; reg < num_regs; reg++) {
            reg_value = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                               CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, reg);

            *(contents + (reg * 4)) = reg_value;
        }
    }

    char file_buffer_contents[FILE_UPLOAD_BATCH_SIZE];
};

class LinuxRemoteTTYCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxRemoteTTYCommand(): CLICommandHandler("remotesh") {}

    std::string getArgList() const override { return "[optional command]"; }
    std::string getHelp() const override {
        return "Launches a remote shell on the host. If the optional command is provided, that is executed instead of "
               "the login shell.";
    }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        (void) args;
        auto canClient = std::dynamic_pointer_cast<Canmore::RegMappedCANClient>(interface.handle->client);
        if (canClient) {
            std::stringstream iss;
            auto itr = args.begin();
            while (itr != args.end()) {
                iss << *itr;
                itr++;
                if (itr != args.end()) {
                    iss << ' ';
                }
            }

            RemoteTTYClientTask ttyClient(interface.handle, canClient, iss.str());
            ttyClient.run();
        }
        else {
            interface.writeLine(COLOR_ERROR "Current interface is not CAN bus, cannot create TTY client" COLOR_RESET);
        }
    }
};

LinuxCLI::LinuxCLI(std::shared_ptr<Canmore::LinuxClient> handle): CLIInterface(handle) {
    registerCommand(std::make_shared<LinuxVersionCommand>());
    registerCommand(std::make_shared<LinuxRebootCommand>());
    registerCommand(std::make_shared<LinuxDaemonRestartCommand>());
    registerCommand(std::make_shared<LinuxFileUploadCommand>());
    registerCommand(std::make_shared<LinuxRemoteTTYCommand>());
    setBackgroundTask(std::make_shared<LinuxKeepaliveTask>());

    auto devMap = DeviceMap::create();
    uint64_t flashId = handle->getFlashId();
    auto devDescr = devMap.lookupSerial(flashId);

    std::cout << std::endl;
    renderHeader("Connecting to Linux Device");
    renderName(devDescr.name);
    if (devDescr.boardType != "unknown")
        renderField("Board Type", devDescr.boardType);
    else if (flashId != 0)
        renderField("Unique ID", devDescr.hexSerialNum());
    renderField("Version", handle->getVersion());
    std::cout << std::endl;
}
