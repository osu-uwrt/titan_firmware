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
#define FILE_BUFFER_SIZE 1024
// previously 1024
#define FILE_UPLOAD_MAX_ERRORS 5

#define CURSOR_UP "\033[1A"
#define CURSOR_BEGIN "\r"
#define ERASE_LINE "\033[K"
#define PROGRESS_BAR_LENGTH 32

#if FILE_BUFFER_SIZE % 4 != 0
#error FILE_UPLOAD_BATCH_SIZE must be a multiple of 4!
#endif

class LinuxFileUploadCommand : public CLICommandHandler<Canmore::LinuxClient> {
public:
    LinuxFileUploadCommand(): CLICommandHandler("upload") {}

    std::string getArgList() const override { return "[src file], [dst file]"; }
    std::string getHelp() const override { return "Uploads a file to the remote client."; }

    void callback(CLIInterface<Canmore::LinuxClient> &interface, std::vector<std::string> const &args) override {
        bool first_write = true;
        error_counter = 0;

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
            reportErrnoError(interface, "Failed to open file");
            return;
        }

        // now begin file upload loop
        memset(file_buffer_contents, 0, sizeof(file_buffer_contents));
        int file_size = std::filesystem::file_size(src_file),  // will use to make a cool progress bar
            data_read = 0, data_len = 0, status = 0;
        uint32_t file_crc = 0xFFFFFFFF;

        interface.writeLine("Uploading file");
        interface.writeLine("");  // makes progress bar play better
        while ((file && !file.eof()) || error_counter != 0) {
            // check for interrupt
            struct pollfd fd;
            fd.fd = STDIN_FILENO;
            fd.events = POLLIN;

            if (poll(&fd, 1, 0) < 0) {
                throw std::system_error(errno, std::generic_category(), "poll");
            }

            // Check for keypress
            if (fd.revents & POLLIN) {
                char c = getchar();
                // If ctrl+c, then break
                if (c == '\x03') {
                    interface.writeLine("Interrupted");
                    return;
                }
            }

            // read file data
            if (error_counter == 0) {
                int bytes_read;
                data_len = readFileIntoPageAndBuffer(interface, dst_file, file, bytes_read, file_crc);
                data_read += bytes_read;
            }

            // pack registers
            // filename length
            interface.handle->client->writeRegister(
                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET, (uint32_t) dst_file.length());

            // data length
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET, (uint32_t) data_len);

            // checksum
            uint32_t crc32 = crc32_compute((uint8_t *) file_buffer_contents, data_len);
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET, crc32);

            // clear file
            interface.handle->client->writeRegister(
                CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                CANMORE_LINUX_FILE_CONTROL_CLEAR_FILE_OFFSET, (first_write ? 1 : 0));

            // file inode
            mode_t mode = 0;  // integer type, so 4 bytes (32 bits)
            interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                    CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                    CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET, mode);

            status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_WRITE);

            if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
                reportRemoteDeviceError(interface, "File write failed");
                return;
            }
            else if (status == CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC) {
                interface.writeLine(COLOR_ERROR "Bad CRC" COLOR_RESET);

                // retry writing the page. dont read anything new
                writeDataIntoBufferPage(interface, file_buffer_contents, data_len, 0);
                error_counter++;
                if (error_counter > FILE_UPLOAD_MAX_ERRORS) {
                    interface.writeLine(COLOR_ERROR "Maximum number of transmission errors reached while trying to "
                                                    "upload the file. Please check the connections on the CAN and "
                                                    "ensure that traffic is at a reasonable level." COLOR_RESET);
                    return;
                }
                continue;  // dont go further or else error_counter will be wrongly reset
            }
            else if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
                interface.writeLine(COLOR_ERROR "Timed out waiting for write status to become SUCCESS" COLOR_RESET);
                return;
            }

            error_counter = 0;  // good transmission, reset error counter
            first_write = false;

            // write progress bar to terminal
            double percent_progress = data_read / (double) file_size;
            int progress_amount = percent_progress * PROGRESS_BAR_LENGTH;
            std::string progress_bar = CURSOR_UP CURSOR_BEGIN ERASE_LINE "[";
            int i;
            for (int i = 0; i < progress_amount; i++) {
                progress_bar += (i == progress_amount - 1 ? ">" : "=");
            }

            for (int i = 0; i < PROGRESS_BAR_LENGTH - progress_amount; i++) {
                progress_bar += " ";
            }

            progress_bar += "] " + std::to_string((int) (percent_progress * 100)) + "%";
            interface.writeLine(progress_bar);
        }

        // check the file crc
        // first compute the file crc
        interface.writeLine("Checking file CRC");

        // write filename to remote (name length is already in the register so we wont update that)
        writeDataIntoBufferPage(interface, dst_file.c_str(), dst_file.size(), 0);

        // write crc into remote register
        interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET, file_crc);

        // do operation
        status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_CHECK_CRC);
        if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
            if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
                reportRemoteDeviceError(interface, "Failed to set remote file permissions");
            }
            else if (status == CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC) {
                interface.writeLine(
                    COLOR_ERROR
                    "Checksum of remote file does not match the local one. Please try uploading again" COLOR_RESET);
            }
        }

        // set remote file permissions
        // first get the local permissions
        interface.writeLine("Setting file permissions");
        struct stat fileStat;
        if (stat(src_file.c_str(), &fileStat) < 0) {
            reportErrnoError(interface, "Failed to get file status");
        }

        // write filename into remote
        writeDataIntoBufferPage(interface, dst_file.c_str(), dst_file.size(), 0);

        // write permissions to remote register
        uint32_t mode_reg_val = (uint32_t) fileStat.st_mode;
        interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET, mode_reg_val);

        // do operation
        status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_SET_MODE);
        if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
            if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
                reportRemoteDeviceError(interface, "Failed to set remote file permissions");
            }
        }

        file.close();
        interface.writeLine("Upload complete.");
    }

private:
    /**
     * @brief Writes a string into the server buffer page.
     * @param interface used to perform the writes.
     * @param data the data to write
     * @param length the number of bytes to write out of the data buffer
     * @param offset the offset to write the string at.
     * @return The number of registers occupied by the previously written string.
     */
    size_t writeDataIntoBufferPage(CLIInterface<Canmore::LinuxClient> &interface, const void *data, size_t length,
                                   int offset) {
        char buf[4];
        int position_in_string = 0;
        memset(buf, 0, sizeof(buf));
        std::vector<uint32_t> regs;
        while (position_in_string < length) {
            size_t num_bytes_to_copy = length - position_in_string;
            if (num_bytes_to_copy > 4) {
                num_bytes_to_copy = 4;
            }

            memcpy(buf, data + position_in_string, num_bytes_to_copy);

            // populate reg val
            uint32_t reg_val = 0;
            reg_val |= static_cast<uint32_t>(buf[0]) & 0xFF;
            reg_val |= (static_cast<uint32_t>(buf[1]) & 0xFF) << 8;
            reg_val |= (static_cast<uint32_t>(buf[2]) & 0xFF) << 16;
            reg_val |= (static_cast<uint32_t>(buf[3]) & 0xFF) << 24;
            regs.push_back(reg_val);

            memset(buf, 0, sizeof(buf));

            position_in_string += 4;
        }

        interface.handle->client->writeArray(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                             CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, offset, regs);

        return offset + regs.size();
    }

    void readFilePage(CLIInterface<Canmore::LinuxClient> &interface, char *contents, size_t num_regs) {
        uint32_t reg_value;
        for (size_t reg = 0; reg < num_regs; reg++) {
            reg_value = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                               CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, reg);

            uint32_t mask = 0xFF;  // mask to filter out everything but the least significant byte in the reg val
            contents[reg * 4] = reg_value & mask;
            contents[reg * 4 + 1] = (reg_value >> 8) & mask;
            contents[reg * 4 + 2] = (reg_value >> 16) & mask;
            contents[reg * 4 + 3] = (reg_value >> 24) & mask;
        }
    }

    int readFileIntoPageAndBuffer(CLIInterface<Canmore::LinuxClient> &interface, const std::string &dst_file,
                                  std::ifstream &file, int &num_bytes_just_read, uint32_t &crc_value) {
        strcpy(file_buffer_contents, dst_file.c_str());
        int offset_after_filename = writeDataIntoBufferPage(interface, dst_file.c_str(), dst_file.length(), 0),
            offset_after_filename_bytes = offset_after_filename * 4;

        file.read(&file_buffer_contents[offset_after_filename_bytes],
                  sizeof(file_buffer_contents) - offset_after_filename_bytes);

        num_bytes_just_read = file.gcount();

        // update the crc32 value
        crc_value = crc32_update((uint8_t *) &file_buffer_contents[offset_after_filename_bytes], num_bytes_just_read,
                                 crc_value);

        writeDataIntoBufferPage(interface, &file_buffer_contents[offset_after_filename_bytes], num_bytes_just_read,
                                offset_after_filename);

        return num_bytes_just_read + offset_after_filename_bytes;
    }

    int doRemoteFileOperation(CLIInterface<Canmore::LinuxClient> &interface, int operation) {
        // get ready for file write. In order to do this, the "write" register must be set to 0 and the "write
        // status" register must have a value of 0 indicating "ready"
        interface.handle->client->writeRegister(
            CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
            CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET, CANMORE_LINUX_FILE_OPERATION_NOP);

        uint32_t write_status = -1;
        auto start_time = std::chrono::system_clock::now();
        while (write_status != CANMORE_LINUX_FILE_STATUS_READY) {
            write_status = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                                  CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                  CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET);

            if (std::chrono::system_clock::now() - start_time > 1s) {
                interface.writeLine(COLOR_ERROR "Timed out waiting for write status to become READY" COLOR_RESET);
                break;
            }
        }

        // command write
        interface.handle->client->writeRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET, operation);

        // wait for write to complete or fail
        start_time = std::chrono::system_clock::now();
        while (write_status == CANMORE_LINUX_FILE_STATUS_READY || write_status == CANMORE_LINUX_FILE_STATUS_BUSY) {
            write_status = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                                  CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                  CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET);

            if (std::chrono::system_clock::now() - start_time > 1s) {
                break;
            }
        }

        return write_status;
    }

    void reportErrnoError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message) {
        std::string err = COLOR_ERROR;
        err += message;
        err += ": " + std::string(strerror(errno));
        err += COLOR_RESET;
        interface.writeLine(err);
    }

    void reportRemoteDeviceError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message) {
        std::string err_msg = COLOR_ERROR;
        err_msg += message + "; Remote device reported error: ";

        uint32_t error_len = interface.handle->client->readRegister(CANMORE_TITAN_CONTROL_INTERFACE_MODE_LINUX,
                                                                    CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                    CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET);

        if (error_len >= FILE_BUFFER_SIZE) {
            error_len = FILE_BUFFER_SIZE - 1;
        }

        size_t error_len_regs = error_len / 4 + 1;

        // read the actual error message, which has been stored in the file buffer
        readFilePage(interface, file_buffer_contents, error_len_regs);
        file_buffer_contents[error_len] = '\0';  // just in case

        // assemble and send error message
        err_msg += std::string(file_buffer_contents);

        err_msg += COLOR_RESET;
        interface.writeLine(err_msg);
    }

    char file_buffer_contents[FILE_BUFFER_SIZE] = { 0 };
    int error_counter = 0;
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
