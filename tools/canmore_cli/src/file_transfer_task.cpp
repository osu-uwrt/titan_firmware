#include "FileTransferTask.hpp"

#include "canmore/crc32.h"
#include "canmore/reg_mapped/interface/linux.h"

#include <filesystem>
#include <fstream>
#include <string.h>
#include <sys/stat.h>
#include <unistd.h>

#define FILE_TRANSFER_SERVER_TMP_FILE "/tmp/canmore_ft_server_buffer"
#define FILE_TRANSFER_CLIENT_TMP_FILE "/tmp/canmore_ft_client_buffer"

using namespace std::chrono_literals;

std::string FileTransferTask::fileName(const std::string &path) {
    size_t last_slash = path.rfind('/');
    if (last_slash == std::string::npos) {
        last_slash = -1;
    }

    return path.substr(last_slash + 1);
}

void FileTransferTask::upload(CLIInterface<Canmore::LinuxClient> &interface, const std::string &src_file,
                              const std::string &dst_file, bool suppress_output) {
    bool first_write = true;
    int error_counter = 0;

    bool output_suppressed_before = outputSuppressed_;
    setOutputSuppressed(suppress_output);

    // open and check source file
    std::ifstream file(src_file.c_str());
    if (!file.good()) {
        reportErrnoError(interface, "Failed to open file");
        return;
    }

    memset(file_buffer_contents, 0, sizeof(file_buffer_contents));
    int file_size = 0;
    try {
        file_size = std::filesystem::file_size(src_file);  // will use to make a cool progress bar
    } catch (std::filesystem::filesystem_error &ex) {
        reportErrnoError(interface, "Failed to get file size");
    }
    int data_read = 0, data_len = 0, status = 0;
    uint32_t file_crc = 0xFFFFFFFF;

    printInfo(interface, "Uploading file");
    printInfo(interface, "");  // makes progress bar play better
    while ((file && !file.eof()) || error_counter != 0) {
        // check for interrupt
        if (checkForInterrupt()) {
            // If ctrl+c, then break
            printInfo(interface, "Interrupted");
            file.close();
            return;
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
            CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
            CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET, (uint32_t) dst_file.length());

        // data length
        interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET, (uint32_t) data_len);

        // checksum
        uint32_t crc32 = crc32_compute((uint8_t *) file_buffer_contents, data_len);
        interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET, crc32);

        // clear file
        interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_CLEAR_OFFSET, (first_write ? 1 : 0));

        status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_WRITE);

        if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "File write failed");
            file.close();
            return;
        }
        else if (status == CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC) {
            printError(interface, "Bad CRC");

            // retry writing the page. dont read anything new
            writeDataIntoBufferPage(interface, file_buffer_contents, data_len, 0);
            error_counter++;
            if (error_counter > FILE_UPLOAD_MAX_ERRORS) {
                printError(interface, "Maximum number of transmission errors reached while trying to "
                                      "upload the file. Please check the connections on the CAN and "
                                      "ensure that traffic is at a reasonable level.");
                file.close();
                return;
            }
            continue;  // dont go further or else error_counter will be wrongly reset
        }
        else if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
            printError(interface, "Timed out waiting for write status to become SUCCESS");
            file.close();
            return;
        }

        error_counter = 0;  // good transmission, reset error counter
        first_write = false;

        // write progress bar to terminal
        if (file_size > 0) {
            double percent_progress = data_read / (double) file_size;
            drawProgressBar(interface, percent_progress);
        }
        else {
            drawProgressBar(interface, 1);
        }
    }

    file.close();

    // check the file crc
    // first compute the file crc
    printInfo(interface, "Checking file CRC");
    if (!checkFileCrc(interface, dst_file, file_crc)) {
        return;
    }

    // set remote file permissions
    // first get the local permissions
    printInfo(interface, "Setting file permissions");
    struct stat fileStat;
    if (stat(src_file.c_str(), &fileStat) < 0) {
        reportErrnoError(interface, "Failed to get file status");
    }

    // write permissions to remote register
    uint32_t mode_reg_val = (uint32_t) fileStat.st_mode;
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET, mode_reg_val);

    // do operation
    status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_SET_MODE);
    if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to set remote file permissions");
            return;
        }
        else {
            printError(interface, "Failed to set remote file permissions: Unknown error");

            return;
        }
    }

    printInfo(interface, "Upload complete.");
    setOutputSuppressed(output_suppressed_before);
}

void FileTransferTask::download(CLIInterface<Canmore::LinuxClient> &interface, const std::string &src_file,
                                const std::string &dst_file, bool suppress_output) {
    bool first_read = true;
    int error_counter = 0, status;

    bool output_suppressed_before = outputSuppressed_;
    setOutputSuppressed(suppress_output);

    // attempt to open local file for write
    std::ofstream file;
    file.open(dst_file.c_str());
    if (!file.good()) {
        std::string err = "Error opening file on local system: ";
        err += strerror(errno);
        printError(interface, err);
        return;
    }

    // query remote for file size (helps for progress bar)
    // first write filename into buffer
    writeDataIntoBufferPage(interface, src_file.c_str(), src_file.size(), 0);

    // write filename length to remote
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET,
                                            (uint32_t) src_file.size());

    status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_GET_FILE_LEN);
    if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to get filename length");
        }
    }

    // start download
    printInfo(interface, "Downloading file");
    printInfo(interface, "");

    uint32_t file_size = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET),
             batch_size_bytes, file_crc32 = 0xFFFFFFFF, read_offset = 0;

    do {
        // get ready for file read
        // first read filename to remote. Dont write the length because its already been written
        writeDataIntoBufferPage(interface, src_file.c_str(), src_file.size(), 0);

        // write file read offset to remote
        interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_READ_OFFSET_OFFSET, read_offset);

        // read clear flag to remote
        interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                CANMORE_LINUX_FILE_CONTROL_CLEAR_OFFSET, (first_read ? 1 : 0));

        status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_READ);
        if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
            if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
                reportRemoteDeviceError(interface, "Failed to read file data");
                file.close();
                return;
            }
            else {
                printError(interface, "Failed to read file data: Unknown error");
                file.close();
                return;
            }
        }

        // data length is present in the DATA_LENGTH register. Read it
        batch_size_bytes = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                                  CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                  CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET);

        // crc of file buffer is present in the CRC register. Read it
        uint32_t expected_buf_crc = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                                           CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                           CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET);

        // load data in
        unsigned int batch_size_regs = (batch_size_bytes + 3) / 4;
        readFilePage(interface, file_buffer_contents, batch_size_regs);

        // check crc
        uint32_t actual_buf_crc = crc32_compute((uint8_t *) file_buffer_contents, batch_size_bytes);
        if (expected_buf_crc != actual_buf_crc) {
            printError(interface, "Bad CRC");
            error_counter++;
            if (error_counter > FILE_UPLOAD_MAX_ERRORS) {
                printError(interface, "Maximum number of transmission errors reached while trying to "
                                      "upload the file. Please check the connections on the CAN and "
                                      "ensure that traffic is at a reasonable level.");
                file.close();
                return;
            }

            continue;
        }
        else {
            read_offset += batch_size_bytes;
        }

        // write data to file
        file.write(file_buffer_contents, batch_size_bytes);

        // update file crc for later check
        file_crc32 = crc32_update((uint8_t *) file_buffer_contents, batch_size_bytes, file_crc32);

        // update progress bar
        if (file_size > 0) {
            double percent_progress = read_offset / (double) file_size;
            drawProgressBar(interface, percent_progress);
        }
        else {
            drawProgressBar(interface, 1);
        }

        error_counter = 0;  // successful transfer
        first_read = false;
    } while (batch_size_bytes == FILE_BUFFER_SIZE);

    file.close();

    // check file crc
    printInfo(interface, "Checking file CRC");
    if (!checkFileCrc(interface, src_file, file_crc32)) {
        return;
    }

    // get file mode
    printInfo(interface, "Setting file permissions");

    // file name and length already in remote from previous operations. just do the operation
    status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_GET_MODE);
    if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to get remote file permissions");
            return;
        }
        else {
            printError(interface, "Failed to get remote file permissions: Unknown error");
            return;
        }
    }

    // get file mode out of the FILE_MODE register
    mode_t new_mode = (mode_t) interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                                      CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                      CANMORE_LINUX_FILE_CONTROL_FILE_MODE_OFFSET);

    if (chmod(dst_file.c_str(), new_mode) < 0) {
        reportErrnoError(interface, "Failed to set local file permissions");
        return;
    }

    printInfo(interface, "Download complete.");

    setOutputSuppressed(output_suppressed_before);
}

void FileTransferTask::cd(CLIInterface<Canmore::LinuxClient> &interface, const std::string &directory,
                          bool suppress_output) {
    bool output_suppressed_before = outputSuppressed_;
    setOutputSuppressed(suppress_output);

    // write desired directory into tmp file
    writeStringToFile(FILE_TRANSFER_CLIENT_TMP_FILE, directory);

    // transfer file to remote
    upload(interface, FILE_TRANSFER_CLIENT_TMP_FILE, FILE_TRANSFER_SERVER_TMP_FILE, true);

    // write tmp file name and length to file buffer and control pages
    writeDataIntoBufferPage(interface, FILE_TRANSFER_SERVER_TMP_FILE, sizeof(FILE_TRANSFER_SERVER_TMP_FILE), 0);
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET,
                                            sizeof(FILE_TRANSFER_SERVER_TMP_FILE));

    // do operation
    int result = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_CD);
    if (result != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (result == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to set working directory on remote");
        }
        else {
            printError(interface, "Unknown error");
        }
    }

    setOutputSuppressed(output_suppressed_before);
}

void FileTransferTask::ls(CLIInterface<Canmore::LinuxClient> &interface, const std::string &directory,
                          bool suppress_output) {
    bool output_suppressed_before = outputSuppressed_;
    setOutputSuppressed(suppress_output);

    // write directory to list to tmp file
    if (!writeStringToFile(FILE_TRANSFER_CLIENT_TMP_FILE, directory)) {
        reportErrnoError(interface, "Failed to write target directory in temporary file");
    }

    // transfer file
    upload(interface, FILE_TRANSFER_CLIENT_TMP_FILE, FILE_TRANSFER_SERVER_TMP_FILE, true);

    // write temp file name and length to file buffer
    writeDataIntoBufferPage(interface, FILE_TRANSFER_SERVER_TMP_FILE, sizeof(FILE_TRANSFER_SERVER_TMP_FILE), 0);
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET,
                                            sizeof(FILE_TRANSFER_SERVER_TMP_FILE));

    // do ls operation
    int result = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_LS);
    if (result != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (result == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to read temporary result from remote");
        }
        else {
            printError(interface, "Unknown error");
        }

        return;
    }

    // if we got here the operation succeeded. Need to transfer result back and read it out
    download(interface, FILE_TRANSFER_SERVER_TMP_FILE, FILE_TRANSFER_CLIENT_TMP_FILE, true);

    std::string report = "";
    if (!readFiletoString(FILE_TRANSFER_CLIENT_TMP_FILE, report)) {
        reportErrnoError(interface, "Failed to read temporary result from local file");
        return;
    }

    printInfo(interface, report);

    setOutputSuppressed(output_suppressed_before);
}

void FileTransferTask::pwd(CLIInterface<Canmore::LinuxClient> &interface, bool suppress_output) {
    bool output_suppressed_before = outputSuppressed_;
    setOutputSuppressed(suppress_output);

    // write tmp file name to the file buffer
    writeDataIntoBufferPage(interface, FILE_TRANSFER_SERVER_TMP_FILE, sizeof(FILE_TRANSFER_SERVER_TMP_FILE), 0);

    // write length
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_FILENAME_LENGTH_OFFSET,
                                            sizeof(FILE_TRANSFER_SERVER_TMP_FILE));

    // do pwd operation
    int result = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_PWD);
    if (result != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (result == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to read temporary result from remote");
        }
        else {
            printError(interface, "Unknown error");
        }

        return;
    }

    // download result to tmp
    download(interface, FILE_TRANSFER_SERVER_TMP_FILE, FILE_TRANSFER_CLIENT_TMP_FILE, true);

    // read and print result
    std::string path;
    if (!readFiletoString(FILE_TRANSFER_CLIENT_TMP_FILE, path)) {
        reportErrnoError(interface, "Could not read temporary result");
    }

    printInfo(interface, path);
    setOutputSuppressed(output_suppressed_before);
}

size_t FileTransferTask::writeDataIntoBufferPage(CLIInterface<Canmore::LinuxClient> &interface, const char *data,
                                                 size_t length, int offset) {
    char buf[4];
    size_t position_in_string = 0;
    memset(buf, 0, sizeof(buf));
    std::vector<uint32_t> regs;
    while (position_in_string < length) {
        size_t num_bytes_to_copy = length - position_in_string;
        if (num_bytes_to_copy > 4) {
            num_bytes_to_copy = 4;
        }

        memcpy(buf, &data[position_in_string], num_bytes_to_copy);

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

    interface.handle->client->writeArray(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_BUFFER_PAGE_NUM,
                                         offset, regs);

    return offset + regs.size();
}

void FileTransferTask::readFilePage(CLIInterface<Canmore::LinuxClient> &interface, char *contents,
                                    unsigned int num_regs) {
    std::vector<uint32_t> regs;
    regs.resize(num_regs);
    interface.handle->client->readArray(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_BUFFER_PAGE_NUM, 0,
                                        regs, num_regs);

    for (size_t reg = 0; reg < regs.size(); reg++) {
        uint32_t reg_value = regs.at(reg);
        contents[reg * 4] = reg_value & 0xFF;
        contents[reg * 4 + 1] = (reg_value >> 8) & 0xFF;
        contents[reg * 4 + 2] = (reg_value >> 16) & 0xFF;
        contents[reg * 4 + 3] = (reg_value >> 24) & 0xFF;
    }
}

int FileTransferTask::readFileIntoPageAndBuffer(CLIInterface<Canmore::LinuxClient> &interface,
                                                const std::string &dst_file, std::ifstream &file,
                                                int &num_bytes_just_read, uint32_t &crc_value) {
    strcpy(file_buffer_contents, dst_file.c_str());
    int offset_after_filename = writeDataIntoBufferPage(interface, dst_file.c_str(), dst_file.length(), 0),
        offset_after_filename_bytes = offset_after_filename * 4;

    file.read(&file_buffer_contents[offset_after_filename_bytes],
              sizeof(file_buffer_contents) - offset_after_filename_bytes);

    num_bytes_just_read = file.gcount();

    // update the crc32 value
    crc_value =
        crc32_update((uint8_t *) &file_buffer_contents[offset_after_filename_bytes], num_bytes_just_read, crc_value);

    writeDataIntoBufferPage(interface, &file_buffer_contents[offset_after_filename_bytes], num_bytes_just_read,
                            offset_after_filename);

    return num_bytes_just_read + offset_after_filename_bytes;
}

bool FileTransferTask::checkFileCrc(CLIInterface<Canmore::LinuxClient> &interface, const std::string &filename,
                                    uint32_t expected_crc) {
    int status;

    // write filename to remote (name length is already in the register so we wont update that)
    writeDataIntoBufferPage(interface, filename.c_str(), filename.size(), 0);

    // write crc into remote register
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_CRC_OFFSET, expected_crc);

    // do operation
    status = doRemoteFileOperation(interface, CANMORE_LINUX_FILE_OPERATION_CHECK_CRC);
    if (status != CANMORE_LINUX_FILE_STATUS_SUCCESS) {
        if (status == CANMORE_LINUX_FILE_STATUS_FAIL_DEVICE_ERROR) {
            reportRemoteDeviceError(interface, "Failed to check remote file CRC");
        }
        else if (status == CANMORE_LINUX_FILE_STATUS_FAIL_BAD_CRC) {
            printError(interface, "Checksum of remote file does not match the local one. Please try again");
        }
        else {
            printError(interface, "Failed to check file CRC: Unknown error");
        }

        return false;
    }

    return true;
}

int FileTransferTask::doRemoteFileOperation(CLIInterface<Canmore::LinuxClient> &interface, int operation) {
    // get ready for file write. In order to do this, the "write" register must be set to 0 and the "write
    // status" register must have a value of 0 indicating "ready"
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET,
                                            CANMORE_LINUX_FILE_OPERATION_NOP);

    uint32_t write_status = -1;
    auto start_time = std::chrono::system_clock::now();
    while (write_status != CANMORE_LINUX_FILE_STATUS_READY) {
        write_status = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                              CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                              CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET);

        if (std::chrono::system_clock::now() - start_time > 1s) {
            printError(interface, "Timed out waiting for write status to become READY");
            break;
        }
    }

    // command write
    interface.handle->client->writeRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX, CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                            CANMORE_LINUX_FILE_CONTROL_OPERATION_OFFSET, operation);

    // wait for write to complete or fail
    start_time = std::chrono::system_clock::now();
    while (write_status == CANMORE_LINUX_FILE_STATUS_READY || write_status == CANMORE_LINUX_FILE_STATUS_BUSY) {
        write_status = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                              CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                              CANMORE_LINUX_FILE_CONTROL_STATUS_OFFSET);

        if (std::chrono::system_clock::now() - start_time > 1s) {
            break;
        }
    }

    return write_status;
}

void FileTransferTask::reportErrnoError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message) {
    std::string err = message;
    err += ": " + std::string(strerror(errno));
    printError(interface, err);
}

void FileTransferTask::reportRemoteDeviceError(CLIInterface<Canmore::LinuxClient> &interface,
                                               const std::string &message) {
    std::string err_msg = message + "; Remote device reported error: ";

    uint32_t error_len = interface.handle->client->readRegister(CANMORE_CONTROL_INTERFACE_MODE_LINUX,
                                                                CANMORE_LINUX_FILE_CONTROL_PAGE_NUM,
                                                                CANMORE_LINUX_FILE_CONTROL_DATA_LENGTH_OFFSET);

    if (error_len >= FILE_BUFFER_SIZE) {
        error_len = FILE_BUFFER_SIZE - 1;
    }

    unsigned int error_len_regs = error_len / 4 + 1;

    // read the actual error message, which has been stored in the file buffer
    readFilePage(interface, file_buffer_contents, error_len_regs);
    file_buffer_contents[error_len] = '\0';  // just in case

    // assemble and send error message
    err_msg += std::string(file_buffer_contents);

    printError(interface, err_msg);
}

void FileTransferTask::drawProgressBar(CLIInterface<Canmore::LinuxClient> &interface, double progress) {
    int progress_amount = progress * PROGRESS_BAR_LENGTH;
    std::string progress_bar = CURSOR_UP CURSOR_BEGIN ERASE_LINE "[";
    for (int i = 0; i < progress_amount; i++) {
        progress_bar += (i == progress_amount - 1 ? ">" : "=");
    }

    for (int i = 0; i < PROGRESS_BAR_LENGTH - progress_amount; i++) {
        progress_bar += " ";
    }

    progress_bar += "] " + std::to_string((int) (progress * 100)) + "%";
    printInfo(interface, progress_bar);
}

bool FileTransferTask::checkForInterrupt() {
    struct pollfd fd;
    fd.fd = STDIN_FILENO;
    fd.events = POLLIN;

    if (poll(&fd, 1, 0) < 0) {
        throw std::system_error(errno, std::generic_category(), "poll");
    }

    // Check for keypress
    if (fd.revents & POLLIN) {
        char c = getchar();
        return c == '\x03';
    }

    return false;
}

bool FileTransferTask::readFiletoString(const std::string &filename, std::string &contents) {
    std::ifstream in;
    in.open(filename);
    if (!in) {
        return false;
    }

    size_t size = std::filesystem::file_size(filename);
    contents.resize(size);
    in.read(contents.data(), size);
    return true;
}

bool FileTransferTask::writeStringToFile(const std::string &filename, const std::string &contents) {
    std::ofstream out;
    out.open(filename);
    if (out) {
        out << contents;
        return true;
    }

    return false;
}

void FileTransferTask::setOutputSuppressed(bool suppressed) {
    outputSuppressed_ = suppressed;
}

void FileTransferTask::printInfo(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message) {
    if (!outputSuppressed_) {
        interface.writeLine(message);
    }
}

void FileTransferTask::printError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message) {
    interface.writeLine(COLOR_ERROR + message + COLOR_RESET);
}
