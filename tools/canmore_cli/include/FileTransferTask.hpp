#pragma once

#include "CLIInterface.hpp"
#include "canmore_cpp/LinuxClient.hpp"

// file upload batch size is conveniently also the same size as the buffer for strerror
// this batch size must be a multiple of 4
#define FILE_BUFFER_SIZE 1024
#define FILE_UPLOAD_MAX_ERRORS 5

#define CURSOR_UP "\033[1A"
#define CURSOR_BEGIN "\r"
#define ERASE_LINE "\033[K"
#define PROGRESS_BAR_LENGTH 32

#if FILE_BUFFER_SIZE % 4 != 0
#error FILE_UPLOAD_BATCH_SIZE must be a multiple of 4!
#endif

class FileTransferTask {
public:
    static std::string fileName(const std::string &path);

    void upload(CLIInterface<Canmore::LinuxClient> &interface, const std::string &src_file,
                const std::string &dst_file);

    void download(CLIInterface<Canmore::LinuxClient> &interface, const std::string &src_file,
                  const std::string &dst_file);

private:
    /**
     * @brief Writes a string into the server buffer page.
     * @param interface used to perform the writes.
     * @param data the data to write
     * @param length the number of bytes to write out of the data buffer
     * @param offset the offset to write the string at.
     * @return The number of registers occupied by the previously written string.
     */
    size_t writeDataIntoBufferPage(CLIInterface<Canmore::LinuxClient> &interface, const char *data, size_t length,
                                   int offset);

    void readFilePage(CLIInterface<Canmore::LinuxClient> &interface, char *contents, size_t num_regs);

    int readFileIntoPageAndBuffer(CLIInterface<Canmore::LinuxClient> &interface, const std::string &dst_file,
                                  std::ifstream &file, int &num_bytes_just_read, uint32_t &crc_value);

    int doRemoteFileOperation(CLIInterface<Canmore::LinuxClient> &interface, int operation);

    void reportErrnoError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message);

    void reportRemoteDeviceError(CLIInterface<Canmore::LinuxClient> &interface, const std::string &message);

    void drawProgressBar(CLIInterface<Canmore::LinuxClient> &interface, double progress);

    bool checkForInterrupt();

    char file_buffer_contents[FILE_BUFFER_SIZE] = { 0 };
};
