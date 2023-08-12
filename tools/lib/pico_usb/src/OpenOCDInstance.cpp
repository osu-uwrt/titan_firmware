#include <chrono>
#include <cstring>
#include <iostream>
#include <sstream>
#include <fcntl.h>
#include <poll.h>
#include <sys/wait.h>
#include <unistd.h>

#include "pico_usb/PicoprobeClient.hpp"

using namespace PicoUSB;

static bool getEnvironmentBool(const char *name, bool defaultValue = false) {
    const char *value = getenv(name);
    if (value == NULL) {
        return defaultValue;
    }
    else if (value[0] == '1' && value[1] == '\0') {
        return true;
    }
    else if (value[0] == '0' && value[1] == '\0') {
        return false;
    }
    else {
        std::cerr << "[WARNING] Invalid boolean value '" << value << "' for enviornment variable '" << name
                  << "'. Expected either '0' or '1'." << std::endl;
        return defaultValue;
    }
}

OpenOCDInstance::OpenOCDInstance():
        ranCustomProbeInit(getenv("UPLOADTOOL_OPENOCD_CUSTOM_INIT_SCRIPT") != NULL),
        enableStderr(getEnvironmentBool("UPLOADTOOL_OPENOCD_EN_STDERR")) {
    // Ignore SIGPIPE since we're going to be messing around with pipes
    signal(SIGPIPE, SIG_IGN);

    int fd_stdin[2];  // 0: Read end of pipe, 1: Write end of pipe
    int fd_stdout[2];
    if (pipe(fd_stdin) < 0) {
        throw std::system_error(errno, std::generic_category(), "pipe");
    }
    if (pipe(fd_stdout) < 0) {
        throw std::system_error(errno, std::generic_category(), "pipe");
    }

    // Set stdout to non-block since we're going to manage polling it
    if (fcntl(fd_stdout[0], F_SETFL, O_NONBLOCK) < 0) {
        throw std::system_error(errno, std::generic_category(), "fcntl");
    }

    openocdPid = fork();
    if (openocdPid == 0) {
        // Setup our pipes
        // First close the sides we aren't going to use as the child
        close(fd_stdin[1]);  // Write end of stdin
        close(fd_stdout[0]);  // Read end of stdout

        // Next replace stdin and stdout file descriptors
        if (dup2(fd_stdin[0], STDIN_FILENO) < 0) {
            perror("dup2");
            exit(1);
        }
        if (dup2(fd_stdout[1], STDOUT_FILENO) < 0) {
            perror("dup2");
            exit(1);
        }

        // Finally close the file descriptors we duplicated
        close(fd_stdin[0]);
        close(fd_stdout[1]);

        // Determine start command from environment variables
        const char *execPath = getenv("UPLOADTOOL_OPENOCD_PATH");
        const char *initScript = getenv("UPLOADTOOL_OPENOCD_CUSTOM_INIT_SCRIPT");

        // If stderr is not explicitely enabled, replace stderr with /dev/null to supress output
        int stderrFd = STDERR_FILENO;
        if (!enableStderr) {
            // Backup stderr so we can still report errors until we exec
            stderrFd = dup(STDERR_FILENO);
            if (stderrFd < 0) {
                perror("dup(STDERR_FILENO)");
                exit(1);
            }
            if (fcntl(stderrFd, F_SETFD, FD_CLOEXEC) == -1) {
                perror("fcntl(FD_CLOEXEC)");
            }

            // Open dev null and replace stderr
            int nullfd = open("/dev/null", O_WRONLY);
            if (nullfd < 0) {
                perror("open(/dev/null)");
                exit(1);
            }
            if (dup2(nullfd, STDERR_FILENO) < 0) {
                perror("dup2");
                exit(1);
            }
            close(nullfd);
        }

        if (execPath == NULL) {
            // Default to openocd
            execPath = "openocd";
        }

        if (initScript) {
            execlp(execPath, execPath, "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", "tcl_port pipe",
                    "-f", initScript, "-c", "noinit", NULL);
        }
        else {
            execlp(execPath, execPath, "-c", "gdb_port disabled", "-c", "telnet_port disabled", "-c", "tcl_port pipe",
                    "-c", "noinit", NULL);
        }

        // Create error message to write to stderr (our duplicated f)
        std::stringstream errStream;
        errStream << "Failed to execute '" << execPath << "': " << std::strerror(errno) << std::endl;
        auto errStr = errStream.str();

        // No use checking for error, as if we can't actually write to stderr, we don't have anywhere to put it
        write(stderrFd, errStr.c_str(), errStr.size());
        exit(1);
    }
    else if (openocdPid < 0) {
        throw std::system_error(errno, std::generic_category(), "fork");
    }

    // After forking, close the ends of the pipe parent process won't use
    close(fd_stdin[0]); // Close read end of openocd stdin
    close(fd_stdout[1]); // Close write end of openocd stdout pipe

    // Store the file descriptors
    stdinFd = fd_stdin[1];
    stdoutFd = fd_stdout[0];
}

OpenOCDInstance::~OpenOCDInstance() {
    if (openocdPid > 0) {
        // First attempt to see if the process already finished
        int status;
        pid_t ret = waitpid(openocdPid, &status, WNOHANG);

        if (ret == 0) {
            // If process is still alive, attempt a graceful shutdown
            try {
                sendCommand("reset");
                sendCommand("shutdown");
            }
            catch (std::exception &e) {}  // Ignore any errors
            catch (...) {}

            std::chrono::time_point start = std::chrono::steady_clock::now();
            while (ret == 0) {
                // Now attempt to see if the shutdown worked
                ret = waitpid(openocdPid, &status, WNOHANG);
                usleep(10000);

                // If more than 1 second elapses, switch to trying to kill openocd
                if(std::chrono::steady_clock::now() - start > std::chrono::seconds(1))
                    break;
            }

            // If the process *still* hasn't terminated yet, kill it
            // Note we aren't checking for negative status codes, as the only error we can get is ECHILD
            // meaning that we don't have any children we're waiting for. Which if that's the case we don't
            // want to try to kill some arbitrary PID which may or may not be used by another process on the system
            if (ret == 0) {
                if (kill(openocdPid, SIGKILL) < 0) {
                    // We should not get here
                    // It either means that we don't have permission to terminate, that SIGKILL isn't a valid POSIX signal,
                    // or the process is already terminated
                    // But to avoid infinitely waiting on a process we failed to kill, just raise an exception
                    // (which might kill the program if we're already processing an exception, but that's probably fine)
                    perror("kill");
                    std::terminate();
                }

                // Now finally wait for the child to exit (blocking)
                waitpid(openocdPid, &status, 0);
            }
        }

        // Report abnormal process termination from the status code
        if (WIFEXITED(status)) {
            int exitCode = WEXITSTATUS(status);
            if (exitCode != 0) {
                std::cerr << "[WARNING] OpenOCD returned non-zero exit code: " << exitCode << std::endl;
            }
        }
        else if (WIFSIGNALED(status)) {
            std::cerr << "[WARNING] OpenOCD was terminated with signal: " << WTERMSIG(status) << std::endl;
        }
        else {
            std::cerr << "[WARNING] OpenOCD terminated with unknown status: " << status << std::endl;
        }
    }
}

void OpenOCDInstance::writeData(std::string data) {
    ssize_t ret;
    ret = write(stdinFd, data.c_str(), data.length());
    if (ret < 0) {
        if (errno == EPIPE) {
            throw PicoprobeError("OpenOCD stdin pipe broken");
        }
        else {
            throw std::system_error(errno, std::generic_category(), "write");
        }
    } else if ((size_t)ret != data.length()) {
        throw PicoprobeError("OpenOCD stdin pipe failed to fully write");
    }
}

std::string OpenOCDInstance::readData(int timeout_ms) {
    struct pollfd fds[1];
    fds[0].revents = 0;
    fds[0].fd = stdoutFd;
    fds[0].events = POLLIN;

    if (poll(fds, 1, timeout_ms) == -1) {
        throw std::system_error(errno, std::generic_category(), "poll");
    }

    if (fds[0].revents & POLLIN) {
        // stdoutFd should now have data for us
        char buf[64];
        ssize_t ret = read(stdoutFd, buf, sizeof(buf));
        if (ret <= 0) {
            throw std::system_error(errno, std::generic_category(), "read");
        }
        else {
            return std::string(buf, ret);
        }
    }
    else if (fds[0].revents & POLLHUP) {
        // If hangup event, then the other end of the pipe was broken
        throw PicoprobeError("OpenOCD stdout pipe broken");
    }
    else {
        // Else then we don't have any events, and the pipe timed out
        throw PicoprobeError("OpenOCD stdout pipe timeout");
    }
}

std::string OpenOCDInstance::sendCommand(std::string cmd, int timeout_ms) {
    // If enabling debug, print the command we're sending to stderr
    if (enableStderr) {
        std::cerr << "> " << cmd << std::endl;
    }

    // Send command and terminator
    writeData(cmd);
    writeData("\x1A");

    std::string dataOut;
    while (true) {
        auto out = readData(timeout_ms);
        dataOut += out;

        if (out.back() == '\x1A') break;

        if (out.find('\x1A') != std::string::npos) {
            throw PicoprobeError("OpenOCD sent extra data after command separator");
        }
    }

    // Remove last character from dataOut as this is the separator character
    return dataOut.substr(0, dataOut.length() - 1);;
}

void OpenOCDInstance::init() {
    sendCommand("init");
    if (sendCommand("command mode") != "exec") {
        throw PicoprobeError("Failed to initialize OpenOCD. Enable openocd stderr for more details");
    }
}

OpenOCDInstance::OpenOCDVersion OpenOCDInstance::getVersion() {
    std::string prefix("Open On-Chip Debugger ");

    // Check if the version matches what we expect
    auto versionStr = sendCommand("version");

    if (versionStr.compare(0, prefix.size(), prefix)) {
        return OpenOCDVersion();
    }

    size_t maj_sep = versionStr.find('.', prefix.size());
    if (maj_sep == std::string::npos) {
        return OpenOCDVersion();
    }

    size_t min_sep = versionStr.find('.', maj_sep + 1);
    if (maj_sep == std::string::npos) {
        return OpenOCDVersion();
    }

    const char* versionStrPtr = versionStr.c_str();
    char* end;

    long versionMajor = std::strtol(versionStrPtr + prefix.size(), &end, 10);
    if (end != (versionStrPtr + maj_sep)) {
        return OpenOCDVersion();
    }

    long versionMinor = std::strtol(versionStrPtr + maj_sep + 1, &end, 10);
    if (end != (versionStrPtr + min_sep)) {
        return OpenOCDVersion();
    }

    if (versionMajor >= (1 << 8) || versionMajor < 0) {
        return OpenOCDVersion();
    }

    if (versionMinor >= (1 << 8) || versionMinor < 0) {
        return OpenOCDVersion();
    }

    return OpenOCDVersion((uint8_t)versionMajor, (uint8_t)versionMinor);
}
