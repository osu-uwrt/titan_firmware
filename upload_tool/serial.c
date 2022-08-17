#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <termios.h>
#include <sys/select.h>
#include <sys/ioctl.h>
#include <linux/sockios.h>
#include <signal.h>

#include "uploader.h"

static volatile int shouldRun;

void signalHandler(int signo) {
    shouldRun = 0;
}

void serial_monitor(char* portname) {
    #define LOCAL_BUFFER_SIZE 32

    int fd = open (portname, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0)
    {
        fprintf(stderr, "error %d opening %s: %s\n", errno, portname, strerror (errno));
        return;
    }

    fprintf(stderr, "Opening %s\n", portname);

    // Setup stdin
    int stdin_fd = fileno(stdin);

    struct termios old = {0};
    struct termios term = {0};
    if (tcgetattr(stdin_fd, &old) < 0)
        perror("tcgetattr()");
    if (tcgetattr(stdin_fd, &term) < 0)
        perror("tcgetattr()");
    term.c_lflag &= ~ICANON;
    term.c_lflag &= ~ECHO;
    term.c_cc[VMIN] = 1;
    term.c_cc[VTIME] = 0;
    if (tcsetattr(stdin_fd, TCSANOW, &term) < 0)
        perror("tcsetattr ICANON");

    // Setup serial port
    struct termios old_ser = {0};
    struct termios new_ser = {0};
    if (tcgetattr(fd, &old_ser) < 0)
        perror("tcgetattr()");
    if (tcgetattr(fd, &new_ser) < 0)
        perror("tcgetattr()");
    new_ser.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL|ICANON);
    new_ser.c_cc[VMIN] = 0;
    new_ser.c_cc[VTIME] = 0;
    if (tcsetattr(fd, TCSANOW, &new_ser) < 0)
        perror("tcsetattr ICANON");

    // Setup signal
    shouldRun = 1;
    if (signal(SIGINT, signalHandler) == SIG_ERR){
        fprintf(stderr, "Unable to catch CTRL-C\n");
    }

    // Define select vars
    fd_set readfds;
    int max_fd;

    while (shouldRun) {
        max_fd = (fd > stdin_fd ? fd : stdin_fd);
        FD_ZERO(&readfds);
        FD_SET(stdin_fd, &readfds);
        FD_SET(fd, &readfds);

        int activity = select(max_fd + 1, &readfds, NULL, NULL, NULL);

        if ((activity < 0) && (errno!=EINTR)) 
        {
            fprintf(stderr, "\nselect error!\n");
            break;
        }

        if (FD_ISSET(stdin_fd, &readfds)) {
            char ch;
            char data[LOCAL_BUFFER_SIZE];
            int buffer_pos = 0;
            int pending = 1;
            while(pending && shouldRun) {
                if (read(stdin_fd, &ch, 1) == 0) {
                    fprintf(stderr, "\nStdin fd closed\n");
                    goto end;
                }
                ioctl(stdin_fd, SIOCINQ, &pending);
                if (buffer_pos == LOCAL_BUFFER_SIZE) {
                    write(fd, data, buffer_pos);
                    buffer_pos = 0;
                }
                data[buffer_pos++] = ch;
            }
            write(fd, data, buffer_pos);
        }

        if (FD_ISSET(fd, &readfds)) {
            char ch;
            char data[LOCAL_BUFFER_SIZE];
            int buffer_pos = 0;
            int pending = 1;
            while(pending && shouldRun) {
                if (read(fd, &ch, 1) == 0) {
                    fprintf(stderr, "\nSerial port fd closed\n");
                    goto end;
                }
                ioctl(fd, SIOCINQ, &pending);
                if (buffer_pos == LOCAL_BUFFER_SIZE) {
                    write(fileno(stdout), data, buffer_pos);
                    buffer_pos = 0;
                }
                if (ch != '\r') {
                    data[buffer_pos++] = ch;
                }
            }
            write(fileno(stdout), data, buffer_pos);
        }
    }


end:
    fprintf(stderr, "\nExiting...\n");
    signal(SIGINT, SIG_DFL);
    if (tcsetattr(stdin_fd, TCSADRAIN, &old) < 0)
        perror ("tcsetattr ~ICANON");
    close(fd);
}