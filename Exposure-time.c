#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <string.h>
#include "edtinc.h"

// Error handling omitted for brevity

void set_exposure_time(const char *serial_port_path, int exposure_time_clock_cycles) {
    int serial_port = open(serial_port_path, O_RDWR | O_NOCTTY | O_SYNC);

    if (serial_port < 0) {
        fprintf(stderr, "Error opening %s: %s\n", serial_port_path, strerror(errno));
        exit(EXIT_FAILURE);
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if (tcgetattr(serial_port, &tty) != 0) {
        fprintf(stderr, "Error from tcgetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(EXIT_FAILURE);
    }

    cfsetospeed(&tty, B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
    tty.c_iflag &= ~IGNBRK;                         // ignore break signal
    tty.c_lflag = 0;                                // no signaling chars, no echo, no canonical processing
    tty.c_oflag = 0;                                // no remapping, no delays
    tty.c_cc[VMIN]  = 0;                            // read doesn't block
    tty.c_cc[VTIME] = 5;                            // 0.5 seconds read timeout

    tty.c_iflag &= ~(IXON | IXOFF | IXANY);         // shut off xon/xoff ctrl
    tty.c_cflag |= (CLOCAL | CREAD);                // ignore modem controls, enable reading
    tty.c_cflag &= ~(PARENB | PARODD);              // shut off parity
    tty.c_cflag |= 0;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;

    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        fprintf(stderr, "Error from tcsetattr: %s\n", strerror(errno));
        close(serial_port);
        exit(EXIT_FAILURE);
    }

    // Construct the command to set the exposure time
    char command[256];
    snprintf(command, sizeof(command), "SENS:EXPPER %d\r\n", exposure_time_clock_cycles);

    // Send the command
    int n_written = write(serial_port, command, strlen(command));

    if (n_written < 0) {
        fprintf(stderr, "Error writing to serial port: %s\n", strerror(errno));
    }

    // Read the response (if any) from the camera for confirmation/acknowledgment
    // Omitted for brevity - you would implement reading and parsing the response

    close(serial_port);
}

int main(int argc, char **argv) {
    // Existing main code...

    // Assuming the camera's serial port is at /dev/ttyS0
    const char *camera_serial_port = "/dev/ttyS0";
    int exposure_time_seconds = 3; // Desired exposure time in seconds
    int clock_frequency = 1000000; // Replace with actual camera clock frequency in Hz
    int exposure_time_clock_cycles = exposure_time_seconds * clock_frequency;

    // Set the camera exposure time before image acquisition
    set_exposure_time(camera_serial_port, exposure_time_clock_cycles);

    // Continue with image acquisition and processing...
    // Existing main code...

    return 0;
}
