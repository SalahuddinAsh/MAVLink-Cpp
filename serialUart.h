#ifndef SERIAL_UART
#define SERIAL_UART

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <fcntl.h>      // File control definitions
#include <termios.h>    // POSIX Terminal control definitions
#include <unistd.h>     // UNIX standard definitions
#include <errno.h>      // Error number definitions
#include <poll.h>       // To use poll()
#include <sys/ioctl.h>  // To use ioctl() for advanced serial port configuratio


// DEBUG is 1 to display the printf messages
#define DEBUG 1

// Open the serial port
int serialOpen(const char *port, const uint baud);

// Close the serial port
void serialClose(int fd);

// Check if there is 1 data byte to receive
int serialHasChar(int fd);

// Return the number of available bytes to be received
int serialNumOfAvailableBytes(int fd);

// Send over uart
int serialWrite(int fd, uint8_t *buf, uint16_t bytesToSend);

// Receive over uart
int serialRead(int fd, uint8_t *buf, const uint bytesToReceive);

// Receive one byte
uint8_t serialReadChar(int fd);

#endif
