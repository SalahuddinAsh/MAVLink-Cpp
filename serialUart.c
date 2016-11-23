
#include "serialUart.h"

// Close the serial port
void serialClose(int fd)
{
  tcflush(fd, TCIOFLUSH);
  close(fd);
  if(DEBUG == 1)
    {
      printf("Serial port successfully closed\n");
    }
}


// Opens the serial port . e.g., openSerial("/dev/ttyAMA0", 115200);
int serialOpen(const char *port, const uint baud)
{
  // File descriptor to handle the serial port
  int fd = -1;

  // open() function */
  // O_RDWR   - Read/Write access to serial port
  // O_NOCTTY - No terminal will control the process
  // Open in blocking mode,read will wait
  // Change /dev/ttyUSB0 to the one corresponding to your system
  fd = open(port, O_RDWR | O_NOCTTY | O_NDELAY);
  // If failed to open the port, print an error message
  if (fd == -1)
    {
      if(DEBUG == 1)
	{
	  printf("[ERROR] Couldn't open port \"%s\": %s\n", port, strerror(errno));
	}
      return -1;
    }
  else
    {
      if(DEBUG == 1)
	{
	  printf("Serial port %s successfully opened\n", port);
	}
    }

  // Configure and initialize the uart link
  struct termios options;
  tcgetattr(fd, &options);	    // Get the current attributes of the Serial port
  options.c_iflag = IGNPAR;
  options.c_oflag = 0;
  options.c_lflag = 0;

  switch(baud)
    {
    case 9600:
      options.c_cflag = B9600 | CS8 | CLOCAL | CREAD; // Baudrate=9600, Data bits=8, Ignore Modem Control Lines, Enable receiver
      break;
    case 115200:
      options.c_cflag = B115200 | CS8 | CLOCAL | CREAD; // Baudrate=115200, Data bits=8, Ignore Modem Control Lines, Enable receiver
      break;
    default:
      if(DEBUG == 1)
	{
	  fprintf(stderr, "Warning: Baudrate not supported\n");
	}
      serialClose(fd);
      return -1;
    }

  tcflush(fd, TCIFLUSH);	      // Discards old data in the rx buffer
  tcsetattr(fd, TCSANOW, &options);   // Set the attributes to the termios structure

  return fd;
}

// Checks if 1 data byte is available in the RX buffer at the moment
int serialHasChar(int fd)
{
  struct pollfd fds;
  fds.fd = fd;
  fds.events = (POLLIN | POLLPRI);  // POLLIN : There is data to read, POLLPRI: There is urgent data to read
  if(poll(&fds, 1, 0) > 0)
    {
      return 1;
    }
  else
    {
      return 0;
    }
}

// Returns the number of available bytes to be received at the moment
// Warning: this function blocks for 1 second
int serialNumOfAvailableBytes(int fd)
{
  int availBytes = 0;
  sleep(1); // Wait 1 second for the input buffer to fill in
  ioctl(fd, FIONREAD, &availBytes);
  return availBytes;
}
			   


// Send "bytesToSend" bytes of the buffer pointed at by "buf" over fd,
// Returns the number of sent bytes
int serialWrite(int fd, uint8_t *buf, uint16_t bytesToSend)
{
  uint16_t sentBytes = write(fd, buf, bytesToSend); // Send write(File Descriptor, buffer to send, number of bytes);

  if (sentBytes < 0) {
    if(DEBUG == 1)
      {
	printf("[ERROR] Serial sending failed ..\n");
      }
    return -1;
  } else {
    if(DEBUG == 1)
      {
	printf("[STATUS: TX %i Bytes]\n", sentBytes);
      }
    return sentBytes;
  }
}

// Receive "bytesToReceive" bytes via "fd", save them in the buffer pointed to by "buf"
// Returns the number of received bytes
int serialRead(int fd, uint8_t *buf, const uint bytesToReceive)
{
  int av_bytes = 0;
  int chars_left = bytesToReceive, chars_read = 0;

  while(chars_left)
    {
      chars_read = read(fd, buf, chars_left);
      if(chars_read == -1)
	{
	  // an error occurred;
	  return -1;
	  break;
	}
      else
	{
	  buf += chars_read;
	  chars_left -= chars_read;
	}
    }

  if(DEBUG == 1)
    {
      printf("[STATUS: RX %i Bytes]\n", bytesToReceive - chars_left);
    }

  return bytesToReceive - chars_left;
}

// Receive one byte
uint8_t serialReadChar(int fd)
{
  uint8_t ch;
  //tcflow(fd, TCOON);
  read(fd, &ch, 1);
  //tcflow(fd, TCOOFF);
  if(DEBUG == 1)
    {
      printf("One byte received : 0x%.2x\n", ch);
    }
  return ch;
}

