/* -----------------------------------------------------------------------
 * --------------------- HMR2300 Magnetometer Driver-- -------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is simple driver interface for the HMR2300 magnetometer via
 * RS-232 serial interface.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: July 18, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "serial_board.h"
#include "HMR2300_mag.h"

// initializes the magnetometer located at path
int init_HMR2300(struct magnetometer* mag, struct serial_board* serial, int port)
{
	// build the location of the serial driver device
	char path[20];
	char nm[10];
	char base[] = "/dev/ser";

	itoa(port+1,nm,10);
	strcpy(path,base);
	strcat(path,nm);

	set_port_config(serial,port,RS232_CONFIG); // RS232 configuration for serial port
	enable_serial_port(serial,port); // enable the magnetometer serial port
	mag->port = port;
	mag->serial = serial;

	mag->fd = open(path, O_RDWR); // connect to port (read and write capable)
	if (mag->fd == -1)
	{
		printf("Error opening serial device.\n");
		return -1;
	}

	struct termios settings;

	tcgetattr(mag->fd, &settings); // get current settings

	cfsetospeed(&settings, B19200); // set baud rate (switched to 19200 bps from 9600 bps)
	cfsetispeed(&settings, B19200); // set baud rate (switched to 19200 bps from 9600 bps)

	settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars

	settings.c_iflag &= ~IGNBRK;         // ignore break signal
	settings.c_cflag |= (CLOCAL | CREAD); //local connection, no modem control; enable receiving characters
	settings.c_cflag &= ~(PARENB | PARODD);   // No Parity
	settings.c_cflag &= ~CSTOPB;   // 1 Stop Bit
	settings.c_cflag &= ~(IHFLOW | OHFLOW);  // flow control off (no RTS/CTS)
	settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	settings.c_lflag = 0;

	settings.c_cc[VMIN]  = 0;            // read doesn't block
	settings.c_cc[VTIME] = 1;            // tenths of seconds for read timeout (integer)

	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	settings.c_oflag = 0;
	settings.c_oflag &= ~OPOST;

	//cfmakeraw(&settings);

	tcsetattr(mag->fd,TCSANOW,&settings);
	tcflush(mag->fd, TCIOFLUSH); // flush the port before use

	sleep(1);


	// apply initial commands
	char message[100];

	strcpy(message,"*00WE *00A\r"); // ascii
	write_HMR2300(mag,message, strlen(message));

	delay(10);

	strcpy(message,"*00WE *00R=10\r"); // ascii
	write_HMR2300(mag,message, strlen(message));

	delay(10);

	strcpy(message,"*00C\r"); // continuous output
	write_HMR2300(mag,message, strlen(message));

	delay(10);



	return 0;
}

// closes the magnetometer
int close_HMR2300(struct magnetometer* mag)
{
	// apply final commands
	char message[100];


	strcpy(message,"*00P\r"); // stop continuous output
	write_HMR2300(mag,message, strlen(message));

	sleep(1);


	disable_serial_port(mag->serial,mag->port); // disable serial port
	close(mag->fd); // close serial port
	return 0;
}

// reads the magnetometer and stores the data in the buffer
int read_HMR2300(struct magnetometer* mag)
{
	int o = 0;
	int n = 0;
	int i,j,ind,dec;
	double sum;
	char digit;

	unsigned char buf;

	// reset the buffer
	memset(mag->buffer,'\0',MAG_MAX_BUF_SIZE);

	// loop through each byte in the message until return character is received
	do
	{
		n = read(mag->fd,&buf,1);

		if (n < 0)
		{
			printf("Read error\n");
			break;
		}
		else if (n > 0)
		{
			mag->buffer[o] = buf;
			o++;
		}
	}
	while ((buf != '\r') && (o<MAG_MAX_BUF_SIZE));


	if (o==28) // have a message with readings
	{
		// parse the message into 3 readings
		for (i=0;i<3;i++)
		{
			ind = i*9;
			sum = 0;
			dec = 4;

			// parse each character of a given reading
			for (j=1;j<7;j++)
			{
				if (j!=3) // skip the comma
				{
					digit = mag->buffer[ind+j];
					sum += atoi(&digit)*pow(10,dec);
					dec--;
				}
			}
			sum = sum/MAG_CONVERSION;
			if (mag->buffer[ind] == '-') mag->reading[i] = -sum;
			else mag->reading[i] = sum;
		}
	}


	tcflush(mag->fd, TCIFLUSH); // flush the port before use
	return o;
}

// writes len characters to the magnetometer from message buffer
int write_HMR2300(struct magnetometer* mag, char *message, unsigned int len)
{
	int i;


	for (i=0;i<len;i++)
		if (message[i] == '\n')
		{
			message[i] = 0x0d;
			break;
		}

	size_t res = write (mag->fd,message,strlen(message));
	// check to see if error has occurred in writing
	if (res < 0)
	{
		printf("Magnetometer serial write error has occured.\n");
		return -1;
	}
	//tcflush(mag->fd, TCOFLUSH); // flush the port before use
	return 0;
}

