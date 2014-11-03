/* -----------------------------------------------------------------------
 * ---------------- DSP 1750 Single-Axis Rate Gyro Driver ----------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom serial driver for the  DSP 1750 Single-Axis Rate Gyro
 * for QNX. This driver is based on the "serial_board" driver developed
 * for the EMM-8P-XT RS-422/485 PC/104 module for QNX
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 18, 2013
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
#include <termios.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "serial_board.h"
#include "DSP1750_gyro.h"

// initializes the DSP 1750 on specified port located on serial board
int init_DSP1750(struct rategyro* gyro, struct serial_board* serial, uint8_t port)
{
	struct termios settings;

	// build the location of the serial driver device
	char path[20];
	char nm[10];
	char base[] = "/dev/ser";

	itoa(port+1,nm,10);
	strcpy(path,base);
	strcat(path,nm);

	// setup gyro
	set_port_config(serial,port,RS422_CONFIG);
	enable_serial_port(serial,port); // enable port
	gyro->port = port;
	gyro->serial = serial;
	gyro->fd = open(path, O_RDWR); // connect to port

	if (gyro->fd == -1)
	{
		printf("Error opening serial device.\n");
		return -1;
	}
	tcgetattr(gyro->fd, &settings); // get current settings

	cfsetospeed(&settings, B115200); // set baud rate
	cfsetispeed(&settings, B115200); // set baud rate

	// standard setting for DSP 1750
	settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
	// disable IGNBRK for mismatched speed tests; otherwise receive break
	// as \000 chars

	settings.c_iflag &= ~IGNBRK;         // ignore break signal
	settings.c_lflag = 0;                // no signaling chars, no echo,
									// no canonical processing
	settings.c_oflag = 0;                // no remapping, no delays
	settings.c_cc[VMIN]  = 0;            // read doesn't block
	settings.c_cc[VTIME] = 1;            // tenths of seconds for read timeout (integer)

	settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
	settings.c_cflag &= ~CSTOPB;
	settings.c_oflag &= ~OPOST;
	settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
									// enable reading
	settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity
	settings.c_cflag &= ~CSTOPB;


	//settings.c_cflag &= ~PARENB; // no parity
	//settings.c_cflag &= ~CSTOPB; // 1 stop bit
	//settings.c_cflag &= ~CSIZE;
	//settings.c_cflag |= CS8; // 8 bits
	//settings.c_lflag &= ~ICANON; // canonical mode
	//settings.c_oflag &= ~OPOST; // raw output
	//settings.c_cflag |= IXOFF; // no flow control


	tcsetattr(gyro->fd, TCSANOW, &settings); // apply the settings
	tcflush(gyro->fd, TCIOFLUSH); // flush the port before use

	return 0;
}

// closes the DSP 1750 on serial board
int close_DSP1750(struct rategyro* gyro)
{
	disable_serial_port(gyro->serial,gyro->port); // disable serial port 0
	close(gyro->fd); // close serial port 0;
	return 0;
}

// reads the DSP 1750, verifies the message, and outputs the converted data
int read_DSP1750(struct rategyro* gyro)
{
	int sign, i;
	static int data;
	float reading;
	uint8_t byte;

	i = 0;
	// loop until the beginning of a valid message is received
	do
	{
		//printf("%d\n",gyro->fd);
		//printf("%d\n",i);
		read(gyro->fd,&byte,1);
		//printf("0x%x\n",byte);
		if (gyro->locked) // locked on to beginning of the message
		{
			// check if the next message is received
			if (((gyro->state+1)%4 == byte/64) && (byte & 16) && !(byte & 32))
			{
				gyro->buffer[0] = byte;
				gyro->state = byte/64;
				break;
			}
		}
		else // haven't yet locked on to the beginning of the message
		{
			// check for valid message counts
			if ((byte == 0x10) || (byte == 0x50) || (byte == 0x90) || (byte == 0xd0))
			{
				gyro->buffer[0] = byte;
				gyro->state = byte/64;
				gyro->locked = 1; // locked on to beginning of message
				break;
			}
		}
		if (i >= 6) return -1; // failed read if cannot retrieve valid byte within 10 tries

		i++;
	}
	while (1);

	// fill the rest of the message with data
	for (i=1;i<DSP_BUF_SIZE;i++)
	{
		read(gyro->fd,&gyro->buffer[i],1);
		//printf("0x%x",gyro->buffer[i]);
	}

	//printf("%\n");



	// check the sign
	sign = 0;
	if (gyro->buffer[3] & 32) sign = 1;

	gyro->buffer[3] &= 255-224; // clear unused bits and the sign bit
	//printf("0x%x\n",gyro->buffer[3]);

	data = -sign*2097152+gyro->buffer[3]*65536+gyro->buffer[4]*256+gyro->buffer[5];
	reading = data*476.8/1.0e6*(PI/180); // LSB is 476.8 udeg/s

	// check if the angular rate value is real
	// ** value may spike if there is EM interference **
	if (fabs(reading) > 490)
	{
		return -1;
	}
	gyro->reading = reading;
	return 0;

}


// initializes the DSP 1750 rate gyro located at the specified port on the serial board
int init_DSP1750_raw(struct rategyro* gyro, struct serial_board* serial, uint8_t port)
{
	// setup gyro
	set_port_config(serial,port,RS422_CONFIG);
	enable_serial_port(serial,port); // enable port
	gyro->port = port;
	gyro->serial = serial;

	// configure serial port
	out8(serial->port_addr[port]+0x01,0x3); // enable interrupts from receive line only
	out8(serial->port_addr[port]+0x02,0x0); // disable FIFO
	out8(serial->port_addr[port]+0x03,0x80); // select device latch registers
	out8(serial->port_addr[port]+0x00,0x1); // set DLL for 115.2 bps
	out8(serial->port_addr[port]+0x01,0x0); // set DLM for 115.2 bps
	out8(serial->port_addr[port]+0x03,0x3); // // set parity=none, stop bits=1, and word length=8

	return 0;
}

// closes the DSP1750
int close_DSP1750_raw(struct rategyro* gyro)
{
	disable_serial_port(gyro->serial,gyro->port); // disable serial port
	return 0;
}

// reads the standard message from the DSP1750 rate gyro
int read_DSP1750_raw(struct rategyro* gyro)
{
	int i, sign;
	static int data;
	float reading;
	uint8_t rcv;

	// loop until first byte of m_oldessage is received
	while (1)
	{
		read_serial_byte(gyro->serial, gyro->port, &rcv);
		//printf("RCV: 0x%x\n",rcv);

		if (gyro->locked) // locked on to beginning of the message
		{
			// check if the next message is received
			if (((gyro->state+1)%4 == rcv/64) && (rcv & 16) && !(rcv & 32))
			{
				gyro->buffer[0] = rcv;
				gyro->state = rcv/64;
				break;
			}
		}
		else // haven't yet locked on to the beginning of the message
		{
			// check for valid message counts
			if ((rcv == 0x10) || (rcv == 0x50) || (rcv == 0x90) || (rcv == 0xd0))
			{
				gyro->buffer[0] = rcv;
				gyro->state = rcv/64;
				gyro->locked = 1; // locked on to beginning of message
				break;
			}
		}
	}
	// retrieve the rest of the message
	for (i=1;i<DSP_BUF_SIZE;i++)
	{
		read_serial_byte(gyro->serial, gyro->port, &rcv);
		gyro->buffer[i] = rcv;
	}

	// check the sign
	sign = 0;
	if (gyro->buffer[3] & 32) sign = 1;

	gyro->buffer[3] &= 255-224; // clear unused bits and the sign bit
	//printf("0x%x\n",gyro->buffer[3]);

	data = -sign*2097152+gyro->buffer[3]*65536+gyro->buffer[4]*256+gyro->buffer[5];
	reading = data*476.8/1.0e6; // LSB is 476.8 udeg/s

	// check if the angular rate value is real
	// ** value may spike if there is EM interference **
	if (fabs(reading) > 490)
	{
		return -1;
	}
	gyro->reading = reading;

	return 0;
}
