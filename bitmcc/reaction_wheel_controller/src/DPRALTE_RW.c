/* -----------------------------------------------------------------------
 * -------------------------- DPRALTE RW Driver---------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is simple driver interface for the DPRALTE reaction wheel
 * controller via RS-232 serial interface.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: July 23, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 * 29/07/13 - added verbosity levels for debugging purposes
 * 		verbosity = 0 => no messages
 * 		verbosity = 1 => display sent and received messages
 * 		verbosity = 2 => check and display generated values
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <sys/mman.h>
#include <termios.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "serial_board.h"
#include "IO_board.h"
#include "DPRALTE_RW.h"
#include "CRC_func.h"

// initializes the RW controller through serial board at specified port and IO board
int init_DPRALTE(struct rw_controller* rw, struct serial_board* serial, struct IO_board* io, uint8_t port)
{
	rw->io = io;
	//printf("Generating CRC table...");
	if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
	{
		printf("mk_crctable() memory allocation failed\n");
		return -1;
	}

	unsigned int i;
	// check CRC table generated values
	if (rw->verbosity == 2)
	{
		for (i=0;i<256;i++)
		{
			printf("0x%x ",crctable[i]);
			if ((i+1)%8 == 0) printf("\n");
		}
	}

	/* Set default RW gains here (I_rw*a_rw = -torque_rw) */
	rw->kp_wrw = 0; // proportional to RW speed error (< 0)
	rw->ki_wrw = 0; // integral to RW speed error (< 0)
	rw->kp_waz = 0; // proportional to gondola speed  (< 0)
	rw->ki_waz = 0; // integral to gondola speed (< 0)
	rw->w_rwd = 0; // desired RW speed [rad/s]

	// initialize serial interface if specified
	if (serial)
	{
		struct termios settings;

		// build the location of the serial driver device
		char path[20];
		char nm[10];
		char base[] = "/dev/ser";

		itoa(port+1,nm,10);
		strcpy(path,base);
		strcat(path,nm);


		// setup the serial port and IO board
		set_port_config(serial,port,RS232_CONFIG); // set RS232 configuration
		set_serial_DIO(serial,port); // set DIO bit to specify RS232 configuration on DPRALTE
		enable_serial_port(serial,port); // enable the serial port
		rw->port = port;
		rw->serial = serial;
		rw->fd = open(path, O_RDWR); // connect to port (read and write capable)

		if (rw->fd == -1)
		{
			printf("Error opening serial device.\n");
			return -1;
		}
		tcgetattr(rw->fd, &settings); // get current settings
		if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
			{
				printf("mk_crctable() memory allocation failed\n");
				return -1;
			}

			unsigned int i;
			// check CRC table generated values
			if (rw->verbosity == 2)
			{
				for (i=0;i<256;i++)
				{
					printf("0x%x ",crctable[i]);
					if ((i+1)%8 == 0) printf("\n");
				}
			}
		cfsetospeed(&settings, B115200); // set baud rate (115200 bps default)
		cfsetispeed(&settings, B115200); // set baud rate (115200 bps default)

		// standard setting for DSP 1750
		settings.c_cflag = (settings.c_cflag & ~CSIZE) | CS8;     // 8-bit chars
		// disable IGNBRK for mismatched speed tests; otherwise receive break
		// as \000 chars

		settings.c_iflag &= ~IGNBRK;         // ignore break signal
		settings.c_lflag = 0;                // no signaling chars, no echo,
										// no canonical processing
		settings.c_oflag = 0;                // no remapping, no delays
		settings.c_cc[VMIN]  = 0;            // read doesn't block
		settings.c_cc[VTIME] = 10;            // milliseconds read timeout

		settings.c_iflag &= ~(IXON | IXOFF | IXANY); // shut off xon/xoff ctrl
		settings.c_cflag &= ~CSTOPB;
		settings.c_oflag &= ~OPOST;                               // enable raw output
		settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);      // enable raw input

		settings.c_cflag |= (CLOCAL | CREAD);// ignore modem controls,
										// enable reading
		settings.c_cflag &= ~(PARENB | PARODD);      // shut off parity

		tcsetattr(rw->fd, TCSANOW, &settings); // apply the settings
		tcflush(rw->fd, TCIOFLUSH); // flush the port before use

		enable_write_access_DPRALTE(rw);
	}
	else rw->serial = NULL;
	return 0;
}

// closes the RW controller
int close_DPRALTE(struct rw_controller* rw)
{
	if (rw->serial)
	{
		disable_write_access_DPRALTE(rw);
		//clear_serial_DIO(serial,rw->port);
		disable_serial_port(rw->serial,rw->port);
		close(rw->fd); // close serial port
	}

	free(crctable); // free memory for the crctable
	return 0;
}

// reads the RW controller and stores the data in the buffer
int read_message_DPRALTE(struct rw_controller* rw)
{
	int i;
	uint8_t byte;
	uint16_t accum = 0;
	unsigned int msg_len = 8; // message is at least 8 bytes long (header length)

	// loop until SOF (0xa5) is read
	do
	{
		read(rw->fd,&byte,1);
		//read_serial_byte(rw->serial,rw->port,&byte); // for raw serial mode
		if (byte == 0xa5) // check to see if SOF byte has been received
		{
			rw->buffer[0] = byte;
			crccheck(byte,&accum,crctable);
			i = 1;
			break;
		}
	}
	while (1);

	// loop until all bytes received
	do
	{
		read(rw->fd,&byte,1);
		//read_serial_byte(rw->serial,rw->port,&byte); // for raw serial mode
		rw->buffer[i] = byte;
		crccheck(byte,&accum,crctable);

		// get data words bytes to read the correct number of bytes
		if ((i == 5) && (byte != 0x0)) msg_len = (2*byte)+10;
		i++;
	}
	while (i < msg_len);

	rw->msg_len = i; // store the received message length

	// check if DPRALTE reports errors
	if (rw->buffer[3] == 0x02)
	{
		printf("Incomplete command.\n");
		return -1;
	}
	else if (rw->buffer[3] == 0x04)
	{
		printf("Invalid command issued.\n");
		return -1;
	}
	else if (rw->buffer[3] == 0x06)
	{
		printf("Write access denied.\n");
		return -1;
	}
	else if (rw->buffer[3] == 0x08)
	{
		printf("Framing/CRC error.\n");
		return -1;
	}

	// verify that received message does not contain data errors
	if (accum != 0)
	{
		printf("Invalid message received.\n");
		return -1;
	}

	// print out received message for debugging purposes
	if (rw->verbosity == 1)
	{
		printf("Message received: ");
		for (i=0;i<rw->msg_len;i++) printf("0x%x ",rw->buffer[i]);
		printf("\n");
	}
	//tcflush(rw->fd, TCIFLUSH); // flush the port
	return i;
}

// sends a message with specified index, offset, and size (number of 2 byte words) to the RW controller
// if message is NULL (contains no data) then the receive buffer will expect size bytes when read
// index and offset values for particular commands are given in the command dictionary in documentation
int write_message_DPRALTE(struct rw_controller* rw, uint8_t index, uint8_t offset, uint8_t size, uint8_t *message)
{
	unsigned int i;
	unsigned int msg_len;
	uint8_t is_data;
	uint8_t seq;
	static uint8_t msg[RW_MAX_BUF_SIZE], temp[2];
	uint16_t accum;

	// check to see if there is data to send or not
	if (message == NULL)
	{
		is_data = 0;
		msg_len = 8; // only send the header
		seq = 0x01; // message does not contain data
	}
	else
	{
		is_data = 1;
		msg_len = (2*size)+10; // send header and data
		seq = 0x02; // message does contain data
	}

	uint8_t header[6] = {0xa5,0x3f,seq,index,offset,size};
	memcpy(msg+0,&header,6);
	accum = 0;
	for (i=0;i<6;i++) crccheck(msg[i],&accum,crctable);
	if (rw->verbosity == 2) printf("Header CRC: 0x%x\n",accum);
	memcpy(temp,&accum,2);
	memcpy(msg+6,&temp[1],1);
	memcpy(msg+7,&temp[0],1);

	// add data field if there is data to add
	if (is_data)
	{
		memcpy(msg+8,message,2*size);
		accum = 0;
		for (i=8;i<(2*size)+8;i++) crccheck(msg[i],&accum,crctable);
		if (rw->verbosity == 2) printf("Data CRC: 0x%x\n",accum);
		memcpy(temp,&accum,2);
		memcpy(msg+(msg_len-2),&temp[1],1);
		memcpy(msg+(msg_len-1),&temp[0],1);
	}

	// perform and CRC check on the assembled message before sending
	accum = 0;
	for (i=0;i<msg_len;i++) crccheck(msg[i],&accum,crctable);
	if (rw->verbosity == 2) printf("CRC Check: 0x%x\n",accum);
	if (accum != 0x00)
	{
		printf("Assembled an invalid message. Invalid CRC: 0x%x\n",accum);
		return -1;
	}

	// display the message for debugging purposes
	if (rw->verbosity == 1)
	{
		printf("Message sent: ");
		for (i=0;i<msg_len;i++) printf("0x%x ",msg[i]);
		printf("\n");
	}

	// send the message
	size_t res = write (rw->fd,&msg,msg_len);
	// check to see if error has occurred in writing to RW controller
	if (res < 0)
	{
		printf("RW controller serial write error has occurred.\n");
		return -1;
	}
	tcflush(rw->fd, TCOFLUSH); // flush the port

	//for (i=0;i<msg_len;i++) write_serial_byte(rw->serial,rw->port,&msg[i]); // for raw serial mode

	return 0;
}

// sets the kp, ki, and kd gains corresponding to a velocity loop PID controller for config=0,1
int set_vel_gains_DPRALTE(struct rw_controller* rw, uint8_t config, uint32_t kp, uint32_t ki, uint32_t kd)
{
	// check for invalid gains
	if ((kp < 0) || (kp > KP_VEL_MAX))
	{
		printf("Invalid proportional gain specified.\n");
		return -1;
	}
	if ((ki < 0) || (ki > KI_VEL_MAX))
	{
		printf("Invalid integral gain specified.\n");
		return -1;
	}
	if ((kd < 0) || (kd > KD_VEL_MAX))
	{
		printf("Invalid derivative gain specified.\n");
		return -1;
	}

	uint8_t temp[4];
	kp *= 0x0666; // scaling factor for kp
	ki *= 0x06666666; // scaling factor for ki
	kd *= 0x0666; // scaling factor for kd

	if (config == 0)
	{
		// set kp
		memcpy(temp,&kp,4);
		write_message_DPRALTE(rw,0x36,0x03,2,temp);
		read_message_DPRALTE(rw);

		// set ki
		memcpy(temp,&ki,4);
		write_message_DPRALTE(rw,0x36,0x05,2,temp);
		read_message_DPRALTE(rw);

		// set kd
		memcpy(temp,&kd,4);
		write_message_DPRALTE(rw,0x36,0x07,2,temp);
		read_message_DPRALTE(rw);
	}
	else if (config == 1)
	{
		// set kp
		memcpy(temp,&kp,4);
		write_message_DPRALTE(rw,0x36,0x0d,2,temp);
		read_message_DPRALTE(rw);

		// set ki
		memcpy(temp,&ki,4);
		write_message_DPRALTE(rw,0x36,0x0f,2,temp);
		read_message_DPRALTE(rw);

		// set kd
		memcpy(temp,&kd,4);
		write_message_DPRALTE(rw,0x36,0x11,2,temp);
		read_message_DPRALTE(rw);
	}
	else
	{
		printf("Configuration must be 0 or 1.\n");
		return -1;
	}
	return 0;
}

// enables write access to DPRALTE RW controller for all parameters
int enable_write_access_DPRALTE(struct rw_controller* rw)
{
	uint16_t data;
	uint8_t temp[2];

	// gain write access to all parameters
	data = 0x000e;
	memcpy(temp,&data,2);
	write_message_DPRALTE(rw,0x07,0x00,1,temp);
	read_message_DPRALTE(rw);
	return 0;
}

// disables write access to DPRALTE RW controller for all parameters
int disable_write_access_DPRALTE(struct rw_controller* rw)
{
	uint16_t data;
	uint8_t temp[2];

	// revoke write access to all parameters
	data = 0x0000;
	memcpy(temp,&data,2);
	write_message_DPRALTE(rw,0x07,0x00,1,temp);
	read_message_DPRALTE(rw);
	return 0;
}

// check the reaction wheel controller drive for any errors and report them
int check_drive_errors_DPRALTE(struct rw_controller* rw)
{
	write_message_DPRALTE(rw,2,1,1,NULL);
	read_message_DPRALTE(rw);
	if (rw->buffer[8] & 0x1) printf("Drive reset.\n");
	if (rw->buffer[8] & 0x2) printf("Drive internal error.\n");
	if (rw->buffer[8] & 0x4) printf("Short circuit.\n");
	if (rw->buffer[8] & 0x8) printf("Current overshoot.\n");
	if (rw->buffer[8] & 0x10) printf("Under voltage.\n");
	if (rw->buffer[8] & 0x20) printf("Over voltage.\n");
	if (rw->buffer[8] & 0x40) printf("Drive over temperature.\n");

	write_message_DPRALTE(rw,2,2,1,NULL);
	read_message_DPRALTE(rw);
	if (rw->buffer[8] & 0x1) printf("Parameter restore error.\n");
	if (rw->buffer[8] & 0x2) printf("Parameter store error.\n");
	if (rw->buffer[8] & 0x4) printf("Invalid hall state.\n");
	if (rw->buffer[8] & 0x8) printf("Phase sync. error.\n");
	if (rw->buffer[8] & 0x10) printf("Motor over temperature.\n");
	if (rw->buffer[8] & 0x20) printf("Phase detection fault.\n");
	if (rw->buffer[8] & 0x40) printf("Feedback sensor error.\n");
	if (rw->buffer[8] & 0x80) printf("Motor over speed.\n");
	if (rw->buffer[9] & 0x1) printf("Max. measured position.\n");
	if (rw->buffer[9] & 0x2) printf("Min. measured position.\n");
	if (rw->buffer[9] & 0x4) printf("Comm. error (node guarding).\n");
	if (rw->buffer[9] & 0x8) printf("Broken wire.\n");
	if (rw->buffer[9] & 0x10) printf("Motion engine fault.\n");

	return 0;
}

// command torque from reaction wheel via analog input from I/O board
int command_torque_RW(struct rw_controller* rw, double torque)
{
	unsigned long code;
	double voltage = torque/RW_KT/RW_CURR_VOLT_RATIO; // get torque from K_t motor constant

	// ensure codes are within 12-bit range
	if (voltage >= 10) code = 0xfff;
	else if (voltage <= -10) code = 0x000;
	else dscVoltageToDACode(rw->io->dscb,rw->io->dscdasettings, voltage, &code);


	dscDAConvert(rw->io->dscb,0,code);
	//printf("DA code issued: 0x%x\n",code);

	return 0;
}

// reads the velocity of the reaction wheel via serial in rad/s and stores in rw.velocity
int read_velocity_serial_RW(struct rw_controller* rw)
{
	int32_t data;
	write_message_DPRALTE(rw,0x11,0x2,2,NULL);
	read_message_DPRALTE(rw);
	memcpy(&data,rw->buffer+8,4);
	//printf("0x%x\n",data);
	rw->velocity = ((int) data)/-131072.0/RW_COUNT_PER_REV*RW_SWITCH_FREQ*2.0*PI; // scale factor of 2^17*RW_COUNT_PER_REV/RW_SWITCH_FREQ

	return 0;
}

// reads the velocity of the reaction wheel via the I/O board in rad/s and stores in rw.velocity
int read_velocity_RW(struct rw_controller* rw)
{
	DSCSAMPLE sample;
	DFLOAT voltage;

	// read the velocity voltage
	if ((rw->io->result = dscADSample(rw->io->dscb, &sample)) != DE_NONE)
	{
		printf("Failed to retrieve RW velocity from I/O board.\n");
		return -1;
	}
	// convert 16-bit signed value to angular speed [rad/s]
	dscADCodeToVoltage(rw->io->dscb,rw->io->dscadsettings,sample,&voltage);
	rw->velocity = -voltage*RW_SPEED_VOLT_RATIO; // switch polarity

	return 0;
}

// reads the current of the reaction wheel via serial in amps and stores in rw.current
int read_current_serial_RW(struct rw_controller* rw)
{
	int16_t data;
	write_message_DPRALTE(rw,0x10,0x3,1,NULL);
	read_message_DPRALTE(rw);
	memcpy(&data,rw->buffer+8,2);
	//printf("0x%x\n",data);
	rw->current = ((int) data)/8192.0*RW_CURR_MAX; // scale factor of 2^13/RW_CURR_MAX

	return 0;
}

// initializes the reaction wheel controller through the specified serial port and IO board
int init_DPRALTE_raw(struct rw_controller* rw, struct serial_board* serial, struct IO_board* io, uint8_t port)
{
	// setup the serial port and IO board
	set_port_config(serial,port,RS232_CONFIG); // set RS232 configuration
	set_serial_DIO(serial,port); // set DIO bit to specify RS232 configuration on DPRALTE
	enable_serial_port(serial,port); // enable the serial port
	rw->port = port;
	rw->io = io;
	rw->serial = serial;

	// generate the CRC table for message checks
	if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
	{
		printf("mk_crctable() memory allocation failed\n");
		return -1;
	}

	// configure the serial port
	out8(serial->port_addr[port]+0x01,0x0); // disable all interrupts
	out8(serial->port_addr[port]+0x02,0x0); // disable FIFO
	out8(serial->port_addr[port]+0x03,0x80); // select device latch registers
	out8(serial->port_addr[port]+0x00,0x1); // set DLL for 115.2 bps
	out8(serial->port_addr[port]+0x01,0x0); // set DLM for 115.2 bps
	out8(serial->port_addr[port]+0x03,0x3); // set parity=none, stop bits=1, and word length=8

	// gain write access to the RW controller
	enable_write_access_DPRALTE(rw);

	return 0;
}

// closes the reaction wheel controller
int close_DPRALTE_raw(struct rw_controller* rw)
{
	disable_write_access_DPRALTE(rw);
	//clear_serial_DIO(rw->serial,rw->port);
	disable_serial_port(rw->serial,rw->port);
	free(crctable); // free memory for the crctable
	return 0;
}

