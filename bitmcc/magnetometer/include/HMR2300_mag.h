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

#ifndef HMR2300_MAG_H_
#define HMR2300_MAG_H_

#define MAG_MAX_BUF_SIZE 255
#define MAG_CONVERSION 15000	// 15000 cnts is 1.0 gauss

struct magnetometer
{
	int fd;
	struct serial_board *serial;
	int port;
	double reading[3];
	uint8_t buffer[MAG_MAX_BUF_SIZE];
};

// define some function prototypes

int init_HMR2300(struct magnetometer* ,struct serial_board* , int);
int close_HMR2300(struct magnetometer* );
int read_HMR2300(struct magnetometer* );
int write_HMR2300(struct magnetometer* , char * , unsigned int );

#endif /* HMR2300_MAG_H_ */



