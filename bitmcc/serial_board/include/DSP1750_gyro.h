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

#ifndef DSP1750_H_
#define DSP1750_H_

#define PI 3.141592653589793238462643383
#define DSP_BUF_SIZE 6

struct rategyro
{
	int fd;
	uint8_t state;
	float reading;
	uint8_t locked;
	uint8_t port;
	struct serial_board* serial;
	uint8_t buffer[10];
};

// some global variables
extern struct serial_board serial;
extern struct rategyro gyro1, gyro2, gyro3;
extern int gyros_active;
extern unsigned int num_samples;

// define some function prototypes
int init_DSP1750(struct rategyro* , struct serial_board* , uint8_t );
int close_DSP1750(struct rategyro* );
int read_DSP1750(struct rategyro* );
int init_DSP1750_raw(struct rategyro* , struct serial_board* , uint8_t );
int close_DSP1750_raw(struct rategyro* );
int read_DSP1750_raw(struct rategyro* );




#endif /* DSP1750_H_ */
