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
 * controller via RS-232 serial interface. This script activates the
 * interface, enables administrator write access to all parameters, and
 * opens a terminal for accessing command parameters. Refer to the
 * command dictionary in the documentation for input command structure
 * and values.
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
 *
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <pthread.h>
#include <termios.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <hw/inout.h>
#include <sys/neutrino.h>

#include "dscud.h"
#include "IO_board.h"
#include "DPRALTE_RW.h"
#include "CRC_func.h"

int logging_active;
static struct rw_controller rw;
static struct IO_board io;

void *begin_mon_thread( )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread
	int count = 0;

	// RW speed control variables
	double kp_rw = 16.0; // RW proportional gain
	double w_rwd = PI; // desired RW speed

	double w_rwe_0 = 0; // initial RW speed
	double w_rwe = 0; // RW speed

	command_torque_RW(&rw,0);
	while (logging_active)
	{
		// read and display the RW velocity and current
		//read_current_RW(&rw);
		read_velocity_RW(&rw);

		// command actuators
		w_rwe_0 = w_rwe;
		w_rwe = rw.velocity-w_rwd;
		rw.torque = kp_rw*w_rwe;
		command_torque_RW(&rw,rw.torque);

		if ((count%1000) == 0) printf("Torque: %f Nm, Speed: %f rad/s\n",rw.torque,w_rwe);
		count++;
	}
	command_torque_RW(&rw,0);
}

void *begin_comm_thread( )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread
	char line[10];
	char *p;
	double value;

	while (1)
	{
		printf("Enter desired RW torque (+/- 15 Nm max; 0 to stop and exit): ");
		p = fgets (line, 10, stdin);
		if (p == NULL) printf("Input error.\n");
		else
		{
			value = atof(line);
			//printf("%f\n",value);
			//command_torque_RW(&rw,value);
			if (value == 0) break;
		}
	}
	logging_active = 0;
}

int main()
{
	unsigned int i;

	pthread_t mon_thread, comm_thread;

	logging_active = 1; // activate logging thread
	rw.verbosity = 0; // be verbose with sent and received messages

	printf("Initializing hardware...\n");
	init_IO(&io,IO_ADDR_DEFAULT,4); // initialize the IO board
	init_DPRALTE(&rw,NULL,&io,0); // initialize DPRALTE
	printf("done!\n\n");
	//printf("0x%x 0x%x\n",serial.config[0],serial.config[1]); // check serial configuration

	// check the drive for errors
	//check_drive_errors_DPRALTE(&rw);

	// start the command and monitoring threads
	pthread_create(&comm_thread, NULL, &begin_comm_thread, NULL);
	pthread_create(&mon_thread, NULL, &begin_mon_thread, NULL);

	pthread_join(comm_thread,NULL);
	pthread_join(mon_thread,NULL);


	printf("Closing hardware...\n");
	close_DPRALTE(&rw); // close DPRALTE
	close_IO(&io); // close IO board
	printf("done!\n");

	return 0;
}
