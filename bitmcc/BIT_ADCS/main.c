/* -----------------------------------------------------------------------
 * ------------------------- BIT ADCS MAIN PROGRAM -----------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL)
 * Version 2 or higher.
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is the main program for the Balloon-borne Imaging Testbed Attitude
 * Determination and Control systems (BIT ADCS).
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 24, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

// standard includes
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <termios.h> // serial port settings
#include <fcntl.h> // serial port control
#include <sys/mman.h> // address mapping for I/O
#include <sys/types.h> // socket stuff
#include <sys/stat.h>
#include <sys/socket.h> // socket stuff
#include <hw/inout.h> // raw PC104 I/O routines
#include <sys/neutrino.h> // Neutrino kernel/thread control
#include <gsl/gsl_blas.h> // GSL math stuff
#include <gsl/gsl_linalg.h> // GSL math stuff

// communication protocol headers
#include "command_list.h" // list of BIT commands
#include "netcmd.h" // generic command processing

// custom driver includes
#include "SUSI.h" // access to CPU voltage, temperature, etc
#include "dscud.h" // universal driver
#include "PWM_board.h" // custom PWM board driver
#include "serial_board.h" // custom serial board driver
#include "DSP1750_gyro.h" // custom DSP1750 rate gyro driver
#include "IO_board.h" // custom PWM board driver
#include "IM483_motor.h" // custom IM483 stepper motor driver (pivot)
#include "DPRALTE_RW.h" // custom driver for DPRALTE RW controller
#include "HMR2300_mag.h" // custom driver for the HMR2300 magnetometer
#include "bitcmd_receive.h" // ground connection via UDP
#include "CRC_func.h" // CRC checks and generators for message validation

// ADCS specific includes
#include "BIT_ADCS.h"

// main program loop
int main()
{
	ThreadCtl(_NTO_TCTL_IO, 0); // request I/O privileges for the thread

	//--------------
	// DECLARATIONS
	//--------------

	int i=0, j=0, BITON; // counter
	struct sigevent event; // event for attached interrupts

	//----------------
	// INITIALIZATION
	//----------------
	printf("Initializing hardware ...");

	// initialize the boards
	init_serial(&serial,SERIAL_ADDR_DEFAULT); // initialize serial board
	init_PWM(&pwm,PWM_ADDR_DEFAULT); // initialize PWM board
	init_IO(&io,IO_ADDR_DEFAULT,IO_IRQ_DEFAULT); // initialize IO board
	init_aux(); // auxiliary initialization procedures

	// attach clock function to PWM board clock (IRQ 7)
	ADCS_clock = 0; // reset the clock
	int interruptID = InterruptAttach (PWM_IRQ_DEFAULT, intHandlerClock, &event, sizeof (event), 0);
	if (interruptID == -1) printf ("InterruptAttach () : Can't attach to IRQ.");

	// connect to BITCMD via UDP
	init_bitcmd(&bitcmd); // initialize BITCMD over UDP

	printf("done!\n");

	//------------
	// OPERATIONS
	//------------

	// activate specific hardware
	//change_hardware_state(BITCMD_ID | GYRO_ID | RW_ID | ADCS_ID | SUSI_ID);
	change_hardware_state(NULL);
	BITON = 1;

	sleep(1); // wait a sec...

	printf("Beginning BITCMD listen...\n");

	// loop continuously to receive and execute commands from BITCMD
	while (BITON)
	{
		read_bitcmd(&bitcmd); // read the command (blocking)

		/* Execute the received command */

		// check single commands first
		if (bitcmd.n_in == 0) // no parameters -> single command
		{
			// find command index
			for (i=0;i<N_SCOMMANDS;i++) if (strcmp(scommands[i].name,bitcmd.id)==0) break;

			// switch through possible commands
			if (strncmp(bitcmd.id,"::group::",9) == 0) {
				send_groupnames(&bitcmd);
			} else if (strncmp(bitcmd.id,"::list::",8) == 0) {
				send_cmdlist(&bitcmd);
			} else if (strncmp(bitcmd.id,"::ping::",8) == 0) {
				send_pong(&bitcmd);
			} else if (strncmp("killprog",bitcmd.id,strlen(bitcmd.id))==0) {
				BITON = 0;
			} else if (strncmp("gyro_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(GYRO_ID);
			} else if (strncmp("gyro_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(GYRO_ID);
			} else if (strncmp("rw_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(RW_ID);
			} else if (strncmp("rw_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(RW_ID);
			} else if (strncmp("pivot_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(PIVOT_ID);
			} else if (strncmp("pivot_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(PIVOT_ID);
			} else if (strncmp("p_stp_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(STEP_PITCH_ID);
			} else if (strncmp("p_stp_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(STEP_PITCH_ID);
			} else if (strncmp("p_frm_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(FRAME_PITCH_ID);
			} else if (strncmp("p_frm_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(FRAME_PITCH_ID);
			} else if (strncmp("r_frm_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(FRAME_ROLL_ID);
			} else if (strncmp("r_frm_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(FRAME_ROLL_ID);
			} else if (strncmp("mag_on",bitcmd.id,strlen(bitcmd.id))==0) {
				open_hardware(MAG_ID);
			} else if (strncmp("mag_off",bitcmd.id,strlen(bitcmd.id))==0) {
				close_hardware(MAG_ID);
			}
		}
		else // there are parameters -> multi-command
		{
			// validate command
			j = mcom_validate(bitcmd.id,bitcmd.inputs,bitcmd.n_in);

			if (j != -1) {

			// switch through possible commands
			if (strncmp("rw_torque",bitcmd.id,strlen(bitcmd.id))==0) {
				rw.torque = bitcmd.inputs[0];
			} else if (strncmp("rw_gains",bitcmd.id,strlen(bitcmd.id))==0) {
				rw.kp_wrw = bitcmd.inputs[0];
				rw.ki_wrw = bitcmd.inputs[1];
				rw.kp_waz = bitcmd.inputs[2];
				rw.ki_waz = bitcmd.inputs[3];
			} else if (strncmp("rw_nspeed",bitcmd.id,strlen(bitcmd.id))==0) {
				rw.w_rwd = bitcmd.inputs[0];
			}



			} else printf("Invalid mcommand received.\n");

		}
		if ((i==N_SCOMMANDS) && (j==N_MCOMMANDS)) printf("Unrecognized command. Check command_list.\n");


	}

	//----------
	// SHUTDOWN
	//----------
	close_bitcmd(&bitcmd); // close BITCMD over UDP

	printf("Closing hardware...");

	InterruptDetach(interruptID); // detach the clock interrupt

	// close boards
	close_aux(); // auxiliary closing procedures
	close_serial(&serial); // close serial board
	close_PWM(&pwm); // close PWM board
	close_IO(&io); // close IO board


	printf("done!\n");

	return 0;
}
