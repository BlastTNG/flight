/* -----------------------------------------------------------------------
 * ------------------------- BIT ADCS MAIN PROGRAM -----------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
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
 * Created: June 26, 2013
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
#include "command_list.h"
#include "netcmd.h"

#include "SUSI.h" // access to CPU voltage, temperature, etc
#include "PWM_board.h" // custom PWM board driver
#include "serial_board.h" // custom serial board driver
#include "DSP1750_gyro.h" // custom DSP1750 rate gyro driver
#include "dscud.h" // universal driver
#include "IO_board.h" // custom PWM board driver
#include "IM483_motor.h" // custom IM483 stepper motor driver (pivot)
#include "DPRALTE_RW.h" // custom driver for DPRALTE RW controller
#include "HMR2300_mag.h" // custom driver for the HMR2300 magnetometer
#include "bitcmd_receive.h" // ground connection via UDP
#include "CRC_func.h" // CRC checks and generators for message validation
#include "BIT_ADCS.h"

// --------------------
// 	GLOBAL DEFINITIONS
// --------------------

// some general definitions
volatile unsigned int bit_adcs_active; // active flag for the entire BIT ADCS system
volatile uint16_t bit_control_word = 0x0000; // control word for BIT software modules

// DMM-32DX-AT Analog/Digital I/O board definitions
struct IO_board io;
volatile float pivot_speed_int; // pivot speed [rad/s]

// EMM-8P-XT RS-232/422/485 serial board definitions
struct serial_board serial;

// RTD DM-6916 PWM/counter board definitions
struct PWM_board pwm;
uint32_t ADCS_clock; // master clock [1/10 ms]

// DSP 1750 rate gyroscope definitions
struct rategyro gyro[3];
FILE* gyrofile;
int gyros_active;
unsigned int num_samples;

// DPRALTE reaction wheel controller definitions
struct rw_controller rw;

// HMR2300 magnetometer definitions
struct magnetometer mag;

// BITCMD structure definitions
struct BITCMD bitcmd; // the BIT command structure

// hardware device threads
pthread_t gyro_thread[3];
pthread_t adcs_thread;
pthread_t pivot_thread;
pthread_t rw_thread;
pthread_t susi_thread;
pthread_t mag_thread;

// ---------------------
//  FUNCTION DEFINTIONS
// ---------------------

// auxiliary procedures for initialization
int init_aux(void )
{
	int i;

	// clear all pins for digital I/O initially (everything OFF)
	for (i=1;i<=24;i++) dscDIOClearPin(io.dscb,i);

	// initialize SUSI for system info (i.e. temperature, voltage, etc)
	if (!SusiDllInit())
	{
		printf("SusiDllInit() failed\n");
	}

	if (!SusiHWMAvailable())
	{
		printf("SusiHWMAvailable() failed\n");
		SusiDllUnInit();
	}

	// initialize counter on PWM board
	enable_IRQ(&pwm); // enable interrupts on PWM board
	set_IRQ_freq(&pwm,CLK_FREQ); // set clock frequency

	// generate CRC table in lieu of init_DPRALTE
	if((crctable = mk_crctable((unsigned short)CRC_POLY,crchware)) == NULL)
	{
		printf("mk_crctable() memory allocation failed\n");
		return -1;
	}

	bit_control_word = 0x0; // all hardware is OFF by default
	sleep(1);

	return 0;
}

// auxiliary procedures for closing
int close_aux(void )
{
	// close SUSI
	if (SusiDllUnInit()== FALSE) printf("SusiDllUnInit() failed\n");

	free(crctable); // free memory for the crctable
	change_hardware_state(NULL); // shut down all hardware
	sleep (1); // wait a sec...

	// disable clock
	disable_IRQ(&pwm); // disable clock on PWM board
	return 0;
}


int g_ind = 0; // index for counting gyros

// loops and stores gyro data to a file while the BIT ADCS is active
void begin_gyro_thread(void *arg)
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	int count = 0;
	int port, ind;
	port = (int) arg; // gyro serial port
	ind = g_ind; // gyro index
	g_ind++; // increment the used gyro index

	// initialize the gyro
	init_DSP1750(&gyro[ind],&serial,port);
	//delay(3000); // wait 3 s for gyros to power up


	// build the filename and open logging file
	FILE* myfile;
	float the_time;
	char fname[80];

	// get current date and time for filename
	time_t time_of_day = time(0); // get time now
	sprintf(fname,"log/%d_gyro%d.txt",time_of_day,ind);
	myfile = fopen(fname,"a");

	//printf("%s\n",fname);

	printf("Beginning gyro %d thread...\n",ind);
	// loop until gyros are no longer active
	while (bit_control_word & GYRO_ID)
	{
		read_DSP1750(&gyro[ind]);
		the_time = ((float) ADCS_clock)/CLK_FREQ;
		fprintf(myfile,"%4.6f %4.6f\n",the_time,gyro[ind].reading);
		//if ((count % 1000) == 0) printf("Gyro #%d: %4.6f %4.6f\n",ind,the_time,gyro[ind].reading);
		count++;
	}
	g_ind--; // decrement the gyro index

	// close gyro and thread
	close_DSP1750(&gyro[ind]);
	printf("Closing gyro %d thread.\n",ind);
	fclose(myfile);
}

// begins the reaction wheel monitoring thread
void begin_rw_thread(void )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	// open file to store RW data
	FILE* myfile;
	char fname[80];

	// get current date and time for filename
	time_t time_of_day = time(0); // get time now
	sprintf(fname,"log/%d_RW.txt",time_of_day);
	myfile = fopen(fname,"a");

	float the_time; // variable to store the current run time from ADCS clock

	// initialize the reaction wheel controller
	init_DPRALTE(&rw,NULL,&io,4);

	struct sigevent event;

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	// attach interrupt level 7
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event, _NTO_INTR_FLAGS_TRK_MSK);
	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");

	printf("Starting RW control thread...\n");

	command_torque_RW(&rw,0); // zero out the RW torque initially
	rw.torque = 0;

	while (bit_control_word & RW_ID)
	{
		InterruptWait (0, NULL);
		InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt

		if ((ADCS_clock%(CLK_FREQ/100)) == 0)
		{//if ((count%1000) == 0)
			read_velocity_RW(&rw);
			command_torque_RW(&rw,rw.torque);

			// give a 4 second burst of full torque (~ 4 rad/s for RW speed)
			the_time = ((float) ADCS_clock)/CLK_FREQ;
			fprintf(myfile,"%4.6f %4.6f\n",the_time,rw.velocity);
			//read_current_RW(&rw);
			//printf("%d %f %f\n",ADCS_clock,rw.velocity,rw.current);
		}

	}
	command_torque_RW(&rw,0); // zero out the RW torque before exiting thread
	rw.torque = 0;

	fclose(myfile); // close the RW data logging file
	InterruptDetach(interruptID);

	// close reaction wheel controller
	close_DPRALTE(&rw);

	printf("Stopping RW control thread.\n");
}

// begins the pivot speed control thread
void begin_pivot_thread(void )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	struct sigevent event;

	unsigned int marker, elapsed_time, dir;
	float pivot_period;
	float ratio = CLK_FREQ*(PI/180.0)*PIVOT_DEG_PER_STP/2.0; // conversion ratio for speed (remember the 2 micros/stp)
	marker = ADCS_clock;

	// initialize digital I/O for pivot motor
	dscDIOClearPin(io.dscb,1); // clear pulse input
	dscDIOClearPin(io.dscb,2); // clear direction input
	dscDIOSetPin(io.dscb,3); // enable outputs
	dscDIOSetPin(io.dscb,4); // turn off reset
	delay(100);

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	pivot_speed_int = 0;

	// attach interrupt level 7
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event, _NTO_INTR_FLAGS_TRK_MSK);
	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");

	printf("Starting pivot control thread...\n");

	while (bit_control_word & PIVOT_ID)
	{
		InterruptWait (0, NULL);
		InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt

		// pulse the pivot motor at the desired frequency/speed
	    if (pivot_speed_int != 0)
	    {
	        // set the direction (MAY HAVE TO SWITCH)
	        if (pivot_speed_int < 0) dir=1;
	        else dir=0;

	        pivot_period = ratio/fabs(pivot_speed_int); // [ticks/stp]
	        //printf("Pivot period: %f\nRatio: %f\n",pivot_period,ratio);

	        if (pivot_period < 1) pivot_period = 1; // set minimum period (maximum speed) to 1 tick/stp

	        // pulse the motor is the specified pivot step period has elapsed
	    	elapsed_time = ADCS_clock-marker;
	        if (elapsed_time >= pivot_period)
	        {
	            pulse_IM483(io.dscb,dir); // pulse the pivot motor
	            marker = ADCS_clock; // reset the marker
	            //printf("Pivot pulse! %d\n",ADCS_clock);
	        }
	    }

	}
	InterruptDetach(interruptID);

	printf("Stopping pivot control thread.\n");

}

// begins the SUSI computer monitoring thread
void begin_SUSI_thread(void )
{
	FILE* myfile;
	char fname[80];
	float cpu_temp;

	// get current date and time for filename
	time_t time_of_day = time(0); // get time now
	sprintf(fname,"log/%d_SUSI.txt",time_of_day);
	myfile = fopen(fname,"a");

	float the_time; // variable to store the current run time from ADCS clock

	printf("Starting SUSI monitor...\n");
	while (bit_control_word & SUSI_ID)
	{
		SusiHWMGetTemperature(TCPU, &cpu_temp, NULL);
		the_time = ((float) ADCS_clock)/CLK_FREQ;
		fprintf(myfile,"%4.6f %4.6f\n",the_time,cpu_temp);
		sleep(10);
	}

	printf("Stopping SUSI monitor.\n");
	fclose(myfile);
}

void begin_mag_thread(void )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	FILE* myfile;
	char fname[80];

	// get current date and time for filename
	time_t time_of_day = time(0); // get time now
	sprintf(fname,"log/%d_magneto.txt",time_of_day);
	myfile = fopen(fname,"a");

	float the_time; // variable to store the current run time from ADCS clock

	init_HMR2300(&mag,&serial,5);

	printf("Starting magnetometer thread...\n");

	while (bit_control_word & MAG_ID)
	{
		read_HMR2300(&mag);
		the_time = ((float) ADCS_clock)/CLK_FREQ;
		fprintf(myfile,"%4.6f %4.6f %4.6f %4.6f\n",the_time,mag.reading[0],mag.reading[1],mag.reading[2]);

	}

	close_HMR2300(&mag);
	fclose(myfile);

	printf("Stopping magnetometer thread.\n");
}

// begins the ADCS thread
void begin_ADCS(void )
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	struct sigevent event;

	float w_bit;
	int count;

	count = 0;

	// set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	// attach interrupt level 7
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event, _NTO_INTR_FLAGS_TRK_MSK);
	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");

	printf("Starting ADCS...\n");

	// set all actuators to zero
	rw.torque = 0;
	pivot_speed_int = 0;

	// RW speed control variables
	double w_rwe_0 = 0; // initial RW speed
	double t_rw_0 = 0; // initial time index for RW
	double sum_rwe = 0; // initial RW speed integral term
	double t_rw = ((float) ADCS_clock)/CLK_FREQ; // time index for RW
	double w_rwe = 0; // RW speed

	while (bit_control_word & ADCS_ID)
	{
		InterruptWait (0, NULL);
		InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt
		if ((ADCS_clock%(CLK_FREQ/CONT_FREQ))==0) // set controller frequency to 500 Hz
		{
			// get sensor data
			w_bit = gyro[2].reading; // get current gyro Z data [rad/s]
			//printf("%d %f rad/s %f rad/s\n",ADCS_clock,w,w_rw);

			// command actuators
			w_rwe_0 = w_rwe;
			t_rw_0 = t_rw;
			w_rwe = rw.velocity-rw.w_rwd;
			t_rw = ((float) ADCS_clock)/CLK_FREQ;
			sum_rwe += 0.5*(w_rwe+w_rwe_0)*(t_rw-t_rw_0);

			rw.torque = -rw.kp_wrw*w_rwe-rw.ki_wrw*sum_rwe; // SIGN FLIP!!
			rw.torque += rw.kp_waz*w_bit;
			//pivot_speed_int = Kp[2]*w_bit+Kp[3]*w_rw;
			//pivot_speed_int = 3.14e-4; // test 1 Hz pivot pulse

			if ((count%CONT_FREQ) == 0)
			{
				//printf("Integral: %f\n",sum_rwe);
				printf("%d %f Nm %f rad/s\n",ADCS_clock,rw.torque,rw.velocity);
			}
			//if ((count%CONT_FREQ) == 0) printf("%d %f Nm %f rad/s\n",ADCS_clock,rw.torque,pivot_speed_int);

			count ++;
		}
	}

	// set all actuators to zero
	rw.torque = 0;
	pivot_speed_int = 0;


	printf("Terminating ADCS.\n");
}

// interrupt handler for PWM clock
const struct sigevent * intHandlerClock (void *arg, int id)
{
	ADCS_clock = (ADCS_clock == CLK_FREQ*86400) ? 1 : ADCS_clock+1;
	clear_IRQ(&pwm);
	return NULL;
}

// opens specific hardware by IDs
int open_hardware(uint16_t id)
{
	return change_hardware_state(bit_control_word | id);
}

// closes specific hardware by IDs
int close_hardware(uint16_t id)
{
	return change_hardware_state(bit_control_word & (65535-id));
}

// changes what hardware devices are active by modifying the bit_control_word
int change_hardware_state(uint16_t new_state)
{
	int i; // counter
	int gyro_port[] = {0,1,2}; // serial ports used by rate gyroscopes

	uint16_t change, turn_on, turn_off;
	change = new_state ^ bit_control_word; // detect changes in control word
	turn_on = change & new_state; // determine hardware turned on
	turn_off = change & bit_control_word; // determine hardware turned off

	// turn off specified hardware
	bit_control_word = new_state;

	// turn on specified hardware
	if (turn_on & GYRO_ID) for (i=0;i<3;i++) pthread_create(&gyro_thread[i], NULL, (void *) &begin_gyro_thread, (void*)gyro_port[i]);
	if (turn_on & RW_ID) pthread_create(&rw_thread, NULL, (void *) &begin_rw_thread, NULL);
	if (turn_on & SUSI_ID) pthread_create(&susi_thread, NULL, (void *) &begin_SUSI_thread, NULL);
	if (turn_on & PIVOT_ID) pthread_create(&pivot_thread, NULL, (void *) &begin_pivot_thread, NULL);
	if (turn_on & MAG_ID) pthread_create(&mag_thread, NULL, (void *) &begin_mag_thread, NULL);
	if (turn_on & STEP_PITCH_ID);
	if (turn_on & FRAME_ROLL_ID);
	if (turn_on & FRAME_PITCH_ID);
	if (turn_on & CURRENT_ID);
	if (turn_on & THERMAL_ID);
	if (turn_on & ADCS_ID) pthread_create(&adcs_thread, NULL, (void *) &begin_ADCS, NULL);
	return 0;
}


/*
void *begin_clock_thread(void *ard)
{
	ThreadCtl( _NTO_TCTL_IO, 0 ); // request I/O privileges for this thread

	struct sigevent event;

    // set the event structure
	memset(&event, 0, sizeof(event));
	event.sigev_notify = SIGEV_INTR;

	printf("Starting clock thread...\n");

	ADCS_clock = 0; // reset the clock

	 // attached interrupt level 7 to clock thread
	int interruptID = InterruptAttachEvent (PWM_IRQ_DEFAULT, &event,_NTO_INTR_FLAGS_END | _NTO_INTR_FLAGS_TRK_MSK);

	if (interruptID < 0) printf("Server : InterruptAttachEvent() failed\n");
	while (bit_adcs_active)
	{
		printf("%d\n",ADCS_clock);
		InterruptWait(0,NULL);
		if (check_IRQ(&pwm))
		{
			clear_IRQ(&pwm); // clear the interrupt status
			InterruptUnmask (PWM_IRQ_DEFAULT,interruptID); // unmask the PWM clock interrupt
			ADCS_clock = (ADCS_clock == CLK_FREQ*86400) ? 1 : ADCS_clock+1;

		}

	}
	InterruptDetach(interruptID);
	printf("Closing clock thread.\n");
}
*/
