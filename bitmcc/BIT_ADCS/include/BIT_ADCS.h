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
 * Created: June 24, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#ifndef BIT_ADCS_H_
#define BIT_ADCS_H_

// general defines
#define CLK_FREQ 10000	// clock frequency [Hz] (also the control loop frequency)
#define CONT_FREQ 500 // ADCS controller command frequency [Hz]
#define W_RW_OFFSET 6.28 // reaction wheel nominal speed [rad/s]

// hardware ID defines for bit_control_word
#define GYRO_ID 0x1
#define RW_ID 0x2
#define SUSI_ID 0x4
#define PIVOT_ID 0x8
#define	MAG_ID 0x10
#define STEP_PITCH_ID 0x20
#define FRAME_ROLL_ID 0x40
#define FRAME_PITCH_ID 0x80
#define CURRENT_ID 0x100
#define THERMAL_ID 0x200
#define ADCS_ID 0x400

// some external variables
extern volatile uint16_t bit_control_word;
extern struct IO_board io;
extern volatile float pivot_speed_int;
extern struct serial_board serial;
extern struct PWM_board pwm;
extern struct rategyro gyro[3];
extern FILE* gyrofile;
extern struct rw_controller rw;
extern struct magnetometer mag;
extern struct BITCMD bitcmd;
extern uint32_t ADCS_clock;
extern pthread_t gyro_thread[3];
extern pthread_t adcs_thread;
extern pthread_t comm_thread;
extern pthread_t pivot_thread;
extern pthread_t rw_thread;
extern pthread_t susi_thread;
extern pthread_t mag_thread;


// some function prototypes
void begin_gyro_thread(void * );
//void *begin_clock_thread(void *);
void begin_ADCS(void );
void begin_pivot_thread(void );
void begin_rw_thread(void );
void begin_SUSI_thread(void );
void begin_mag_thread(void );
const struct sigevent * intHandlerClock (void *, int );

int open_hardware(uint16_t );
int close_hardware(uint16_t );
int change_hardware_state(uint16_t );
int init_aux(void );
int close_aux(void );



#endif /* BIT_ADCS_H_ */
