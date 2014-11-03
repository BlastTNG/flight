/* -----------------------------------------------------------------------
 * ------------------- K089150 Frameless Motor Driver --------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the the K089150 frameless
 * motors via the AMC AZbdc12a8 driver. The motor is driven via PWM
 * command from the DM6916 PWM board.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: September 3, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

#ifndef K089150_FRAME_H_
#define K089150_FRAME_H_

// some defines
#define FRAMELESS_MOTOR_CNST 1.070319411377828	// motor constant kt [Nm/A]
#define FRAMELESS_MOTOR_MAX_CURRENT 12.0	// maximum current of motor [A]

struct frameless_motor
{
	uint8_t port;
	float current;
	double torque;
	struct PWM_board *pwm;
};

// some function prototypes
int init_frameless(struct frameless_motor *, struct PWM_board *, uint8_t );
int close_frameless(struct frameless_motor *);
int command_torque_frameless(struct frameless_motor *, double );

#endif /* K089150_FRAME_H_ */
