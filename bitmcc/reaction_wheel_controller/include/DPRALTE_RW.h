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
 *
 *
 */

#ifndef DPRALTE_RW_H_
#define DPRALTE_RW_H_

#define PI 3.141592653589793238462643383

#define RW_MAX_BUF_SIZE 550		// maximum number of bytes in message
#define KP_VEL_MAX 1310700		// maximum value for velocity loop proportional gain
#define KI_VEL_MAX 20		// maximum value for velocity loop proportional gain
#define KD_VEL_MAX 80		// maximum value for velocity loop proportional gain
#define RW_KT 0.4995	// reaction wheel motor constant [Nm/A]
#define RW_CURR_VOLT_RATIO 3.0	// reaction wheel current-to-voltage ratio [A/V] (for current commands)
#define RW_CURR_MAX 40.0	// maximum current for the drive [A]
#define RW_SWITCH_FREQ 20000.0	// switching frequency of the drive [Hz]
#define RW_COUNT_PER_REV 3.0 	// the number of counts per revolution [rev/cnt]
#define RW_SPEED_VOLT_RATIO 5.0	// reaction wheel speed-to-voltage ratio [rad/s/V] (for speed measurements)

struct rw_controller
{
	int fd;
	unsigned int msg_len;
	uint8_t port;
	uint8_t verbosity;
	float torque;
	float current;
	float velocity;
	float kp_wrw;
	float kp_waz;
	float ki_wrw;
	float ki_waz;
	float w_rwd;
	struct IO_board *io;
	struct serial_board *serial;
	uint8_t buffer[RW_MAX_BUF_SIZE];
};

// define some function prototypes

int init_DPRALTE(struct rw_controller* , struct serial_board* , struct IO_board* , uint8_t );
int close_DPRALTE(struct rw_controller* );

int init_DPRALTE_raw(struct rw_controller* , struct serial_board* , struct IO_board* , uint8_t );
int close_DPRALTE_raw(struct rw_controller* );

int write_message_DPRALTE(struct rw_controller* , uint8_t , uint8_t , uint8_t , uint8_t *);
int read_message_DPRALTE(struct rw_controller* );
int set_vel_gains_DPRALTE(struct rw_controller* , uint8_t , uint32_t , uint32_t , uint32_t );
int enable_write_access_DPRALTE(struct rw_controller* );
int disable_write_access_DPRALTE(struct rw_controller* );
int command_torque_RW(struct rw_controller* , double );
int read_velocity_serial_RW(struct rw_controller* );
int read_velocity_RW(struct rw_controller* );
int read_current_serial_RW(struct rw_controller* );
int check_drive_errors_DPRALTE(struct rw_controller* );

#endif /* DPRALTE_RW_H_ */



