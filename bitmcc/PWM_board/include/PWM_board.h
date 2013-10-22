/* -----------------------------------------------------------------------
 * --------------------- RTD DM6916 PWM PC/104 Driver --------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver created to run the DM6916 PWM PC/104 module in
 * QNX.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 10, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 *
 */

// define register addresses
#define PWM_00 0x00
#define PWM_01 0x01
#define PWM_02 0x02
#define PWM_0 0x03

#define PWM_10 0x04
#define PWM_11 0x05
#define PWM_12 0x06
#define PWM_1 0x07

#define PWM_20 0x08
#define PWM_21 0x09
#define PWM_22 0x0a
#define PWM_2 0x0b

// read/write modes for counters
#define LATCH 0x0
#define LSBONLY 0x1
#define MSBONLY 0x2
#define LSBMSB 0x3

// counter formats
#define BINARY 0x0
#define BCD 0x1

// define default base address
#define PWM_ADDR_DEFAULT 0x360
#define PWM_IRQ_DEFAULT 0x7
#define PWM_REG_SIZE 20

#ifndef PWM_BOARD_H_
#define PWM_BOARD_H_


struct PWM_board
{
	uintptr_t base_addr;
	uint8_t port[PWM_REG_SIZE];
	uint8_t IRQ_enable;
	uint32_t IRQ_freq;
};

// define function prototypes
int init_PWM(struct PWM_board* , uint64_t );
int close_PWM(struct PWM_board* );
int enable_PWM_port(struct PWM_board* , uint8_t );
int disable_PWM_port(struct PWM_board* , uint8_t );
int set_PWM_DIO(struct PWM_board* , uint8_t );
int clear_PWM_DIO(struct PWM_board* , uint8_t );
int set_duty_cycle(struct PWM_board* , uint8_t, int );
int configure_counter(struct PWM_board* , uint8_t , uint8_t , uint8_t , uint8_t );
int set_counter(struct PWM_board*, uint8_t , uint16_t );
int enable_IRQ(struct PWM_board* );
int disable_IRQ(struct PWM_board* );
int check_IRQ(struct PWM_board* );
int clear_IRQ(struct PWM_board* );
int set_IRQ_freq(struct PWM_board* , float );

#endif /* PWM_BOARD_H_ */
