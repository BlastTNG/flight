/* -----------------------------------------------------------------------
 * --------------- EMM-8P-XT RevC RS-422/485 PC/104 Driver ---------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is a custom driver for the EMM-8P-XT RS-422/485 PC/104 module for
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

// define some default memory addresses
#define SERIAL_ADDR_DEFAULT 0x200
#define SERIAL_REG_SIZE 18

// define serial configurations
#define RS232_CONFIG 0x0
#define RS422_CONFIG 0x1
#define RS485_ECHO_CONFIG 0x2
#define RS485_NOECHO_CONFIG 0x3

#ifndef SERIAL_BOARD_H_
#define SERIAL_BOARD_H_

struct serial_board
{
	uintptr_t base_addr;
	uint64_t port_addr[8];
	uint8_t port_IRQ[8];
	uint8_t port_enable[8];
	uint8_t config[2];
	uint8_t DIO_config;
	uint8_t DIO_output;
};

// define function prototypes
int init_serial(struct serial_board* , uint64_t );
int close_serial(struct serial_board* );
int read_serial_byte(struct serial_board* , uint8_t , uint8_t* );
int write_serial_byte(struct serial_board* , uint8_t , uint8_t* );
int enable_serial_port(struct serial_board* , uint8_t );
int disable_serial_port(struct serial_board* , uint8_t );
int set_port_addr(struct serial_board* , uint8_t , uint64_t );
int set_port_IRQ(struct serial_board* , uint8_t , uint8_t );
int save_to_EEPROM(struct serial_board* );
int reload_from_EEPROM(struct serial_board* );
int set_port_config(struct serial_board* , uint8_t , uint8_t );
int set_serial_DIO(struct serial_board* , uint8_t );
int clear_serial_DIO(struct serial_board* , uint8_t );
int read_serial_DIO(struct serial_board* , uint8_t , uint8_t *);

#endif /* SERIAL_BOARD_H_ */
