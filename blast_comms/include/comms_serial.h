/**
 * @file comms_serial.h
 *
 * @date 2011-02-07
 * @author Seth Hillbrand
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2011-2015 Seth Hillbrand
 *
 * FCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * FCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with FCP; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef COMMS_SERIAL_H_
#define COMMS_SERIAL_H_
#include <stdbool.h>
#include <stddef.h>
#include <termios.h>

#include <comms_common.h>
#include <comms_netsock.h>
#include <pthread.h>

typedef struct comms_serial
{
	comms_socket_t  *sock;
	struct termios  term;
	pthread_mutex_t mutex;
} comms_serial_t;


comms_serial_t *ebex_serial_new(void);
void ebex_serial_reset(comms_serial_t **m_serial);
void ebex_serial_free(void *m_serial);
void ebex_serial_close(comms_serial_t *m_serial);
void ebex_serial_set_fd(comms_serial_t *m_serial, socket_t m_fd);
int ebex_serial_write(comms_serial_t *m_serial, const void *m_buf, size_t m_len);
int ebex_serial_flush(comms_serial_t *m_serial);
bool ebex_serial_setspeed(comms_serial_t *m_serial, speed_t m_speed);

int ebex_serial_connect(comms_serial_t *m_serial, const char *m_terminal);
int ebex_fifo_connect(comms_serial_t *m_serial, const char *m_fifo, int m_flags);
#endif /* COMMS_SERIAL_H_ */
