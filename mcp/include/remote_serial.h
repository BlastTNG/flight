/*
 * remote_serial.h
 *
 * This software is copyright (C) 2013-2014 University of Pennsylvania
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Dec 20, 2015 by seth
 */

#ifndef INCLUDE_REMOTE_SERIAL_H_
#define INCLUDE_REMOTE_SERIAL_H_
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <unistd.h>
#include "phenom/defs.h"
#include "phenom/listener.h"
#include "phenom/socket.h"
#include "phenom/memory.h"

typedef struct remote_serial {
    int which;
    int port;
    bool connected;
    bool have_warned_version;
    uint8_t have_warned_connect;
    uint32_t backoff_sec;
    struct timeval timeout;
    ph_job_t connect_job;
    ph_sock_t *sock;
    ph_bufq_t *input_buffer;
} remote_serial_t;

int remote_serial_write_data(remote_serial_t *m_serial, uint8_t *m_data, size_t m_len);
int remote_serial_read_data(remote_serial_t *m_serial, uint8_t *m_buffer, size_t m_size);
int remote_serial_flush(remote_serial_t *m_serial);
void remote_serial_shutdown(remote_serial_t *m_serial);
remote_serial_t *remote_serial_init(int m_which, int m_port, int has_warned);

#endif /* INCLUDE_REMOTE_SERIAL_H_ */
