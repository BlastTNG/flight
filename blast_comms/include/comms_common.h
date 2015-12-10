/**
 * @file comms_common.h
 *
 * @date Jan 17, 2011
 * @author Seth Hillbrand
 * 
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010-2015 Seth Hillbrand
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


#ifndef INCLUDE_COMMS_COMMON_H
#define INCLUDE_COMMS_COMMON_H
#include <stdio.h>
#include <stdlib.h>

#include <blast.h>

#define DEFAULT_MTU 1432

typedef int socket_t;
#define INVALID_SOCK ((socket_t) -1)

/**
 * Socket status signals
 */
#define NETSOCK_NOT_WRITABLE 1
#define NETSOCK_WRITABLE 2

/**
 * Socket Return codes
 */
#define NETSOCK_OK                  0
#define NETSOCK_ERR                -1
#define NETSOCK_CONNECT_TIMEOUT    -2
#define NETSOCK_AGAIN              -3
#define NETSOCK_DATA_ERR           -4

typedef enum
{
    NETSOCK_STATE_NONE,
    NETSOCK_STATE_CONNECTING,
    NETSOCK_STATE_CONNECTED,
    NETSOCK_STATE_EOS,
    NETSOCK_STATE_CLOSED,
    NETSOCK_STATE_ERROR,
    NETSOCK_STATE_END,
} e_comms_socket_state;

typedef void (*netsock_int_callback_t)(int m_code, void *m_priv);
typedef int (*netsock_data_callback_t)(const void *m_data, size_t m_len, void *m_priv);
typedef void (*netsock_intint_callback_t)(int m_code, int m_errcode, void *m_priv);

typedef struct netsock_callbacks
{
    void *priv;
    netsock_data_callback_t data;
    netsock_int_callback_t control;
    netsock_int_callback_t error;
    netsock_data_callback_t finished;
    netsock_intint_callback_t connected;
} netsock_callbacks_t;

/// Clear definitions unless needed for call tracing
#define log_enter(...)
#define log_leave(...)

#endif /* COMMS_COMMON_H_ */
