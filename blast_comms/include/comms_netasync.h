/**
 * @file comms_netasync.h
 *
 * @date Jan 16, 2011
 * @author seth
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


#ifndef BLAST_NETASYNC_H_
#define BLAST_NETASYNC_H_

#include <poll.h>
#include <stdint.h>
#include <pthread.h>

#include <blast.h>
#include <comms_netasync.h>
#include <comms_common.h>

#define BLAST_NET_DEFAULT_CTX_STEP 4

struct comms_net_async_ctx;
struct comms_net_async_handle;
struct comms_socket;

/**
 * Callback fired when an event is caught on the socket
 * @param m_handle Async event handler
 * @param m_socket Which socket was caught
 * @param m_events bitfield returned by poll()
 * @param m_priv Arbitrary data pointer
 * @return
 */
typedef int (*comms_net_async_callback_t)(struct comms_net_async_handle *m_handle, socket_t m_socket, uint16_t m_events, void *m_priv);

typedef struct comms_net_async_handle
{
    struct comms_net_async_ctx *ctx;
    union
    {
        socket_t fd;
        size_t index;
    };
    uint16_t events;
    comms_net_async_callback_t process;
    void *priv;
} comms_net_async_handle_t;

typedef struct comms_net_async_ctx
{
    pthread_mutex_t mutex;
    comms_net_async_handle_t **async_handlers;
    size_t num_handlers;
    struct pollfd *pollfds;
    size_t num_fds;
    size_t step_size;
} comms_net_async_ctx_t;

comms_net_async_handle_t *comms_net_async_handler_new(socket_t fd, uint16_t events, comms_net_async_callback_t cb, void *userdata);
void comms_net_async_handler_free(comms_net_async_handle_t *p);
void comms_net_async_set_events(comms_net_async_handle_t *p, uint16_t events);
void comms_net_async_add_events(comms_net_async_handle_t *p, uint16_t events);
void comms_net_async_del_events(comms_net_async_handle_t *p, uint16_t events);
socket_t comms_net_async_sock(comms_net_async_handle_t *p);
void comms_net_async_set_sock(comms_net_async_handle_t *p, socket_t fd);
void comms_net_async_set_callback(comms_net_async_handle_t *p, comms_net_async_callback_t cb, void *userdata);
comms_net_async_ctx_t *comms_net_async_ctx_new(size_t chunk_size);
void comms_net_async_ctx_free(comms_net_async_ctx_t *ctx);
int comms_net_async_ctx_add_sock(comms_net_async_ctx_t *m_ctx, struct comms_socket *m_sock);
void comms_net_async_handler_disconnect_ctx(comms_net_async_handle_t *p);
int comms_net_async_poll(comms_net_async_ctx_t *ctx, int timeout);

#endif /* BLAST_NETASYNC_H_ */
