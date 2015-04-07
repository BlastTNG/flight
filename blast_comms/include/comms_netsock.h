/**
 * @file comms_netsock.h
 *
 * @date Jan 17, 2011
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


#ifndef BLAST_NETSOCK_H_
#define BLAST_NETSOCK_H_
#include <stdbool.h>
#include <netinet/in.h>
#include <unistd.h>

#include <comms_netbuf.h>
#include <comms_netasync.h>
#include <lookup.h>
#include <comms_common.h>

#define COMMS_NETSOCK_BUF_SIZE 8192 /**< COMMS_NETSOCK_BUF_SIZE 8192 is the maximum size of an atomic read/write */

typedef struct comms_socket
{
    comms_net_async_handle_t *poll_handle;
    netsock_callbacks_t *callbacks;
    socket_t fd;

    char *host;
    unsigned long timeout;
    unsigned int port;

    struct sockaddr_in addr;
    socklen_t addrlen;

    bool is_socket;
    bool can_read;
    bool can_write;

    bool have_exception;
    int last_errno;

    int flags;
    int udp;

    e_comms_socket_state state;
    comms_netbuf_t *out_buffer;
    comms_netbuf_t *in_buffer;

    void *priv_data;

} comms_socket_t;

comms_socket_t *comms_sock_new(void);
void comms_sock_reset(comms_socket_t **m_sock);
void comms_sock_free(comms_socket_t *m_sock);
void comms_sock_close(comms_socket_t *m_sock);
void comms_sock_set_fd(comms_socket_t *m_sock, socket_t m_fd);
int comms_sock_write(comms_socket_t *m_sock, const void *m_buf, size_t m_len);
int comms_sock_flush(comms_socket_t *m_sock);
int comms_sock_connect(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port);
int comms_sock_connect_udp(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port);
int comms_sock_listen(comms_socket_t *m_socket, const char *m_hostname, uint32_t m_port);
int comms_sock_listen_udp(comms_socket_t *m_socket, const char *m_hostname, uint32_t m_port);
socket_t comms_sock_accept(comms_socket_t *m_socket);
int comms_sock_multicast_listen(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port);
int comms_sock_set_buffer(comms_socket_t *m_socket, int m_size);
void comms_sock_set_flags(comms_socket_t *m_sock, int m_flags);
int comms_sock_join_mcast(comms_socket_t *m_socket, in_addr_t m_group);

comms_net_async_handle_t *comms_sock_get_poll_handle(comms_socket_t *m_sock);

#endif /* BLAST_NETSOCK_H_ */
