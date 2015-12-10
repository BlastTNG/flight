/**
 * @file comms_netsock.c
 *
 * @date Jan 16, 2011
 * @author seth
 *
 * @brief This file is part of FCP, created for the EBEX project
 *
 * This software is copyright (C) 2010 Columbia University
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

#include <errno.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include <blast.h>
#include <comms_netsock.h>

static int comms_sock_raw_read(comms_socket_t *m_sock, void *m_buf, size_t m_len);
static int comms_sock_raw_write(comms_socket_t *m_sock, const void *m_buf, size_t m_len);
static int comms_sock_get_addrinfo(const char *m_hostname, uint32_t m_port, bool m_udp, struct addrinfo **m_ai);
static socket_t comms_sock_bind(const char *m_hostname, uint32_t m_port, bool m_tcp);
static socket_t comms_sock_connect_fd(const char *m_hostname, uint32_t m_port, bool m_udp);
static int comms_sock_poll(comms_net_async_handle_t *m_handle, socket_t m_fd, uint16_t m_events, void *m_sock);

/**
 * Retrieves the address info structure for the given hostname:port combination.  This assumes a TCP/IP
 * connection
 * @param m_hostname Destination hostname in either DNS or xx.xx.xx.xx format
 * @param m_port Port number for the service
 * @param m_udp True if the port is a UDP port
 * @param m_ai Double pointer that will be allocated by by getaddrinfo()
 * @return return code from getaddrinfo
 */
static int comms_sock_get_addrinfo(const char *m_hostname, uint32_t m_port, bool m_udp, struct addrinfo **m_ai)
{
    char *service = NULL;
    struct addrinfo request = { 0 };
    int retval = 0;

    log_enter();

    if (!m_udp) {
        request.ai_protocol = IPPROTO_TCP;
        request.ai_socktype = SOCK_STREAM;
    } else {
        request.ai_protocol = IPPROTO_UDP;
        request.ai_socktype = SOCK_DGRAM;
    }

    request.ai_family = PF_UNSPEC;
    request.ai_flags = AI_PASSIVE;
    if (m_port) {
        blast_tmp_sprintf(service, "%hu", m_port);
        request.ai_flags = AI_NUMERICSERV;
    }
    retval = getaddrinfo(m_hostname, service, &request, m_ai);
    log_leave();

    return retval;
}

/**
 * Connects an allocated socket to a given remote host:port combination
 * @param m_hostname
 * @param m_port
 * @return
 */
static socket_t comms_sock_connect_fd(const char *m_hostname, uint32_t m_port, bool m_udp)
{
    socket_t sock = -1;
    int retval;
    struct addrinfo *addrinfo_group;
    struct addrinfo *ai;

    log_enter();

    retval = comms_sock_get_addrinfo(m_hostname, m_port, m_udp, &addrinfo_group);
    if (retval != 0) {
        blast_err("Failed to resolve hostname %s (%s)", m_hostname, gai_strerror(retval));
        log_leave();
        return NETSOCK_ERR;
    }

    for (ai = addrinfo_group; ai != NULL; ai = ai->ai_next) {
        /* create socket */
        sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock < 0) {
            blast_err("Socket create failed: %s", strerror(errno));
            continue;
        }

        fcntl(sock, F_SETFL, O_NONBLOCK);

        if (!m_udp) connect(sock, ai->ai_addr, ai->ai_addrlen);
        break;
    }

    freeaddrinfo(addrinfo_group);
    log_leave();

    return sock;
}

/**
 * Binds a socket structure to a specific host:port combination.
 * @param m_hostname
 * @param m_port Port number to connect
 * @param m_udp If false, build a stream-based tcp connection
 * @return
 */
static socket_t comms_sock_bind(const char *m_hostname, uint32_t m_port, bool m_udp)
{
    socket_t sock = -1;
    int retval;
    struct addrinfo *addrinfo_group;
    struct addrinfo *ai;
    int sock_opt = 1;

    log_enter();

    retval = comms_sock_get_addrinfo(m_hostname, m_port, m_udp, &addrinfo_group);
    if (retval != 0) {
        blast_err("Failed to resolve hostname %s (%s)", m_hostname, gai_strerror(retval));
        log_leave();
        return NETSOCK_ERR;
    }

    for (ai = addrinfo_group; ai != NULL; ai = ai->ai_next) {
        /* create socket */
        sock = socket(ai->ai_family, ai->ai_socktype, ai->ai_protocol);
        if (sock < 0) {
            blast_err("Socket create failed: %s", strerror(errno));
            continue;
        }

        if (setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &sock_opt, sizeof(sock_opt)) < 0) {
            freeaddrinfo(addrinfo_group);
            close(sock);
            log_leave();
            blast_err("Couldn't reuse socket for port %d", m_port);
            return NETSOCK_ERR;
        }

        if (bind(sock, ai->ai_addr, ai->ai_addrlen)) {
            freeaddrinfo(addrinfo_group);
            close(sock);
            log_leave();
            blast_err("Couldn't bind to socket for port %d", m_port);
            return NETSOCK_ERR;
        }

        break;
    }

    freeaddrinfo(addrinfo_group);
    log_leave();

    return sock;
}

/**
 * Joins an comms_socket to a multicast group.  Typical groups will be specified in in.h as
 * INADDR_UNSPEC_GROUP, INADDR_ALLHOSTS_GROUP, INADDR_ALLRTRS_GROUP or INADDR_MAX_LOCAL_GROUP,
 * corresponding to 224.0.0.0, .1, .2, .255.
 * @param m_socket A bound comms_socket
 * @param m_group
 * @return
 */
int comms_sock_join_mcast(comms_socket_t *m_socket, in_addr_t m_group)
{
    struct ip_mreq multicast_req = { { 0 }, { 0 } };

    multicast_req.imr_multiaddr.s_addr = m_group;		/// Group membership
    multicast_req.imr_interface.s_addr = INADDR_ANY;	/// Local interface address (any)

    if (setsockopt(m_socket->fd, IPPROTO_IP, IP_ADD_MEMBERSHIP, &multicast_req, sizeof(multicast_req))) {
        blast_strerror("Cannot join multicast group!");
        return NETSOCK_ERR;
    }

    return NETSOCK_OK;
}

int comms_sock_set_buffer(comms_socket_t *m_socket, int m_size)
{
    if (setsockopt(m_socket->fd, SOL_SOCKET, SO_RCVBUF, &m_size, sizeof(int))) {
        blast_strerror("Cannot set socket receive buffer!");
        return NETSOCK_ERR;
    }

    return NETSOCK_OK;
}

static int comms_sock_listen_generic(comms_socket_t *m_socket, const char *m_hostname, uint32_t m_port, bool m_udp)
{
    socket_t fd;

    BLAST_SAFE_FREE(m_socket->host);
    if (!m_hostname) {
        m_socket->host = strdup("0.0.0.0");
    } else {
        m_socket->host = strdup(m_hostname);
    }

    m_socket->port = m_port;

    if ((fd = comms_sock_bind(m_socket->host, m_port, m_udp)) == NETSOCK_ERR) {
        BLAST_SAFE_FREE(m_socket->host);
        return NETSOCK_ERR;
    }

    comms_sock_set_fd(m_socket, fd);

    if (listen(fd, 16)) {
        close(fd);
        BLAST_SAFE_FREE(m_socket->host);
        return NETSOCK_ERR;
    }

    return NETSOCK_OK;
}
/**
 * Starts listening on a given port.
 * @param m_socket Pointer to an allocated socket
 * @param m_hostname Hostname on which to listen for connections (or NULL for default)
 * @param m_port Numerical port to listen on.
 * @return NETSOCK_ERR on failure, NETSOCK_OK on success
 */
int comms_sock_listen(comms_socket_t *m_socket, const char *m_hostname, uint32_t m_port)
{
    return comms_sock_listen_generic(m_socket, m_hostname, m_port, false);
}

/**
 * Starts listening on a given port for UDP
 * @param m_socket Pointer to an allocated socket
 * @param m_hostname Hostname on which to listen for connections (or NULL for ALL)
 * @param m_port Numerical port to listen on.
 * @return NETSOCK_ERR on failure, NETSOCK_OK on success
 */
int comms_sock_listen_udp(comms_socket_t *m_socket, const char *m_hostname, uint32_t m_port)
{
    return comms_sock_listen_generic(m_socket, m_hostname, m_port, true);
}

/**
 * Accepts the next connection available on a listening socket.  This call will block until the connection
 * is available.  This is a cancellation point for pthreads.
 * @param m_socket
 * @return
 */
socket_t comms_sock_accept(comms_socket_t *m_socket)
{
    socket_t fd = 0;
    struct sockaddr_in sa;
    socklen_t sl = sizeof(sa);

    if ((fd = accept(m_socket->fd, &sa, &sl)) < 0) {
        m_socket->last_errno = errno;
        switch (m_socket->last_errno) {
            case EAGAIN:
                return NETSOCK_AGAIN;
            case EMFILE:
            case ENFILE:
                blast_err("WARNING!  File number limitations have been reached! ");
                return NETSOCK_ERR;
            default:
                blast_strerror("Could not accept new connection");
                return NETSOCK_ERR;
        }
    }

    fcntl(fd, F_SETFL, O_NONBLOCK);

    if (m_socket->host) free(m_socket->host);
    memcpy(&m_socket->addr, &sa, sl);
    m_socket->host = strdup(inet_ntoa(sa.sin_addr));

    blast_info("Received connection from %s", m_socket->host);
    return fd;
}
/**
 * Allocates space for a new socket structure
 * @return Pointer to the allocated structure or NULL on failure
 */
comms_socket_t *comms_sock_new(void)
{
    comms_socket_t *sock;

    sock = (comms_socket_t *) calloc(1, sizeof(struct comms_socket));
    if (!sock) return NULL;

    log_enter();

    sock->can_read = false;
    sock->can_write = false;
    sock->have_exception = false;
    sock->poll_handle = NULL;
    sock->state = NETSOCK_STATE_NONE;
    sock->fd = INVALID_SOCK;
    sock->last_errno = -1;
    sock->is_socket = true;
    sock->in_buffer = netbuf_new(1, 4 * 1024 * 1024);
    if (!sock->in_buffer) {
        BLAST_SAFE_FREE(sock);
        blast_err("Could not allocate in_buffer");
        return NULL;
    }
    sock->out_buffer = netbuf_new(1, 1024 * 1024);
    if (!sock->out_buffer) {
        netbuf_free(sock->in_buffer);
        BLAST_SAFE_FREE(sock);
        log_leave("Could not allocate out_buffer");
        return NULL;
    }

    log_leave();
    return sock;
}

/**
 * Takes a double-pointer to a socket and ensures that it is in a pristine state, ready for a new connection.
 * @param m_sock Double pointer to a socket
 */
void comms_sock_reset(comms_socket_t **m_sock)
{
    log_enter();
    if (!(*m_sock)) {
        *m_sock = comms_sock_new();
        return;
    }
    comms_sock_close(*m_sock);

    (*m_sock)->fd = INVALID_SOCK;
    (*m_sock)->last_errno = -1;
    (*m_sock)->is_socket = true;
    netbuf_reinit((*m_sock)->in_buffer);
    netbuf_reinit((*m_sock)->out_buffer);
    (*m_sock)->can_read = false;
    (*m_sock)->can_write = false;
    (*m_sock)->have_exception = false;
    BLAST_ZERO((*m_sock)->callbacks);
    (*m_sock)->poll_handle = comms_sock_get_poll_handle(*m_sock);
    (*m_sock)->state = NETSOCK_STATE_NONE;
    (*m_sock)->flags = 0;
    log_leave();
}

void comms_sock_set_flags(comms_socket_t *m_sock, int m_flags)
{
    m_sock->flags = m_flags;
}

static inline bool comms_sock_is_open(comms_socket_t *s)
{
    return (s->fd != INVALID_SOCK);
}

/**
 * Main routine providing callback handling for asynchronous socket connections.
 * @param m_handle Pointer to the asynchronous handler
 * @param m_fd File descriptor for the network socket
 * @param m_events Received events flags
 * @param m_sock Pointer to the comms_socket structure
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
static int comms_sock_poll(comms_net_async_handle_t *m_handle, socket_t m_fd, uint16_t m_events, void *m_sock)
{
    comms_socket_t *sock = (comms_socket_t*) m_sock;
    char buffer[COMMS_NETSOCK_BUF_SIZE];
    int retval = 0;
    socklen_t errlen = sizeof(sock->last_errno);

    log_enter();
    if (!comms_sock_is_open(sock)) {
        log_leave();
        return NETSOCK_ERR;
    }

    if (m_events & POLLERR) {
        /**
         * If we catch an error while connecting, we use getsockopt to try and determine the cause of the error.
         * The connected callback in then fired with NETSOCK_ERR
         */
        if (sock->state == NETSOCK_STATE_CONNECTING) {
            sock->state = NETSOCK_STATE_ERROR;
            blast_err("Got error while connecting to %s", sock->host);
            if (sock->is_socket) getsockopt(m_fd, SOL_SOCKET, SO_ERROR, (void *) &sock->last_errno, &errlen);

            comms_sock_close(sock);
            if (sock->callbacks.connected) {
                sock->callbacks.connected(NETSOCK_ERR, sock->last_errno, sock->callbacks.priv);
            }log_leave();
            return NETSOCK_ERR;
        }
        /**
         * If we catch an error and we are not trying to connect, then we fall through to the POLLIN handler to
         * process the error
         */

        comms_net_async_del_events(m_handle, POLLERR);
        m_events |= POLLIN;
    }
    if (m_events & (POLLIN | POLLPRI)) {
        sock->can_read = true;
        retval = comms_sock_raw_read(sock, buffer, sizeof(buffer));
        /**
         * Caught an error.  Likely broken pipe.  Errno is stored in the sock struct.
         */
        if (retval < 0) {
            retval = -1;
            if (m_handle) {
                comms_net_async_del_events(m_handle, POLLIN | POLLERR);
            }
            if (sock->callbacks.error) {
                sock->callbacks.error(sock->last_errno, sock->callbacks.priv);
            }
        }

        /**
         * No more data are available.  This is the EOF condition
         */
        if (retval == 0) {
            comms_net_async_del_events(m_handle, POLLIN);
            m_events |= POLLHUP;
        }

        /**
         * Got actual data.  Buffer it and pass on to the user callback function
         */
        if (retval > 0) {
            size_t written = netbuf_write(sock->in_buffer, buffer, retval);
            size_t to_write = retval - written;
            if (sock->callbacks.data) {
                void *data;
                size_t len = netbuf_peek(sock->in_buffer, &data);

                if (len) {
                    retval = sock->callbacks.data(data, len, sock->callbacks.priv);

                    if (retval > 0) netbuf_eat(sock->in_buffer, retval);

                    if (netbuf_bytes_available(sock->in_buffer)) comms_net_async_add_events(m_handle, POLLIN);
                    free(data);
                }
            }
            /// If we didn't get everything in, try one more time
            if (to_write) {
                if (netbuf_write(sock->in_buffer, buffer + written, to_write)) blast_warn(
                        "Discarding data due to overfull in_buffer on %s", sock->host ? sock->host : "(NULL)");
            }
        }
    }
    if (m_events & POLLHUP) {
        if (sock->callbacks.finished) {
            void *data;
            size_t len = netbuf_peek(sock->in_buffer, &data);

            if (len) {
                sock->callbacks.finished(data, len, sock->callbacks.priv);
                if (sock->in_buffer) netbuf_eat(sock->in_buffer, len);
                free(data);
            }
            sock->state = NETSOCK_STATE_CLOSED;
        }log_leave();
        return NETSOCK_OK;
    }
    if (m_events & POLLOUT) {
        if (sock->state == NETSOCK_STATE_CONNECTING) {
            sock->state = NETSOCK_STATE_CONNECTED;
            comms_net_async_set_events(m_handle, POLLPRI | POLLIN | POLLOUT);
            fcntl(sock->fd, F_SETFL, 0);

            if (sock->callbacks.connected) {
                sock->callbacks.connected(NETSOCK_OK, 0, sock->callbacks.priv);
            }

            log_leave();
            return NETSOCK_OK;
        }

        sock->can_write = true;
        comms_net_async_del_events(m_handle, POLLOUT);

        if (netbuf_bytes_available(sock->out_buffer)) {
            comms_sock_flush(sock);
        } else if (sock->callbacks.control) {
            sock->callbacks.control(NETSOCK_WRITABLE, sock->callbacks.priv);
        }
    }

    log_leave();
    return retval;
}

comms_net_async_handle_t *comms_sock_get_poll_handle(comms_socket_t *m_sock)
{
    log_enter();
    if (!m_sock->poll_handle) {
        m_sock->poll_handle = comms_net_async_handler_new(m_sock->fd, 0, comms_sock_poll, m_sock);
    }log_leave();
    return m_sock->poll_handle;
}

void comms_sock_free(comms_socket_t *m_sock)
{
    log_enter();
    if (!m_sock) {
        log_leave("No socket");
        return;
    }

    comms_sock_close(m_sock);
    netbuf_free(m_sock->in_buffer);
    netbuf_free(m_sock->out_buffer);
    BLAST_SAFE_FREE(m_sock->host);
    BLAST_SAFE_FREE(m_sock);
    log_leave("Freed");
}

void comms_sock_close(comms_socket_t *m_sock)
{
    log_enter();
    if (comms_sock_is_open(m_sock)) {
        if (close(m_sock->fd) == -1) m_sock->last_errno = errno;

        m_sock->fd = INVALID_SOCK;
        m_sock->state = NETSOCK_STATE_CLOSED;
    }
    if (m_sock->poll_handle) {
        comms_net_async_handler_free(m_sock->poll_handle);
        m_sock->poll_handle = NULL;
    }log_leave("Closed");
}

void comms_sock_set_fd(comms_socket_t *m_sock, socket_t m_fd)
{
    log_enter();
    m_sock->fd = m_fd;
    if (m_sock->poll_handle) {
        comms_net_async_set_sock(m_sock->poll_handle, m_fd);
    }

    log_leave();
}

static int comms_sock_raw_read(comms_socket_t *m_sock, void *m_buf, size_t m_len)
{
    int retval = NETSOCK_ERR;

    log_enter();
    if (m_sock->have_exception) {
        log_leave();
        return NETSOCK_ERR;
    }
    if (m_sock->is_socket) {
        m_sock->addrlen = sizeof(m_sock->addr);
        retval = recvfrom(m_sock->fd, m_buf, m_len, 0, &m_sock->addr, &m_sock->addrlen);
    } else {
        retval = read(m_sock->fd, m_buf, m_len);
    }

    m_sock->last_errno = errno;
    m_sock->can_read = false;

    if (retval < 0) {
        m_sock->have_exception = true;
    }

    log_leave();
    return retval;
}

static int comms_sock_raw_write(comms_socket_t *m_sock, const void *m_buf, size_t m_len)
{
    ssize_t retval = NETSOCK_ERR;

    log_enter();
    if (m_sock->have_exception) {
        log_leave();
        return NETSOCK_ERR;
    }

    if (m_sock->is_socket) {
        if (m_sock->udp) {
            retval = sendto(m_sock->fd, m_buf, m_len, m_sock->flags, &m_sock->addr, m_sock->addrlen);
        } else {
            /// Cap the maximum we send at any given point to the default segment size for TCP
            m_len = min(m_len, DEFAULT_MTU);
            retval = send(m_sock->fd, m_buf, m_len, m_sock->flags);
        }
    } else {
        retval = write(m_sock->fd, m_buf, m_len);
    }

    m_sock->last_errno = errno;

    if (m_sock->poll_handle) {
        comms_net_async_add_events(m_sock->poll_handle, POLLOUT);
    }
    if (retval < 0) {
        m_sock->have_exception = true;
    }

    log_leave();
    return retval;
}

int comms_sock_write(comms_socket_t *m_sock, const void *m_buf, size_t m_len)
{
    if (m_len) {
        if (!m_sock) return NETSOCK_ERR;

        if (m_sock->state != NETSOCK_STATE_CONNECTING && m_sock->state != NETSOCK_STATE_CONNECTED) {
            blast_err("Can not write to socket %s in state %d", m_sock->host ? m_sock->host : "(Unknown host)",
                      m_sock->state);
            return NETSOCK_ERR;
        }
        if (!netbuf_write(m_sock->out_buffer, m_buf, m_len)) {
            blast_err("Could not buffer data on %s", m_sock->host ? m_sock->host : "(NULL)");
            return NETSOCK_ERR;
        }
        if (m_sock->state == NETSOCK_STATE_CONNECTED) return comms_sock_flush(m_sock);
    }

    return NETSOCK_OK;
}

int comms_sock_flush(comms_socket_t *m_sock)
{
    uint32_t len;
    int retval;

    log_enter();

    if (!comms_sock_is_open(m_sock)) {
        blast_err("Could not write to closed socket %s", m_sock->host ? m_sock->host : "(Unknown host)");

        log_leave();
        return NETSOCK_ERR;
    }

    len = netbuf_bytes_available(m_sock->out_buffer);
    if (m_sock->poll_handle && len > 0) {
        if (!__sync_lock_test_and_set(&m_sock->can_write, false)) {
            comms_net_async_add_events(m_sock->poll_handle, POLLOUT);
            log_leave();
            return NETSOCK_AGAIN;
        }

        void *data;
        len = netbuf_peek(m_sock->out_buffer, &data);

        if (len) {
            retval = comms_sock_raw_write(m_sock, data, len);
            free(data);
            if (retval < 0) {
                comms_sock_close(m_sock);
                blast_err("Could not write socket: %s", strerror(m_sock->last_errno));
                log_leave();
                return NETSOCK_ERR;
            }
            netbuf_eat(m_sock->out_buffer, retval);
        }
    }

    /**
     * If we have data left to write to the socket, then we force the POLLOUT event to allow
     * our async handler to catch and process the data in the next poll call.
     */
    len = netbuf_bytes_available(m_sock->out_buffer);
    if (m_sock->poll_handle && len > 0) {
        comms_net_async_add_events(m_sock->poll_handle, POLLOUT);
        log_leave();
        return NETSOCK_AGAIN;
    }

    log_leave();
    return NETSOCK_OK;
}

static int comms_sock_connect_generic(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port, bool m_udp)
{
    socket_t fd;
    log_enter();

    if (!m_sock || m_sock->state != NETSOCK_STATE_NONE) return NETSOCK_ERR;
    fd = comms_sock_connect_fd(m_hostname, m_port, m_udp);

    if (fd == INVALID_SOCK) return NETSOCK_ERR;
    comms_sock_set_fd(m_sock, fd);

    BLAST_SAFE_FREE(m_sock->host);
    m_sock->host = strdup(m_hostname);
    m_sock->port = m_port;
    m_sock->udp = m_udp;

    m_sock->state = NETSOCK_STATE_CONNECTING;

    comms_net_async_set_events(comms_sock_get_poll_handle(m_sock), POLLOUT);
    log_leave();
    return NETSOCK_OK;
}

int comms_sock_reconnect(comms_socket_t *m_sock)
{
    socket_t fd;

    if (!m_sock) return NETSOCK_ERR;
    if (m_sock->state != NETSOCK_STATE_CLOSED && m_sock->state != NETSOCK_STATE_ERROR) {
        blast_err("Could not reconnect socket (%s) in state %d", m_sock->host ? m_sock->host : "unknown host",
                  m_sock->state);
    }
    if (!m_sock->host) {
        blast_err("Could not reconnect socket without host");
        return NETSOCK_ERR;
    }

    fd = comms_sock_connect_fd(m_sock->host, m_sock->port, m_sock->udp);

    if (fd == INVALID_SOCK) return NETSOCK_ERR;
    comms_sock_set_fd(m_sock, fd);

    m_sock->state = NETSOCK_STATE_CONNECTING;

    comms_net_async_set_events(comms_sock_get_poll_handle(m_sock), POLLOUT);
    log_leave();
    return NETSOCK_OK;
}

int comms_sock_connect(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port)
{
    return comms_sock_connect_generic(m_sock, m_hostname, m_port, false);
}

int comms_sock_connect_udp(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port)
{
    return comms_sock_connect_generic(m_sock, m_hostname, m_port, true);
}

/**
 * Sets up a multicast listening port.
 * @param m_sock Allocated socket
 * @param m_hostname Should be "224.0.0.x" where x is between 0 and 255
 * @param m_port Multicast port number
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
int comms_sock_multicast_listen(comms_socket_t *m_sock, const char *m_hostname, uint32_t m_port)
{
    socket_t fd;
    in_addr_t group;
    int loop = 0;
    log_enter();

    if (!m_sock || m_sock->state != NETSOCK_STATE_NONE) return NETSOCK_ERR;
    fd = comms_sock_bind("0.0.0.0", m_port, true);
    if (fd == INVALID_SOCK) return NETSOCK_ERR;

    comms_sock_set_fd(m_sock, fd);
    if ((group = inet_addr(m_hostname)) == INADDR_NONE) return NETSOCK_ERR;

    if (comms_sock_join_mcast(m_sock, group) == NETSOCK_ERR) return NETSOCK_ERR;

    setsockopt(m_sock->fd, IPPROTO_IP, IP_MULTICAST_LOOP, &loop, sizeof(loop));

    BLAST_SAFE_FREE(m_sock->host);

    m_sock->addr.sin_family = AF_INET;
    m_sock->addr.sin_addr.s_addr = group;
    m_sock->addr.sin_port = htons(m_port);
    m_sock->addrlen = sizeof(m_sock->addr);

    m_sock->host = strdup(m_hostname);
    m_sock->port = m_port;
    m_sock->udp = 1;

    m_sock->state = NETSOCK_STATE_CONNECTING;

    comms_net_async_set_events(comms_sock_get_poll_handle(m_sock), POLLOUT);
    log_leave();
    return NETSOCK_OK;
}
