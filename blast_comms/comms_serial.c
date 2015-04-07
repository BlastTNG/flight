/**
 * @file comms_serial.c
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
#include <stdbool.h>
#include <termios.h>
#include <fcntl.h>

#include <blast.h>
#include <comms_common.h>
#include <comms_serial.h>
#include <comms_netsock.h>

/**
 * The allocate function for a serial line is the same as for a network socket.  The exception is that we set #is_socket
 * to false to ensure we use "write" instead of "send" and "read" instead of "recv"
 * @return New socket pointer on success, NULL on failure
 */
comms_serial_t *comms_serial_new(void)
{
	comms_serial_t *serial;

	serial = (comms_serial_t *)malloc(sizeof(comms_serial_t));
	if (!serial) return NULL;
	memset(serial, 0, sizeof(comms_serial_t));

	serial->sock = comms_sock_new();
	if (!serial->sock)
	{
		log_leave("Could not register serial socket");
		return NULL;
	}

	serial->sock->is_socket = false;
	serial->sock->priv_data = serial;

	/**
	 * Sets the terminal to RAW mode (no echo, no special processing, character at a time IO
	 */

	serial->term.c_cflag = CS8 | B1200 | CLOCAL | CREAD;

	log_leave();
	return serial;
}

/**
 * Takes a double-pointer to a serial fd and ensures that it is in a pristine state, ready for a new connection.
 * @param m_sock Double pointer to a serial fd
 */
void comms_serial_reset(comms_serial_t **m_serial)
{
	log_enter();
	if (!(*m_serial))
	{
		*m_serial = comms_serial_new();
	}
	else
	{
		if (!pthread_mutex_lock(&(*m_serial)->mutex))
		{
			comms_sock_reset(&(*m_serial)->sock);
			(*m_serial)->sock->is_socket = false;
			pthread_mutex_unlock(&(*m_serial)->mutex);
		}
	}
	log_leave();
}

void comms_serial_close(comms_serial_t *m_serial)
{
	if (m_serial && m_serial->sock)
	{
		if (!pthread_mutex_lock(&m_serial->mutex))
		{
			comms_sock_close(m_serial->sock);
			pthread_mutex_unlock(&m_serial->mutex);
		}
	}

}

void comms_serial_free(void *m_serial)
{
    /// This will close and free the socket and its buffers
    if(m_serial)
    {
        comms_serial_close(m_serial);
        free(m_serial);
    }
}

bool comms_serial_setspeed(comms_serial_t *m_serial, speed_t m_speed)
{
	if (cfsetospeed(&m_serial->term, m_speed) || cfsetispeed(&m_serial->term, m_speed))
	{
		blast_err("Could not set speed to %d", (int)m_speed);
		return false;
	}

	if (m_serial->sock->fd != INVALID_SOCK)
	{
		tcsetattr(m_serial->sock->fd, TCSANOW, &m_serial->term);
	}
	return true;
}

/**
 * Buffer data for asynchronous writing to the serial device
 * @param m_serial Pointer to the serial structure
 * @param m_buf Pointer to the data stream from which to write
 * @param m_len Length in bytes to write
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
int comms_serial_write(comms_serial_t *m_serial, const void *m_buf, size_t m_len)
{
	int retval;

	if (m_serial && m_serial->sock)
	{
		if (!pthread_mutex_lock(&m_serial->mutex))
		{
			retval = comms_sock_write(m_serial->sock, m_buf, m_len);
			pthread_mutex_unlock(&m_serial->mutex);
			if (retval != NETSOCK_ERR) return NETSOCK_OK;
		}
		else
		{
			blast_err("Could not lock mutex!");
			return NETSOCK_ERR;
		}
	}
	else
	{
		blast_err("Attempted to write to NULL Socket for serial port");
	}
	return NETSOCK_ERR;
}

/**
 * Open a serial port device for bi-directional, asynchronous communication
 * @param m_serial Pointer to the serial structure
 * @param m_terminal Name of the tty serial device
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
int comms_serial_connect(comms_serial_t *m_serial, const char *m_terminal)
{
	log_enter();

	if (!m_terminal)
	{
		blast_err("Passed NULL pointer to terminal name");
		log_leave("NULL");
		return NETSOCK_ERR;
	}

	if (m_serial->sock->fd != INVALID_SOCK) comms_serial_close(m_serial);

	if ((m_serial->sock->fd = open (m_terminal, O_NOCTTY | O_RDWR | O_NONBLOCK)) == INVALID_SOCK)
	{
		blast_err("Could not open terminal '%s'", m_terminal);
		blast_strerror("\t");
		log_leave("Open Error");
		return NETSOCK_ERR;
	}

	if (!m_serial->sock->host)
	{
		blast_info("Successfully opened %s for serial communication", m_terminal);
		m_serial->sock->host = bstrdup(err, m_terminal);
	}
	else if (strcmp(m_serial->sock->host, m_terminal))
	{
		blast_info("Successfully opened %s for serial communication", m_terminal);
		bfree(err,m_serial->sock->host);
		m_serial->sock->host = bstrdup(err, m_terminal);
	}

	m_serial->sock->state = NETSOCK_STATE_CONNECTING;

	tcsetattr(m_serial->sock->fd, TCSANOW, &m_serial->term);

	comms_net_async_set_events(comms_sock_get_poll_handle(m_serial->sock), POLLOUT);

	m_serial->sock->can_write = true;

	log_leave();
	return NETSOCK_OK;
}

/**
 * Connects the #comms_serial_t structure to a known FIFO file.
 * @param m_serial Pointer to the serial structure
 * @param m_fifo Name of the FIFO to open
 * @param m_flags flags that will be passed to open(2) (e.g. O_RDONLY, O_WRONLY).  O_NONBLOCK is assumed.
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
int comms_fifo_connect(comms_serial_t *m_serial, const char *m_fifo, int m_flags)
{
	log_enter();

	if (!m_fifo)
	{
		blast_err("Passed NULL pointer to FIFO name");
		log_leave("NULL");
		return NETSOCK_ERR;
	}
	if (!m_serial)
	{
		blast_err("Passed NULL pointer to serial device");
		log_leave("NULL");
		return NETSOCK_ERR;
	}

	if (m_serial->sock->fd != INVALID_SOCK) comms_serial_close(m_serial);

	if ((m_serial->sock->fd = open (m_fifo, m_flags|O_NONBLOCK)) == INVALID_SOCK)
	{
		blast_err("Could not open FIFO '%s'", m_fifo);
		blast_strerror("\t");
		log_leave("Open Error");
		return NETSOCK_ERR;
	}

	m_serial->sock->host = bstrdup(err, m_fifo);
	m_serial->sock->state = NETSOCK_STATE_CONNECTING;

	comms_net_async_set_events(comms_sock_get_poll_handle(m_serial->sock), POLLOUT);

	blast_info("Successfully opened %s for FIFO communication", m_fifo);

	log_leave();
	return NETSOCK_OK;
}
