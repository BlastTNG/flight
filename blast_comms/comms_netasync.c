/**
 * @file comms_netasync.c
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

#include <errno.h>
#include <poll.h>
#include <stdint.h>
#include <pthread.h>
#include <string.h>

#include <blast.h>
#include <comms_common.h>
#include <comms_netasync.h>
#include <comms_netsock.h>

/**
 * Creates a new asynchronous handler for a given socket that will respond to specific events.
 * @param m_fd Number of a previously allocated socket
 * @param m_events Bitfield of poll events that this handler can process
 * @param m_process Pointer to the callback function that will process the event
 * @param m_priv Pointer to unspecified private data that will also be provided to the callback function
 * @return Pointer to the new asynchronous handler or NULL on failure
 */
comms_net_async_handle_t *comms_net_async_handler_new(socket_t m_fd, uint16_t m_events, comms_net_async_callback_t m_process,
		void *m_priv)
{
	comms_net_async_handle_t *handle = NULL;
	log_enter();

	handle = balloc(err, sizeof(struct comms_net_async_handle));
	if (handle)
	{
		BLAST_ZERO_P(handle);

		handle->fd = m_fd;
		handle->events = m_events;
		handle->process = m_process;
		handle->priv = m_priv;
	}

	log_leave();
	return handle;
}

/**
 * Frees the memory associated with the asynchronous handler after disconnecting its context
 * @param m_handle Pointer to the async handler
 */
void comms_net_async_handler_free(comms_net_async_handle_t *m_handle)
{
	log_enter();
	if (m_handle && m_handle->ctx)
	{
		comms_net_async_handler_disconnect_ctx(m_handle);
		m_handle->ctx = NULL;
	}

	BLAST_SAFE_FREE(m_handle);
	log_leave("Freed");
}

void comms_net_async_set_sock(comms_net_async_handle_t *m_handle, socket_t m_fd)
{
	log_enter();
	if (m_handle->ctx != NULL)
	{
		m_handle->ctx->pollfds[m_handle->index].fd = m_fd;
	}
	else
	{
		m_handle->fd = m_fd;
	}
	log_leave();
}

void comms_net_async_set_events(comms_net_async_handle_t *m_handle, uint16_t m_events)
{
	log_enter();
	m_handle->events = m_events;
	if (m_handle->ctx != NULL)
	{
		m_handle->ctx->pollfds[m_handle->index].events = m_events;
	}
	log_leave();
}

void comms_net_async_add_events(comms_net_async_handle_t *m_handle, uint16_t m_events)
{
	log_enter();
	if (m_handle->ctx)
	{
		__sync_or_and_fetch(&(m_handle->ctx->pollfds[m_handle->index].events), m_events);
	}
	log_leave();
}

void comms_net_async_del_events(comms_net_async_handle_t *m_handle, uint16_t m_events)
{
	log_enter();
	if (m_handle->ctx)
	{
		__sync_and_and_fetch(&(m_handle->ctx->pollfds[m_handle->index].events), ~m_events);
	}
	log_leave();
}


socket_t comms_net_async_sock(comms_net_async_handle_t *m_handle)
{
	log_enter();
	if (m_handle->ctx != NULL)
	{
		return m_handle->ctx->pollfds[m_handle->index].fd;
	}

	log_leave();
	return m_handle->fd;
}

void comms_net_async_set_callback(comms_net_async_handle_t *m_handle, comms_net_async_callback_t m_callback, void *m_priv)
{
	log_enter();
	if (m_callback)
	{
		m_handle->process = m_callback;
		m_handle->priv = m_priv;
	}
	log_leave();
}

comms_net_async_ctx_t *comms_net_async_ctx_new(size_t m_step_size)
{
	comms_net_async_ctx_t *ctx;
	log_enter();

	ctx = (comms_net_async_ctx_t*) balloc(err,sizeof(comms_net_async_ctx_t));
	if (ctx)
	{
		BLAST_ZERO_P(ctx);
		ctx->step_size = m_step_size ? m_step_size : BLAST_NET_DEFAULT_CTX_STEP;
		pthread_mutex_init(&ctx->mutex, NULL);
	}

	log_leave();
	return ctx;
}

/**
 * Frees all memory associated with the context, including the context itself.  The calling function should
 * set #m_ctx to NULL following this function call.
 * @param m_ctx Pointer to the context to free
 */
void comms_net_async_ctx_free(comms_net_async_ctx_t *m_ctx)
{
	comms_net_async_handle_t *handle = NULL;

	if (!m_ctx) return;

	log_enter();
	if (m_ctx->num_handlers)
	{
		while(m_ctx->num_fds)
		{
			handle = m_ctx->async_handlers[0];

			if (handle->process && handle->process(handle, m_ctx->pollfds[0].fd, POLLERR, handle->priv) < 0)
			{
				blast_err("POLLERR handler failed while freeing asynchronous context");
			}
			comms_net_async_handler_disconnect_ctx(handle);
		}

		BLAST_SAFE_FREE(m_ctx->async_handlers);
		BLAST_SAFE_FREE(m_ctx->pollfds);
	}

	BLAST_SAFE_FREE(m_ctx);
	log_leave("Success");
}

/**
 * Resizes the asynchronous handler array.  This can resize up or down in value.
 * @param m_ctx Pointer to the polling context
 * @param m_size Number of asynchronous handlers (and sockets) to allocate in the context
 * @return NETSOCK_OK on success, NETSOCK_ERR on failure
 */
static int comms_net_async_ctx_resize(comms_net_async_ctx_t *m_ctx, size_t m_size)
{
	comms_net_async_handle_t **handle = NULL;
	struct pollfd *poll_fds = NULL;

	log_enter();
	if (!pthread_mutex_lock(&m_ctx->mutex))
	{
		if (!m_size)
		{
			blast_err("Tried to resize async context to zero!  You really want comms_net_async_ctx_free()");
			log_leave("Invalid m_size");
			return NETSOCK_ERR;
		}
		else
		{
			if(!(handle = (comms_net_async_handle_t**)
					reballoc(err, m_ctx->async_handlers, sizeof(comms_net_async_handle_t*) * m_size)))
			{
				pthread_mutex_unlock(&m_ctx->mutex);
				log_leave("Could not allocate space for asynchronous handler array");
				return NETSOCK_ERR;
			}

			blast_dbg("Allocated space for %d handles at %p", (int)m_size, (void*)handle);

			if (!(poll_fds = (struct pollfd *)reballoc(err, m_ctx->pollfds, sizeof(struct pollfd) * m_size)))
			{
				m_ctx->async_handlers = (comms_net_async_handle_t**)
						reballoc(err, m_ctx->async_handlers, sizeof(comms_net_async_handle_t*) * m_ctx->num_handlers);

				pthread_mutex_unlock(&m_ctx->mutex);
				log_leave("Could not re-allocate space for polling socket array");
				return NETSOCK_ERR;
			}
			blast_dbg("Allocated space for %d poll file descriptors at %p", (int)m_size, (void*)poll_fds);
		}

		m_ctx->async_handlers = handle;
		m_ctx->pollfds = poll_fds;
		m_ctx->num_handlers = m_size;

		pthread_mutex_unlock(&m_ctx->mutex);
	}

	log_leave("Success");
	return NETSOCK_OK;
}

/**
 * Adds an allocated asynchronous handler to a known context
 * @param m_ctx Pointer to a previously allocated context
 * @param m_async Pointer to the asynchronous handler to add to #m_ctx
 * @return NETSOCK_ERR on failure NETSOCK_OK on success
 */
static int comms_net_async_ctx_add(comms_net_async_ctx_t *m_ctx, comms_net_async_handle_t *m_handle)
{
	socket_t fd;

	log_enter();
	if (!m_ctx || !m_handle)
	{
		log_leave("Passed NULL pointer");
		return NETSOCK_ERR;
	}

	if (m_handle->ctx)
	{
		log_leave("Asynchronous handler context already allocated");
		return NETSOCK_ERR;
	}

	if (m_ctx->num_fds == m_ctx->num_handlers &&
			comms_net_async_ctx_resize(m_ctx, m_ctx->num_handlers + m_ctx->step_size) < 0)
	{
		log_leave("Could not resize context handler array");
		return NETSOCK_ERR;
	}

	fd = m_handle->fd;
	m_handle->index = m_ctx->num_fds++;

	m_ctx->async_handlers[m_handle->index] = m_handle;
	m_ctx->pollfds[m_handle->index].fd = fd;
	m_ctx->pollfds[m_handle->index].events = m_handle->events;
	m_ctx->pollfds[m_handle->index].revents = 0;
	m_handle->ctx = m_ctx;

	log_leave("Success.  %p ctx has %d fds", m_ctx, m_ctx->num_fds);
	return NETSOCK_OK;
}

/**
 * Convenience function to add an allocated comms_socket to an allocated polling context without
 * first going through a handle
 * @param m_ctx Pointer to the destination polling context
 * @param m_sock Pointer to the allocated #comms_socket structure
 * @return NETSOCK_OK on success.  NETSOCK_ERR on failure
 */
int comms_net_async_ctx_add_sock(comms_net_async_ctx_t *m_ctx, struct comms_socket *m_sock)
{
	comms_net_async_handle_t *poll_handle = NULL;
	int retval = NETSOCK_ERR;

	log_enter();
	if (!m_ctx || !m_sock)
	{
		log_leave("Received NULL pointer");
		return NETSOCK_ERR;
	}
	poll_handle = comms_sock_get_poll_handle(m_sock);

	if (!poll_handle)
	{
		log_leave("Could not get socket polling handle");
		return NETSOCK_ERR;
	}

	retval = comms_net_async_ctx_add(m_ctx, poll_handle);
	log_leave("Returning %d", (int)retval);
	return retval;
}

/**
 * Removes an asynchronous handler from the polling context array.  The handler's socket value is
 * properly reset.  Both the handler and polling context remain valid at the end of this function.
 * @param m_handle Pointer to the asynchronous handler that should be removed from the context.
 */
void comms_net_async_handler_disconnect_ctx(comms_net_async_handle_t *m_handle)
{
	size_t fd_index;
	comms_net_async_ctx_t *handler_context = NULL;

	log_enter();
	if (!m_handle)
	{
		log_leave("NULL Pointer");
		return;
	}

    handler_context = m_handle->ctx;
    if (!handler_context)
    {
        log_leave("Handle not currently connected to context.");
        return;
    }


    if (!pthread_mutex_lock(&handler_context->mutex))
    {

        if (handler_context->num_handlers <= m_handle->index || handler_context->async_handlers[m_handle->index] != m_handle)
        {
            /**
             * In the case that a user passes
             */
            blast_err("Mis-formed context/handle pairing.  Attempting to clean.  You may have leaked memory.");
            m_handle->ctx = NULL;
            for (fd_index = 0; fd_index < handler_context->num_handlers; fd_index++)
            {
                if (handler_context->async_handlers[fd_index] == m_handle)
                {
                    m_handle->index = fd_index;
                    m_handle->ctx = handler_context;
                }
            }

            if (!m_handle->ctx)
            {
                blast_err("No match found in context handler");
                pthread_mutex_unlock(&handler_context->mutex);
                log_leave();
                return;
            }
        }

        fd_index = m_handle->index;

        m_handle->fd = handler_context->pollfds[fd_index].fd;
        m_handle->ctx = NULL;
        handler_context->num_fds--;

        /**
         * If we have removed a socket from the middle of the pollfds array, take the last socket (and its handler) in the array and move
         * it into the now empty slot.
         */
        if (handler_context->num_fds > 0 && handler_context->num_fds != fd_index)
        {
            handler_context->pollfds[fd_index] = handler_context->pollfds[handler_context->num_fds];
            handler_context->async_handlers[fd_index] = handler_context->async_handlers[handler_context->num_fds];
            handler_context->async_handlers[fd_index]->index = fd_index;
        }

        log_leave("Success %p ctx now has %d fds", handler_context, handler_context->num_fds);
        pthread_mutex_unlock(&handler_context->mutex);
    }
}

/**
 * Provides an asynchronous wrapper to poll a group of associated sockets and individually
 * handle their returns.
 * @param m_ctx Pointer to a previously allocated asynchronous polling context
 * @param m_timeout Number of milliseconds after which to timeout the poll event
 * @return zero on success, -1 on error
 */
int comms_net_async_poll(comms_net_async_ctx_t *m_ctx, int m_timeout)
{
	int retval = 0;
	int event_count = 0;
	size_t i = 0;
	size_t num_fd = m_ctx->num_fds;
	short recv_events = 0;
	struct pollfd *pollfds;
	comms_net_async_handle_t **async_handlers;

	log_enter();
	if (!num_fd)
	{
		log_leave("No sockets to poll in ctx %p", m_ctx);
		return 0;
	}

	if (!pthread_mutex_lock(&m_ctx->mutex))
	{
		pollfds = alloca(sizeof(struct pollfd) * num_fd);
		async_handlers = alloca(sizeof(comms_net_async_handle_t*) * num_fd);
		memcpy(pollfds, m_ctx->pollfds, sizeof(struct pollfd) * num_fd);
		memcpy(async_handlers, m_ctx->async_handlers, sizeof(comms_net_async_handle_t*) * num_fd);
		pthread_mutex_unlock(&m_ctx->mutex);

		if ((retval = poll(pollfds, num_fd, m_timeout)) <= 0)
		{
			log_leave("No Events. Polling Retval=%d", retval);
			return (retval ? -1 : 0);
		}

		event_count = retval;

		while (event_count && i < num_fd)
		{
			pthread_testcancel();
			if (!pollfds[i].revents)
			{
				i++;
				continue;
			}
			event_count--;

			recv_events = pollfds[i].revents;
			pollfds[i].revents = 0;

			if ((recv_events & pollfds[i].events) &&
					async_handlers[i] && async_handlers[i]->process)
			{
				async_handlers[i]->process(	async_handlers[i],
												pollfds[i].fd,
												recv_events,
												async_handlers[i]->priv);
			}

			i++;

		}
	}

	log_leave("Processed %d events in ctx %p", (int) retval, m_ctx);
	return retval;
}
