/**
 * @file comms_netbuf.h
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


#ifndef COMMS_NET_BUFFER_H_
#define COMMS_NET_BUFFER_H_
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <pthread.h>

struct netbuf_t {
    size_t elem_size;  // The size of each element.
    size_t min_cap;    // The smallest capacity of the buffer
    size_t max_cap;    // The maximum capacity of the buffer

    char*  buffer;     // The internal buffer, holding the enqueued elements.
    char*  bufend;     // One past the end of the buffer, so that the actual
                       // elements are stored in in interval [buffer, bufend).
    char*  begin;      // Always points to the sentinel element. `begin + elem_size`
                       // points to the left-most element in the pipe.
    char*  end;        // Always points past the right-most element in the pipe.

    // The number of producers/consumers in the pipe.
    size_t producer_refcount, // Guarded by begin_lock.
           consumer_refcount; // Guarded by end_lock.

    pthread_mutex_t begin_lock;
    pthread_mutex_t end_lock;

    pthread_cond_t just_pushed; // Signaled immediately after a push.
    pthread_cond_t just_popped; // Signaled immediately after a pop.
};

typedef struct netbuf_t          netbuf_t;

// Returns the number of bytes currently in use in the buffer, excluding the
// sentinel element.
static inline size_t netbuf_bytes_available(netbuf_t *m_buffer)
{
    return ((m_buffer->begin >= m_buffer->end)
            ? ((m_buffer->end - m_buffer->buffer) + (m_buffer->bufend - m_buffer->begin))
            : (m_buffer->end - m_buffer->begin))
        - m_buffer->elem_size;
}

netbuf_t* __attribute__((__malloc__,warn_unused_result)) netbuf_new(size_t elem_size, size_t limit);
netbuf_t* __attribute__((nonnull,warn_unused_result)) netbuf_producer_new(netbuf_t*);
netbuf_t* __attribute__((nonnull,warn_unused_result)) netbuf_consumer_new(netbuf_t*);
void netbuf_free(netbuf_t*);
void netbuf_producer_free(netbuf_t*);
void netbuf_consumer_free(netbuf_t*);
void netbuf_reinit(netbuf_t *m_buf);

size_t __attribute__((nonnull,warn_unused_result)) netbuf_write(netbuf_t*, const void* elems, size_t count);
size_t __attribute__((nonnull,warn_unused_result)) netbuf_read(netbuf_t*, void* target, size_t count);
size_t __attribute__((nonnull,warn_unused_result)) netbuf_read_wait(netbuf_t*, void* target, size_t count);
size_t __attribute__((nonnull,warn_unused_result)) netbuf_peek(netbuf_t*, void**);
size_t __attribute__((nonnull,warn_unused_result)) netbuf_peek_noalloc(netbuf_t*, void*, size_t);
void __attribute__((nonnull)) netbuf_reserve(netbuf_t*, size_t count);
void __attribute__((nonnull)) netbuf_eat(netbuf_t *m_buf, size_t m_len);

#endif /* BLAST_NET_BUFFER_H_ */
