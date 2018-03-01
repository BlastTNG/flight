/**
 * file_buffer_tng.c
 *
 *  Created on: Mar 27, 2010, updated for BLAST-TNG on May 5, 2017
 *      Author: Seth Hillbrand,Laura Fissel
 *
 * This file is part of MCP, the EBEX flight control program
 *
 * This software is copyright (C) 2010-2017 Columbia University, University of Pennsylvania
 *
 * fcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <malloc.h>
#include <string.h>
#include <blast.h>
#include <errno.h>
#include <time.h>
#include "file_buffer_tng.h"

static void filebuffer_clear(filebuffer_t *m_buffer);
static ssize_t filebuffer_append_space(filebuffer_t *m_buffer, char **m_datap, size_t len);
static void filebuffer_defrag(filebuffer_t *m_buffer);
static int filebuffer_realloc(filebuffer_t *m_buffer, size_t len);
static void filebuffer_consume(filebuffer_t *m_buffer, size_t len);
static void filebuffer_cleanup(filebuffer_t *m_buffer);

/* What overage should we include in a re-allocation (bytes) */
static const unsigned int filebuffer_expansion_unit = 32768;

/* How much do we discount the recent size reqs in the average */
static const float filebuffer_historical_weight = 0.01;

extern int16_t SouthIAm;
uint32_t filebuffer_cached_bytes[2] = {0};

/**
 * Initializes a buffer structure and allocates the memory.
 *
 * @param m_buffer Pointer to the #filebuffer_t structure
 * @param m_len Initial buffer size in bytes
 *
 * @return -1 on error, 0 on success
 */
int
filebuffer_init(filebuffer_t *m_buffer, size_t m_len)
{
	m_buffer->alloc = m_len;
	m_buffer->buf = balloc(fatal, m_buffer->alloc);
	m_buffer->initial_size = m_len;
	m_buffer->moving_average_size = m_len;
	m_buffer->begin = 0;
	m_buffer->end = 0;

	if (m_buffer->buf == NULL) {
		blast_err("file_buffer::filebuffer_init : Could not allocate buffer of size %u", (unsigned)m_buffer->alloc);
		return -1;
	}
	memset(m_buffer->buf, 0, m_buffer->alloc);
	if(pthread_mutex_init(&(m_buffer->append_lock), NULL) == -1) {
		blast_err("file_buffer::filebuffer_init : Could not initialize append mutex");
		return -1;
	}
	if(pthread_mutex_init(&(m_buffer->consume_lock), NULL) == -1) {
		blast_err("file_buffer::filebuffer_init : Could not initialize consume mutex");
		pthread_mutex_destroy(&(m_buffer->append_lock));
		return -1;
	}

	return 0;
}

/**
 * Free memory associated with the filebuffer
 * @param m_buffer Buffer structure to free
 * @return nothing
 */
void
filebuffer_free(filebuffer_t *m_buffer)
{
	pthread_mutex_destroy(&(m_buffer->append_lock));
	pthread_mutex_destroy(&(m_buffer->consume_lock));
	filebuffer_clear(m_buffer);
	BLAST_SAFE_FREE(m_buffer->buf);
}

/**
 * Add data to the end of the buffer, expanding it if needed.
 *
 * N.B. filebuffer_append_space may lock the consume mutex if it needs to
 * expand the buffer.
 *
 * @param m_buffer Buffer structure
 * @param m_data Data to append
 * @param len Number of bytes to append from #m_data to #m_buffer->buf
 *
 * @return -1 on error, positive number of bytes buffered on success
 */
ssize_t
filebuffer_append(filebuffer_t *m_buffer, const char *m_data, size_t len)
{
	char 	*insertion_ptr = NULL;
	ssize_t retval = -1;

	pthread_mutex_lock(&(m_buffer->append_lock));
	retval = filebuffer_append_space(m_buffer, &insertion_ptr, len);

	if (retval != -1) {
		memcpy(insertion_ptr, m_data, len);
		filebuffer_cached_bytes[SouthIAm] += len;
	} else {
		blast_info("Failed to append data to buffer");
	}

	pthread_mutex_unlock(&(m_buffer->append_lock));
	return retval;
}

/**
 * Returns the number of bytes of data stored in the buffer
 * @param m_buffer Buffer structure
 * @return number of bytes of data
 */
size_t
filebuffer_len(filebuffer_t *m_buffer)
{
	if (m_buffer->end > m_buffer->begin) {
		return (size_t) (m_buffer->end - m_buffer->begin);
	}

	return 0;
}

/**
 * Returns the fraction of the #m_buffer->preferred_size that is currently in use
 * @param m_buffer Buffer structure
 * @return fraction of the buffer's preferred size currently used
 */
float
filebuffer_fraction_full(filebuffer_t *m_buffer)
{
	if (m_buffer->alloc == 0) {
		return 1.0F;
	} else {
		return ((float)filebuffer_len(m_buffer))/((float)m_buffer->alloc);
	}
}
/**
 * Writes the full contents of the buffer to the file stream and then flushes the stream.
 * If successful, the buffer contents are consumed.
 * This function will fail if either the write or flush command fails.
 *
 * @param m_buffer Buffer structure to write out
 * @param m_fp File pointer to an open stream
 *
 * @return -1 on failure, 0 on success
 */
int filebuffer_writeout(filebuffer_t *m_buffer, FILE *m_fp)
{
	size_t 				len = 0;
	size_t 				writelen = 0;
	int					retval = -1;

	if (m_buffer && m_fp) {
		pthread_mutex_lock(&(m_buffer->consume_lock));
		len = filebuffer_len(m_buffer);
		if (len > 0) {
			writelen = fwrite(m_buffer->buf + m_buffer->begin, sizeof(char), len, m_fp);
			if (len == writelen) {
				filebuffer_consume(m_buffer, len);
				retval = 0;
			} else {
			    blast_strerror("Could not write %zu bytes to disk", len);
			}
		} else {
		    pthread_mutex_unlock(&(m_buffer->consume_lock));
			/**
			 * If there are no bytes to write, we succeed automatically
			 */
			return 0;
		}
        /**
         * We allow locking of the consume mutex above as the write operation may take time where we still
         * want to allow writing additional data.  But before we clean our buffer, we MUST maintain
         * appropriate consistent ordering of locks throughout.
         */
        pthread_mutex_unlock(&(m_buffer->consume_lock));

		/* We try to get full control of the buffer here and if we are successful, we
		 * will clean the buffer, consolidating and re-allocating as needed.
		 */
		if (pthread_mutex_lock(&(m_buffer->append_lock)) == 0) {
		    pthread_mutex_lock(&(m_buffer->consume_lock));
			filebuffer_cleanup(m_buffer);
            pthread_mutex_unlock(&(m_buffer->consume_lock));
			pthread_mutex_unlock(&(m_buffer->append_lock));
		}
	}

	return retval;
}

/**
 * Clear the data from buffer.  This does not zero out memory.
 *
 * @param m_buffer Buffer structure to clear
 * @return nothing
 */
static void filebuffer_clear(filebuffer_t *m_buffer)
{
	m_buffer->begin = 0;
	m_buffer->end = 0;
}

/**
 * Append space to the buffer and expand the allocation if necessary.
 *
 * @param [in, out] m_buffer Buffer structure
 * @param [out] m_datap Pointer to a pointer to the first available byte in the buffer
 * @param [in] len Length of space needed
 *
 * @return -1 on error, positive number of bytes written on success
 */
static ssize_t filebuffer_append_space(filebuffer_t *m_buffer, char **m_datap, size_t len)
{
	while (m_buffer->end + len >= m_buffer->alloc) {
	    pthread_mutex_lock(&(m_buffer->consume_lock));
		/* If all the data are gathered at the end, then move to the beginning */
		if (m_buffer->begin > m_buffer->alloc/2) {
			filebuffer_defrag(m_buffer);
		}

		if (filebuffer_realloc(m_buffer, len) == -1) {
			if (m_buffer->begin != 0) {
				filebuffer_defrag(m_buffer);
			} else {
			    pthread_mutex_unlock(&(m_buffer->consume_lock));
				blast_err("file_buffer::filebuffer_append_space : Could not expand buffer");
				return -1;
			}
		}
		pthread_mutex_unlock(&(m_buffer->consume_lock));
	}

	*m_datap = m_buffer->buf + m_buffer->end;
	m_buffer->end += (unsigned int) len;

	if (m_buffer->buf == NULL) {
		*m_datap = NULL;
		blast_info("Returning NULL buffer.");
		return -1;
	}
	return (ssize_t)len;
}

/**
 * Increase the allocation size of the buffer
 *
 * @param [in,out] m_buffer Buffer structure to modify
 * @param [in] len Number of bytes that need to be added
 *
 * @return -1 on failure, 0 on success. On failure, the buffer is unchanged
 */

static int filebuffer_realloc(filebuffer_t *m_buffer, size_t len)
{
	/**
	 * Since we need to expand, toss in some extra padding for the next append to
	 * prevent a costly reallocation cycle for every buffer write if we are stalled
	 * in disk writing.
	 */
	void *temp = NULL;
	m_buffer->alloc += (unsigned int) len + filebuffer_expansion_unit;

	/**
	 * Reset the moving average size of the buffer.  Since re-allocations are expensive, we
	 * weight the new size at 100% thus the moving average will take a while to decrease
	 */
	m_buffer->moving_average_size = m_buffer->alloc;

	temp = realloc(m_buffer->buf, m_buffer->alloc);
	if (temp != NULL) {
		m_buffer->buf = temp;
		return 0;
	}

	blast_info("Failed to allocate %u bytes", (unsigned) m_buffer->alloc);

	/* Try one more time without the extra padding */
	m_buffer->alloc -= filebuffer_expansion_unit;
	temp = realloc(m_buffer->buf, m_buffer->alloc);
	if (temp != NULL) {
		m_buffer->buf = temp;
		return 0;
	}

	return -1;
}

/**
 * Move buffer data to the beginning of the buffer memory and update the
 * moving average.
 *
 * @param [in, out] m_buffer Buffer structure to modify
 * @return Nothing
 */
static void filebuffer_defrag(filebuffer_t *m_buffer)
{
	memmove(m_buffer->buf, m_buffer->buf + m_buffer->begin,
			m_buffer->end - m_buffer->begin);
	m_buffer->end -= m_buffer->begin;
	m_buffer->begin = 0;
	m_buffer->moving_average_size += filebuffer_historical_weight
			* (float)((m_buffer->end - m_buffer->begin) - m_buffer->moving_average_size);
}

/**
 * Eat data from the beginning of the buffer.  Defrag and re-allocate as needed.
 *
 * @param [in, out] m_buffer Buffer structure to modify
 * @param [in] len Number of bytes to eat
 *
 * @return Nothing
 */

static void filebuffer_consume(filebuffer_t *m_buffer, size_t len)
{
	size_t buffer_len = 0;

	buffer_len = filebuffer_len(m_buffer);

	if (buffer_len < len) {
		blast_warn("Tried to eat more data than are in the buffer");
		m_buffer->begin += buffer_len;
	} else {
		m_buffer->begin += (unsigned int) len;
	}
	filebuffer_cached_bytes[SouthIAm] -= len;
}

/**
 * Clean the buffer by consolidating data at the beginning and, if needed,
 * reducing the buffer allocation
 * @param [in] m_buffer Target file buffer
 */
static void filebuffer_cleanup(filebuffer_t *m_buffer)
{
	char *temp = NULL;
	size_t buffer_len = 0;

	buffer_len = filebuffer_len(m_buffer);

	/* if the buffer is empty, reset it to start from the beginning */
	if (m_buffer->begin == m_buffer->end) {
		filebuffer_clear(m_buffer);
	} else {
		filebuffer_defrag(m_buffer);
	}

	/**
	 * If we are sufficiently under the moving average in buffer usage and we have allocated
	 * more space than we need on average
	 */
	if ((buffer_len < (m_buffer->moving_average_size - filebuffer_expansion_unit))
			&& (m_buffer->alloc > (m_buffer->moving_average_size + filebuffer_expansion_unit))) {
		temp = realloc(m_buffer->buf, m_buffer->moving_average_size);
		if (temp != NULL) {
			m_buffer->buf = temp;
			m_buffer->alloc = m_buffer->moving_average_size;
		}
	}
}

