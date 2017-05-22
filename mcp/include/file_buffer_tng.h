/*
 * file_buffer_tng.h
 *
 *  Created on: Mar 27, 2010 originally, updated May 5th 2017 for BLAST-TNG
 *      Author: Seth Hillbrand, Laura Fissel
 *
 * This file is part of MCP, the BLAST flight control program
 *
 * This software is copyright (C) 2010 Columbia University
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

#ifndef INCLUDE_FILE_BUFFER_TNG_H_
#define INCLUDE_FILE_BUFFER_TNG_H_

#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/types.h>

typedef struct
{
	char 			*buf;				/* The data buffer pointer */
	size_t		 	alloc;				/* Number of bytes allocated */
	unsigned int	begin;				/* Offset of the first data-containing byte from #buf */
	unsigned int	end;				/* Offset of the last data-containing byte from #buf */
	size_t			initial_size;		/* Initial guess as to the best buffer size (in bytes) */
	size_t			moving_average_size;/* Calculated optimal size of buffer */
	pthread_mutex_t	append_lock;		/* Gets locked when appending to buffer */
	pthread_mutex_t	consume_lock;		/* Gets locked when consuming buffer or writing data to file */
} filebuffer_t;

int filebuffer_init(filebuffer_t *m_buffer, size_t len);
void filebuffer_free(filebuffer_t *m_buffer);
ssize_t filebuffer_append(filebuffer_t *m_buffer, const char *m_data, size_t len);
size_t filebuffer_len(filebuffer_t *m_buffer);
int filebuffer_writeout(filebuffer_t *m_buffer, FILE *m_fp);
float filebuffer_fraction_full(filebuffer_t *m_buffer);

/**
 * Correctly sets a timeout value some positive offset from now.  This really doesn't belong in
 * this header file.
 * @TODO: Move set_timeout to general utility header file after created
 * @param m_timer Pointer to the timeout timer
 * @param m_sec Number of seconds from now to timeout (cumulative with m_nsec)
 * @param m_nsec Number of nano seconds from now to timeout (cumulative with m_sec)
 */
static __inline__ void set_timeout(struct timespec *m_timer, uint32_t m_sec, uint64_t m_nsec)
{
	clock_gettime(CLOCK_REALTIME, m_timer);
	m_timer->tv_nsec += (typeof(m_timer->tv_nsec))m_nsec;
	if (m_timer->tv_nsec > 1000000000) {
		m_timer->tv_sec++;
		m_timer->tv_nsec -= 1000000000;
	}
	m_timer->tv_sec += (typeof(m_timer->tv_sec))m_sec;
}

#endif /* INCLUDE_FILE_BUFFER_TNG_H_ */
