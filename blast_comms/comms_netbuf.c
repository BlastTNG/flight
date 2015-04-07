/**
 * @file comms_netbuf.c
 *
 * @date Jan 16, 2011
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

#include <stdlib.h>
#include <string.h>
#include <stdint.h>

#include <blast.h>
#include <comms_netbuf.h>

comms_netbuf_t *comms_netbuf_new(void)
{
	comms_netbuf_t *new_buf = balloc(err, sizeof(comms_netbuf_t));

	if (!new_buf)
	{
		return NULL;
	}
	memset(new_buf, 0, sizeof(comms_netbuf_t));
	return new_buf;
}

void comms_netbuf_free(comms_netbuf_t *m_buf)
{
	if (m_buf)
	{
		BLAST_SAFE_FREE(m_buf->data);
		bfree(err, m_buf);
	}
}

static int comms_netbuf_realloc(comms_netbuf_t *m_buf, size_t m_size)
{
	size_t new_size = 64;
	uint8_t *new = NULL;

	while (new_size <= m_size)
	{
		new_size <<= 1;
	}

	new = (uint8_t*) reballoc(err, m_buf->data, new_size);
	if (!new)
	{
		return -1;
	}
	if (new_size > m_buf->allocated) memset(new + m_buf->produced, 0, new_size - m_buf->produced);
	m_buf->data = new;
	m_buf->allocated = new_size;
	return 0;
}

static void comms_netbuf_defrag(comms_netbuf_t *m_buf)
{
	if (!m_buf->consumed) return;

	memmove(m_buf->data, m_buf->data + m_buf->consumed, m_buf->produced - m_buf->consumed);
	m_buf->produced -= m_buf->consumed;
	memset(m_buf->data + m_buf->produced, 0, m_buf->consumed);
	m_buf->consumed = 0;
}

int comms_netbuf_reinit(comms_netbuf_t *m_buf)
{
	memset(m_buf->data, 0, m_buf->produced);
	m_buf->produced = 0;
	m_buf->consumed = 0;
	if (m_buf->allocated > 127)
	{
		return(comms_netbuf_realloc(m_buf, 127) < 0);
	}
	return 0;
}

int comms_netbuf_add(comms_netbuf_t *m_buf, const void *m_data, size_t m_len)
{
	if (m_buf->allocated < (m_buf->produced + m_len + 1))
	{
		if (m_buf->consumed > 0)
			comms_netbuf_defrag(m_buf);
		if (comms_netbuf_realloc(m_buf, m_buf->produced + m_len + 1) < 0)
		{
			return -1;
		}
	}

	memcpy(m_buf->data + m_buf->produced, m_data, m_len);
	m_buf->produced += m_len;
	m_buf->data[m_buf->produced] = 0;
	return 0;
}


int comms_netbuf_cat(comms_netbuf_t *m_dest, comms_netbuf_t *m_src)
{
	if (comms_netbuf_add(m_dest, comms_netbuf_get_head(m_src), comms_netbuf_remaining(m_src)) < 0)
	{
		return -1;
	}

	return 0;
}

inline void *comms_netbuf_start(comms_netbuf_t *m_buf)
{
	return m_buf->data;
}

inline void *comms_netbuf_get_head(comms_netbuf_t *m_buf)
{
	return m_buf->data + m_buf->consumed;
}

inline size_t comms_netbuf_remaining(comms_netbuf_t *m_buf)
{
	return m_buf->produced - m_buf->consumed;
}


size_t comms_netbuf_eat(comms_netbuf_t *m_buf, size_t m_len)
{
	if (m_buf->produced < m_buf->consumed + m_len)
		return 0;
	m_buf->consumed += m_len;

	if (m_buf->consumed == m_buf->produced)
	{
		m_buf->consumed = 0;
		m_buf->produced = 0;
	}
	return m_len;
}


size_t comms_netbuf_read(comms_netbuf_t *m_buf, void *m_data, size_t m_len)
{
	size_t remaining = comms_netbuf_remaining(m_buf);
	if (!m_len || !remaining)
	{
		return 0;
	}

	m_len = m_len > remaining? remaining: m_len;
	memcpy(m_data, m_buf->data + m_buf->consumed, m_len);
	m_buf->consumed += m_len;

	if (m_buf->consumed == m_buf->produced)
	{
		m_buf->consumed = 0;
		m_buf->produced = 0;
	}

	return m_len;
}

