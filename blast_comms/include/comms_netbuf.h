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

typedef struct comms_netbuf {
    uint8_t	*data;
    size_t 	produced;
    size_t 	allocated;
    size_t 	consumed;
} comms_netbuf_t;

void comms_netbuf_free(comms_netbuf_t *m_buf);
void *comms_netbuf_start(comms_netbuf_t *m_buf);
comms_netbuf_t *comms_netbuf_new(void);
int comms_netbuf_add(comms_netbuf_t *m_buf, const void *m_data, size_t m_len);
int comms_netbuf_cat(comms_netbuf_t *m_dest, comms_netbuf_t *m_src);
int comms_netbuf_reinit(comms_netbuf_t *m_buf);
void *comms_netbuf_get_head(comms_netbuf_t *m_buf);
size_t comms_netbuf_remaining(comms_netbuf_t *m_buf);
size_t comms_netbuf_read(comms_netbuf_t *m_buf, void *data, size_t m_len);
size_t buffer_pass_bytes_end(comms_netbuf_t *m_buf, size_t m_len);
size_t comms_netbuf_eat(comms_netbuf_t *m_buf, size_t m_len);

#endif /* EBEX_NET_BUFFER_H_ */
