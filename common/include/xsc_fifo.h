/**
 * @file fifo.h
 *
 * @date Feb 14, 2011
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


#ifndef INCLUDE_XSC_FIFO_H_
#define INCLUDE_XSC_FIFO_H_
#include <unistd.h>
#include <atomic.h>
#include <stdbool.h>

typedef struct xsc_fifo_node
{
    struct xsc_fifo_node    *next;
    void                *data;
} __attribute__((packed)) xsc_fifo_node_t;

typedef struct fifo
{
    xsc_fifo_node_t     *head;
    intptr_t        pop_count;
    xsc_fifo_node_t     *tail;
    intptr_t        push_count;
} __attribute__((packed, aligned(ALIGN_SIZE))) xsc_fifo_t;

bool xsc_fifo_push(xsc_fifo_t *m_fifo, void *m_data);
void *xsc_fifo_pop(xsc_fifo_t *m_fifo);
xsc_fifo_t *fifo_new();
void xsc_fifo_free(xsc_fifo_t *m_fifo, void (*m_free)(void*) );

static inline intptr_t fifo_count(xsc_fifo_t *m_fifo)
{
    if (!m_fifo) return 0;
    return (m_fifo->push_count - m_fifo->pop_count);
}

#define END_XSC_FIFO(...) NULL

#endif /* BLAST_FIFO_H_ */
