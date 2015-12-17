/**
 * @file fifo.c
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

#include <sched.h>
#include <stdbool.h>

#include <blast.h>
#include <atomic.h>
#include <fifo.h>

typedef struct __attribute__((packed)) fifo_element
{
    fifo_node_t *node;
    intptr_t count;
}  fifo_element_t __attribute__((aligned(ALIGN_SIZE)));
/**
 * Pushes a new chunk of data on the FIFO queue
 * @param m_fifo Pointer to the queue
 * @param m_data Pointer to the data to add
 */
bool fifo_push(fifo_t *m_fifo, void *m_data)
{
    fifo_node_t *new_node = malloc(sizeof(fifo_node_t));
    fifo_element_t push_struct[2];

    if (!m_fifo || !m_data) return false;

    new_node->next = END_FIFO(m_fifo);
    new_node->data = m_data;
    while (1) {
        push_struct[0].node = m_fifo->tail;
        push_struct[0].count = m_fifo->push_count;
        push_struct[1].node = m_fifo->tail->next;
        push_struct[1].count = m_fifo->push_count + 1;

        if (CAS_BOOL(&push_struct[0].node->next, END_FIFO(m_fifo), new_node)) break;

        /**
         * If we didn't succeed, then tail did not have the FIFO tail, in which
         * case we need to swing tail around and try again
         */
        sched_yield();
        CAS2(&m_fifo->tail, push_struct[0], push_struct[1]);
    }

    /**
     * We've successfully queued the node.  Now try and point our fifo tail to the newly inserted
     * node.  If we fail, then another node has been inserted behind us and we allow it to be set as
     * the tail.
     */
    push_struct[1].node = new_node;
    CAS2(&m_fifo->tail, push_struct[0], push_struct[1]);
    return true;
}

/**
 * Removes the oldest chunk of data from the FIFO queue
 * @param m_fifo Pointer to the FIFO queue
 * @return Pointer to the data chunk or NULL on failure
 */
void *fifo_pop(fifo_t *m_fifo)
{
    void *retval = NULL;
    fifo_element_t pop_struct[2];

    if (!m_fifo) return NULL;

    while (1) {
        pop_struct[0].node = m_fifo->head;
        pop_struct[0].count = m_fifo->pop_count;
        pop_struct[1].node = m_fifo->head->next;
        pop_struct[1].count = m_fifo->pop_count + 1;
        if (pop_struct[0].node == m_fifo->tail) {
            if (pop_struct[1].node == END_FIFO(m_fifo)) return NULL; /** If our queue is empty, exit */

            pop_struct[0].count = m_fifo->push_count;
            pop_struct[1].count = m_fifo->push_count + 1;
            CAS2(&m_fifo->tail, pop_struct[0], pop_struct[1]);
        } else if (pop_struct[1].node != END_FIFO(m_fifo)) {
            /**
             * Extract the data from our new dummy node
             */
            retval = pop_struct[1].node->data;
            if (CAS2_BOOL(&m_fifo->head, pop_struct[0], pop_struct[1])) break;
        }
        sched_yield();
    }

    /**
     * Free the dummy node
     */
    free(pop_struct[0].node);
    return retval;
}

/**
 * Creates a new FIFO structure and initializes the dummy node
 * @return Pointer to the FIFO structure
 */
fifo_t *fifo_new()
{
    fifo_t *new_fifo = NULL;
    fifo_node_t *new_node = NULL;

    if ((new_node = calloc(1, sizeof(fifo_node_t)))
            && (new_fifo = calloc(1, sizeof(fifo_t)))) {
        new_fifo->head = new_fifo->tail = (fifo_node_t*) new_node;
    } else {
        if (new_node) free(new_node);
    }
    return new_fifo;
}

/**
 * Frees the FIFO data structure.  This will free internally allocated nodes as well, using either
 * a user-defined free() equivalent or the system free()
 * @param m_fifo Pointer to the FIFO data structure
 * @param m_free Pointer to the free()-equivalent function
 */
void fifo_free(fifo_t *m_fifo, void (*m_free)(void*))
{
    void *m_data = NULL;

    if (m_fifo) {
        while ((m_data = fifo_pop(m_fifo))) {
            if (m_free) m_free(m_data);
        }
        if (m_fifo->head) {
            if (m_free) free(m_fifo->head);
        }
        free(m_fifo);
    }
}
