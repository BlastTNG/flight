/*
 * pq.h
 *
 * @date Jan 31, 2011
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
 * Based on fdqueue.c from the Apache httpd project.  Original code is
 * licensed under Apache License version 2.0.  A copy of this is
 * available at http://www.apache.org/licenses/LICENSE-2.0
 *
 */

#ifndef INCLUDE_PQ_H_
#define INCLUDE_PQ_H_

#include <stdint.h>
#include <unistd.h>

/** priority data type */
typedef uint64_t pq_pri_t;

/** callback functions to get/set/compare the priority of an element */
typedef pq_pri_t (*pq_get_pri_f)(void *a);
typedef void (*pq_set_pri_f)(void *a, pq_pri_t pri);
typedef int (*pq_cmp_pri_f)(pq_pri_t next, pq_pri_t curr);


/** callback functions to get/set the position of an element */
typedef size_t (*pq_get_pos_f)(void *a);
typedef void (*pq_set_pos_f)(void *a, size_t pos);


/** debug callback function to print a entry */
typedef void (*pq_print_entry_f)(FILE *out, void *a);


/** the priority queue handle */
typedef struct pq_t
{
    size_t size;            /**< number of elements in this queue */
    size_t avail;           /**< slots available in this queue */
    size_t step;            /**< growth stepping setting */
    pq_cmp_pri_f cmppri;    /**< callback to compare nodes */
    pq_get_pri_f getpri;    /**< callback to get priority of a node */
    pq_set_pri_f setpri;    /**< callback to set priority of a node */
    pq_get_pos_f getpos;    /**< callback to get position of a node */
    pq_set_pos_f setpos;    /**< callback to set position of a node */
    void **d;               /**< The actual queue in binary heap form */
} pq_t;


/**
 * initialize the queue
 *
 * @param n the initial estimate of the number of queue items for which memory
 *     should be preallocated
 * @param cmppri The callback function to run to compare two elements
 *     This callback should return 0 for 'lower' and non-zero
 *     for 'higher', or vice versa if reverse priority is desired
 * @param setpri the callback function to run to assign a score to an element
 * @param getpri the callback function to run to set a score to an element
 * @param getpos the callback function to get the current element's position
 * @param setpos the callback function to set the current element's position
 *
 * @return the handle or NULL for insufficent memory
 */
pq_t *
pq_init(size_t n, pq_cmp_pri_f cmppri, pq_get_pri_f getpri, pq_set_pri_f setpri,
                  pq_get_pos_f getpos, pq_set_pos_f setpos);


/**
 * free all memory used by the queue
 * @param q the queue
 */
void pq_free(pq_t *q);


/**
 * return the size of the queue.
 * @param q the queue
 */
size_t pq_size(pq_t *q);


/**
 * insert an item into the queue.
 * @param q the queue
 * @param d the item
 * @return 0 on success
 */
int pq_insert(pq_t *q, void *d);


/**
 * move an existing entry to a different priority
 * @param q the queue
 * @param new_pri the new priority
 * @param d the entry
 */
void pq_change_priority(pq_t *q, pq_pri_t new_pri, void *d);


/**
 * pop the highest-ranking item from the queue.
 * @param q the queue
 * @return NULL on error, otherwise the entry
 */
void *pq_pop(pq_t *q);


/**
 * remove an item from the queue.
 * @param q the queue
 * @param d the entry
 * @return 0 on success
 */
int pq_remove(pq_t *q, void *d);


/**
 * access highest-ranking item without removing it.
 * @param q the queue
 * @return NULL on error, otherwise the entry
 */
void *pq_peek(pq_t *q);


/**
 * print the queue
 * @internal
 * DEBUG function only
 * @param q the queue
 * @param out the output handle
 * @param the callback function to print the entry
 */
void pq_print(pq_t *q, FILE *out, pq_print_entry_f print);


/**
 * dump the queue and it's internal structure
 * @internal
 * debug function only
 * @param q the queue
 * @param out the output handle
 * @param the callback function to print the entry
 */
void pq_dump(pq_t *q, FILE *out, pq_print_entry_f print);


/**
 * checks that the pq is in the right order, etc
 * @internal
 * debug function only
 * @param q the queue
 */
int pq_is_valid(pq_t *q);

#endif /* INCLUDE_PQ_H_ */
