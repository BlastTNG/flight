/*
 * pq.c
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


#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pq.h"


#define LEFT(i)   ((i) << 1)
#define RIGHT(i)  (((i) << 1) + 1)
#define PARENT(i) ((i) >> 1)


pq_t *pq_init(size_t n,
              pq_cmp_pri_f cmppri, pq_get_pri_f getpri,
              pq_set_pri_f setpri, pq_get_pos_f getpos,
              pq_set_pos_f setpos)
{
    pq_t *q;

    if (!(q = malloc(sizeof(pq_t)))) return NULL;

    /* Need to allocate n+1 elements since element 0 isn't used. */
    if (!(q->d = malloc((n + 1) * sizeof(void *)))) {
        free(q);
        return NULL;
    }

    q->size = 1;
    q->avail = q->step = (n + 1); /* see comment above about n+1 */
    q->cmppri = cmppri;
    q->setpri = setpri;
    q->getpri = getpri;
    q->getpos = getpos;
    q->setpos = setpos;

    return q;
}


void pq_free(pq_t *q)
{
    free(q->d);
    free(q);
}


size_t pq_size(pq_t *q)
{
    /* queue element 0 exists but doesn't count since it isn't used. */
    return (q->size - 1);
}


static void bubble_up(pq_t *q, size_t i)
{
    size_t parent_node;
    void *moving_node = q->d[i];
    pq_pri_t moving_pri = q->getpri(moving_node);

    for (parent_node = PARENT(i);
            ((i > 1) && q->cmppri(q->getpri(q->d[parent_node]), moving_pri));
            i = parent_node, parent_node = PARENT(i)) {
        q->d[i] = q->d[parent_node];
        q->setpos(q->d[i], i);
    }

    q->d[i] = moving_node;
    q->setpos(moving_node, i);
}


static size_t maxchild(pq_t *q, size_t i)
{
    size_t child_node = LEFT(i);

    if (child_node >= q->size) return 0;

    if ((child_node + 1) < q->size &&
            q->cmppri(q->getpri(q->d[child_node]), q->getpri(q->d[child_node + 1]))) {
        child_node++; /* use right child instead of left */
    }

    return child_node;
}

static void percolate_down(pq_t *q, size_t i)
{
    size_t child_node;
    void *moving_node = q->d[i];
    pq_pri_t moving_pri = q->getpri(moving_node);

    while ((child_node = maxchild(q, i)) && q->cmppri(moving_pri, q->getpri(q->d[child_node]))) {
        q->d[i] = q->d[child_node];
        q->setpos(q->d[i], i);
        i = child_node;
    }

    q->d[i] = moving_node;
    q->setpos(moving_node, i);
}

int pq_insert(pq_t *q, void *d)
{
    void *tmp;
    size_t i;
    size_t newsize;

    if (!q) return 1;

    /* allocate more memory if necessary */
    if (q->size >= q->avail) {
        newsize = q->size + q->step;
        if (!(tmp = realloc(q->d, sizeof(void *) * newsize))) return 1;
        q->d = tmp;
        q->avail = newsize;
    }

    /* insert item */
    i = q->size++;
    q->d[i] = d;
    bubble_up(q, i);

    return 0;
}

void pq_change_priority(pq_t *q, pq_pri_t new_pri, void *d)
{
    size_t posn;
    pq_pri_t old_pri = q->getpri(d);

    q->setpri(d, new_pri);
    posn = q->getpos(d);
    if (q->cmppri(old_pri, new_pri))
        bubble_up(q, posn);
    else
        percolate_down(q, posn);
}

int pq_remove(pq_t *q, void *d)
{
    size_t posn = q->getpos(d);
    q->d[posn] = q->d[--q->size];
    if (q->cmppri(q->getpri(d), q->getpri(q->d[posn])))
        bubble_up(q, posn);
    else
        percolate_down(q, posn);

    return 0;
}

void *pq_pop(pq_t *q)
{
    void *head;

    if (!q || q->size == 1) return NULL;

    head = q->d[1];
    q->d[1] = q->d[--q->size];
    percolate_down(q, 1);

    return head;
}

void *pq_peek(pq_t *q)
{
    void *d;
    if (!q || q->size == 1) return NULL;
    d = q->d[1];
    return d;
}

void pq_dump(pq_t *q, FILE *out, pq_print_entry_f print)
{
    int i;

    fprintf(stdout, "posn\tleft\tright\tparent\tmaxchild\t...\n");
    for (i = 1; i < q->size; i++) {
        fprintf(stdout, "%d\t%d\t%d\t%d\t%ul\t",
                i, LEFT(i), RIGHT(i), PARENT(i), (unsigned int) maxchild(q, i));
        print(out, q->d[i]);
    }
}


static void set_pos(void *d, size_t val)
{
    /* do nothing */
}


static void set_pri(void *d, pq_pri_t pri)
{
    /* do nothing */
}


void pq_print(pq_t *q, FILE *out, pq_print_entry_f print)
{
    pq_t *dup;
    void *e;

    dup = pq_init(q->size, q->cmppri, q->getpri, set_pri, q->getpos, set_pos);
    dup->size = q->size;
    dup->avail = q->avail;
    dup->step = q->step;

    memcpy(dup->d, q->d, (q->size * sizeof(void *)));

    while ((e = pq_pop(dup)))
        print(out, e);

    pq_free(dup);
}


static int subtree_is_valid(pq_t *q, int pos)
{
    if (LEFT(pos) < q->size) {
        /* has a left child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[LEFT(pos)]))) return 0;
        if (!subtree_is_valid(q, LEFT(pos))) return 0;
    }
    if (RIGHT(pos) < q->size) {
        /* has a right child */
        if (q->cmppri(q->getpri(q->d[pos]), q->getpri(q->d[RIGHT(pos)]))) return 0;
        if (!subtree_is_valid(q, RIGHT(pos))) return 0;
    }
    return 1;
}


int pq_is_valid(pq_t *q)
{
    return subtree_is_valid(q, 1);
}
