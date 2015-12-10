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

#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stddef.h>
#include <pthread.h>

#include <blast.h>
#include <comms_netbuf.h>


#define likely(cond)   __builtin_expect(!!(cond), 1)
#define unlikely(cond) __builtin_expect((cond), 0)
#define DEFAULT_MINCAP  32

static inline __attribute__((pure)) size_t netbuf_elem_size(netbuf_t* p)
{
    return p->elem_size;
}

typedef struct {
    char *buffer;
    char *bufend;
    char *begin;
    char *end;
    size_t elem_size;
} buf_snapshot_t;

static inline buf_snapshot_t make_snapshot(netbuf_t* p)
{
    return (buf_snapshot_t) {
        .buffer = p->buffer,
        .bufend = p->bufend,
        .begin  = p->begin,
        .end    = p->end,
        .elem_size = netbuf_elem_size(p),
    };
}

// Returns the maximum number of bytes the buffer can hold, excluding the
// sentinel element.
static inline size_t capacity(buf_snapshot_t s)
{
    return s.bufend - s.buffer - s.elem_size;
}

// Does the buffer wrap around?
//   true  -> wrap
//   false -> nowrap
static inline bool wraps_around(buf_snapshot_t s)
{
    return unlikely(s.begin >= s.end);
}

// Returns the number of bytes currently in use in the buffer, excluding the
// sentinel element.
static inline size_t bytes_in_use(buf_snapshot_t s)
{
    return (wraps_around(s)
    //         v   right half   v   v     left half    v
            ? ((s.end - s.buffer) + (s.bufend - s.begin))
            : (s.end - s.begin))
        // exclude the sentinel element.
        - s.elem_size;
}

static inline char* wrap_ptr_if_necessary(char* buffer,
                                          char* p,
                                          char* bufend)
{
    if (p >= bufend) {
        size_t diff = p - bufend;
        return buffer + diff;
    } else {
        return p;
    }
}

static inline char* rev_wrap_ptr_if_necessary(char* buffer,
                                              char* p,
                                              char* bufend)
{
    if (p < buffer) {
        size_t diff = buffer - p;
        return bufend - diff;
    } else {
        return p;
    }
}

static inline void* offset_memcpy(void* restrict dest, const void* restrict src, size_t n)
{
    memcpy(dest, src, n);
    return (char*) dest + n;
}

static size_t __attribute__((const)) next_pow2(size_t n)
{
    if (unlikely(!n)) return 2;

    // If when we round up we will overflow our size_t
    if (unlikely(n >= ((~(size_t )0 >> 1) + 1))) return n;

    // Algorithm idea from
    // https://groups.google.com/d/msg/comp.lang.python/xNOq4N-RffU/HObqrKz59sQJ
    n--;

    for (size_t shift = 1; shift < (sizeof n) * 8; shift <<= 1)
        n |= n >> shift;

    n++;

    return n;
}

#define in_bounds(left, x, right) ((x) >= (left) && (x) <= (right))


static inline void check_invariants(netbuf_t* p)
{
    if (p == NULL) return;

    if (p->buffer == NULL) {
        assert(p->consumer_refcount == 0);
        return;
    } else {
        assert(p->consumer_refcount != 0);
    }

    buf_snapshot_t s = make_snapshot(p);

    assert(s.begin);
    assert(s.end);
    assert(s.bufend);

    assert(p->elem_size != 0);

    assert(bytes_in_use(s) <= capacity(s) && "There are more elements in the buffer than its capacity.");

    assert(in_bounds(s.buffer, s.begin, s.bufend));
    assert(in_bounds(s.buffer, s.end, s.bufend));

    if (s.begin == s.end)
    assert(bytes_in_use(s) == capacity(s));

    assert(in_bounds(DEFAULT_MINCAP*p->elem_size, p->min_cap, p->max_cap));
    assert(in_bounds(p->min_cap, capacity(s) + p->elem_size, p->max_cap));
}


static inline void lock_pipe(netbuf_t* p)
{
    pthread_mutex_lock(&p->end_lock);
    pthread_mutex_lock(&p->begin_lock);
    check_invariants(p);
}

static inline void unlock_pipe(netbuf_t* p)
{
    check_invariants(p);
    pthread_mutex_unlock(&p->begin_lock);
    pthread_mutex_unlock(&p->end_lock);
}

// runs some code while automatically locking and unlocking the pipe. If `break'
// is used, the pipe will be unlocked before control returns from the macro.
#define WHILE_LOCKED(stuff) do {    \
    lock_pipe(m_buf);               \
    do { stuff; } while (0);        \
    unlock_pipe(m_buf);             \
    } while (0)

netbuf_t* netbuf_new(size_t elem_size, size_t limit)
{
    assert(elem_size != 0);

    if (elem_size == 0)
        return NULL;

    netbuf_t* p = malloc(sizeof *p);

    assert(DEFAULT_MINCAP >= 1);

    size_t cap = DEFAULT_MINCAP * elem_size;
    char*  buf = malloc(cap);

    // Change the limit from being in "elements" to being in "bytes", and make
    // room for the sentinel element.
    limit = (limit + 1) * elem_size;

    if (unlikely(p == NULL || buf == NULL))
        return free(p), free(buf), NULL;

    *p = (netbuf_t) {
        .elem_size  = elem_size,
        .min_cap = cap,
        .max_cap = limit ? next_pow2(max(limit, cap)) : ~(size_t)0,

        .buffer = buf,
        .bufend = buf + cap,
        .begin  = buf,
        .end    = buf + elem_size,

        // Since we're issuing a netbuf_t, it counts as both a producer and a
        // consumer since it can issue new instances of both. Therefore, the
        // refcounts both start at 1; not the intuitive 0.
        .producer_refcount = 1,
        .consumer_refcount = 1,
    };

    pthread_mutex_init(&p->begin_lock, NULL);
    pthread_mutex_init(&p->end_lock, NULL);

    pthread_cond_init(&p->just_pushed, NULL);
    pthread_cond_init(&p->just_popped, NULL);

    check_invariants(p);

    return p;
}

netbuf_t* netbuf_producer_new(netbuf_t* p)
{
    pthread_mutex_lock(&p->begin_lock);
        p->producer_refcount++;
    pthread_mutex_unlock(&p->begin_lock);

    return (netbuf_t*)p;
}

netbuf_t* netbuf_consumer_new(netbuf_t* p)
{
    pthread_mutex_lock(&p->end_lock);
        p->consumer_refcount++;
    pthread_mutex_unlock(&p->end_lock);

    return (netbuf_t*)p;
}

static void deallocate(netbuf_t* p)
{
    assert(p->producer_refcount == 0);
    assert(p->consumer_refcount == 0);

    pthread_mutex_destroy(&p->begin_lock);
    pthread_mutex_destroy(&p->end_lock);

    pthread_cond_destroy(&p->just_pushed);
    pthread_cond_destroy(&p->just_popped);

    free(p->buffer);
    free(p);
}

void netbuf_free(netbuf_t* p)
{
    size_t new_producer_refcount,
           new_consumer_refcount;

    pthread_mutex_lock(&p->begin_lock);
        assert(p->producer_refcount > 0);
        new_producer_refcount = --p->producer_refcount;
    pthread_mutex_unlock(&p->begin_lock);

    pthread_mutex_lock(&p->end_lock);
        assert(p->consumer_refcount > 0);
        new_consumer_refcount = --p->consumer_refcount;
    pthread_mutex_unlock(&p->end_lock);

    if (unlikely(new_consumer_refcount == 0)) {
        p->buffer = (free(p->buffer), NULL);

        if (likely(new_producer_refcount > 0))
            pthread_cond_broadcast(&p->just_popped);
        else
            deallocate(p);
    } else {
        if (unlikely(new_producer_refcount == 0)) pthread_cond_broadcast(&p->just_pushed);
    }
}

void netbuf_producer_free(netbuf_t* p)
{
    size_t new_producer_refcount;

    pthread_mutex_lock(&p->begin_lock);
        assert(p->producer_refcount > 0);
        new_producer_refcount = --p->producer_refcount;
    pthread_mutex_unlock(&p->begin_lock);

    if (unlikely(new_producer_refcount == 0)) {
        size_t consumer_refcount;

        pthread_mutex_lock(&p->end_lock);
        consumer_refcount = p->consumer_refcount;
        pthread_mutex_unlock(&p->end_lock);

        // If there are still consumers, wake them up if they're waiting on
        // input from a producer. Otherwise, since we're the last handle
        // altogether, we can free the pipe.
        if (likely(consumer_refcount > 0))
            pthread_cond_broadcast(&p->just_pushed);
        else
            deallocate(p);
    }
}

void netbuf_consumer_free(netbuf_t* p)
{
    size_t new_consumer_refcount;

    pthread_mutex_lock(&p->end_lock);
        new_consumer_refcount = --p->consumer_refcount;
    pthread_mutex_unlock(&p->end_lock);

    if (unlikely(new_consumer_refcount == 0)) {
        size_t producer_refcount;

        pthread_mutex_lock(&p->begin_lock);
        producer_refcount = p->producer_refcount;
        pthread_mutex_unlock(&p->begin_lock);

        // If there are still producers, wake them up if they're waiting on
        // room to free up from a consumer. Otherwise, since we're the last
        // handle altogether, we can free the pipe.
        if (likely(producer_refcount > 0))
            pthread_cond_broadcast(&p->just_popped);
        else
            deallocate(p);
    }
}

// Returns the end of the buffer (buf + number_of_bytes_copied).
static inline char* copy_netbuf_into_new_buf(buf_snapshot_t s, char* restrict buf) {
    if (wraps_around(s)) {
        buf = offset_memcpy(buf, s.begin, s.bufend - s.begin);
        buf = offset_memcpy(buf, s.buffer, s.end - s.buffer);
    } else {
        buf = offset_memcpy(buf, s.begin, s.end - s.begin);
    }

    return buf;
}


// Returns the number of bytes copied
static inline size_t copy_netbuf_data(buf_snapshot_t s, char* restrict buf) {
    size_t retval = 0;

    if (wraps_around(s)) {
        buf = offset_memcpy(buf, s.begin + s.elem_size, s.bufend - s.begin - s.elem_size);
        retval = s.bufend - s.begin - s.elem_size;

        buf = offset_memcpy(buf, s.buffer, s.end - s.buffer);
        retval += (s.end - s.buffer - s.elem_size);
    } else {
        buf = offset_memcpy(buf, s.begin + s.elem_size, s.end - s.begin - s.elem_size);
        retval = s.end - s.begin - s.elem_size;
    }

    return retval;
}

// Resizes the buffer to make room for at least 'new_size' elements, returning
// an updated snapshot of the pipe state.
//
// The new size MUST be bigger than the number of elements currently in the
// pipe.
//
// The pipe must be fully locked on entrance to this function.
static buf_snapshot_t resize_buffer(netbuf_t* p, size_t new_size)
{
    check_invariants(p);

    const size_t max_cap   = p->max_cap,
                 min_cap   = p->min_cap,
                 elem_size = netbuf_elem_size(p);

    assert(new_size >= bytes_in_use(make_snapshot(p)));

    if (unlikely(new_size >= max_cap)) new_size = max_cap;

    if (new_size <= min_cap) return make_snapshot(p);

    char* new_buf = malloc(new_size + elem_size);
    p->end = copy_netbuf_into_new_buf(make_snapshot(p), new_buf);

    p->begin  =
    p->buffer = (free(p->buffer), new_buf);

    p->bufend = new_buf + new_size + elem_size;

    check_invariants(p);

    return make_snapshot(p);
}

static inline buf_snapshot_t validate_size(netbuf_t* p, buf_snapshot_t s, size_t new_bytes)
{
    size_t elem_size = netbuf_elem_size(p);
    size_t cap = capacity(s);
    size_t bytes_needed = bytes_in_use(s) + new_bytes;

    if (unlikely(bytes_needed > cap)) {
        // upgrade our lock, then re-check. By taking both locks (end and begin)
        // in order, we have an equivalent operation to lock_pipe().
        {
            pthread_mutex_lock(&p->begin_lock);

            s = make_snapshot(p);
            bytes_needed = bytes_in_use(s) + new_bytes;
            size_t elems_needed = bytes_needed / elem_size;

            if (likely(bytes_needed > cap))
                s = resize_buffer(p, next_pow2(elems_needed) * elem_size);
        }

        // Unlock the pipe if requested.
        pthread_mutex_unlock(&p->begin_lock);
    }

    return s;
}

// Runs the actual push, assuming there is enough room in the buffer.
//
// Returns the new 'end' pointer.
static inline char* process_push(buf_snapshot_t s,
                                const void* restrict elems,
                                size_t bytes_to_copy
                               )
{
    assert(bytes_to_copy != 0);

    // This shouldn't be necessary.
    // s.end = wrap_ptr_if_necessary(s.buffer, s.end, s.bufend);
    assert(s.end != s.bufend);

    // If we currently have a nowrap buffer, we may have to wrap the new
    // elements. Copy as many as we can at the end, then start copying into the
    // beginning. This basically reduces the problem to only deal with wrapped
    // buffers, which can be dealt with using a single offset_memcpy.
    if (!wraps_around(s)) {
        size_t at_end = min(bytes_to_copy, (size_t) (s.bufend - s.end));

        s.end = offset_memcpy(s.end, elems, at_end);

        elems = (const char*) elems + at_end;
        bytes_to_copy -= at_end;
    }

    // Now copy any remaining data...
    if (unlikely(bytes_to_copy)) {
        s.end = wrap_ptr_if_necessary(s.buffer, s.end, s.bufend);
        s.end = offset_memcpy(s.end, elems, bytes_to_copy);
    }

    s.end = wrap_ptr_if_necessary(s.buffer, s.end, s.bufend);

    // ...and update the end pointer!
    return s.end;
}

// Will spin until there is enough room in the buffer to push any elements.
// Returns the number of elements currently in the buffer. `end_lock` should be
// locked on entrance to this function.
static inline buf_snapshot_t wait_for_room(netbuf_t* m_buf, size_t* max_cap)
{
    buf_snapshot_t s = make_snapshot(m_buf);

    size_t bytes_used = bytes_in_use(s);

    size_t consumer_refcount = m_buf->consumer_refcount;

    *max_cap = m_buf->max_cap;

    for (; unlikely(bytes_used == *max_cap) && likely(consumer_refcount > 0);
          s                 = make_snapshot(m_buf),
          bytes_used        = bytes_in_use(s),
          consumer_refcount = m_buf->consumer_refcount,
          *max_cap          = m_buf->max_cap)
        pthread_cond_wait(&m_buf->just_popped, &m_buf->end_lock);

    return s;
}

size_t __netbuf_write(netbuf_t* m_buf,
                 const void* restrict m_src,
                 size_t m_count)
{
    size_t elem_size = netbuf_elem_size(m_buf);

    if (unlikely(m_count == 0))
        return 0;

    size_t pushed = 0;

    { pthread_mutex_lock(&m_buf->end_lock);
        buf_snapshot_t s = make_snapshot(m_buf);
        size_t max_cap = m_buf->max_cap;

        // if no more consumers...
        if (unlikely(m_buf->consumer_refcount == 0)) {
            pthread_mutex_unlock(&m_buf->end_lock);
            return 0;
        }

        s = validate_size(m_buf, s, m_count);

        // Finally, we can now begin with pushing as many elements into the
        // queue as possible.
        m_buf->end = process_push(s, m_src,
                     pushed = min(m_count, max_cap - bytes_in_use(s)));
    } pthread_mutex_unlock(&m_buf->end_lock);

    assert(pushed > 0);

    // Signal if we've only pushed one element, broadcast if we've pushed more.
    if (unlikely(pushed == elem_size))
        pthread_cond_signal(&m_buf->just_pushed);
    else
        pthread_cond_broadcast(&m_buf->just_pushed);

    return pushed;
}

size_t netbuf_write(netbuf_t* m_buf, const void* restrict m_src, size_t m_count)
{
    m_count *= netbuf_elem_size(m_buf);
    return __netbuf_write(m_buf, m_src, m_count);
}


#ifdef NETBUF_DEBUG
#include <stdio.h>
void netbuf_debug_int(netbuf_t* p, const char* id)
{
    char debug_string[1025];
    int len = 0;
    len = snprintf(debug_string, sizeof(debug_string), "%s: [ ", id);
    for (int* ptr = (int*)p->buffer; ptr != (int*)p->bufend; ++ptr) {
        len += snprintf(debug_string + len, 1024 - len, "%i ", *ptr);
        if (len > 1024) break;
    }
    if (len <=1024) snprintf(debug_string + len, sizeof(debug_string + len), "]");
    blast_dbg("%s", debug_string);
    blast_dbg("begin: %lu    end: %lu\n", p->begin - p->buffer, p->end - p->buffer);
}
void netbuf_debug_string(netbuf_t* p, const char* id)
{
    char debug_string[1025];
    int len = 0;
    len = snprintf(debug_string, sizeof(debug_string), "%s: [ ", id);
    for (char* ptr = (char*)p->buffer; ptr != (char*)p->bufend; ++ptr) {
        len += snprintf(debug_string + len, 1024 - len, "%c ", *ptr);
        if (len > 1024) break;
    }
    if (len <=1024) snprintf(debug_string + len, sizeof(debug_string + len), "]");
    blast_dbg("%s", debug_string);
    blast_dbg("begin: %lu    end: %lu\n", p->begin - p->buffer, p->end - p->buffer);
}
#endif


// Waits for at least one element to be in the pipe. p->begin_lock must be
// locked when entering this function, and a new, valid snapshot is returned.
static inline buf_snapshot_t wait_for_elements(netbuf_t* p, bool m_immediate)
{
    buf_snapshot_t s = make_snapshot(p);

    size_t bytes_used = bytes_in_use(s);

    if (m_immediate && !bytes_used) return s;

    for (; unlikely(bytes_used == 0) && likely(p->producer_refcount > 0);
          s = make_snapshot(p),
          bytes_used = bytes_in_use(s))
        pthread_cond_wait(&p->just_pushed, &p->begin_lock);

    return s;
}

// wow, I didn't even intend for the name to work like that...
// returns a new snapshot, with the updated changes also reflected onto the
// pipe.
static inline buf_snapshot_t pop_without_locking(buf_snapshot_t s,
                                             void* restrict target,
                                             size_t bytes_to_copy,
                                             char** begin // [out]
                                            )
{
    assert(s.begin != s.bufend);

    size_t elem_size = s.elem_size;

    // Copy either as many bytes as requested, or the available bytes in the RHS
    // of a wrapped buffer - whichever is smaller.
    {
        size_t first_bytes_to_copy = min(bytes_to_copy, (size_t)(s.bufend - s.begin - elem_size));

        target = offset_memcpy(target, s.begin + elem_size, first_bytes_to_copy);

        bytes_to_copy -= first_bytes_to_copy;

        s.begin       += first_bytes_to_copy;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);
    }

    if (unlikely(bytes_to_copy > 0)) {
        s.begin += elem_size;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);

        memcpy(target, s.begin, bytes_to_copy);

        s.begin += bytes_to_copy;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);

        s.begin -= elem_size;
        s.begin = rev_wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);
    }

    // Since we cached begin on the stack, we need to reflect our changes back
    // on the pipe.
    *begin = s.begin;

    return s;
}

// If the buffer is shrunk to something a lot smaller than our current
// capacity, resize it to something sane. This function must be entered with
// only p->begin_lock locked, and will automatically unlock p->begin_lock on
// exit.
static inline void trim_buffer(netbuf_t* p, buf_snapshot_t s)
{
    size_t cap = capacity(s);

    // We have a sane size. We're done here.
    if (likely(bytes_in_use(s) > cap / 4)) {
        pthread_mutex_unlock(&p->begin_lock);
        return;
    }

    // Okay, we need to resize now. Upgrade our lock so we can check again. The
    // weird lock/unlock order is to make sure we always acquire the end_lock
    // before begin_lock. Deadlock can arise otherwise.
    pthread_mutex_unlock(&p->begin_lock);
    pthread_mutex_lock(&p->end_lock);
    pthread_mutex_lock(&p->begin_lock);

    s   = make_snapshot(p);
    cap = capacity(s);

    // To conserve space like the good computizens we are, we'll shrink
    // our buffer if our memory usage efficiency drops below 25%. However,
    // since shrinking/growing the buffer is the most expensive part of a push
    // or pop, we only shrink it to bring us up to a 50% efficiency. A common
    // pipe usage pattern is sudden bursts of pushes and pops. This ensures it
    // doesn't get too time-inefficient.
    if (likely(bytes_in_use(s) <= cap / 4))
        resize_buffer(p, cap / 2);

    // All done. Unlock the pipe. The reason we don't let the calling function
    // unlock begin_lock is so that we can do it BEFORE end_lock. This prevents
    // the lock order from being fudged.
    pthread_mutex_unlock(&p->begin_lock);
    pthread_mutex_unlock(&p->end_lock);
}

// Performs the actual pop, except `requested' is now in bytes as opposed to
// elements.
//
// This will behave eagerly, returning as many elements that it can into
// `target' as it can fill right now.
static inline size_t __netbuf_pop(netbuf_t* p, void* restrict target, size_t requested,
bool m_nowait)
{
    if (unlikely(requested == 0)) return 0;

    size_t popped = 0;

    {
        pthread_mutex_lock(&p->begin_lock);
        buf_snapshot_t s = wait_for_elements(p, m_nowait);
        size_t bytes_used = bytes_in_use(s);

        if (unlikely(bytes_used == 0)) {
            pthread_mutex_unlock(&p->begin_lock);
            return 0;
        }

        check_invariants(p);

        s = pop_without_locking(s, target, popped = min(requested, bytes_used), &p->begin);

        check_invariants(p);

        trim_buffer(p, s);
    } // p->begin_lock was unlocked by trim_buffer.

    assert(popped);

    if (unlikely(popped == netbuf_elem_size(p)))
        pthread_cond_signal(&p->just_popped);
    else
        pthread_cond_broadcast(&p->just_popped);

    return popped;
}

static size_t __netbuf_read(netbuf_t *p, void *m_target, size_t m_count, bool m_nonblock)
{
    size_t elem_size = netbuf_elem_size(p);
    size_t bytes_left = m_count * elem_size;
    size_t bytes_popped = 0;
    size_t ret = -1;

    do {
        ret = __netbuf_pop(p, m_target, bytes_left, m_nonblock);
        m_target = (void*) ((char*) m_target + ret);
        bytes_popped += ret;
        bytes_left -= ret;
    } while (ret && bytes_left);

    return bytes_popped / elem_size;
}

size_t netbuf_read(netbuf_t* p, void* m_target, size_t count)
{
    return __netbuf_read(p, m_target, count, true);
}

size_t netbuf_read_wait(netbuf_t *p, void *m_target, size_t count)
{
    return __netbuf_read(p, m_target, count, false);
}

size_t netbuf_peek_noalloc(netbuf_t *m_buf, void *m_target, size_t m_size)
{
    void *dest = m_target;
    buf_snapshot_t s = make_snapshot(m_buf);
    size_t bytes_to_copy = min(m_size, bytes_in_use(s));

    if (wraps_around(s) && (bytes_to_copy > (s.bufend - s.begin))) {
        dest = offset_memcpy(dest, s.begin + s.elem_size, s.bufend - s.begin - s.elem_size);
        bytes_to_copy -= (s.bufend - s.begin);

        memcpy(dest, s.buffer, bytes_to_copy);
    } else {
        memcpy(dest, s.begin + s.elem_size, bytes_to_copy);
    }

    return bytes_to_copy;
}

size_t netbuf_peek(netbuf_t *m_buf, void **m_target)
{
    size_t retval;

    pthread_mutex_lock(&m_buf->begin_lock);
    buf_snapshot_t s = make_snapshot(m_buf);
    size_t bytes_used = bytes_in_use(s); /// Add the sentinel element back here

    if (unlikely(bytes_used == 0)) {
        pthread_mutex_unlock(&m_buf->begin_lock);
        *m_target = NULL;
        return 0;
    }

    check_invariants(m_buf);
    *m_target = calloc(bytes_used, 1);
    retval = copy_netbuf_data(s, *m_target);

    pthread_mutex_unlock(&m_buf->begin_lock);

    return retval;
}

void netbuf_reserve(netbuf_t *m_buf, size_t m_count)
{
    m_count *= netbuf_elem_size(m_buf); // now `count' is in "bytes" instead of "elements".

    if (m_count == 0) m_count = DEFAULT_MINCAP;

    size_t max_cap = m_buf->max_cap;

    WHILE_LOCKED(if(unlikely(m_count <= bytes_in_use(make_snapshot(m_buf)))) break;

    m_buf->min_cap = min(m_count, max_cap); resize_buffer(m_buf, m_count););
}

void netbuf_eat(netbuf_t *m_buf, size_t m_len)
{
    buf_snapshot_t s;

    pthread_mutex_lock(&m_buf->begin_lock);

    if (unlikely(m_len == 0)) {
        pthread_mutex_unlock(&m_buf->begin_lock);
        return;
    }
    s  = make_snapshot(m_buf);
    check_invariants(m_buf);
    {
        size_t first_bytes_to_copy = min(m_len, (size_t )(s.bufend - s.begin - s.elem_size));

        m_len -= first_bytes_to_copy;

        s.begin += first_bytes_to_copy;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);
    }

    if (unlikely(m_len > 0)) {
        s.begin += s.elem_size;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);

        s.begin += m_len;
        s.begin = wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);

        s.begin -= s.elem_size;
        s.begin = rev_wrap_ptr_if_necessary(s.buffer, s.begin, s.bufend);
    }

    m_buf->begin = s.begin;
    check_invariants(m_buf);

    trim_buffer(m_buf, s); // Unlocks begin_lock
}


void netbuf_reinit(netbuf_t *m_buf)
{
    buf_snapshot_t s  = make_snapshot(m_buf);
    size_t bytes_used = bytes_in_use(s);

    netbuf_eat(m_buf, bytes_used);
}


