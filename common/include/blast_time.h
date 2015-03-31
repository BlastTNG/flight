/* 
 * blast_time.h: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of blast_common, created for the BLASTPol Project.
 *
 * blast_common is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * blast_common is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with blast_common; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Mar 31, 2015 by Seth Hillbrand
 */

#ifndef INCLUDE_BLAST_TIME_H_
#define INCLUDE_BLAST_TIME_H_

#include <time.h>
#include <stdint.h>

/* Parameters used to convert the timespec values: */
#define MSEC_PER_SEC    1000L
#define USEC_PER_MSEC   1000L
#define NSEC_PER_USEC   1000L
#define NSEC_PER_MSEC   1000000L
#define USEC_PER_SEC    1000000L
#define NSEC_PER_SEC    1000000000L
#define FSEC_PER_SEC    1000000000000000LL

#define TIME_T_MAX      (time_t)((1UL << ((sizeof(time_t) << 3) - 1)) - 1)


static inline int timespec_equal(const struct timespec *a,
                                 const struct timespec *b)
{
        return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec);
}

/*
 * lhs < rhs:  return <0
 * lhs == rhs: return 0
 * lhs > rhs:  return >0
 */
static inline int timespec_compare(const struct timespec *lhs, const struct timespec *rhs)
{
        if (lhs->tv_sec < rhs->tv_sec)
                return -1;
        if (lhs->tv_sec > rhs->tv_sec)
                return 1;
        return lhs->tv_nsec - rhs->tv_nsec;
}

static inline int timeval_compare(const struct timeval *lhs, const struct timeval *rhs)
{
        if (lhs->tv_sec < rhs->tv_sec)
                return -1;
        if (lhs->tv_sec > rhs->tv_sec)
                return 1;
        return lhs->tv_usec - rhs->tv_usec;
}

static inline void set_normalized_timespec(struct timespec *ts, time_t sec, int64_t nsec)
{
    while (nsec >= NSEC_PER_SEC) {
        /*
         * The following asm() prevents the compiler from
         * optimising this loop into a modulo operation. See
         * also __iter_div_u64_rem() in include/linux/time.h
         */
        asm("" : "+rm"(nsec));
        nsec -= NSEC_PER_SEC;
        ++sec;
    }
    while (nsec < 0) {
        asm("" : "+rm"(nsec));
        nsec += NSEC_PER_SEC;
        --sec;
    }
    ts->tv_sec = sec;
    ts->tv_nsec = nsec;
}
/*
 * Add two timespec values and do a safety check for overflow.
 * It's assumed that both values are valid (>= 0)
 */
static inline struct timespec timespec_add(const struct timespec lhs,
                                  const struct timespec rhs)
{
        struct timespec res;

        set_normalized_timespec(&res, lhs.tv_sec + rhs.tv_sec,
                                lhs.tv_nsec + rhs.tv_nsec);

        if (res.tv_sec < lhs.tv_sec || res.tv_sec < rhs.tv_sec)
                res.tv_sec = TIME_T_MAX;

        return res;
}
#endif /* INCLUDE_BLAST_TIME_H_ */
