/**
 * @file atomic.h
 *
 * @date Feb 14, 2011
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


#ifndef BLAST_ATOMIC_H_
#define BLAST_ATOMIC_H_
#include <stdint.h>
#include <malloc.h>

#define CACHE_LINE_SIZE 64

#define BLAST_ATOMIC_MEM_BLOCK()  __asm__ __volatile__ ("mfence" ::: "memory")
#define BLAST_ATOMIC_STORE_MEM_BLOCK() __asm__ __volatile__ ("mfence" ::: "memory")

#define BLAST_ATOMIC_BURY(_p)        ((void *)(((uintptr_t)(_p)) | 1))
#define BLAST_ATOMIC_UNBURY(_p)      ((void *)(((uintptr_t)(_p)) & ~1))
#define BLAST_ATOMIC_IS_BURIED(_p)   (((uintptr_t)(_p)) & 1)

/**
 * If our pointers are 64-bits, we need to use a 128-bit compare and swap for the double-word.  Otherwise, we can
 * use standard 64-bit
 */
#if INTPTR_MAX == INT64_MAX
#define ALIGN_SIZE 16
    typedef int __attribute__ ((mode (TI), may_alias)) dbl_ptr;
#else
#define ALIGN_SIZE 8
    typedef uint64_t __attribute__ ((may_alias)) dbl_ptr;
#endif

#define CAS(_ptr, _oldval, _newval) __sync_val_compare_and_swap ((typeof(_oldval)*)(_ptr), _oldval, _newval)
#define CAS_BOOL(_ptr, _oldval, _newval) __sync_bool_compare_and_swap ((typeof(_oldval)*)(_ptr), _oldval, _newval)
#define CAS2(_ptr, _oldval, _newval) CAS((dbl_ptr*)(_ptr), *((dbl_ptr *)(&(_oldval))), *((dbl_ptr *)(&(_newval))))
#define CAS2_BOOL(_ptr, _oldval, _newval) CAS_BOOL((dbl_ptr*)(_ptr), *((dbl_ptr *)(&(_oldval))), *((dbl_ptr *)(&(_newval))))

#define PAD_WRAP(_x) char __pad ## _x [CACHE_LINE_SIZE]
#define ALIGNED_MALLOC(_s)  ((void *)(memalign(CACHE_LINE_SIZE*2, (_s))))

#define ADD_TO(_v,_x) __sync_add_and_fetch(&(_v), (typeof(_v))(_x))

#endif /* BLAST_ATOMIC_H_ */
