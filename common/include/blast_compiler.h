/* 
 * blast_compiler.h: 
 *
 * This software is copyright 
 *  (C) 2013-2014 California State University, Sacramento
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Feb 18, 2015 by seth
 */

#ifndef BLAST_COMPILER_H_
#define BLAST_COMPILER_H_

#define likely(x)      __builtin_expect(!!(x), 1)
#define unlikely(x)    __builtin_expect(!!(x), 0)

#define tole(x) ((__force uint32_t) __constant_cpu_to_le32(x))
#define tobe(x) ((__force uint32_t) __constant_cpu_to_be32(x))

/* Are two types/vars the same type (ignoring qualifiers)? */
#ifndef __same_type
# define __same_type(a, b) __builtin_types_compatible_p(typeof(a), typeof(b))
#endif

#ifndef __compiletime_warning
# define __compiletime_warning(message)
#endif
#ifndef __compiletime_error
# define __compiletime_error(message)
# define __compiletime_error_fallback(condition) \
        do { ((void)sizeof(char[1 - 2 * condition])); } while (0)
#else
# define __compiletime_error_fallback(condition) do { } while (0)
#endif

#define __compiletime_assert(condition, msg, prefix, suffix)            \
        do {                                                            \
                bool __cond = !(condition);                             \
                extern void prefix ## suffix(void) __compiletime_error(msg); \
                if (__cond)                                             \
                        prefix ## suffix();                             \
                __compiletime_error_fallback(__cond);                   \
        } while (0)

#define _compiletime_assert(condition, msg, prefix, suffix) \
        __compiletime_assert(condition, msg, prefix, suffix)

/**
 * compiletime_assert - break build and emit msg if condition is false
 * @condition: a compile-time constant condition to check
 * @msg:       a message to emit if condition is false
 *
 * In tradition of POSIX assert, this macro will break the build if the
 * supplied condition is *false*, emitting the supplied error message if the
 * compiler has support to do so.
 */
#define compiletime_assert(condition, msg) \
        _compiletime_assert(condition, msg, __compiletime_assert_, __LINE__)

#define always_inline                   inline  __attribute__((always_inline))
#define __deprecated                    __attribute__((deprecated))
#define __packed                        __attribute__((packed))
#define __weak                          __attribute__((weak))

/*
 * From the GCC manual:
 *
 * Many functions have no effects except the return value and their
 * return value depends only on the parameters and/or global
 * variables.  Such a function can be subject to common subexpression
 * elimination and loop optimization just as an arithmetic operator
 * would be.
 * [...]
 */
#define __pure                          __attribute__((pure))
#define __aligned(x)                    __attribute__((aligned(x)))
#define __printf(a, b)                  __attribute__((format(printf, a, b)))
#define __scanf(a, b)                   __attribute__((format(scanf, a, b)))
#define  noinline                       __attribute__((noinline))
#define __attribute_const__             __attribute__((__const__))
#define __maybe_unused                  __attribute__((unused))
#define __always_unused                 __attribute__((unused))
/*
 * A trick to suppress uninitialized variable warning without generating any
 * code
 */
#define uninitialized_var(x) x = x


#endif /* BLAST_COMPILER_H_ */
