/* blast.h: the BLAST flight code common header
 *
 * This software is copyright (C) 2004-2010 University of Toronto
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_BLAST_H
#define INCLUDE_BLAST_H

#include <stdlib.h>     /* free() */
#include <stdarg.h>     /* ANSI C variable arguments (va_list, va_start, va_end) */
#include <sys/types.h>  /* for size_t */
#include <string.h>     /* for memset */
#include <errno.h>      /* for errno */

/* Commonly used seed values (generally SYNC words for telemetry)*/
#define BLAST_MAGIC8    0xEB
#define BLAST_MAGIC16   0xEB90
#define BLAST_MAGIC32   0xEB90146F

/* BUOS (BLAST Unified Output Scheme) definitions */
#define BUOS_MAX 2048
typedef enum {none, info, warning, err, tfatal, fatal, startup, sched, mem}
  buos_t;

void bputs_stdio(buos_t l, const char* s);
void bputs_syslog(buos_t l, const char* s);
void bprintf(buos_t, const char*, ...) __attribute__((format(printf, 2, 3)));
void berror(buos_t, const char*, ...) __attribute__((format(printf, 2, 3)));
void bputs(buos_t, const char*);
void buos_use_func(void (*puts_func)(buos_t, const char*));
void buos_use_stdio(void);
void buos_use_syslog(void);
void buos_disallow_mem(void);
void buos_allow_mem(void);
void buos_disable_exit(void);
void buos_enable_exit(void);

#ifndef __FILENAME__
# define __FILENAME__ __FILE__
#endif

/* BLAMM (BLAST Memory Manager) definitions */
void *_balloc(buos_t, size_t, const char*, int, const char*) __attribute__((malloc));
void _bfree(buos_t, void*, const char*, int, const char*);
void _basprintf(buos_t, char**, const char*, const char*, int, const char*, ...)
                                 __attribute__((format(printf, 3, 7), noinline));
void *_reballoc(buos_t, void*, size_t, const char*, int, const char*);
char *_bstrdup(buos_t, const char*, const char*, int, const char*) __attribute__((malloc));
char *_bstrndup(buos_t, const char*, size_t n, const char*, int, const char*) __attribute__((malloc));
void *_memdup(buos_t l, const void *m_src, size_t n, const char* m_func, int m_line, const char *m_file)
                                                                                 __attribute__((malloc));

#define balloc(lvl, size) _balloc(lvl, size, __FUNCTION__ , __LINE__ , __FILENAME__ )
#define bfree(lvl, ptr) _bfree(lvl, ptr, __FUNCTION__ , __LINE__ , __FILENAME__ )
#define reballoc(lvl, ptr, newsize) _reballoc(lvl, ptr, newsize, __FUNCTION__ , __LINE__ , __FILENAME__)
#define bstrdup(lvl, ptr) _bstrdup(lvl , ptr , __FUNCTION__ , __LINE__ , __FILENAME__)
#define bstrndup(lvl, ptr, len) _bstrndup(lvl , ptr, len , __FUNCTION__ , __LINE__ , __FILENAME__)
#define basprintf(lvl, ptr, fmt, ...) _basprintf(lvl, ptr, fmt, __FUNCTION__ , __LINE__ , __FILENAME__, ##__VA_ARGS__)
#define memdup(level, ptr, size) _memdup(level, ptr, size, __FUNCTION__ , __LINE__ , __FILENAME__)

/** Free memory space */
#define BLAST_SAFE_FREE(_ptr) do { if (_ptr) {bfree(mem, _ptr); (_ptr) = NULL;} } while (0)

/** Zero an element */
#define BLAST_ZERO(x) memset((void*)&(x), 0, sizeof(x))

/** Zero dereferenced pointer */
#define BLAST_ZERO_P(x) do { if (x) memset((void*)(x), 0, sizeof(*(x))); } while (0)

#define blast_fatal(fmt, ...) \
    do {                                                                \
        bprintf(fatal, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_tfatal(fmt, ...) \
    do {                                                                \
        bprintf(tfatal, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_err(fmt, ...) \
    do {                                                                \
        bprintf(err, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_info(fmt, ...) \
    do {                                                                \
        bprintf(info, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_warn(fmt, ...) \
    do {                                                                \
        bprintf(warning, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_startup(fmt, ...) \
        do {                                                                \
            bprintf(startup, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_sched(fmt, ...) \
        do {                                                                \
            bprintf(sched, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#define blast_mem(fmt, ...) \
        do {                                                                \
            bprintf(mem, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#ifndef NDEBUG
#define blast_dbg(fmt, ...) \
        do {                                                                \
            bprintf(info, "%s:%d (%s):" fmt, __FILENAME__, __LINE__, __func__, ##__VA_ARGS__); \
        }while(0)
#else
#define blast_dbg(...)
#endif


/**
 * Prints the error message followed by an explanation of the errno code
 */
#define blast_strerror(fmt, ...) \
    do {                                                        \
        blast_err(fmt ":%s", ##__VA_ARGS__, strerror(errno));    \
    } while (0)

/**
 * Allocates a temporary, formated string on the stack.  This memory will be automatically freed
 * when the function exits, so do not call free or any variant on the pointer
 */
#define blast_tmp_sprintf(ptr, format, ...)                 \
    ({                                                      \
        int bytes;                                          \
                                                            \
        bytes = snprintf(NULL, 0, format, ##__VA_ARGS__)+1; \
        ptr = alloca(bytes * sizeof(char));                 \
        snprintf(ptr, bytes, format, ##__VA_ARGS__);        \
        bytes;                                              \
    })

/**
 * Allocates a temporary, formated string on the stack.  This memory will be automatically freed
 * when the function exits, so do not call free or any variant on the pointer
 */
#define blast_tmp_vsprintf(ptr, format, ...)                 \
    ({                                                       \
        int bytes;                                           \
                                                             \
        bytes = vsnprintf(NULL, 0, format, ##__VA_ARGS__)+1; \
        ptr = alloca(bytes * sizeof(char));                  \
        vsnprintf(ptr, bytes, format, ##__VA_ARGS__);        \
        bytes;                                               \
    })
#endif

#define BLAST_SWAP(_x, _y)          \
    do {                            \
        typeof(_x) _temp = (_x);    \
        (_x) = (_y);                \
        (_y) = _temp;               \
    } while (0)

/** Min/Max common use */
#undef max
#define max(a, b) ((a) >= (b) ? (a) : (b))
#undef min
#define min(a, b) ((a) <= (b) ? (a) : (b))

#define min_safe(x, y) ({                   \
    typeof(x) _min1 = (x);                  \
    typeof(y) _min2 = (y);                  \
    (void) (&_min1 == &_min2);              \
    _min1 < _min2 ? _min1 : _min2; })

#define max_safe(x, y) ({                   \
    typeof(x) _max1 = (x);                  \
    typeof(y) _max2 = (y);                  \
    (void) (&_max1 == &_max2);              \
    _max1 > _max2 ? _max1 : _max2; })
