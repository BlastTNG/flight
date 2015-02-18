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

#ifndef BLAST_H
#define BLAST_H

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */
#include <sys/types.h>  /* for size_t */

/* If we're not using GNU C, elide __attribute__ */
#ifndef __GNUC__
#  define  __attribute__(x)  /*NOTHING*/
#endif

/* BUOS (BLAST Unified Output Scheme) definitions */
#define BUOS_MAX 2048
typedef enum {none, info, warning, err, tfatal, fatal, startup, sched, mem}
  buos_t;

void bputs_stdio(buos_t l, const char* s);
void bputs_syslog(buos_t l, const char* s);
void bprintf(buos_t, const char*, ...) __attribute__((format(printf,2,3)));
void berror(buos_t, const char*, ...) __attribute__((format(printf,2,3)));
void bputs(buos_t, const char*);
void buos_use_func(void (*puts_func)(buos_t, const char*));
void buos_use_stdio(void);
void buos_use_syslog(void);
void buos_disallow_mem(void);
void buos_allow_mem(void);
void buos_disable_exit(void);
void buos_enable_exit(void);

/* BLAMM (BLAST Memory Manager) definitions */
void *_balloc(buos_t, size_t, const char*, int, const char*);
void _bfree(buos_t, void*, const char*, int, const char*);
void *_reballoc(buos_t, void*, size_t, const char*, int, const char*);
char *_bstrdup(buos_t, const char*, const char*, int, const char*);
#define balloc(x,y) _balloc( x , y , __FUNCTION__ , __LINE__ , __FILE__ )
#define bfree(x,y) _bfree( x , y , __FUNCTION__ , __LINE__ , __FILE__ )
#define reballoc(x,y,z) \
  _reballoc( x , y , z , __FUNCTION__ , __LINE__ , __FILE__)
#define bstrdup(x,y) _bstrdup( x , y , __FUNCTION__ , __LINE__ , __FILE__)

#endif
