/* blast.h: the BLAST flight code common header
 *
 * This software is copyright (C) 2004 University of Toronto
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

#include <stdarg.h>

#define BLOG_MAX 2048
/* logging definitions */
typedef enum {info, warning, err, tfatal, fatal, startup, sched} blog_t;

void bputs_stdio(blog_t l, const char* s);
void bputs_syslog(blog_t l, const char* s);
void bprintf(blog_t, const char*, ...);
void berror(blog_t, const char*, ...);
void bputs(blog_t, const char*);
void blog_use_func(void (*puts_func)(blog_t, const char*));
void blog_use_stdio(void);
void blog_use_syslog(void);

#endif
