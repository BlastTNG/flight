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

#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <syslog.h>
#include <errno.h>
#include <string.h>
#include <pthread.h>
#include "blast.h"

void (*real_bputs)(blog_t, const char*) = NULL;

void bputs_stdio(blog_t l, const char* s)
{
  FILE* stream = stderr;

  switch (l) {
    case info:
    case startup:
    case sched:
      stream = stdout;
    case warning:
    case err:
    case tfatal:
    case fatal:
      fputs(s, stream);
      if (strstr(s, "\n") == NULL)
        fputs("\n", stream);
  }

  if (l == fatal)
    exit(1);
  else if (l == tfatal)
    pthread_exit(NULL);
}

void bputs_syslog(blog_t l, const char* s)
{
  int level = LOG_INFO;

  switch(l) {
    case info:
      level = LOG_INFO;
      break;
    case warning:
      level = LOG_WARNING;
      break;
    case err:
      level = LOG_ERR;
      break;
    case tfatal:
      level = LOG_CRIT;
      break;
    case fatal:
      level = LOG_ALERT;
      break;
    case startup:
      level = LOG_NOTICE;
      break;
    case sched:
      level = LOG_DEBUG;
      break;
  }

  syslog(level, "%s", s);

  if (l == fatal)
    exit(1);
  else if (l == tfatal)
    pthread_exit(NULL);
}

void bputs(blog_t l, const char* s)
{
  if (real_bputs)
    (*real_bputs)(l, s);
  else
    bputs_stdio(l, s);
}

void bprintf(blog_t l, const char* fmt, ...) {
  char message[BLOG_MAX];
  va_list argptr;

  va_start(argptr, fmt);
  vsnprintf(message, BLOG_MAX, fmt, argptr);
  va_end(argptr);

  bputs(l, message);
}

void berror(blog_t l, const char* fmt, ...) {
  char message[BLOG_MAX];
  va_list argptr;
  int error = errno;

  va_start(argptr, fmt);
  vsnprintf(message, BLOG_MAX, fmt, argptr);
  va_end(argptr);

  /* add a colon */
  strncat(message, ": ", BLOG_MAX - strlen(message));

  message[BLOG_MAX - 1] = '\0';
  /* copy error message into remainder of string -- Note: sterror is reentrant
   * despite what strerror(3) insinuates (and strerror_r is horribly b0rked) */
  strcat(message, strerror(error));
  message[BLOG_MAX - 1] = '\0';

  bputs(l, message);
}

void blog_use_func(void (*puts_func)(blog_t, const char*))
{
  real_bputs = puts_func;
}

void blog_use_syslog(void)
{
  real_bputs = bputs_syslog;
}

void blog_use_stdio(void)
{
  real_bputs = bputs_stdio;
}
