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

int __buos_disable_exit = 0;
void (*__buos_real_bputs)(buos_t, const char*) = NULL;

void bputs_stdio(buos_t l, const char* s)
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

  if (!__buos_disable_exit) {
    if (l == fatal)
      exit(1);
    else if (l == tfatal)
      pthread_exit(NULL);
  }
}

void bputs_syslog(buos_t l, const char* s)
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

  if (!__buos_disable_exit) {
    if (l == fatal)
      exit(1);
    else if (l == tfatal)
      pthread_exit(NULL);
  }
}

void bputs(buos_t l, const char* s)
{
  if (__buos_real_bputs)
    (*__buos_real_bputs)(l, s);
  else
    bputs_stdio(l, s);
}

void bprintf(buos_t l, const char* fmt, ...) {
  char message[BUOS_MAX];
  va_list argptr;

  va_start(argptr, fmt);
  vsnprintf(message, BUOS_MAX, fmt, argptr);
  va_end(argptr);

  bputs(l, message);
}

void berror(buos_t l, const char* fmt, ...) {
  char message[BUOS_MAX];
  va_list argptr;
  int error = errno;

  va_start(argptr, fmt);
  vsnprintf(message, BUOS_MAX, fmt, argptr);
  va_end(argptr);

  /* add a colon */
  strncat(message, ": ", BUOS_MAX - strlen(message));

  message[BUOS_MAX - 1] = '\0';
  /* copy error message into remainder of string -- Note: sterror is reentrant
   * despite what strerror(3) insinuates (and strerror_r is horribly b0rked) */
  strcat(message, strerror(error));
  message[BUOS_MAX - 1] = '\0';

  bputs(l, message);
}

void buos_disable_exit(void)
{
  __buos_disable_exit = 1;
}

void buos_enable_exit(void)
{
  __buos_disable_exit = 0;
}

void buos_use_func(void (*puts_func)(buos_t, const char*))
{
  __buos_real_bputs = puts_func;
}

void buos_use_syslog(void)
{
  __buos_real_bputs = bputs_syslog;
}

void buos_use_stdio(void)
{
  __buos_real_bputs = bputs_stdio;
}
