/*
 * pivotcommand.h: Header file for commands to be sent to 
 * the pivot motor controller on SPIDER.
 *
 */

#include <stdarg.h>  /* ANSI C variable arguments (va_list, va_start, va_end) */
struct PivotInfoStruct {
  int fd; // File descriptor for the pivot.
  int open; // 0 is closed, 1 is open
  int init; // 0 has not yet been initialized
            // 1 is has with no errors
            // -1 initialization was attempted but failed
};


void open_port(char *address);

void close_pivot();
