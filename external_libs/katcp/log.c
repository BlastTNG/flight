/* (c) 2010,2011 SKA SA */
/* Released under the GNU GPLv3 - see COPYING */

#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include <sys/time.h>

#include "katpriv.h"
#include "katcp.h"
#include "katcl.h"

/*************************************************************/

static int vector_sum(int *result, int size)
{
  int i, sum;

  sum = 0;
  for(i = 0; i < size; i++){
    if(result[i] < 0){
      return -1;
    }
    sum += result[i];
  }

  return sum;
}

/*************************************************************/

static char *log_levels_vector[KATCP_MAX_LEVELS] = {
  "trace", 
  "debug", 
  "info", 
  "warn", 
  "error", 
  "fatal",
  "off"
};

int log_to_code_katcl(char *name)
{
  int code;

  if(name == NULL){
    return -1;
  }

  for(code = 0; code < KATCP_MAX_LEVELS; code++){
    if(!strncmp(name, log_levels_vector[code], 4)){
      return code;
    }
  }

  /* ugly and pointless all alias */
  if(!strncmp(name, "all", 4)){
    return KATCP_LEVEL_TRACE;
  }

  return -1;
}

char *log_to_string_katcl(int code)
{
  if((code < 0) || (code >= KATCP_MAX_LEVELS)){
    return NULL;
  }

  return log_levels_vector[code];
}

int sync_message_katcl(struct katcl_line *cl, int level, char *name, char *fmt, ...)
{
  va_list args;
  int result;

  if(cl == NULL){
    return -1;
  }

  va_start(args, fmt);
  result = vlog_message_katcl(cl, level, name, fmt, args);
  va_end(args);

  if(result < 0){
    return -1;
  }

  while((result = write_katcl(cl)) == 0);

  if(result < 0){
    return -1;
  }

  return 0;
}

int vlog_parse_katcl(struct katcl_parse *px, int level, char *name, char *fmt, va_list args)
{
  int result[5];

  char *subsystem, *logstring;

  if((level >= KATCP_LEVEL_OFF) || (level < 0)){
#ifdef DEBUG
    fprintf(stderr, "log: bad form to a message of level off or worse\n");
#endif
    return -1;
  }

  logstring = log_to_string_katcl(level);
  if(logstring == NULL){
#ifdef KATCP_CONSISTENCY_CHECKS
    fprintf(stderr, "log: using unknown log level %d in log function call\n", level);
    abort();
#endif
    return -1;
  }

  subsystem = name ? name : "unknown" ;


  result[0] = add_string_parse_katcl(px, KATCP_FLAG_FIRST, KATCP_LOG_INFORM);
  result[1] = add_string_parse_katcl(px, 0, logstring);
  result[2] = add_timestamp_parse_katcl(px, 0, NULL);
  result[3] = add_string_parse_katcl(px, 0, subsystem);

#if DEBUG > 1
  fprintf(stderr, "log: my fmt string is <%s>, milli=%u\n", fmt, milli);
#endif

  result[4] = add_vargs_parse_katcl(px, KATCP_FLAG_LAST, fmt, args);

  return vector_sum(result, 5);
}

int log_message_katcl(struct katcl_line *cl, int level, char *name, char *fmt, ...)
{
  va_list args;
  int result;

  if(cl == NULL){
    return -1;
  }

  va_start(args, fmt);
  result = vlog_message_katcl(cl, level, name, fmt, args);
  va_end(args);

  return result;
}

int vlog_message_katcl(struct katcl_line *cl, int level, char *name, char *fmt, va_list args)
{
  struct katcl_parse *px;
  int result;

  px = create_referenced_parse_katcl();
  if(px == NULL){
    return -1;
  }

  result = vlog_parse_katcl(px, level, name, fmt, args);
  if(result < 0){
    destroy_parse_katcl(px);
    return -1;
  }

  if(append_parse_katcl(cl, px) < 0){
    destroy_parse_katcl(px);
    return -1;
  }

  /* WARNING: we count on append_parse to hang onto it ... */
  destroy_parse_katcl(px);

  return result;
}

#if 0
int basic_inform_katcl(struct katcl_line *cl, char *name, char *arg)
{
  int result[2];

  if(arg){
    result[0] = append_string_katcl(cl, KATCP_FLAG_FIRST, name);
    result[1] = append_string_katcl(cl, KATCP_FLAG_LAST, arg);
    return vector_sum(result, 2);
  } else {
    return append_string_katcl(cl, KATCP_FLAG_FIRST | KATCP_FLAG_LAST, name);
  }
}
#endif

int extra_response_katcl(struct katcl_line *cl, int code, char *fmt, ...)
{
  va_list args;
  int result;

  va_start(args, fmt);
  result = vextra_response_katcl(cl, code, fmt, args);
  va_end(args);

  return result;
}

int vextra_response_katcl(struct katcl_line *cl, int code, char *fmt, va_list args)
{
  /* WARNING: this function may not fit... we had it disabled for a while */

  int result[3];
  char *name, *status;

  if(code > KATCP_RESULT_OK){
    return -1;
  }

  name = arg_copy_string_katcl(cl, 0);
  if(name == NULL){
    return -1;
  }
  name[0] = KATCP_REPLY;

  status = code_to_name_katcm(code);
  if(status == NULL){
    free(name);
    return -1;
  }

  result[0] = append_string_katcl(cl, KATCP_FLAG_FIRST, name);
  free(name);

  if(fmt == NULL){
    result[1] = append_string_katcl(cl, KATCP_FLAG_LAST, status);
    return vector_sum(result, 2);
  } else {
    result[1] = append_string_katcl(cl, 0, status);
    result[2] = append_vargs_katcl(cl, KATCP_FLAG_LAST, fmt, args);
    return vector_sum(result, 3);
  }

}

