#include <stdio.h>
#include <time.h>
#include <string.h>

#include "command_struct.h"
#include "pointing_struct.h"

#define SCHEDULEFILE "/data/etc/schedule.mcp"

#define MAX_LINE_LENGTH 120
#define MAX_NSCHED 8000
struct ScheduleType S;

/***************************************************************************/
/*    GetLine: read non-comment line from file                             */
/*        The line is placed   in *line.                                   */
/*        Returns 1 if succesful, 0 if unsuccesful                         */
/***************************************************************************/
int GetLine(FILE *fp, char *line) {
  char *ret_val;
  int first_char;

  do {
    ret_val=fgets(line, MAX_LINE_LENGTH, fp);
    first_char=0;
    while ((line[first_char]==' ') || (line[first_char]=='\t')) first_char++;
    line += first_char;
  } while (((line[0] =='#') || (strlen(line)<2)) && (ret_val!=NULL));

  if (ret_val!=NULL) {
    return (1); /* a line was read */
  } else {
    return(0);  /* there were no valid lines */
  }
}

/*********************************************************************/
/*            Init Sched Structure                                   */
/*********************************************************************/
void InitSched(void) {
  FILE *fp;
  char line_in[162];
  long day, hr, min, sec;
  struct tm ts;

  int i,j, entry_ok;
  int n_fields;

  /*******************************************/
  /*** Count number of schedule file lines ***/
  fp = fopen(SCHEDULEFILE,"r");
  if (fp==NULL) {
    S.n_sched=0;
    return;
  }
  while (GetLine(fp, line_in)) {
    S.n_sched++;
  }
  S.n_sched--; /* don't count date line */

  if (S.n_sched>MAX_NSCHED) S.n_sched = MAX_NSCHED;
  S.e = (struct EventType *)malloc(S.n_sched*sizeof(struct EventType));
  if (S.e == NULL)
    perror("sched: Unable to malloc");
  
  if (fclose(fp) == EOF) {
    perror("sched: Error on close");
  }

  /**************************/
  /*** Read Starting Time ***/
  fp = fopen(SCHEDULEFILE,"r");
  if (fp==NULL) {
    S.n_sched=0;
    return;
  }
  GetLine(fp, line_in);
  sscanf(line_in,"%d/%d/%d %d:%d:%d", &(ts.tm_mon), &(ts.tm_mday),
	 &(ts.tm_year),&(ts.tm_hour), &(ts.tm_min), &(ts.tm_sec));
  if (ts.tm_year<50) ts.tm_year += 100;
  else ts.tm_year-=1900;
  
  ts.tm_isdst = 0;
  ts.tm_mon--; /* Jan is 0 in struct tm.tm_mon, not 1 */

  S.t0 = mktime(&ts)-timezone; 

  /***********************/
  /*** Read the events ***/
  for (i=j=0; i<S.n_sched; i++) {
    entry_ok=1;
    GetLine(fp, line_in);

    n_fields = sscanf(line_in, "%ld %ld:%ld:%ld %lg %lg %lg %lg %lg",
		      &day, &hr, &min, &sec,
		      &(S.e[j].ra), &(S.e[j].dec), &(S.e[j].r),
		      &(S.e[j].el_vel), &(S.e[j].az_vel));
    if (n_fields != 9) entry_ok = 0;
    S.e[j].t = day*24l*3600l + hr*3600l + min*60l + sec;
    if (entry_ok) j++;
  }
  if (fclose(fp) == EOF) {
    perror("sched: Error on close");
  }
  
}

void DoSched(void) {
  time_t t;
  double dt;
  double d_lon;
  static int last_is=-1;
  int i_sched, i_point;

  i_point = GETREADINDEX(point_index);

  t = PointingData[i_point].t;
  if (t < CommandData.pointing_mode.t_start_sched) {
    last_is = -1;
    return;
  }
  
  /*************************************************************/
  /** find local comoving siderial date (in siderial seconds) **/
  dt = (PointingData[i_point].t-S.t0)*1.002737909; /*Ref Siderial Time */
  d_lon = PointingData[i_point].lon;
  while (d_lon<0) d_lon+=360.0;
  while (d_lon>=360.0) d_lon-=360.0;
  dt -= ((d_lon) * 3600.00 * 24.00/360.0); /* add longitude correction */

  /******************/
  /** find i_sched **/
  i_sched = last_is;
  if (i_sched<0) i_sched=0;
  while ((dt>S.e[i_sched].t) && (i_sched<S.n_sched-1)) i_sched++;
  while ((dt<S.e[i_sched].t) && (i_sched>0)) i_sched--;

  if (i_sched!=last_is) {
    /************************************************/
    /** Copy scheduled scan mode into current mode **/
    CommandData.pointing_mode.az_mode = POINT_RASTER;
    CommandData.pointing_mode.el_mode = POINT_RASTER;
    CommandData.pointing_mode.az1 = CommandData.pointing_mode.az2 = 0;
    CommandData.pointing_mode.el1 = CommandData.pointing_mode.el2 = 0;
    CommandData.pointing_mode.az_vel = S.e[i_sched].az_vel;
    CommandData.pointing_mode.el_vel = S.e[i_sched].el_vel;
    CommandData.pointing_mode.ra = S.e[i_sched].ra;
    CommandData.pointing_mode.dec = S.e[i_sched].dec;
    CommandData.pointing_mode.r = S.e[i_sched].r;
  }
  last_is = i_sched;
}
