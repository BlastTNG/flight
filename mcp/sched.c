#include <stdio.h>
#include <time.h>
#include <string.h>

#include "command_struct.h"
#include "pointing_struct.h"

#define SCHEDULEFILE "/data/etc/schedule.mcp"

#define MAX_LINE_LENGTH 120
#define MAX_NSCHED 8000
struct ScheduleType S;
void StarPos(double t, double ra0, double dec0, double mra, double mdec,
	     double pi, double rvel, double *ra, double *dec);
double GetJulian(time_t t);


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
  double ra,dec;

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
  S.p =
    (struct PointingModeStruct *)
    malloc(S.n_sched*sizeof(struct PointingModeStruct));
  if (S.p == NULL)
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

  printf("JD: %.4f\n", GetJulian(S.t0));
  
  /***********************/
  /*** Read the events ***/
  for (i=j=0; i<S.n_sched; i++) {
    entry_ok=1;
    GetLine(fp, line_in);

    switch (line_in[0]) {
    case 'v':
    case 'V':
      n_fields = sscanf(line_in, "%*s %ld %ld:%ld:%ld %lg %lg %lg %lg %lg",
			&day, &hr, &min, &sec,
			&ra, &dec, &(S.p[j].w),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_VCAP;
      if (n_fields != 9) entry_ok = 0;
      break;
    case 'b':
    case 'B':
      n_fields = sscanf(line_in, "%*s %ld %ld:%ld:%ld %lg %lg %lg %lg %lg %lg",
			&day, &hr, &min, &sec,
			&ra, &dec, &(S.p[j].w), &(S.p[j].h),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_BOX;
      if (n_fields != 10) entry_ok = 0;
      break;
    case 'c':
    case 'C':
      n_fields = sscanf(line_in, "%*s %ld %ld:%ld:%ld %lg %lg %lg %lg %lg",
			&day, &hr, &min, &sec,
			&ra, &dec, &(S.p[j].w),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_CAP;
      if (n_fields != 9) entry_ok = 0;
      break;
    default:
      break;
    }
    
    S.p[j].t = day*24l*3600l + hr*3600l + min*60l + sec;
    StarPos(GetJulian(S.t0), ra*(M_PI/12.0), dec*(M_PI/180.0),
	    0.0, 0.0, 0.0, 0.0, // proper motion, etc
	    &(S.p[j].X), &(S.p[j].Y));

    S.p[j].X*=12.0/M_PI;
    S.p[j].Y*=180.0/M_PI;
    
    printf("%d %d ra: %.4f %.4f  dec: %.4f %.4f\n", entry_ok, S.p[j].mode,
	   ra, S.p[j].X, dec, S.p[j].Y);
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
  if (t < CommandData.pointing_mode.t) {
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
  while ((dt>S.p[i_sched].t) && (i_sched<S.n_sched-1)) i_sched++;
  while ((dt<S.p[i_sched].t) && (i_sched>0)) i_sched--;

  if (i_sched!=last_is) {
    /************************************************/
    /** Copy scheduled scan mode into current mode **/
    CommandData.pointing_mode.mode = S.p[i_sched].mode;
    CommandData.pointing_mode.X = S.p[i_sched].X;
    CommandData.pointing_mode.Y = S.p[i_sched].Y;
    CommandData.pointing_mode.w = S.p[i_sched].w;
    CommandData.pointing_mode.h = S.p[i_sched].h;
    CommandData.pointing_mode.vaz = S.p[i_sched].vaz;
    CommandData.pointing_mode.del = S.p[i_sched].del;
  }
  last_is = i_sched;
}
