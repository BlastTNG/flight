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
void radec2azel(double ra, double dec, time_t lst, double lat, double *az,
		double *el);
int pinIsIn();

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
#define CHECK_LAT 39.49
#define CHECK_LON 104.22
/*********************************************************************/
/*            Init Sched Structure                                   */
/*********************************************************************/
void InitSched(void) {
  FILE *fp;
  char line_in[162];
  double hours;
  struct tm ts;
  double ra,dec;
  double dt;
  double d_lon;
  int day;
  
  double az1, az2, el1, el2, rh;
  
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

  /*************************************************************/
  /** find local comoving siderial date (in siderial seconds) **/
  dt = (time(NULL)-S.t0)*1.002737909; /*Ref Siderial Time */
  d_lon = CHECK_LON;
  while (d_lon<0) d_lon+=360.0;
  while (d_lon>=360.0) d_lon-=360.0;
  dt -= ((d_lon) * 3600.00 * 24.00/360.0); /* add longitude correction */

  dt/=3600.0;
  
  printf("***********************************************************\n"
	 "***       Schedule File:\n"
	 "*** Current local siderial date (hours relative to epoch): %g\n"
	 "*** Assuming LAT = %g , LON = %g for checks\n", dt,
	 CHECK_LAT, CHECK_LON);
  
  /***********************/
  /*** Read the events ***/
  for (i=j=0; i<S.n_sched; i++) {
    entry_ok=1;
    GetLine(fp, line_in);

    switch (line_in[0]) {
    case 'v':
    case 'V':
      n_fields = sscanf(line_in, "%*s %d %lg %lg %lg %lg %lg %lg %lg",
			&day, &hours,
			&ra, &dec, &(S.p[j].w), &(S.p[j].h),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_VBOX;
      rh = S.p[j].h;
      if (n_fields != 8) entry_ok = 0;
      break;
    case 'b':
    case 'B':
      n_fields = sscanf(line_in, "%*s %d %lg %lg %lg %lg %lg %lg %lg",
			&day, &hours,
			&ra, &dec, &(S.p[j].w), &(S.p[j].h),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_BOX;
      rh = S.p[j].h;
      if (n_fields != 8) entry_ok = 0;
      break;
    case 'c':
    case 'C':
      n_fields = sscanf(line_in, "%*s %d %lg %lg %lg %lg %lg %lg",
			&day, &hours,
			&ra, &dec, &(S.p[j].w),
			&(S.p[j].vaz), &(S.p[j].del));
      S.p[j].mode = P_CAP;
      if (n_fields != 7) entry_ok = 0;
      break;
    default:
      break;
    }
    
    S.p[j].t = day*24l*3600l + hours*3600l;
    StarPos(GetJulian(S.t0), ra*(M_PI/12.0), dec*(M_PI/180.0),
	    0.0, 0.0, 0.0, 0.0, // proper motion, etc
	    &(S.p[j].X), &(S.p[j].Y));

    S.p[j].X*=12.0/M_PI;
    S.p[j].Y*=180.0/M_PI;

    if (!entry_ok) {
      printf("****** Warning Entry %d is Malformed: Skipping *****\n", j);
    } 
    if (entry_ok) j++;
  }
  if (fclose(fp) == EOF) {
    perror("sched: Error on close");
  }
  
  for (i=0; i<S.n_sched; i++) {
    radec2azel(S.p[i].X, S.p[i].Y, S.p[i].t, CHECK_LAT, &az1, &el1);
    if (i==S.n_sched-1) {
      radec2azel(S.p[i].X, S.p[i].Y, S.p[i].t, CHECK_LAT, &az2, &el2);
    } else {
      radec2azel(S.p[i].X, S.p[i].Y, S.p[i+1].t, CHECK_LAT, &az2, &el2);
    }

    if (S.p[i].mode == P_CAP) {
      rh = S.p[i].w;
    } else {
      rh = S.p[i].h;
    }

    if (el1>el2) {
      el1+= rh;
      if (el1> 60.0) printf("******************************************\n"
			    "*** Warning: El high\n");
      el2-= rh;
      if (el2 < 27.0) printf("******************************************\n"
			     "*** Warning: El low\n");
    } else {
      el1-= rh;
      el2+= rh;
      if (el2> 60.0) printf("******************************************\n"
			    "*** Warning: El high\n");
      if (el1 < 27.0) printf("******************************************\n"
			     "*** Warning: El low\n");
    }
    
    printf("*** %2d Az: %8.3f - %8.3f El: %8.3f - %8.3f\n", i,
	   az1, az2, el1, el2);
  }
  printf("***********************************************************\n");
}

void DoSched(void) {
  time_t t;
  double dt;
  double d_lon;
  static int last_is=-1;
  int i_sched, i_point;
  int i_dgps;

  i_point = GETREADINDEX(point_index);

  t = PointingData[i_point].t;
  if (t < CommandData.pointing_mode.t) {
    last_is = -1;
    return;
  }

  i_dgps = GETREADINDEX(dgpspos_index);
  if (DGPSPos[i_dgps].at_float) {
    if (pinIsIn()) {
      printf("unlocking pin\n");
      CommandData.pumps.lock_out = 1;
      CommandData.disable_az = 0;
      // Point North
      CommandData.pointing_mode.mode = P_RADEC_GOTO;
      CommandData.pointing_mode.X = 2.6139; /* ra */
      CommandData.pointing_mode.Y = 89.275; /* dec */
      CommandData.pointing_mode.w = 0;
      CommandData.pointing_mode.vaz = 0;
      CommandData.pointing_mode.del = 0;
      CommandData.pointing_mode.h = 0;
      // start autofocus
      CommandData.ISCState.abort = 1;
      CommandData.ISC_autofocus =300; 
      CommandData.old_ISC_focus = CommandData.ISCState.focus_pos;
      CommandData.ISCState.focus_pos = FOCUS_RANGE;
      // out of sched mode for a while
      CommandData.pointing_mode.t = t + 600;
      return;
    }
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
