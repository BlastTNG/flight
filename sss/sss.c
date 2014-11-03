/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/

#define _GNU_SOURCE
#include <gsl/gsl_multimin.h>
#include <sys/types.h>
#include <stdio.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>
#include <time.h>
#include <math.h>

#define NP 3 //number of parameters to fit
#define NOAS 1 //Number of samples On A Side of the fit

#define PI180 (3.14159265 / 180.0)

//#define VERBOSE     //Print a bunch of crap to the screen
//#define FAKE_DATA   //Generate fake data instead of reading from the mpc driver
//#define CALIBRATED_SENSOR_OUT  //Send calibrated module values instead of uncalibrated values
//#define READ_OLD_DATA  //Read in old data as if it came from the mpc.
//#define NO_NET         //Don't listen on some port; just print out the data to stdout.

#include "sss.h"
#include "mpc.h"
#include "mpc_funcs.h"
#include "sss_struct.h"
#include "net.h"
#include "lut.h"

double module_calibration[] =
{
  5842.0 / 5842.0,
  5842.0 / 8816.0,
  5842.0 / 7147.0,
  5842.0 / 8951.0,
  5842.0 / 5914.0,
  5842.0 / 5769.0,
  5842.0 / 6790.0,
  5842.0 / 5761.0,
  5842.0 / 8989.0,
  5842.0 / 5928.0,
  5842.0 / 7140.0,
  5842.0 / 6360.0
};

double module_offsets[] =
{
  360.,
  330.,
  300.,
  270.,
  240.,
  210.,
  180.,
  150.,
  120.,
  90.,
  60.,
  30.,
};

struct ls_func_struct
{
  double f[NOAS * 2 + 1];
};

double renorm90 (double x) //return a number between -45 and 45 degrees.
{
  if (x > 45)
    return renorm90(x - 90.);
  if (x < -45)
    return renorm90(x + 90.);
  return x;
}

double renorm360 (double x) //return the angle in the range [0,360).
{
  if (x < 0)
    return renorm360(x + 360.);
  if (x >= 360.)
    return renorm360(x - 360.);
  return x;
}

/* fill up the transmission packet with all the raw and housekeeping values */
void populate_packet(unsigned * sensor, unsigned * housekeeping, 
    struct timeb tp, sss_packet_data * dat)
{
  dat->m01 = sensor[0];
  dat->m02 = sensor[1];
  dat->m03 = sensor[2];
  dat->m04 = sensor[3];
  dat->m05 = sensor[4];
  dat->m06 = sensor[5];
  dat->m07 = sensor[6];
  dat->m08 = sensor[7];
  dat->m09 = sensor[8];
  dat->m10 = sensor[9];
  dat->m11 = sensor[10];
  dat->m12 = sensor[11];

#ifdef CALIBRATED_SENSOR_OUT
  dat->m01 *= module_calibration[0];
  dat->m02 *= module_calibration[1];
  dat->m03 *= module_calibration[2];
  dat->m04 *= module_calibration[3];
  dat->m05 *= module_calibration[4];
  dat->m06 *= module_calibration[5];
  dat->m07 *= module_calibration[6];
  dat->m08 *= module_calibration[7];
  dat->m09 *= module_calibration[8];
  dat->m10 *= module_calibration[9];
  dat->m11 *= module_calibration[10];
  dat->m12 *= module_calibration[11];
#endif

  dat->sun_time = (double)tp.time + ((double)tp.millitm / 1000.);

  dat->v5    = ((double)housekeeping[0] / SLOW_G * C2V_M + C2V_B) * 2;
  dat->v12   = ((double)housekeeping[1] / SLOW_G * C2V_M + C2V_B) * 2;
  dat->vbatt = ((double)housekeeping[2] / SLOW_G * C2V_M + C2V_B) * 4;

  dat->t_cpu       = ((double)housekeeping[3] / SLOW_G * C2V_M + C2V_B) * V2K;
  dat->t_hdd       = ((double)housekeeping[4] / SLOW_G * C2V_M + C2V_B) * V2K;
  dat->t_case      = ((double)housekeeping[5] / SLOW_G * C2V_M + C2V_B) * V2K;
  dat->t_port      = ((double)housekeeping[6] / SLOW_G * C2V_M + C2V_B) * V2K;
  dat->t_starboard = ((double)housekeeping[7] / SLOW_G * C2V_M + C2V_B) * V2K;
}

double ls_func (const gsl_vector *v, void *params)
{
  double chi = 0.0;
  double diff;
  double theta;
  double cos_theta;
  double fit_func;
  int i;
  double A = gsl_vector_get(v, 0);
  double phi = gsl_vector_get(v, 1);
  double dc = gsl_vector_get(v, 2);
  struct ls_func_struct * p = (struct ls_func_struct *) params;

  /* We fit to the function ( A * cos^2(theta) + dc )
     The first cos(theta) is for the amount of incident light on a flat surface
     at angle theta normal to the surface.
     The second cos(theta) is for the extra pathlength of filter you have to go
     through when you are at angle theta normal to the surface.
   */
  for (i = -1 * NOAS; i <= NOAS; i++)
  {
    theta = PI180 * (i * 30 + phi);
    cos_theta = cos(theta);
    fit_func = A * cos_theta * cos_theta + dc;
    diff = fit_func - p->f[i + NOAS];
#ifdef VERBOSE
    fprintf(stderr, "param: %e (%e) [%d]\n", p->f[i+NOAS], fit_func, i + NOAS);
#endif
    chi += diff * diff;
  }

  return chi;

}
/* calculate the az of the sun */
void calculate_az(unsigned * sensor_uint, sss_packet_data * dat)
{
  double sensors[N_FAST_CHAN];
  double x;
  double offset;
  int i;
  int sens_max = -1;

  //gsl stuff inits
  gsl_multimin_function fit_func;
  const gsl_multimin_fminimizer_type *T = gsl_multimin_fminimizer_nmsimplex;
  gsl_multimin_fminimizer *s;
  gsl_vector *v;
  gsl_vector *step_size;
  int iter;
  int status;
  double size = 0.0;
  struct ls_func_struct params;

  //Setup all gsl stuff
  fit_func.f = &ls_func;
  fit_func.n = NP;
  fit_func.params = (void *)&params;
  v  = gsl_vector_alloc(NP);
  step_size = gsl_vector_alloc(NP);
  s = gsl_multimin_fminimizer_alloc (T, NP);
  gsl_vector_set_all (step_size, 1.0);

  /* calibrate modules and determine module with max intensity */
  x = 0;
  for (i = 0; i < N_FAST_CHAN; i++)
  {
    sensors[i] = module_calibration[i] * (double)sensor_uint[i];
    if (sensors[i] > x)
    {
      x = sensors[i];
      sens_max = i;
    }
  }

  offset = module_offsets[sens_max];

  //fill the parameters with the sensor values in question, mindful of the wrap-around
  for(i = -1 * NOAS; i <= NOAS; i++)
  {
    params.f[i + NOAS] = sensors[(sens_max + i + 12) % 12];
  }

  //setup the initial values for the fit
  gsl_vector_set(v, 0, params.f[NOAS] - params.f[0]);  //amplitude
  gsl_vector_set(step_size, 0, 10.0);
  gsl_vector_set(v, 1, 0.0); //phase
  gsl_vector_set(step_size, 1, 0.5);
  gsl_vector_set(v, 2, params.f[0]); //dc offset
  gsl_vector_set(step_size, 2, 5.0);
  gsl_multimin_fminimizer_set (s, &fit_func, v, step_size);

  iter = 0;
  do
  {
    iter++;
    status = gsl_multimin_fminimizer_iterate(s);
    if (status) break;
    size = gsl_multimin_fminimizer_size(s);
    status = gsl_multimin_test_size (size, 1e-5);
#ifdef VERBOSE
    fprintf(stderr, "%d ", iter);
    for (i = 0; i < NP; i++)
    {
      fprintf(stderr, "%.13e ", gsl_vector_get(s->x, i));
    }
    fprintf(stderr, "%.13e %.13e\n", s->fval, size);
#endif
  } while (iter < MAX_ITERATIONS && status == GSL_CONTINUE);

  dat->amp = gsl_vector_get(s->x, 0);
  dat->phase = renorm90(gsl_vector_get(s->x, 1));
  dat->az_rel_sun = renorm360(dat->phase + offset);
  dat->dc_off = gsl_vector_get(s->x, 2);
  dat->chi = size;
  dat->iter = iter;
}

int main (void)
{
  int fp;

#ifdef READ_OLD_DATA
  double dtime = 0;
  size_t n = 0;
  char * line = NULL;
#endif

  struct mpc_channel_struct chan[NUM_CHANS];
  struct timeb tp;
  struct LutType sssAzLut = {"sss.lut",0,NULL,NULL,0};

  unsigned sensor[N_FAST_CHAN];
  unsigned housekeeping[N_SLOW_CHAN];

  sss_packet_data dat;

  fp = setupmpc(chan);

  LutInit(&sssAzLut);

  while (1) {  /* main loop */


/*  From here to #else is non-flight only */
#if defined FAKE_DATA  //We generate fake data with time
    sensor[0] = (10 + tp.time) % 120;
    sensor[1] = (20 + tp.time) % 120;
    sensor[2] = (30 + tp.time) % 120;
    sensor[3] = (40 + tp.time) % 120;
    sensor[4] = (50 + tp.time) % 120;
    sensor[5] = (60 + tp.time) % 120;
    sensor[6] = (55 + tp.time) % 120;
    sensor[7] = (45 + tp.time) % 120;
    sensor[8] = (35 + tp.time) % 120;
    sensor[9] = (25 + tp.time) % 120;
    sensor[10] = (15 + tp.time) % 120;
    sensor[11] = (tp.time) % 120;
#elif defined READ_OLD_DATA  //We read old data from stdin
    if (dtime == 0.0)
    {
#ifndef NO_NET
      SendData(&dat);
#endif
      sleep(10);
#ifndef NO_NET
      SendData(&dat);
#endif
    }

    if (getline(&line, &n, stdin) < 1)
    {
      fprintf(stderr, "End of file reached.  Have a nice day!\n");
      return 0;
    }

    sscanf(line, 
        "%lf %u %u %u %u %u %u %u %u %u %u %u %u %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*f %*e %*f",
        &dtime,
        &sensor[0], &sensor[1], &sensor[2], &sensor[3], &sensor[4], &sensor[5], 
        &sensor[6], &sensor[7], &sensor[8], &sensor[9], &sensor[10], &sensor[11]);

    tp.time = (time_t)dtime;
    tp.millitm = (unsigned short)((dtime - (double)tp.time) * 1000);
    //usleep(1);
#else  //We do what we should do; read from the mpc card.
    coaddfast(fp, chan, sensor);
    ftime(&tp);
    coaddslow(fp, chan, housekeeping);
#endif

    populate_packet(sensor, housekeeping, tp, &dat);

    calculate_az(sensor, &dat);
    dat.az_rel_sun = LutCal(&sssAzLut, dat.az_rel_sun);

/* From here to #else is non-flight only */
#ifdef NO_NET
    printf("%.3f", dat.sun_time);
    printf(" %u %u %u %u %u %u %u %u %u %u %u %u",
        dat.m01, dat.m02, dat.m03, dat.m04, dat.m05, dat.m06,
        dat.m07, dat.m08, dat.m09, dat.m10, dat.m11, dat.m12);
    printf(" %.3f %.3f %.3f", dat.v5, dat.v12, dat.vbatt);
    printf(" %.2f %.2f %.2f %.2f %.2f",
        dat.t_cpu, dat.t_hdd, dat.t_case, dat.t_port, dat.t_starboard);
    printf(" %.3f %.3f %.3f %.3f %.3e %u", dat.az_rel_sun, dat.amp,
        dat.dc_off, dat.phase, dat.chi, dat.iter);
    printf("\n");
#else //do what we should do; send the data out on the network.
    SendData(&dat);
#endif
  }
  return 1;
}
