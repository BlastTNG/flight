/**************************************************************
 * sss source code
 *
 * Copyright 2005 (C) Matthew Truch
 *
 * Released under the GPL
 *
 ***************************************************************/


#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/timeb.h>
#include <time.h>

#include "sss.h"
#include "mpc.h"
#include "mpc_funcs.h"

/*********************************************************
  Set up the MPC ADC card with all the appropriate one time
  settings
 **********************************************************/

int setupmpc(struct mpc_channel_struct * chan)
{
  int fp, x, i;

  fp = open("/dev/mpc", O_RDWR);
  if (fp < 0) {
    fprintf(stderr, "Error opening mpc (open() returned %d)\n", fp);
    fprintf(stderr, "Everything from here on out is bupkiss!\n");
    return 0; 
  }
  x=NUM_CHANS; 

  ioctl(fp, MPC_NUM_CHANNELS, x); //Set num channels
  ioctl(fp, MPC_RESET_AS); //Reset Auto Scan system 
  //(as we aren't using it) 

  x=16;
  ioctl(fp, MPC_SCAN_RATE, x); //Set scan rate 
  //(even though we aren't using it)

  x=1;
  ioctl(fp, MPC_EOC_IRQ, x); //Use IRQ for readback

  for(i = 0; i < N_FAST_CHAN; i++) { //First 24 channels are doubled into
    chan[i*2].active = 1;   //differential channels for the sensors
    chan[i*2].diff = 1;
    chan[i*2].gain_double = 0;
    chan[i*2].filter_low = 0;
    chan[i*2].channel_num = i*2;
    chan[i*2+1].active = 0;
    chan[i*2+1].diff = 0;
    chan[i*2+1].gain_double = 0;
    chan[i*2+1].filter_low = 0;
    chan[i*2+1].channel_num = i*2+1;
  }

  for(i = N_FAST_CHAN*2; i < NUM_CHANS; i++) { //remaining 8 channels
    //are single-ended
    chan[i].active = 1;                        //for housekeeping.
    chan[i].diff = 0;
    chan[i].gain_double = 0;
    chan[i].filter_low = 0;
    chan[i].channel_num = i;
  }

  for(i = 0; i < NUM_CHANS; i++) { //run through all the channels quickly
    ioctl(fp, MPC_SET_CHANNEL, &chan[i]);   //just to set them up.
  }

  return fp;
}

/* coadd the 'fast' channels.  These are the photo modules */

void coaddfast (int fp, struct mpc_channel_struct * chan, unsigned * sensors)
{
  int i, j, k;
  struct timespec tm;

  tm.tv_sec = 0;
  tm.tv_nsec = 1000000;

  for (i = 0; i < N_FAST_CHAN; i++) sensors[i] = 0;

  for (i = 0; i < N_OUTER_LOOP; i++)
  {
    for (j = 0; j < N_FAST_CHAN; j++)
    {
      ioctl(fp, MPC_SET_CHANNEL, &chan[j*2]);
      nanosleep(&tm, NULL);
      for (k = 0; k < N_INNER_LOOP; k++)
      {
        sensors[j] += ioctl(fp, MPC_BASIC_RUN);
      }
    }
  }
}

/* coadd the 'slow' channels.  These are the voltages and temperatures */

void coaddslow (int fp, struct mpc_channel_struct * chan, unsigned * housekeeping)
{
  int i, j, k;
  struct timespec tm;

  tm.tv_sec = 0;
  tm.tv_nsec = 1000000;

  for (i = 0; i < N_SLOW_CHAN; i++) housekeeping[i] = 0;

  for (i = 0; i < N_OUTER_LOOP_SLOW; i++)
  {
    for (j = 0; j < N_SLOW_CHAN; j++)
    {
      ioctl(fp, MPC_SET_CHANNEL, &chan[j + N_FAST_CHAN*2]);
      nanosleep(&tm, NULL);
      for (k = 0; k < N_INNER_LOOP_SLOW; k++)
      {
        housekeeping[j] += ioctl(fp, MPC_BASIC_RUN);
      }
    }
  }
}

