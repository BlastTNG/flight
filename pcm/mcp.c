/* mcp: the Spider master control program
 *
 * mcp.c: contains the main loop and creates all threads used by mcp
 *
 * This software is copyright (C) 2002-2011 University of Toronto
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/statvfs.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include <sys/select.h>
#include "bbc_pci.h"

#include "blast.h"
#include "command_list.h"
#include "command_struct.h"
#include "crc.h"
#include "mcp.h"
#include "pointing_struct.h"
#include "starpos.h"
#include "channels.h"
#include "mputs.h"
#include "flcdataswap.h"
#include "tx.h"
#include "hwpr.h"
#include "mceserv.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250 /* "frames" */
#define BI0_VETO_LENGTH 5 /* seconds */

// Used by higain and bi0.  Number of frames in buffer.
#define BI0_FRAME_BUFLEN (400)

#define MCE_RATE ( 25.0e6/(50.0*33.0*120.0) ) // 25 MHz/(rl*nr*fr) ~ 119 Hz
                                              // default, still required by pcm

/* Define global variables */
//flc_ip[0] = bitsy, flc_ip[1] = itsy, so that flc_ip[BitsyIAm] gives other flc
// so flc_ip[2] = {Bitsy_IP, Itsy_IP}
char* flc_ip[2] = {"192.168.1.251", "192.168.1.250"};

int bbc_fp = -1;
unsigned int debug = 0;
short int BitsyIAm;
struct ACSDataStruct ACSData;

unsigned int BBFrameIndex;
unsigned short* slow_data[FAST_PER_SLOW];
unsigned short* RxFrame;
pthread_t watchdog_id;

int StartupVeto = STARTUP_VETO_LENGTH + 1;
int UsefulnessVeto = 2*STARTUP_VETO_LENGTH;
int BLASTBusUseful = 0;

static int bi0_fp = -2;
static int Death = -STARTUP_VETO_LENGTH * 2;
static int RxFrameMultiplexIndex;

unsigned long int mccSlowCount[6]; // count time between MCE slow packets

extern short int InCharge; /* tx.c */
extern pthread_mutex_t mutex;       //commands.c

void Pointing();
void WatchPort(void*);
void WatchDGPS(void);
void Magnetometer(void);
void IntegratingStarCamera(void);
void WatchFIFO(void*);               //commands.c
void FrameFileWriter(void);         //framefile.c
void CompressionWriter(void);
void StageBus(void);
void openSC(void);

void InitialiseFrameFile(char);
void pushDiskFrame(unsigned short *RxFrame);
void ShutdownFrameFile();

void SunSensor(void);

void InitSched();

void updateSlowDL(); // common/slowdl.c

struct frameBuffer {
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
  unsigned short** slow_data_list[BI0_FRAME_BUFLEN];
  int has_bi0_padding;
};

static struct frameBuffer bi0_buffer;
struct frameBuffer hiGain_buffer;

time_t biphase_timer;
int biphase_is_on = 0;

struct chat_buf chatter_buffer;

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

void openMotors();    // motors.c
void closeMotors();

void openTable();     // table.cpp
void closeTable();

void startChrgCtrl(); // chrgctrl.c
void endChrgCtrl();

void startSync();     // sync_comms.c
void endSync();

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

static void SensorReader(void)
{
  int data;
  int nr;
  struct statvfs vfsbuf;
  int sensor_error = 0;

  FILE *stream;

  nameThread("Sensor");

  while (1) {
    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp1_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp1 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp1 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp2_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp2 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp2 from I2C bus");
      sensor_error = 5;
    }

    if ((stream = fopen("/sys/bus/i2c/devices/0-002d/temp3_input", "r"))
        != NULL) {
      if ((nr = fscanf(stream, "%i\n", &data)) == 1)
        CommandData.temp3 = data / 10;
      fclose(stream);
    } else {
      if (!sensor_error)
        berror(warning, "Cannot read temp3 from I2C bus");
      sensor_error = 5;
    }

    if (statvfs("/data", &vfsbuf))
      berror(warning, "Cannot stat filesystem");
    else {
      /* vfsbuf.f_bavail is the # of blocks, the blocksize is vfsbuf.f_bsize
       * which, in this case is 4096 bytes, so CommandData.df ends up in units
       * of 4000kb */
      CommandData.df = vfsbuf.f_bfree / 1000;
    }

    if (sensor_error)
      sensor_error--;

    usleep(100000);
  }
}

static void Chatter(void* arg)
{
  int fd;
  char ch;
  ssize_t ch_got;
  off_t fpos;

  fpos = *(off_t*)arg;

  nameThread("Chat");

  fd = open("/data/etc/spider/pcm.log", O_RDONLY|O_NONBLOCK);

  if (fd == -1)
  {
    bprintf(tfatal, "Failed to open /data/etc/spider/pcm.log (%d)\n", errno);
  }

  if (fpos == -1) {
    if (lseek(fd, -500, SEEK_END) == -1)
    {
      if (errno == EINVAL)
      {
	if (lseek(fd, 0, SEEK_SET) == -1)
	{
	  bprintf(tfatal, "Failed to rewind /data/etc/spider/pcm.log (%d)\n", errno);
	}
      } else {
	bprintf(tfatal, "Failed to seek /data/etc/spider/pcm.log (%d)\n", errno);
      }
    }
  } else {
    if (lseek(fd, fpos, SEEK_SET) == -1)
    {
      if (lseek(fd, 0, SEEK_END) == -1)
      {
	bprintf(tfatal, "Failed to rewind /data/etc/spider/pcm.log (%d)\n", errno);
      }
    }
  }

  while (read(fd, &ch, 1) == 1 && ch != '\n'); /* Find start of next message */

  chatter_buffer.reading = chatter_buffer.writing = 0;
      /* decimal 22 is "Synchronous Idle" in ascii */
  memset(chatter_buffer.msg, 22, sizeof(char) * FAST_PER_SLOW * 2 * 4);

  while (1)
  {
    if (chatter_buffer.writing != ((chatter_buffer.reading - 1) & 0x3))
    {
      ch_got = read(fd, chatter_buffer.msg[chatter_buffer.writing], 2 * FAST_PER_SLOW * sizeof(char));
      if (ch_got == -1)
      {
        bprintf(tfatal, "Error reading from /data/etc/spider/pcm.log (%d)\n", errno);
      }
      if (ch_got < (2 * FAST_PER_SLOW * sizeof(char)))
      {
        memset(&(chatter_buffer.msg[chatter_buffer.writing][ch_got]), 22, (2 * FAST_PER_SLOW * sizeof(char)) - ch_got);
      }
      chatter_buffer.writing = ((chatter_buffer.writing + 1) & 0x3);
    }
    usleep(100000);
  }
}

static void GetACS()
{
  double enc_mean_el, enc_diff_el, ofpch_gy, ofroll_gy, ofyaw_gy, ofaz_gy;
  double enc_raw_el_1; // PORT
  double enc_raw_el_2; // STARBOARD
  double enc_table;
  unsigned int uTab;
  double pss1_i1, pss1_i2, pss1_i3, pss1_i4;
  double pss2_i1, pss2_i2, pss2_i3, pss2_i4;
  double pss3_i1, pss3_i2, pss3_i3, pss3_i4;
  double pss4_i1, pss4_i2, pss4_i3, pss4_i4;
  double pss5_i1, pss5_i2, pss5_i3, pss5_i4;
  double pss6_i1, pss6_i2, pss6_i3, pss6_i4;
  double vel_rw;
  double res_piv;
  double bbc_rate; // BBC frame rate in Hz 
  double mce_rate; // MCE data rate in Hz
  double fr_nr_rl_product; // product of three sync box params
  double adc_rate; // BLASTbus ADC sample rate in Hz

  static struct BiPhaseStruct* ofPchgyAddr;
  static struct BiPhaseStruct* ifRollgyAddr;
  static struct BiPhaseStruct* ofYawgyAddr;
  static struct BiPhaseStruct* ofAzGyAddr;
  static struct BiPhaseStruct* velSerRWAddr;
  static struct BiPhaseStruct* resPivAddr;
  static struct BiPhaseStruct* v11PssAddr;
  static struct BiPhaseStruct* v21PssAddr;
  static struct BiPhaseStruct* v31PssAddr;
  static struct BiPhaseStruct* v41PssAddr;
  static struct BiPhaseStruct* v12PssAddr;
  static struct BiPhaseStruct* v22PssAddr;
  static struct BiPhaseStruct* v32PssAddr;
  static struct BiPhaseStruct* v42PssAddr;
  static struct BiPhaseStruct* v13PssAddr;
  static struct BiPhaseStruct* v23PssAddr;
  static struct BiPhaseStruct* v33PssAddr;
  static struct BiPhaseStruct* v43PssAddr;
  static struct BiPhaseStruct* v14PssAddr;
  static struct BiPhaseStruct* v24PssAddr;
  static struct BiPhaseStruct* v34PssAddr;
  static struct BiPhaseStruct* v44PssAddr;
  static struct BiPhaseStruct* v15PssAddr;
  static struct BiPhaseStruct* v25PssAddr;
  static struct BiPhaseStruct* v35PssAddr;
  static struct BiPhaseStruct* v45PssAddr;
  static struct BiPhaseStruct* v16PssAddr;
  static struct BiPhaseStruct* v26PssAddr;
  static struct BiPhaseStruct* v36PssAddr;
  static struct BiPhaseStruct* v46PssAddr;
  static struct BiPhaseStruct* encTableAddr;
  static struct BiPhaseStruct* elRaw1EncAddr; // Spider PORT el encoder
  static struct BiPhaseStruct* elRaw2EncAddr; // Souder STARBOARD el encoder

  static int firsttime = 1;
  if (firsttime) {
    firsttime = 0;
    ofPchgyAddr = GetBiPhaseAddr("ofpch_gy");
    ifRollgyAddr = GetBiPhaseAddr("ofroll_gy");
    ofYawgyAddr = GetBiPhaseAddr("ofyaw_gy");
    ofAzGyAddr = GetBiPhaseAddr("ofaz_gy");
    velSerRWAddr = GetBiPhaseAddr("vel_ser_rw");
    resPivAddr = GetBiPhaseAddr("res_piv");
    v11PssAddr = GetBiPhaseAddr("v1_1_pss");
    v21PssAddr = GetBiPhaseAddr("v2_1_pss");
    v31PssAddr = GetBiPhaseAddr("v3_1_pss");
    v41PssAddr = GetBiPhaseAddr("v4_1_pss");
    v12PssAddr = GetBiPhaseAddr("v1_2_pss");
    v22PssAddr = GetBiPhaseAddr("v2_2_pss");
    v32PssAddr = GetBiPhaseAddr("v3_2_pss");
    v42PssAddr = GetBiPhaseAddr("v4_2_pss");
    v13PssAddr = GetBiPhaseAddr("v1_3_pss");
    v23PssAddr = GetBiPhaseAddr("v2_3_pss");
    v33PssAddr = GetBiPhaseAddr("v3_3_pss");
    v43PssAddr = GetBiPhaseAddr("v4_3_pss");
    v14PssAddr = GetBiPhaseAddr("v1_4_pss");
    v24PssAddr = GetBiPhaseAddr("v2_4_pss");
    v34PssAddr = GetBiPhaseAddr("v3_4_pss");
    v44PssAddr = GetBiPhaseAddr("v4_4_pss");
    v15PssAddr = GetBiPhaseAddr("v1_5_pss");
    v25PssAddr = GetBiPhaseAddr("v2_5_pss");
    v35PssAddr = GetBiPhaseAddr("v3_5_pss");
    v45PssAddr = GetBiPhaseAddr("v4_5_pss");
    v16PssAddr = GetBiPhaseAddr("v1_6_pss");
    v26PssAddr = GetBiPhaseAddr("v2_6_pss");
    v36PssAddr = GetBiPhaseAddr("v3_6_pss");
    v46PssAddr = GetBiPhaseAddr("v4_6_pss");
    encTableAddr = GetBiPhaseAddr("enc_table");
    elRaw1EncAddr = GetBiPhaseAddr("el_raw_1_enc");
    elRaw2EncAddr = GetBiPhaseAddr("el_raw_2_enc");
  }

  //enc_raw_el = (((double)RxFrame[elRawEncAddr->channel])/DEG2I);

  enc_raw_el_1 = ReadCalData(elRaw1EncAddr);
  enc_raw_el_2 = ReadCalData(elRaw2EncAddr);

  if (CommandData.use_elenc1 && !CommandData.use_elenc2) {
    enc_mean_el = enc_raw_el_1;
    enc_diff_el = 0.0;
  } else if (!CommandData.use_elenc1 && CommandData.use_elenc2) {
    enc_mean_el = enc_raw_el_2;
    enc_diff_el = 0.0;
  } else if (CommandData.use_elenc1 && CommandData.use_elenc2) {
    /* use mean encoder position */
    enc_mean_el = (enc_raw_el_1 + enc_raw_el_2)/2.0; 
    enc_diff_el = enc_raw_el_1 - enc_raw_el_2;
  } else {
    /*don't update ACSData -- deal with this problem in pointing.c */
    enc_diff_el = ACSData.enc_diff_el;
    enc_mean_el = ACSData.enc_mean_el;
  }

  vel_rw = ReadCalData(velSerRWAddr); 
  ofpch_gy = ReadCalData(ofPchgyAddr); //(double)((RxFrame[ofPchgyAddr->channel])-GY16_OFFSET)*GY16_TO_DPS;
  ofroll_gy = ReadCalData(ifRollgyAddr); //(double)(RxFrame[ifRollgyAddr->channel]-GY16_OFFSET)*GY16_TO_DPS;
  ofyaw_gy = ReadCalData(ofYawgyAddr); //(double)(RxFrame[ofYawgyAddr->channel]-GY16_OFFSET)*GY16_TO_DPS;

  ofaz_gy = ReadCalData(ofAzGyAddr);

  res_piv = (((double)
        ((short)slow_data[resPivAddr->index][resPivAddr->channel]))/DEG2I);

  pss1_i1 = (double)(slow_data[v11PssAddr->index][v11PssAddr->channel]);
  pss1_i2 = (double)(slow_data[v21PssAddr->index][v21PssAddr->channel]);
  pss1_i3 = (double)(slow_data[v31PssAddr->index][v31PssAddr->channel]);
  pss1_i4 = (double)(slow_data[v41PssAddr->index][v41PssAddr->channel]);
  pss2_i1 = (double)(slow_data[v12PssAddr->index][v12PssAddr->channel]);
  pss2_i2 = (double)(slow_data[v22PssAddr->index][v22PssAddr->channel]);
  pss2_i3 = (double)(slow_data[v32PssAddr->index][v32PssAddr->channel]);
  pss2_i4 = (double)(slow_data[v42PssAddr->index][v42PssAddr->channel]);
  pss3_i1 = (double)(slow_data[v13PssAddr->index][v13PssAddr->channel]);
  pss3_i2 = (double)(slow_data[v23PssAddr->index][v23PssAddr->channel]);
  pss3_i3 = (double)(slow_data[v33PssAddr->index][v33PssAddr->channel]);
  pss3_i4 = (double)(slow_data[v43PssAddr->index][v43PssAddr->channel]);
  pss4_i1 = (double)(slow_data[v14PssAddr->index][v14PssAddr->channel]);
  pss4_i2 = (double)(slow_data[v24PssAddr->index][v24PssAddr->channel]);
  pss4_i3 = (double)(slow_data[v34PssAddr->index][v34PssAddr->channel]);
  pss4_i4 = (double)(slow_data[v44PssAddr->index][v44PssAddr->channel]);
  pss5_i1 = (double)(slow_data[v15PssAddr->index][v15PssAddr->channel]);
  pss5_i2 = (double)(slow_data[v25PssAddr->index][v25PssAddr->channel]);
  pss5_i3 = (double)(slow_data[v35PssAddr->index][v35PssAddr->channel]);
  pss5_i4 = (double)(slow_data[v45PssAddr->index][v45PssAddr->channel]);
  pss6_i1 = (double)(slow_data[v16PssAddr->index][v16PssAddr->channel]);
  pss6_i2 = (double)(slow_data[v26PssAddr->index][v26PssAddr->channel]);
  pss6_i3 = (double)(slow_data[v36PssAddr->index][v36PssAddr->channel]);
  pss6_i4 = (double)(slow_data[v46PssAddr->index][v46PssAddr->channel]);

  uTab = (RxFrame[encTableAddr->channel+1] << 16 | 
      RxFrame[encTableAddr->channel]);
  if ((enc_table = ((double)uTab * (360.0 / 144000.0) + ENC_TABLE_OFFSET)) < 0)
    enc_table += 360;
  else if (enc_table > 360.0) enc_table -= 360;

  if (CommandData.bbcIsExt) {
    fr_nr_rl_product = ( CommandData.sync_box.rl_value 
        * CommandData.sync_box.nr_value 
        * CommandData.sync_box.fr_value );     

    /* MCE rate = 25 MHz/(rl*nr*fr) */
    if (fr_nr_rl_product == 0) {
      mce_rate = MCE_RATE;
    } else {
      mce_rate = ( 25.0e6 / ((double)fr_nr_rl_product));
    }
    bbc_rate = (double) mce_rate/CommandData.bbcExtFrameRate;
    adc_rate = 5.0e6/384.0;
  } else {
    bbc_rate = (4.0e6/384.0) / CommandData.bbcIntFrameRate;
    adc_rate = 4.0e6/384.0;
  }

  ACSData.t = mcp_systime(NULL);
  ACSData.enc_mean_el = enc_mean_el;
  ACSData.enc_diff_el = enc_diff_el;
  ACSData.ofpch_gy = ofpch_gy;
  ACSData.ofroll_gy = ofroll_gy;
  ACSData.ofyaw_gy = ofyaw_gy;
  ACSData.ofaz_gy = ofaz_gy;
  ACSData.vel_rw = vel_rw;
  ACSData.res_piv = res_piv;
  ACSData.pss1_i1 = pss1_i1;
  ACSData.pss1_i2 = pss1_i2;
  ACSData.pss1_i3 = pss1_i3;
  ACSData.pss1_i4 = pss1_i4;
  ACSData.pss2_i1 = pss2_i1;
  ACSData.pss2_i2 = pss2_i2;
  ACSData.pss2_i3 = pss2_i3;
  ACSData.pss2_i4 = pss2_i4;
  ACSData.pss3_i1 = pss3_i1;
  ACSData.pss3_i2 = pss3_i2;
  ACSData.pss3_i3 = pss3_i3;
  ACSData.pss3_i4 = pss3_i4;
  ACSData.pss4_i1 = pss4_i1;
  ACSData.pss4_i2 = pss4_i2;
  ACSData.pss4_i3 = pss4_i3;
  ACSData.pss4_i4 = pss4_i4;
  ACSData.pss5_i1 = pss5_i1;
  ACSData.pss5_i2 = pss5_i2;
  ACSData.pss5_i3 = pss5_i3;
  ACSData.pss5_i4 = pss5_i4;
  ACSData.pss6_i1 = pss6_i1;
  ACSData.pss6_i2 = pss6_i2;
  ACSData.pss6_i3 = pss6_i3;
  ACSData.pss6_i4 = pss6_i4;
  // so it can be read in one atomic cycle...
  ACSData.enc_table = enc_table;
  ACSData.bbc_rate = bbc_rate;
  ACSData.adc_rate = adc_rate;
}

/* sole purpose of following function is to add a field that reads the total current */

static void GetCurrents()
{

  double i_trans;
  double i_das;
  double i_acs;
  double i_mcc;
  double i_sc;
  double i_dgps;
  double i_step;
  double i_flc;
  double i_rw;
  double i_el;
  double i_piv;
  double i_tot;

  static struct BiPhaseStruct* i_transAddr;
  static struct BiPhaseStruct* i_dasAddr;
  static struct BiPhaseStruct* i_acsAddr;
  static struct BiPhaseStruct* i_mccAddr;
  static struct BiPhaseStruct* i_scAddr;
  static struct BiPhaseStruct* i_dgpsAddr;
  static struct BiPhaseStruct* i_stepAddr;
  static struct BiPhaseStruct* i_flcAddr;
  static struct BiPhaseStruct* i_rwAddr;
  static struct BiPhaseStruct* i_elAddr;
  static struct BiPhaseStruct* i_pivAddr;

  static struct NiosStruct* i_transNios;
  static struct NiosStruct* i_dasNios;
  static struct NiosStruct* i_acsNios;
  static struct NiosStruct* i_mccNios;
  static struct NiosStruct* i_scNios;
  static struct NiosStruct* i_dgpsNios;
  static struct NiosStruct* i_stepNios;
  static struct NiosStruct* i_flcNios;
  static struct NiosStruct* i_rwNios;
  static struct NiosStruct* i_elNios;
  static struct NiosStruct* i_pivNios;

  static struct NiosStruct* i_totNios;

  static int firsttime = 1;

  if (firsttime) {

    firsttime = 0;

    i_transAddr = GetBiPhaseAddr("i_trans");
    i_dasAddr = GetBiPhaseAddr("i_das");
    i_acsAddr = GetBiPhaseAddr("i_acs");
    i_mccAddr = GetBiPhaseAddr("i_mcc");
    i_scAddr = GetBiPhaseAddr("i_sc");
    i_dgpsAddr = GetBiPhaseAddr("i_dgps");
    i_stepAddr = GetBiPhaseAddr("i_step");
    i_flcAddr = GetBiPhaseAddr("i_flc");
    i_rwAddr = GetBiPhaseAddr("i_rw");
    i_elAddr = GetBiPhaseAddr("i_el");
    i_pivAddr = GetBiPhaseAddr("i_piv");

    i_transNios = GetNiosAddr("i_trans");
    i_dasNios = GetNiosAddr("i_das");
    i_acsNios = GetNiosAddr("i_acs");
    i_mccNios = GetNiosAddr("i_mcc");
    i_scNios = GetNiosAddr("i_sc");
    i_dgpsNios = GetNiosAddr("i_dgps");
    i_stepNios = GetNiosAddr("i_step");
    i_flcNios  = GetNiosAddr("i_flc");
    i_rwNios = GetNiosAddr("i_rw");
    i_elNios = GetNiosAddr("i_el");
    i_pivNios = GetNiosAddr("i_piv");

    i_totNios = GetNiosAddr("i_of_tot");

  }

  i_trans = (double)(slow_data[i_transAddr->index][i_transAddr->channel])*i_transNios->m + i_transNios->b;
  i_das = (double)(slow_data[i_dasAddr->index][i_dasAddr->channel])*i_dasNios->m + i_dasNios->b;
  i_acs = (double)(slow_data[i_acsAddr->index][i_acsAddr->channel])*i_acsNios->m + i_acsNios->b;
  i_mcc = (double)(slow_data[i_mccAddr->index][i_mccAddr->channel])*i_mccNios->m + i_mccNios->b;
  i_sc = (double)(slow_data[i_scAddr->index][i_scAddr->channel])*i_scNios->m + i_scNios->b;
  i_dgps = (double)(slow_data[i_dgpsAddr->index][i_dgpsAddr->channel])*i_dgpsNios->m + i_dgpsNios->b;
  i_step = (double)(slow_data[i_stepAddr->index][i_stepAddr->channel])*i_stepNios->m + i_stepNios->b;
  i_flc = (double)(slow_data[i_flcAddr->index][i_flcAddr->channel])*i_flcNios->m + i_flcNios->b;
  i_rw = (double)(slow_data[i_rwAddr->index][i_rwAddr->channel])*i_rwNios->m + i_rwNios->b;
  i_el = (double)(slow_data[i_elAddr->index][i_elAddr->channel])*i_elNios->m + i_elNios->b;
  i_piv = (double)(slow_data[i_pivAddr->index][i_pivAddr->channel])*i_pivNios->m + i_pivNios->b;

  /* check to see if the elevation drive is powered on */
  if (i_el > 0.3) {
    CommandData.power.elmot_is_on = 1;
  } else {
    CommandData.power.elmot_is_on = 0;
  }

  i_tot = i_trans + i_das + i_acs + i_mcc + i_sc + i_dgps + i_step + i_flc + i_rw + i_el + i_piv;

  WriteData(i_totNios, 1000*i_tot, NIOS_QUEUE);

}

/* setup the bbcpci device in internal or external (sync box) mode
*/
void setup_bbc()
{
  int setup_test = 0;
  static buos_t mode = startup;

  if (ioctl(bbc_fp, BBCPCI_IOC_OFF_IRQ) < 0) setup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_SYNC) < 0) setup_test = -1;
  usleep(100000);

  //external rates in units of DV pulse rate (200Hz on test box)
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE_EXT, 1) < 0) setup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_FRAME_RATE_EXT, CommandData.bbcExtFrameRate) < 0)
    setup_test = -1;

  //internal rates in units of bbc clock rate (4MHz), command in ADC samples
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE_INT,
        CommandData.bbcIntFrameRate*384) < 0) setup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_FRAME_RATE_INT,
        CommandData.bbcIntFrameRate*384) < 0) setup_test = -1;

  if (CommandData.bbcIsExt) {
    bprintf(mode, "External sync");
    if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_ON) < 0) setup_test = -1;
  } else {
    bprintf(mode, "Internal sync");
    if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_OFF) < 0) setup_test = -1;
  }

  //after startup, messges become warnings
  mode = warning;

  if (setup_test < 0)
    bprintf(fatal, "System: BBC failed to set synchronization mode");

}

void check_bbc_sync()
{
  CommandData.bbcExtFrameMeas = ioctl(bbc_fp, BBCPCI_IOC_FRAME_COUNT);

  if (CommandData.bbcAutoExt && CommandData.bbcIsExt
      && CommandData.bbcExtFrameMeas > BBC_SYNC_TIMEOUT) {
    bprintf(warning, "Sync box stall. Fallback to internal\n");
    CommandData.bbcIsExt = 0;
    setup_bbc();
  }

  if (CommandData.bbcAutoExt && !CommandData.bbcIsExt
      && CommandData.bbcExtFrameMeas < BBC_SYNC_TIMEOUT) {
    bprintf(warning, "Sync box alive. Will use it\n");
    CommandData.bbcIsExt = 1;
    setup_bbc();
  }
}

unsigned int read_from_bbc() {
  unsigned int in_data = 0;
  int size_read;
  int num_delay = 0;
  
  do {
    size_read = read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int));
    if (size_read<=0) {
      num_delay++;
      if (num_delay > 700) {
        check_bbc_sync();
        num_delay = 0;
      }
      usleep(1000);
    }
  } while (size_read <= 0);
  
  return in_data;
}

unsigned int old_read_from_bbc()
{
  unsigned int in_data = 0;
  int fd;
  fd_set rfds;
  struct timeval timeout = {.tv_sec = 1, .tv_usec = 0};

  do {
    FD_ZERO(&rfds);
    FD_SET(bbc_fp, &rfds);

    fd = select(bbc_fp + 1, &rfds, NULL, NULL, &timeout);

    if (fd < 0) {	    /* error */
      berror(err, "System: Error waiting for BBC read");
    } else if (!fd) { /* Timeout */
      check_bbc_sync();
    } else {
      if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
        berror(err, "System: Error on BBC read");
    }
  } while (!in_data);

  return in_data;
}

/* fill_Rx_frame: places one 32 bit word into the RxFrame. 
 * Returns true on success */
static int fill_Rx_frame(unsigned int in_data)
{
  static int n_not_found = 0;
  struct BiPhaseStruct BiPhaseData;

  if (in_data == 0xffffffff)
    return 1;

  /* discard ADC sync words */
  if (in_data & BBC_ADC_SYNC)
    return 1;

  /* words with no write flag are ignored, don't process them */
  if (~in_data & BBC_WRITE)
    return 1;

  /* BBC reverse address lookup */
  BiPhaseData = BiPhaseLookup[BI0_MAGIC(in_data)];

  /* Return error if the lookup failed */
  if (BiPhaseData.index == -1) {
    if (++n_not_found > 2)
      return 0;
    else
      return 1;
  }

  n_not_found = 0;

  /* Discard words we're not saving */
  if (BiPhaseData.index == DISCARD_WORD)
    return 1;

  /* Write the data to the local buffers */
  if (BiPhaseData.index == NOT_MULTIPLEXED)
    RxFrame[BiPhaseData.channel] = BBC_DATA(in_data);
  else
    slow_data[BiPhaseData.index][BiPhaseData.channel] = BBC_DATA(in_data);

  return(1);
}

static void WatchDog (void)
{
  nameThread("WDog");

  /* Allow other threads to kill this one at any time */
  pthread_setcanceltype(PTHREAD_CANCEL_ASYNCHRONOUS, NULL);

  if (ioperm(0x378, 0x0F, 1) != 0)
    berror(tfatal, "Error setting watchdog permissions");
  ioperm(0x80, 1, 1);

  for (;;) {
    outb(0xAA, 0x378);
    usleep(40000);
    outb(0x55, 0x378);
    usleep(40000);
  }
}

// Write to the bi-phase.
// the format is: 4 word header (0xeb90 c5c5 3a3a 146f) | the frame | 1 word crc
// The header has already been prepended in InitFrameBuffer()
// the CRC is only over the frame, and does not include the header or crc.
// the first word of the frame has been replaced with the length of the frame.

// make sure there is at least 1000 words of data in the buffer to avoid
// having the bi-0 writer run out of data while writing to the pci card.
#define BI0_PADDING_MIN 1000
static void write_to_biphase(unsigned short *frame)
{
  int i;
  static char *padding = 0;

  if (padding == 0) {
    padding = (char *)malloc(BI0_PADDING_MIN*sizeof(unsigned short));
    memset(padding, 0x55, BI0_PADDING_MIN*sizeof(unsigned short));
  }


  if (bi0_fp >= 0) { // should never be false!
    // bi0FifoSize holds number of words in the fifo buffer.
    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);
    if (CommandData.bi0FifoSize<BI0_PADDING_MIN) {
      i = write(bi0_fp, padding,
          (BI0_PADDING_MIN-CommandData.bi0FifoSize) * sizeof(unsigned short));
      if (i < 0)
        berror(err, "bi-phase write for padding failed");
    }
    CommandData.bi0FifoSize = ioctl(bi0_fp, BBCPCI_IOC_BI0_FIONREAD);

    //for (j=BiPhaseFrameWords-10; j<BiPhaseFrameWords+4; j++) frame[j] = j;
    //frame[BiPhaseFrameWords+4] = 17;
    frame[BiPhaseFrameWords+4] = CalculateCRC(0, frame+5, 
        sizeof(short)*(BiPhaseFrameWords-1));
    frame[4] = BiPhaseFrameWords;

    i = write(bi0_fp, frame, (BiPhaseFrameWords + 5) * sizeof(unsigned short));
    if (i < 0) {
      berror(err, "bi-phase write for frame failed");
    } else if (i != (BiPhaseFrameWords + 5) * sizeof(unsigned short)) {
      bprintf(err, "Short write for biphase frame: %i of %u", i,
          (unsigned int)(BiPhaseFrameWords * sizeof(unsigned short)));
    }
  }
}

static void InitFrameBuffer(struct frameBuffer *buffer, int has_bi0_padding) {
  int i,j;
  int words;

  buffer->has_bi0_padding = has_bi0_padding;
  buffer->i_in = 0;
  buffer->i_out = 0;

  words = BiPhaseFrameWords;
  if (has_bi0_padding) {
    words += 5;
  }
  for (i = 0; i<BI0_FRAME_BUFLEN; i++) {
    buffer->framelist[i] = balloc(fatal, words *
        sizeof(unsigned short));

    memset(buffer->framelist[i], 0, words*sizeof(unsigned short));

    if (has_bi0_padding) {
      buffer->framelist[i][0] = 0xeb90;
      buffer->framelist[i][1] = 0xc5c5;
      buffer->framelist[i][2] = 0x3a3a;
      buffer->framelist[i][3] = 0x146f;
    }

    buffer->slow_data_list[i] = (unsigned short **)balloc(fatal, FAST_PER_SLOW*sizeof(unsigned short *));
    for (j = 0; j < FAST_PER_SLOW; ++j) {
      buffer->slow_data_list[i][j] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
      memset(buffer->slow_data_list[i][j], 0, slowsPerBi0Frame * sizeof(unsigned short));
    }
  }
}

// warning: there is no checking for an overfull buffer.  Just make sure it is big
// enough that it can never happen.
static void PushFrameBuffer(struct frameBuffer *buffer) {
  int i, j, fw, i_in;
  int i0 = 0;

  if (buffer->has_bi0_padding) {
    i0 = 4;
  }

  i_in = buffer->i_in + 1;
  if (i_in>=BI0_FRAME_BUFLEN)
    i_in = 0;

  fw = BiPhaseFrameWords;

  for (i = 0; i<fw; i++) {
    buffer->framelist[i_in][i+i0] = RxFrame[i];
  }

  for (i=0; i<FAST_PER_SLOW; i++) {
    for (j=0; j<slowsPerBi0Frame; j++) {
      buffer->slow_data_list[i_in][i][j] = slow_data[i][j];
    }
  }

  buffer->i_in = i_in;
}

void ClearBuffer(struct frameBuffer *buffer) {
  buffer->i_out = buffer->i_in;
}

unsigned short *PopFrameBufferAndSlow(struct frameBuffer *buffer, unsigned short ***slow) {
  unsigned short *frame;
  int i_out = buffer->i_out;

  if (buffer->i_in == i_out) { // no data
    return (NULL);
  }
  frame = buffer->framelist[i_out];

  *slow = buffer->slow_data_list[i_out];

  i_out++;
  if (i_out>=BI0_FRAME_BUFLEN) {
    i_out = 0;
  }
  buffer->i_out = i_out;
  return (frame);
}

unsigned short *PopFrameBuffer(struct frameBuffer *buffer) {
  unsigned short *frame;
  int i_out = buffer->i_out;

  if (buffer->i_in == i_out) { // no data
    return (NULL);
  }
  frame = buffer->framelist[i_out];
  i_out++;
  if (i_out>=BI0_FRAME_BUFLEN) {
    i_out = 0;
  }
  buffer->i_out = i_out;
  return (frame);
}


static void zero()
{
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

static void BiPhaseWriter(void)
{
  unsigned short *frame;

  nameThread("Bi0");

  while (!biphase_is_on)
    usleep(10000);

  bi0_fp = open("/dev/bbc_bi0", O_RDWR);
  if (bi0_fp == -1) {
    berror(tfatal, "Error opening biphase device");
  }

  while (1) {
    frame = PopFrameBuffer(&bi0_buffer);

    if (!frame) { 
      /* Death meausres how long the BiPhaseWriter has gone without receiving
       * any data -- an indication that we aren't receiving FSYNCs from the
       * BLASTBus anymore */
      /* NB: Death should be over 1.25s, so check_bbc_sync can work */
      if (InCharge && (++Death > 150)) {
        bprintf(err, "Death is reaping the watchdog tickle.");
        pthread_cancel(watchdog_id);
      }
      usleep(10000); // 100 Hz
    } else {
      write_to_biphase(frame);
      if (Death > 0) {
        Death = 0;
      }
    }
  }
}


/******************************************************************/
/*                                                                */
/* IsNewFrame: returns true if d is a begining of frame marker,   */
/*    unless this is the first beginning of frame.                */
/*                                                                */
/******************************************************************/
static int IsNewFrame(unsigned int d)
{
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_FSYNC | BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | 0xEB90));
  if (is_bof && first_bof) {
    is_bof = first_bof = 0;
  }

  return is_bof;
}

/* Polarity crisis: am I north or south? */
static int AmIBitsy()
{
  char buffer[16];

  if (gethostname(buffer, 15) == -1 && errno != ENAMETOOLONG) {
    berror(err, "System: Unable to get hostname");
  }

#ifdef TEST_RUN
  //check that flight code is not compiled as a test run
  if (strncmp(buffer, "itsy", 15) == 0 || strncmp(buffer, "bitsy", 15) == 0)
    bputs(fatal, "Flight code can't be compiled with TEST_RUN\n");
#endif
  return (buffer[0] == 'b') ? 1 : 0;
}

/* Signal handler called when we get a hup, int or term */
static void CloseBBC(int signo)
{
  bprintf(err, "System: Caught signal %i; stopping NIOS", signo);
  closeMotors();
  closeTable(); //table.cpp

  endChrgCtrl();  // is this needed?
  endSync();
  RawNiosWrite(0, BBC_ENDWORD, NIOS_FLUSH);
  RawNiosWrite(BBCPCI_MAX_FRAME_SIZE, BBC_ENDWORD, NIOS_FLUSH);
  bprintf(err, "System: Closing BBC and Bi0");
  if (bi0_fp >= 0)
    close(bi0_fp);
  if (bbc_fp >= 0)
    close(bbc_fp);

  ShutdownFrameFile();

  /* restore default handler and raise the signal again */
  signal(signo, SIG_DFL);
  raise(signo);
}

void insertMCEData(unsigned short *RxFrame)
{
  static int no_data = 0;
  static int *offset = 0;
  static int mce_frameno;
  static int mce_mplex_offset;
  static int mce_index_offset;

  static int arraystats_data_offset;
  static int arraystats_index_offset;

  static int mce_blob_offset;

  static int i_tmp = 0;  
  static unsigned short mplex_index = 0;
  static unsigned short arraystats_index=0;

  static int mce_blob_pos = -1;
  static int mce_last_blob = 0; /* last blob serial number */

  const struct tes_frame *data;

  static struct NiosStruct *tesNfifoAddr;

  uint32_t D;

  if (offset == 0) {
    char mcefield[10];
    int i;
    struct BiPhaseStruct *bi0;

    tesNfifoAddr = GetNiosAddr("tes_nfifo");

    offset = (int *) malloc(NUM_MCE_FIELDS * sizeof(int));
    for (i=0; i< NUM_MCE_FIELDS; i++) {
      sprintf(mcefield, "mce%03d", i);
      bi0 = GetBiPhaseAddr(mcefield);
      offset[i] = bi0->channel;
    }

    bi0 = GetBiPhaseAddr("mce_frameno");
    mce_frameno = bi0->channel;

    bi0 = GetBiPhaseAddr("mce_mplex");
    mce_mplex_offset = bi0->channel;

    bi0 = GetBiPhaseAddr("mce_index");
    mce_index_offset = bi0->channel;

    bi0 = GetBiPhaseAddr("bolo_stats_index");
    arraystats_index_offset = bi0->channel;

    bi0 = GetBiPhaseAddr("bolo_stats_mplex");
    arraystats_data_offset = bi0->channel;

    bi0 = GetBiPhaseAddr("mce_blob");
    mce_blob_offset = bi0->channel;
  }
  i_tmp++;

  /* empty the fifo when asked */
  if (empty_tes_fifo) {
    while (tes_nfifo() > 1)
      tes_pop();
    empty_tes_fifo = 0;
  }

  mplex_index++;
  if (mplex_index>=N_MCE_STAT*NUM_MCE) 
    mplex_index = 0;

  D = mce_param[mplex_index];

  RxFrame[mce_index_offset] = mplex_index;
  RxFrame[mce_mplex_offset] = (unsigned short)(D & 0xFFFF);
  RxFrame[mce_mplex_offset + 1] = (unsigned short)(D >> 16);

  arraystats_index+=2;
  if (arraystats_index>NUM_ARRAY_STAT-2) arraystats_index = 0;

  RxFrame[arraystats_index_offset] = arraystats_index;
  RxFrame[arraystats_data_offset] = array_statistics[arraystats_index] | 
    (array_statistics[arraystats_index+1] << 8);

  /* MCE blobs -- the MCEserv will increment mce_blob_num if a new blob comes
   * in. If mce_blob_pos is -1, we're just idling with zeroes.  The
   * MCEserv also takes care of the payload CRC and adding the synchronisation
   * intro and outro (both defined in mceserv.h) */
  if (CommandData.mce_blob_num != mce_last_blob) {
    mce_last_blob = CommandData.mce_blob_num;
    mce_blob_pos = 0;
  }

  if (mce_blob_pos == -1)
    RxFrame[mce_blob_offset] = 0;
  else {
    RxFrame[mce_blob_offset] = mce_blob_envelope[mce_blob_pos++];
    if (mce_blob_pos >= mce_blob_size) /* done; start idling */
      /*  there's a race condition here.  Sometimes we won't notice new
       *  blobs... meh */
      mce_blob_pos = -1;
  }

  WriteData(tesNfifoAddr, tes_nfifo(), NIOS_QUEUE);
  if (tes_nfifo() > 0) {
    int i;
    no_data = 0;
    data = tes_data();
    /* write the 32-bit MCE frame number */
    RxFrame[mce_frameno] = (unsigned short)(data->frameno & 0xFFFF);
    RxFrame[mce_frameno + 1] = (unsigned short)(data->frameno >> 16);

    /* now the TES data */
    for (i = 0; i < NUM_MCE_FIELDS; i++) {
      RxFrame[offset[i]] = (unsigned short) (data->data[i]);
    }
    tes_pop();
  } else if (~mccs_reporting & ((1U << NUM_MCE) - 1)) {
    if (no_data++ > 10) {
      /* mccs_reporting is active low */
      bprintf(warning, "No MCEs reporting.");
      mccs_reporting = (1U << NUM_MCE) - 1;
    }
  }
}

int main(int argc, char *argv[])
{
  unsigned int in_data, i;
#ifdef USE_SIP_CMD
  pthread_t CommandDatacomm1;
  pthread_t CommandDatacomm2;
#endif
#ifdef USE_FIFO_CMD
  pthread_t CommandDatafifo;
#endif
  pthread_t disk_id;


  pthread_t compression_id;
  pthread_t bi0_id;
  pthread_t sensors_id;
  pthread_t dgps_id;
  pthread_t mag_id;
#ifdef USE_XY_THREAD
  pthread_t xy_id;
#endif
  pthread_t chatter_id;
  pthread_t hwpr_id;
  pthread_t mcesend_id;
  pthread_t mcerecv_id;
  off_t log_offset;

  if (argc == 1) {
    fprintf(stderr, "Must specify file type:\n"
        "p  pointing\n"
        "m  maps\n"
        "c  cryo\n"
        "n  noise\n"
        "x  software test\n"
        "f  flight\n");
    exit(0);
  }

  umask(0);  /* clear umask */

  /* register the output function */
  buos_use_func(mputs);
  nameThread("Main");

  /* openMCElog opens a log file, returning the offset to the start of the
   * new part of the log.  This function can be called multiple times to
   * open multiple log files.
   */
  log_offset = openMCElog("/data/etc/spider/pcm.log");

  /* Watchdog */
  pthread_create(&watchdog_id, NULL, (void*)&WatchDog, NULL);

  biphase_timer = mcp_systime(NULL) + BI0_VETO_LENGTH;

  /* Find out whether I'm itsy or bitsy */
  BitsyIAm = AmIBitsy();
  if (BitsyIAm) bputs(info, "I am Bitsy.\n");
  else bputs(info, "I am not Bitsy.\n");


#if (TEMPORAL_OFFSET > 0)
  bprintf(warning, "System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  //populate nios addresses, based off of tx_struct, derived
  MakeAddressLookups("/data/etc/spider/Nios.map");

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  if ((bbc_fp = open("/dev/bbcpci", O_RDWR | O_NONBLOCK)) < 0)
    berror(fatal, "System: Error opening BBC");
  setup_bbc();

  bprintf(info, "Command List Version: %s", command_list_serial);
#ifdef USE_FIFO_CMD
  pthread_create(&CommandDatafifo, NULL, (void*)&WatchFIFO, (void*)flc_ip[BitsyIAm]);
#endif
#ifdef USE_SIP_CMD
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchPort, (void*)0);
  pthread_create(&CommandDatacomm2, NULL, (void*)&WatchPort, (void*)1);
#endif

  /* Initialize the Ephemeris */
  ReductionInit("/data/etc/spider/ephem.2000");

  InitFrameBuffer(&bi0_buffer, 1);
  InitFrameBuffer(&hiGain_buffer, 0);

  memset(PointingData, 0, 3 * sizeof(struct PointingDataStruct));

  InitialiseFrameFile(argv[1][0]);
  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  /* Allocate the local data buffers */
  RxFrame = balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
    memset(slow_data[i], 0, slowsPerBi0Frame * sizeof(unsigned short));
  }

  pthread_create(&chatter_id, NULL, (void*)&Chatter, (void*)&log_offset);

  InitSched();
  openMotors();  //open communications with peripherals, creates threads
  // in motors.c
  openTable();	// opens communications and creates thread in table.cpp

#ifndef TEST_RUN //ethernet threads should start in test versions
  openSC();  // SC - creates threads in sc.cpp
#endif


  startChrgCtrl(); // create charge controller serial thread
  // defined in chrgctrl.c

  startSync();     // create sync box serial thread defined in sync_comms.c

  /* initialise the MPC protocol subsystem */
  if (mpc_init())
    bprintf(fatal, "Unable to initialise MPC protocol subsystem.");

  InitTxFrame();

#ifdef USE_XY_THREAD
  pthread_create(&xy_id, NULL, (void*)&StageBus, NULL);
#endif
  pthread_create(&dgps_id, NULL, (void*)&WatchDGPS, NULL);
  pthread_create(&mag_id, NULL, (void*)&Magnetometer, NULL);
  pthread_create(&sensors_id, NULL, (void*)&SensorReader, NULL);
  pthread_create(&compression_id, NULL, (void*)&CompressionWriter, NULL);
  pthread_create(&bi0_id, NULL, (void*)&BiPhaseWriter, NULL);
  pthread_create(&hwpr_id, NULL, (void*)&StartHWP, NULL);
#ifndef TEST_RUN
  pthread_create(&mcesend_id, NULL, (void*)&mcesend, NULL);
  pthread_create(&mcerecv_id, NULL, (void*)&mcerecv, NULL);
#endif

  start_flc_data_swapper(flc_ip[BitsyIAm]);

  while (1) {
    in_data = read_from_bbc();

    if (!fill_Rx_frame(in_data))
      bprintf(err, "System: Unrecognised word received from BBC (%08x)",
          in_data);

    if (IsNewFrame(in_data)) {
      if (UsefulnessVeto > 0) {
        UsefulnessVeto--;
        BLASTBusUseful = 0;
      } else BLASTBusUseful = 1;
      if (StartupVeto > 1) {
        --StartupVeto;
      } else {
        GetACS();
        GetCurrents();
        Pointing();

        check_bbc_sync();   /* check sync box aliveness */

        /* Frame sequencing check */
        if (StartupVeto) {
          bputs(info, "Startup Veto Ends\n");
          StartupVeto = 0;
          Death = 0;
        } else if (RxFrame[3] != (RxFrameMultiplexIndex + 1) % FAST_PER_SLOW
            && RxFrameMultiplexIndex >= 0) {
          bprintf(err, "System: Frame sequencing error detected: wanted %i, "
              "got %i\n", RxFrameMultiplexIndex + 1, RxFrame[3]);
        }
        RxFrameMultiplexIndex = RxFrame[3];

        /* Save current fastsamp */
        BBFrameIndex = (RxFrame[1] + RxFrame[2] * 0x10000);

        UpdateBBCFrame();
        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);
        insertMCEData(RxFrame);

        /* pushDiskFrame must be called before PushBi0Buffer to get the slow
           data right */
        pushDiskFrame(RxFrame);
        if (biphase_is_on) {
          PushFrameBuffer(&bi0_buffer);
          PushFrameBuffer(&hiGain_buffer);
        } else if (biphase_timer < mcp_systime(NULL)) {
          biphase_is_on = 1;
        }

        updateSlowDL();
        zero();
      }
    }
  }
  return(0);
}
