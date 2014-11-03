/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2003-2006 University of Toronto
 *
 * This file is part of mcp.
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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/select.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <time.h>
#include <sys/time.h>
#include <math.h>
#include <stddef.h>
#include <sys/ipc.h>
#include <sys/shm.h>

#include <ctype.h>

#include "pointing_struct.h"
#include "command_struct.h"
#include "mcp.h"

typedef float          htF32_t;
typedef double         htF64_t;
typedef signed char    htI08_t;
typedef signed short   htI16_t;
typedef signed long    htI32_t;
typedef unsigned char  htUI08_t;
typedef unsigned short htUI16_t;
typedef unsigned long  htUI32_t;
typedef int            htBool_t;
typedef signed char    htChar_t;

#define NTPD_BASE 0x4e545030 //shared memory segment key

#define htTrue         ((htBool_t)1)
#define htFalse        ((htBool_t)0)

#define GPSCOM1 "/dev/ttySI5" // Time, Position, Attitude in SBF
extern short int InCharge; /* tx.c */

/* min and max SBF IDs */
#define MIN_CMDID      128
#define MAX_CMDID      767

#define MIN_SBFID      4000 /* Range extended to support SBF 2.0 messages */
#define MAX_SBFID      6015

#define MAX_SBFSIZE    4096   /* maximum size of a SBF block in bytes */
#define MIN_SBFSIZE    8      /* minimum size of a SBF block in bytes */

#define HEADER_SIZE    8 /* size of the SBF header in bytes */

#define SBFID_PVTGEODETIC       5904
#define SBFID_RECEIVERTIME      5914
#define SBFID_ATTEULER          5938
#define SBFID_ATTCOVEULER       5939
#define SBFID_AUXPOS            5942
#define DONOTUSE  -2e10

#define BDRATE B115200
// #define SELECT_GPS_MUS_OUT  200000

void nameThread(const char*); /* mcp.c */

struct shmTime {
  int   mode;
  int   count;
  time_t  clockTimeStampSec; /* external clock */
  int   clockTimeStampUsec;
  time_t  receiveTimeStampSec; /* internal clock, when external value was received */
  int   receiveTimeStampUsec; 
  int     leap;
  int     precision;
  int     nsamples;
  int     valid;
  int     dummy[10];
};

typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;
} VoidBlock_t;

typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;

  htUI32_t            TOW;
  htUI16_t            WNc;

  htI08_t             UTCYear;
  htI08_t             UTCMonth;
  htI08_t             UTCDay;
  htI08_t             UTCHour;
  htI08_t             UTCMin;
  htI08_t             UTCSec;
  htI08_t             DeltaLS;
  htUI08_t            SyncLevel;

} ReceiverTimeBlock_t;

typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;

  htUI32_t            TOW;
  htUI16_t            WNc;
  htUI08_t            NrSV;
  htUI08_t            Error;
  htUI08_t            Mode;
  htUI08_t            System;
  htUI08_t            Info;
  htUI08_t            SBASprn;
  htF64_t             Lat;
  htF64_t             Lon;
  htF64_t             Alt;
  htF32_t             Vn;
  htF32_t             Ve;
  htF32_t             Vu;
  htF64_t             RxClkBias;
  htF32_t             RxClkDrift;
  htF32_t             GeoidHeight;
  htUI16_t            MeanCorrAge;
  htUI16_t            BaseStationID;
  htF32_t             Cog;
} PVTGeodeticBlock_t;

typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;

  htUI32_t            TOW;
  htUI16_t            WNc;
  htUI08_t            NrSV;
  htUI08_t            Error;

  htUI16_t            Mode;
  htUI16_t            Reserved;

  htF32_t             Heading;
  htF32_t             Pitch;
  htF32_t             Roll;

  htF32_t             omega_x;
  htF32_t             omega_y;
  htF32_t             omega_z;
} AttitudeEulerBlock_t;

typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;

  htUI32_t            TOW;
  htUI16_t            WNc;
  htUI08_t            Reserved;
  htUI08_t            Error;

  htF32_t             Cov_HeadHead;
  htF32_t             Cov_PitchPitch;
  htF32_t             Cov_RollRoll;
  htF32_t             Cov_HeadPitch;
  htF32_t             Cov_HeadRoll;
  htF32_t             Cov_PitchRoll;
} AttitudeCovEulerBlock_t;


typedef struct {
  htUI16_t            Sync;
  htUI16_t            CRC;
  htUI16_t            ID;
  htUI16_t            Length;

  htUI32_t            TOW;
  htUI16_t            WNc;

  htUI08_t            N;
  htUI08_t            SBLength;

  htUI08_t            NRSV;
  htUI08_t            Error;
  htUI08_t            AmbiguityType;
  htUI08_t            AuxAntID;

  htF64_t             DeltaEast;
  htF64_t             DeltaNorth;
  htF64_t             DeltaUp;
  htF64_t             EastVelocity;
  htF64_t             NorthVelocity;
  htF64_t             UpVelocity;
} AuxAntPositions_t;


/* SBF sync bytes */
static const char SYNC_STRING[3]="$@";

struct DGPSAttStruct DGPSAtt[3];
int dgpsatt_index = 0;

struct DGPSPosStruct DGPSPos[3];
int dgpspos_index = 0;

time_t DGPSTime;

struct DgpsInfoStruct {
  int open; // 0 is closed, 1 is open
  int init; // 0 has not yet been initialized
  // 1 has been initialized with no errors
  // 2 initialization was attempted but failed
} dgpsinfo;

#define LEAP_SECONDS 0

static const htUI16_t CRCLookUp[256] = {

  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
  0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
  0x1231, 0x0210, 0x3273, 0x2252, 0x52b5, 0x4294, 0x72f7, 0x62d6,
  0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c, 0xf3ff, 0xe3de,
  0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d,
  0x3653, 0x2672, 0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4,
  0xb75b, 0xa77a, 0x9719, 0x8738, 0xf7df, 0xe7fe, 0xd79d, 0xc7bc,
  0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861, 0x2802, 0x3823,
  0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12,
  0xdbfd, 0xcbdc, 0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a,
  0x6ca6, 0x7c87, 0x4ce4, 0x5cc5, 0x2c22, 0x3c03, 0x0c60, 0x1c41,
  0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b, 0x8d68, 0x9d49,
  0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78,
  0x9188, 0x81a9, 0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f,
  0x1080, 0x00a1, 0x30c2, 0x20e3, 0x5004, 0x4025, 0x7046, 0x6067,
  0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c, 0xe37f, 0xf35e,
  0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d,
  0x34e2, 0x24c3, 0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
  0xa7db, 0xb7fa, 0x8799, 0x97b8, 0xe75f, 0xf77e, 0xc71d, 0xd73c,
  0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676, 0x4615, 0x5634,
  0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3,
  0xcb7d, 0xdb5c, 0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a,
  0x4a75, 0x5a54, 0x6a37, 0x7a16, 0x0af1, 0x1ad0, 0x2ab3, 0x3a92,
  0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b, 0x9de8, 0x8dc9,
  0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8,
  0x6e17, 0x7e36, 0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

htUI16_t ComputeCRC(void * buf, size_t buf_length)
{
  htUI32_t  i;
  htUI16_t  crc = 0;
  htUI08_t *buf8 = buf;

  for (i=0; i<buf_length; i++) {
    crc = (crc << 8) ^ CRCLookUp[ (crc >> 8) ^ buf8[i] ];
  }

  return crc;
}

htBool_t CRCIsValid(void *Mess)
{
  VoidBlock_t * Mess2 = Mess;
  htUI16_t crc;

  crc = ComputeCRC( &(Mess2->ID), Mess2->Length-2*sizeof(htUI16_t) );

  return (crc ==  Mess2->CRC) ? htTrue:htFalse;
}



static htI32_t CheckBlock(int fd, htUI08_t* Buffer)
{
  VoidBlock_t* VoidBlock = (VoidBlock_t*)Buffer;

  /* Read the block header, remember that the first '$' is already present in Buffer[0]. */
  int hdrexp=HEADER_SIZE-1;
  int cnt=0;
  do {
    int retn=read(fd,&Buffer[1+cnt],1);
    if (retn<0) {
      return -1;
      //bprintf(err,"DGPS: no response");
    }
    cnt +=retn;
  } while (cnt < hdrexp);

  //printf("\nBuffer: %3u %3u %3u %3u\n",Buffer[0],Buffer[1],Buffer[2],Buffer[3]);
  //printf("Header: %3u %3u %3u\n\n",VoidBlock->Sync, VoidBlock->ID, VoidBlock->Length);
  /*Check block header*/
  if ((VoidBlock->Sync !=
        ((htUI16_t)SYNC_STRING[0] | (htUI16_t)SYNC_STRING[1]<<8))        ||
      (VoidBlock->ID       < MIN_CMDID)                                ||
      ((VoidBlock->ID      > MAX_CMDID) & (VoidBlock->ID < MIN_SBFID)) ||
      (VoidBlock->ID       > MAX_SBFID)                                ||
      (VoidBlock->Length   > MAX_SBFSIZE)                              ||
      (VoidBlock->Length   < MIN_SBFSIZE)) {
    //bprintf(err,"DGPS: Something wrong with block header");
    return -1;
  }

  /* Read one at a time */
  int expected=VoidBlock->Length-HEADER_SIZE;
  int count=0; 
  do { 
    int ret=read(fd,&Buffer[HEADER_SIZE+count],1); 
    if (ret<0) { 
      //      bprintf(err,"DGPS: no response");
      return -1; 
    } 
    count +=ret; 
  } while (count < expected); 

  //if (VoidBlock->ID == SBFID_PVTGEODETIC || VoidBlock->ID ==  SBFID_ATTEULER) {
  /* Check the CRC field */
  if (CRCIsValid(Buffer) == htFalse){
    //    bprintf(err,"DGPS: CRC invalid");
    return -1;
  }
  //}
  return 0;
}

htI32_t GetNextBlock(int fd, void* SBFBlock)
{
  htBool_t     BlockFound;
  htUI08_t     Buffer[MAX_SBFSIZE];
  VoidBlock_t* VBlock = (VoidBlock_t*)Buffer;

  /*struct timeval timeout;
      timeout.tv_sec =0;
      timeout.tv_usec = SELECT_GPS_MUS_OUT;
      int maxfd = fd+1;
      fd_set output;
      FD_ZERO(&output);
      FD_SET(fd,&output);
      select(maxfd, &output, NULL, NULL, &timeout);
      */
  BlockFound = htFalse;
  //if (FD_ISSET(fd,&output)) {
  do {
    //char c[1];
    //int n = read(fd,&c[0],1);
    char c;
    int n = read(fd,&c,1);
    if (n==0) usleep(10000);

    if (n<0) {
      //bprintf(err,"read GPSblock failed!");
    }

    //if (n>0 && c[0]==SYNC_STRING[0]) {
    //Buffer[0] = (htUI08_t)c[0];
    if (n>0 && c==SYNC_STRING[0]) {
      Buffer[0] = (htUI08_t)c;
      if (CheckBlock(fd, Buffer) == 0) {
        BlockFound = htTrue;
        memcpy(SBFBlock, Buffer, (size_t)VBlock->Length);
      }
    }
  } while (BlockFound == htFalse);
    //}
  return (BlockFound == htTrue) ? 0 : -1;

}

struct shmTime *getShmTime() {
  int shmid;
  if ((shmid = shmget (NTPD_BASE, sizeof (struct shmTime), IPC_CREAT|0660))== -1) {
    return 0;
  } else {
    struct shmTime *p=(struct shmTime *)shmat (shmid, 0, 0);
    if ((int)(long)p==-1) {
      return 0;
    }
    return p;
  }
}

int ntpshm_put(double fixtime) {
  static struct shmTime *shmTime = NULL;
  struct timeval tv;
  double seconds, microseconds;

  (void)gettimeofday(&tv,NULL);
  microseconds = 1000000.0 * modf(fixtime,&seconds);

  if (shmTime == NULL) {
    shmTime = getShmTime();
  }

  shmTime->valid =0;
  shmTime->count++;
  shmTime->clockTimeStampSec = (time_t)seconds;
  shmTime->clockTimeStampUsec = (int)microseconds;
  shmTime->receiveTimeStampSec = (time_t)tv.tv_sec;
  shmTime->receiveTimeStampUsec = (int)tv.tv_usec;
  shmTime->count++;
  shmTime->valid = 1;

  return 1;

}

void WatchDGPS()
{
  htUI08_t SBFBlock[MAX_SBFSIZE];
  VoidBlock_t *VBlock = (VoidBlock_t*)SBFBlock;
  double lat=0,lon=0,dir=0;
  struct tm ts;
  static int firsttime = 1;
  static struct BiPhaseStruct* dgpsAzAddr;
  static struct BiPhaseStruct* dgpsPitchAddr;
  static struct BiPhaseStruct* dgpsRollAddr;
  static struct BiPhaseStruct* dgpsAzCovAddr;
  static struct BiPhaseStruct* dgpsPitchCovAddr;
  static struct BiPhaseStruct* dgpsRollCovAddr; 
  static struct BiPhaseStruct* dgpsAntEAddr; 
  static struct BiPhaseStruct* dgpsAntNAddr; 
  static struct BiPhaseStruct* dgpsAntUAddr; 
  static struct BiPhaseStruct* attOkAddr;
  static struct BiPhaseStruct* dgpsLatAddr;
  static struct BiPhaseStruct* dgpsLonAddr;
  static struct BiPhaseStruct* dgpsAltAddr;
  static struct BiPhaseStruct* dgpsSpeedAddr;
  static struct BiPhaseStruct* dgpsDirAddr;
  static struct BiPhaseStruct* dgpsClimbAddr;
  static struct BiPhaseStruct* dgpsNsatAddr;
  static struct BiPhaseStruct* dgpsTimeAddr;
  nameThread("dGPS");

  int attempts = 0;
  /* initialize values in dgpsinfo structure */
  dgpsinfo.init = 0;

  if (firsttime) {
    firsttime = 0;
    dgpsAzAddr = GetBiPhaseAddr("az_raw_dgps");     
    dgpsPitchAddr = GetBiPhaseAddr("pitch_raw_dgps");
    dgpsRollAddr = GetBiPhaseAddr("roll_raw_dgps"); 
    dgpsAzCovAddr = GetBiPhaseAddr("az_cov_dgps");     
    dgpsPitchCovAddr = GetBiPhaseAddr("pitch_cov_dgps");
    dgpsRollCovAddr = GetBiPhaseAddr("roll_cov_dgps"); 
    dgpsAntEAddr = GetBiPhaseAddr("ant_e_dgps"); 
    dgpsAntNAddr = GetBiPhaseAddr("ant_n_dgps"); 
    dgpsAntUAddr = GetBiPhaseAddr("ant_u_dgps"); 
    attOkAddr = GetBiPhaseAddr("att_ok");
    dgpsLatAddr = GetBiPhaseAddr("lat_dgps");       
    dgpsLonAddr = GetBiPhaseAddr("lon_dgps");      
    dgpsAltAddr = GetBiPhaseAddr("alt_dgps");
    dgpsSpeedAddr = GetBiPhaseAddr("speed_dgps");   
    dgpsDirAddr = GetBiPhaseAddr("dir_dgps");       
    dgpsClimbAddr = GetBiPhaseAddr("climb_dgps");   
    dgpsNsatAddr = GetBiPhaseAddr("n_sat_dgps");
    dgpsTimeAddr = GetBiPhaseAddr("time_dgps");
  } 

  /* Initialize values in DGPSAtt and DGPSPos structures */
  DGPSAtt[0].az = 0;
  DGPSAtt[0].pitch = 0;
  DGPSAtt[0].roll = 0;
  DGPSAtt[0].az_cov = 0;
  DGPSAtt[0].pitch_cov = 0;
  DGPSAtt[0].roll_cov = 0;
  DGPSAtt[0].ant_E = 0;
  DGPSAtt[0].ant_N = 0;
  DGPSAtt[0].ant_U = 0;
  DGPSAtt[0].att_ok = 0;
  dgpsatt_index = 1;

  DGPSPos[0].lat = 0;
  DGPSPos[0].lon = 0;
  DGPSPos[0].alt = 0;
  DGPSPos[0].speed = 0;
  DGPSPos[0].direction = 0;
  DGPSPos[0].climb = 0;
  DGPSPos[0].n_sat = 0;
  dgpspos_index = 1;

  DGPSTime = 0;

  while(!InCharge) {
    DGPSAtt[0].az = (((double)slow_data[dgpsAzAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[1].az = (((double)slow_data[dgpsAzAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[2].az = (((double)slow_data[dgpsAzAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[0].pitch = (((double)slow_data[dgpsPitchAddr->index][dgpsPitchAddr->channel])/DEG2I);
    DGPSAtt[1].pitch = (((double)slow_data[dgpsPitchAddr->index][dgpsPitchAddr->channel])/DEG2I);
    DGPSAtt[2].pitch = (((double)slow_data[dgpsPitchAddr->index][dgpsPitchAddr->channel])/DEG2I);
    DGPSAtt[0].roll = (((double)slow_data[dgpsRollAddr->index][dgpsRollAddr->channel])/DEG2I);
    DGPSAtt[1].roll = (((double)slow_data[dgpsRollAddr->index][dgpsRollAddr->channel])/DEG2I);
    DGPSAtt[2].roll = (((double)slow_data[dgpsRollAddr->index][dgpsRollAddr->channel])/DEG2I);
    DGPSAtt[0].az_cov = (((double)slow_data[dgpsAzCovAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[1].az_cov = (((double)slow_data[dgpsAzCovAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[2].az_cov = (((double)slow_data[dgpsAzCovAddr->index][dgpsAzAddr->channel])/DEG2I);
    DGPSAtt[0].pitch_cov = (((double)slow_data[dgpsPitchCovAddr->index][dgpsPitchCovAddr->channel])/DEG2I);
    DGPSAtt[1].pitch_cov = (((double)slow_data[dgpsPitchCovAddr->index][dgpsPitchCovAddr->channel])/DEG2I);
    DGPSAtt[2].pitch_cov = (((double)slow_data[dgpsPitchCovAddr->index][dgpsPitchCovAddr->channel])/DEG2I);
    DGPSAtt[0].roll_cov = (((double)slow_data[dgpsRollCovAddr->index][dgpsRollCovAddr->channel])/DEG2I);
    DGPSAtt[1].roll_cov = (((double)slow_data[dgpsRollCovAddr->index][dgpsRollCovAddr->channel])/DEG2I);
    DGPSAtt[2].roll_cov = (((double)slow_data[dgpsRollCovAddr->index][dgpsRollCovAddr->channel])/DEG2I);
    DGPSAtt[0].ant_E = (((double)slow_data[dgpsAntEAddr->index][dgpsAntEAddr->channel])/100);
    DGPSAtt[1].ant_E = (((double)slow_data[dgpsAntEAddr->index][dgpsAntEAddr->channel])/100);
    DGPSAtt[2].ant_E = (((double)slow_data[dgpsAntEAddr->index][dgpsAntEAddr->channel])/100);
    DGPSAtt[0].ant_N = (((double)slow_data[dgpsAntNAddr->index][dgpsAntNAddr->channel])/100);
    DGPSAtt[1].ant_N = (((double)slow_data[dgpsAntNAddr->index][dgpsAntNAddr->channel])/100);
    DGPSAtt[2].ant_N = (((double)slow_data[dgpsAntNAddr->index][dgpsAntNAddr->channel])/100);
    DGPSAtt[0].ant_U = (((double)slow_data[dgpsAntUAddr->index][dgpsAntUAddr->channel])/100);
    DGPSAtt[1].ant_U = (((double)slow_data[dgpsAntUAddr->index][dgpsAntUAddr->channel])/100);
    DGPSAtt[2].ant_U = (((double)slow_data[dgpsAntUAddr->index][dgpsAntUAddr->channel])/100);
    DGPSAtt[0].att_ok = (slow_data[attOkAddr->index][attOkAddr->channel]) & 0x01;
    DGPSAtt[1].att_ok = (slow_data[attOkAddr->index][attOkAddr->channel]) & 0x01;
    DGPSAtt[2].att_ok = (slow_data[attOkAddr->index][attOkAddr->channel]) & 0x01;
    DGPSPos[0].lat = (((double)slow_data[dgpsLatAddr->index][dgpsLatAddr->channel])/DEG2I);
    DGPSPos[1].lat = (((double)slow_data[dgpsLatAddr->index][dgpsLatAddr->channel])/DEG2I);
    DGPSPos[2].lat = (((double)slow_data[dgpsLatAddr->index][dgpsLatAddr->channel])/DEG2I);
    DGPSPos[0].lon = (((double)slow_data[dgpsLonAddr->index][dgpsLonAddr->channel])/DEG2I);
    DGPSPos[1].lon = (((double)slow_data[dgpsLonAddr->index][dgpsLonAddr->channel])/DEG2I);
    DGPSPos[2].lon = (((double)slow_data[dgpsLonAddr->index][dgpsLonAddr->channel])/DEG2I);
    DGPSPos[0].alt = ((double)slow_data[dgpsAltAddr->index][dgpsAltAddr->channel]);
    DGPSPos[1].alt = ((double)slow_data[dgpsAltAddr->index][dgpsAltAddr->channel]);
    DGPSPos[2].alt = ((double)slow_data[dgpsAltAddr->index][dgpsAltAddr->channel]);
    DGPSPos[0].speed = (((double)slow_data[dgpsSpeedAddr->index][dgpsSpeedAddr->channel])/100);
    DGPSPos[1].speed = (((double)slow_data[dgpsSpeedAddr->index][dgpsSpeedAddr->channel])/100);
    DGPSPos[2].speed = (((double)slow_data[dgpsSpeedAddr->index][dgpsSpeedAddr->channel])/100);
    DGPSPos[0].direction = (((double)slow_data[dgpsDirAddr->index][dgpsDirAddr->channel])/DEG2I);
    DGPSPos[1].direction = (((double)slow_data[dgpsDirAddr->index][dgpsDirAddr->channel])/DEG2I);
    DGPSPos[2].direction = (((double)slow_data[dgpsDirAddr->index][dgpsDirAddr->channel])/DEG2I);
    DGPSPos[0].climb = (((double)slow_data[dgpsClimbAddr->index][dgpsClimbAddr->channel])/100);
    DGPSPos[1].climb = (((double)slow_data[dgpsClimbAddr->index][dgpsClimbAddr->channel])/100);
    DGPSPos[2].climb = (((double)slow_data[dgpsClimbAddr->index][dgpsClimbAddr->channel])/100);
    DGPSPos[0].n_sat = (slow_data[dgpsNsatAddr->index][dgpsNsatAddr->channel]);
    DGPSPos[1].n_sat = (slow_data[dgpsNsatAddr->index][dgpsNsatAddr->channel]);
    DGPSPos[2].n_sat = (slow_data[dgpsNsatAddr->index][dgpsNsatAddr->channel]);
    DGPSTime = (slow_data[dgpsTimeAddr->index][dgpsTimeAddr->channel]); 
    usleep(20000);
  }

  int fd;
  struct termios term;

  //try to open the port
  dgpsinfo.open = 0;
  while (dgpsinfo.open == 0) {
    //if ((fd = open(GPSCOM1, O_RDWR | O_NOCTTY | O_NDELAY)) < 0) {
    if ((fd = open(GPSCOM1, O_RDWR | O_NOCTTY)) < 0) {
      attempts++;
      usleep(20000);
    } else {
      dgpsinfo.open = 1;
      if (attempts>=10) bprintf(info,"open on attempt #%i",attempts);
    }
  }

  //Set options
  tcgetattr(fd,&term);
  cfsetispeed(&term,BDRATE);
  cfsetospeed(&term,BDRATE);

  /*Control Modes*/
  term.c_cflag |= (CLOCAL | CREAD); //local connection, no modem control; enable receiving characters   
  term.c_cflag &= ~PARENB;   // No Parity
  term.c_cflag &= ~CSTOPB;   // 1 Stop Bit 
  term.c_cflag &= ~CSIZE;    // Mask the character size bits
  term.c_cflag |= CS8;       // 8 data bits
  term.c_cflag &= ~CRTSCTS;  //flow control off (no RTS/CTS)
  /*Local Modes*/
  term.c_lflag =0;
  //term.c_cc[VTIME]=0;
  //term.c_cc[VMIN]=0;
  //term.c_lflag |= ICANON;    // enable canonical (line-based) input
  term.c_lflag &= ~ICANON;    // disable canonical (line-based) input
  /*Input Modes*/
  //term.c_iflag = 0;
  //term.c_iflag = ICRNL;      // map CR to NL on input
  /*Output Modes*/
  term.c_oflag = 0;

  cfmakeraw(&term);

  /*Activate settings for the port*/
  tcsetattr(fd,TCSANOW,&term);

  /*Read in SBF data blocks*/
  while (GetNextBlock(fd, SBFBlock) == 0) {
    /* Time */
    if (VBlock->ID == SBFID_RECEIVERTIME) {
      ReceiverTimeBlock_t* RXTIME = (ReceiverTimeBlock_t*) SBFBlock;
      /*bprintf(info,"TIME: %-2i %13.1f %3i %3i %3i %3i %3i %3i\n",
        -5,
        RXTIME->WNc*86400.0*7.0+RXTIME->TOW/1000.0,
        RXTIME->UTCYear,
        RXTIME->UTCMonth,
        RXTIME->UTCDay,
        RXTIME->UTCHour,
        RXTIME->UTCMin,
        RXTIME->UTCSec
      //RXTIME->SyncLevel
      );*/
      ts.tm_year=RXTIME->UTCYear;
      ts.tm_mon=RXTIME->UTCMonth;
      ts.tm_mday=RXTIME->UTCDay;
      ts.tm_hour=RXTIME->UTCHour;
      ts.tm_min=RXTIME->UTCMin;
      ts.tm_sec=RXTIME->UTCSec;
      ts.tm_isdst = 0;
      ts.tm_year += 100; //converts from year in 2-digit UTC
      ts.tm_mon--; // Jan is 1 in UTC, 0 in Unix time

      if (RXTIME->UTCSec != -128)  {
        //DGPSTime = mktime(&ts) - timezone + LEAP_SECONDS;
        DGPSTime = timegm(&ts) + LEAP_SECONDS;
        ntpshm_put((double)DGPSTime); //segmentation fault unless run mcp as sudo
      }

    } else if (VBlock->ID == SBFID_PVTGEODETIC) {

      /* Position & Velocity */
      PVTGeodeticBlock_t* PVT = (PVTGeodeticBlock_t*)SBFBlock;
      /*bprintf(info,"POSITION: %-2i %13.1f %21.10f %21.10f %14.3f %10.3f"
        " %10.3f %15.8f %13.6e %14.3e %3i %3u\n",
        -1,
        PVT->WNc*86400.0*7.0+PVT->TOW/1000.0,
        PVT->Lat,
        PVT->Lon,
        PVT->Alt,
        PVT->Vn,
        PVT->Ve,
        PVT->Vu,
        PVT->RxClkBias,
        PVT->RxClkDrift,
        (int)(PVT->NrSV),
        (unsigned int)(PVT->Mode)
        );*/
      if (PVT->Lat != DONOTUSE) lat = PVT->Lat; // Latitude in radians
      DGPSPos[dgpspos_index].lat = lat*180/M_PI; // Latitude in degrees
      if (PVT->Lon != DONOTUSE) lon = -PVT->Lon; // *** west *** Longitude in radians
      DGPSPos[dgpspos_index].lon = lon*180/M_PI; // Longitude in degrees
      if ((PVT->Alt != DONOTUSE) && (PVT->GeoidHeight != DONOTUSE)) DGPSPos[dgpspos_index].alt = PVT->Alt - PVT->GeoidHeight; // Altitude above geoid in metres
      DGPSPos[dgpspos_index].n_sat = (int)(PVT->NrSV); // # Satellites
      if ((PVT->Vn != DONOTUSE) && (PVT->Ve != DONOTUSE)) {
        DGPSPos[dgpspos_index].speed = (PVT->Vn+PVT->Ve)*60*60/1000;// speed over ground in km/hr (0 to 999.9)
        //if ((PVT->Vn > 0) && (PVT->Ve > 0)) {
        dir = (180/M_PI)*atan2(PVT->Ve,PVT->Vn);// course over ground in degrees from N (due E is 90 deg)
        /*} else if ((PVT->Vn < 0) && (PVT->Ve > 0)) {
          DGPSPos[dgpspos_index].direction = 180 - (180/M_PI)*atan2(PVT->Ve,PVT->Vn);
          } else if ((PVT->Vn < 0) && (PVT->Ve < 0)) {
          DGPSPos[dgpspos_index].direction = 180 + (180/M_PI)*atan2(PVT->Ve,PVT->Vn);
          } else if ((PVT->Vn >0) && (PVT->Ve < 0)) {
          DGPSPos[dgpspos_index].direction = 360 - (180/M_PI)*atan2(PVT->Ve,PVT->Vn);
          }*/
        if (dir < 0) dir +=360;
        DGPSPos[dgpspos_index].direction = dir;
      }
      if (PVT->Vu != DONOTUSE) DGPSPos[dgpspos_index].climb = PVT->Vu; // vertical velocity in m/s (-999.9 to 999.9)      
      if ((PVT->Lat == DONOTUSE)  ||
          (PVT->Lon == DONOTUSE)  || 
          (DGPSPos[dgpspos_index].n_sat < 4)) {
        ;
      } else {
        dgpspos_index = INC_INDEX(dgpspos_index);
      }
    } else if  (VBlock->ID == SBFID_ATTEULER) {

      /* Attitude */
      AttitudeEulerBlock_t* ATTEULER = (AttitudeEulerBlock_t*) SBFBlock;
      /*bprintf(info,"ATTITUDE: %-2i %13.1f %14.5f %14.5f %14.5f"
        " %3u %3u %3u\n",
        -3,
        ATTEULER->WNc*86400.0*7.0+ATTEULER->TOW/1000.0,
        ATTEULER->Heading,
        ATTEULER->Pitch,
        ATTEULER->Roll,
        (unsigned int)(ATTEULER->Error),
        ATTEULER->Mode,
        (unsigned int)(ATTEULER->NrSV)
        );*/
      if (ATTEULER->Heading != DONOTUSE )//&& (DGPSAtt[dgpsatt_index-1].az_cov <= CommandData.dgps_cov_limit)) 
        DGPSAtt[dgpsatt_index].az = ATTEULER->Heading;
      if (ATTEULER->Pitch != DONOTUSE )//&& (DGPSAtt[dgpsatt_index-1].pitch_cov <= CommandData.dgps_cov_limit)) 
        DGPSAtt[dgpsatt_index].pitch = ATTEULER->Pitch;
      if (ATTEULER->Roll != DONOTUSE )//&& (DGPSAtt[dgpsatt_index-1].roll_cov <= CommandData.dgps_cov_limit)) 
        DGPSAtt[dgpsatt_index].roll = ATTEULER->Roll;
      if ((ATTEULER->Heading == DONOTUSE)  || 
          (ATTEULER->Pitch == DONOTUSE)  || 
          (ATTEULER->Roll == DONOTUSE)  ||
          (DGPSAtt[dgpsatt_index].az_cov <=0.001)  ||
          ( ( ( sqrt((DGPSAtt[dgpsatt_index].ant_E)*(DGPSAtt[dgpsatt_index].ant_E) + 
                (DGPSAtt[dgpsatt_index].ant_N)*(DGPSAtt[dgpsatt_index].ant_N)) ) - 2.57) > CommandData.dgps_ants_limit)  ||
          ((fabs(DGPSAtt[dgpsatt_index].ant_U - 1.0)) > CommandData.dgps_ants_limit)  ||
          (DGPSAtt[dgpsatt_index].az_cov > CommandData.dgps_cov_limit)) {
        DGPSAtt[dgpsatt_index].att_ok = 0;
      } else {
        DGPSAtt[dgpsatt_index].att_ok = 1;
      }
      dgpsatt_index = INC_INDEX(dgpsatt_index);
    } else if  (VBlock->ID == SBFID_ATTCOVEULER) {

      /* Attitude Covariance*/
      AttitudeCovEulerBlock_t* ATTCOVEULER = (AttitudeCovEulerBlock_t*) SBFBlock;
      if (ATTCOVEULER->Cov_HeadHead != DONOTUSE) 
        DGPSAtt[dgpsatt_index].az_cov = ATTCOVEULER->Cov_HeadHead;
      if (ATTCOVEULER->Cov_PitchPitch != DONOTUSE) 
        DGPSAtt[dgpsatt_index].pitch_cov = ATTCOVEULER->Cov_PitchPitch;
      if (ATTCOVEULER->Cov_RollRoll != DONOTUSE) 
        DGPSAtt[dgpsatt_index].roll_cov = ATTCOVEULER->Cov_RollRoll; 
    } else if  (VBlock->ID == SBFID_AUXPOS) {

      /* Antenna Position*/
      AuxAntPositions_t* AUXPOSITIONS = (AuxAntPositions_t*) SBFBlock;
      if (AUXPOSITIONS->DeltaEast != DONOTUSE) 
        DGPSAtt[dgpsatt_index].ant_E = AUXPOSITIONS->DeltaEast;
      if (AUXPOSITIONS->DeltaNorth != DONOTUSE) 
        DGPSAtt[dgpsatt_index].ant_N = AUXPOSITIONS->DeltaNorth;
      if (AUXPOSITIONS->DeltaUp != DONOTUSE) 
        DGPSAtt[dgpsatt_index].ant_U = AUXPOSITIONS->DeltaUp;
    } 
  }
  return;

  }
