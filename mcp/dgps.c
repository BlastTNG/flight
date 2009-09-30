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

#include <ctype.h>

#include "pointing_struct.h"
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

#define htTrue         ((htBool_t)1)
#define htFalse        ((htBool_t)0)

#define GPSCOM "/dev/ttyS1"

/* min and max SBF IDs */
#define MIN_CMDID      128
#define MAX_CMDID      767

#define MIN_SBFID      4000 /* Range extended to support SBF 2.0 messages */
#define MAX_SBFID      6015

#define MAX_SBFSIZE    4096   /* maximum size of a SBF block in bytes */
#define MIN_SBFSIZE    8      /* minimum size of a SBF block in bytes */

//#define START_POS_CURRENT 0x00LU

//#define END_POS_AFTER_BLOCK   0x00LU

#define pi    3.14159265358979323846 /* Pi */

#define HEADER_SIZE    8 /* size of the SBF header in bytes */

#define SBFID_PVTGEODETIC       5904
#define SBFID_POSCOVGEODETIC    5906
#define SBFID_VELCOVGEODETIC    5908
#define SBFID_RECEIVERTIME      5914
#define SBFID_ATTEULER          5938
#define SBFID_ATTCOVEULER       5939
#define SBFID_ALL   (MAX_SBFID+1)

#define bdrate B115200

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

/* SBF sync bytes */
static const char SYNC_STRING[3]="$@";

struct DGPSAttStruct DGPSAtt[3];
int dgpsatt_index = 0;

struct DGPSPosStruct DGPSPos[3];
int dgpspos_index = 0;

time_t DGPSTime;

#define LEAP_SECONDS 0		

static htI32_t CheckBlock(int fd, htUI08_t* Buffer)
{
	VoidBlock_t* VoidBlock = (VoidBlock_t*)Buffer;
	
	///* Read the block header, remember that the first '$' is already
	// * present in Buffer[0]. */
	//if (read(fd,&Buffer[1],HEADER_SIZE-1) != HEADER_SIZE-1) {
	//    printf("\nDGPS: returning - couldn't read header\n\n");  	
	//    return -1;
	//}	
	//printf("\nDGPS: reading header\n\n");
	/*Read header one at a time*/
	int hdrexp=HEADER_SIZE-1;
	int cnt=0;
	do {
	  int retn=read(fd,&Buffer[1+cnt],1);
	  if (retn<0) {
	    //printf("\nDGPS: can't read header\n\n");
	    return -1;
	  }
	  //printf("\nDGPS: read %u bytes\n\n",retn);
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
		//printf("\nDGPS: returning - header no good\n\n");
		return -1;
	}
	
	//printf("\nDGPS: reading block\n\n");
	/* Read one at a time */
	int expected=VoidBlock->Length-HEADER_SIZE;
	int count=0; 
	do { 
	   //char testbuf[256]; 
	    int ret=read(fd,&Buffer[HEADER_SIZE+count],1); 
	     if (ret<0) { 
	       //printf("\nDGPS: can't read block\n\n");
	       return -1; 
	     } 
	//     printf("read %u bytes\t",ret); 
	    count +=ret; 
	} while (count < expected); 
	
	//  /*Fetch the block body */
	//if ((htUI16_t)read(fd, &Buffer[HEADER_SIZE], VoidBlock->Length-HEADER_SIZE) != VoidBlock->Length - HEADER_SIZE) {
	//	//perror("returning: can't read");
	//	return -1;
	//}
	
	/* Check the CRC field */
	//if (CRCIsValid(Buffer) == htFalse){
	// printf("returning: crc no good\n");
	// return -1;
	//}
	//printf("\nDGPS: block good, returning from CheckBlock\n\n");
	return 0;
}

htI32_t GetNextBlock(int fd, void* SBFBlock)
{
	htBool_t     BlockFound;
	htUI08_t     Buffer[MAX_SBFSIZE];
		
	//printf("\nDGPS: In GNB now\n\n");
	BlockFound = htFalse;
	do {
		char c[1];
		int n = read(fd,&c[0],1);
		if (n<0)
		  berror(err,"dGPS: read GPSblock failed!");
		if (c[0]==SYNC_STRING[0]) {
			Buffer[0] = (htUI08_t)c[0];
			//printf("\nDGPS:Checking Block\n\n");	
			if (CheckBlock(fd, Buffer) == 0) //&&
			//	((BlockID1 == SBFID_ALL)     ||
			//	 (BlockID1 == ((VoidBlock_t*)Buffer)->ID) ||
			//	 (BlockID2 == SBFID_ALL)     ||
			//	 (BlockID2 == ((VoidBlock_t*)Buffer)->ID))
				  {
				BlockFound = htTrue;	
				memcpy(SBFBlock, Buffer, (size_t)((VoidBlock_t*)Buffer)->Length);	
			}
			//printf("\nDGPS: no find block\n\n");
		}
		
	} while (BlockFound == htFalse);
	return (BlockFound == htTrue) ? 0 : -1;
}


void WatchDGPS()
{
	htUI08_t SBFBlock[MAX_SBFSIZE];
	//htUI32_t LastMeasTOW = 0xffffffff;
	//htUI32_t CurrentTOW,CurrentWNc;
	//htF64_t TimeStamp;
	//FILE *F;
	int	m,n,k,l;
	double lat,lon;
	struct tm ts;
	int pos_ok;

	bputs(startup, "dGPS: WatchDGPS startup\n");
	
	DGPSAtt[0].az = 0;
	DGPSAtt[0].pitch = 0;
	DGPSAtt[0].roll = 0;
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
	
	int fd; //file descriptor (-1 if an error occurred)
	
	struct termios term;
	/*Open device for reading and writing | not as controlling tty | ignore the DCD signal line*/
	if ((fd = open(GPSCOM, O_RDWR | O_NOCTTY | O_NDELAY)) < 0)
		berror(tfatal, "dGPS: Unable to open dgps serial port");
	//printf("\nDGPS: opened serial port\n\n");	
	/*Save current serial port settings*/
	if (tcgetattr(fd, &term))
		berror(tfatal, "dGPS: Unable to get dgps serial port attributes");
	
	/*Input and Output Baud Rates*/
	if (cfsetispeed(&term,bdrate)) //input speed
		berror(tfatal, "dGPS: error setting serial input speed");
	if (cfsetospeed(&term,bdrate)) //input speed
		berror(tfatal, "dGPS: error setting serial input speed");

	/*Control Modes*/
	term.c_cflag |= (CLOCAL | CREAD); //local connection, no modem control; enable receiving characters   
	term.c_cflag &= ~PARENB;   // No Parity
	term.c_cflag &= ~CSTOPB;   // 1 Stop Bit 
	term.c_cflag &= ~CSIZE;    // Mask the character size bits
	term.c_cflag |= CS8;       // 8 data bits
	term.c_cflag &= ~CRTSCTS;  //flow control off (no RTS/CTS)
	/*Local Modes*/
	term.c_lflag =0;
	term.c_cc[VTIME]=0;
	term.c_cc[VMIN]=0;
	//term.c_lflag |= ICANON;    // enable canonical (line-based) input
	/*Input Modes*/
	//term.c_iflag = 0;
	//term.c_iflag = ICRNL;      // map CR to NL on input
	/*Output Modes*/
	term.c_oflag = 0;

	/*Activate settings for the port*/
	if (tcsetattr(fd,TCSANOW,&term))
		berror(tfatal, "dGPS: Unable to set serial attributes");
	
	/* Set interval at which receiver outputs data to 0.1s (10Hz) */
	char cmd1[] = "spi 0.1\n";
	k = strlen(cmd1);
	m = write(fd,cmd1,k);
	if (m<0)
		berror(err,"dGPS: Send command failed!");
	
	/* Set which data blocks receiver will output (see p.129 of manual)
	 *		Status(includes receiver time) =    32768
	 *		PVTGeo(position,velocity,time) =      512
	 *		AttEule(attitude)	       = 16777216
	 *					total  = 16810496
	 */
	char cmd2[] = "sso com1 16777728";
	l = strlen(cmd2);
	n = write(fd,cmd2,l);
	if (n<0)
		berror(err,"dGPS: Send command failed!");
	//printf("\nDGPS: sent commands\n\n");
	/*Read in data blocks*/
	while (GetNextBlock(fd, SBFBlock) == 0) {
		
		usleep(100000);
		/* Time */
		if (((VoidBlock_t*)SBFBlock)->ID == SBFID_RECEIVERTIME) {
			ReceiverTimeBlock_t* RXTIME = (ReceiverTimeBlock_t*) SBFBlock;
			printf("%-2i %13.1f %3i %3i %3i %3i %3i %3i "
				"0 0 0 0 0 0 0 0 0 0 0\n",
				-5,
				RXTIME->WNc*86400.0*7.0+RXTIME->TOW/1000.0,
				RXTIME->UTCYear,
				RXTIME->UTCMonth,
				RXTIME->UTCDay,
				RXTIME->UTCHour,
				RXTIME->UTCMin,
				RXTIME->UTCSec
				);
			ts.tm_year=RXTIME->UTCYear;
			ts.tm_mon=RXTIME->UTCMonth;
			ts.tm_mday=RXTIME->UTCDay;
			ts.tm_hour=RXTIME->UTCHour;
			ts.tm_min=RXTIME->UTCMin;
			ts.tm_sec=RXTIME->UTCSec;
			ts.tm_isdst = 0;
			ts.tm_year += 100; //converts from year in 2-digit UTC
			ts.tm_mon--; // Jan is 1 in UTC, 0 in Unix time
				
			DGPSTime = mktime(&ts) - timezone + LEAP_SECONDS;
		}
		/* Position & Velocity */
		else if (((VoidBlock_t*)SBFBlock)->ID == SBFID_PVTGEODETIC) {
			PVTGeodeticBlock_t* PVT = (PVTGeodeticBlock_t*)SBFBlock;
			printf("%-2i %13.1f %21.10f %21.10f %14.3f %10.3f"
				" %10.3f %15.8f %13.6e %14.3e %3i %3u 0 0\n",
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
				);
			pos_ok = 1;
			lat = PVT->Lat; // Latitude in radians
			DGPSPos[dgpspos_index].lat = lat*180/pi; // Latitude in degrees
			lon = PVT->Lon; // Longitude in radians
			DGPSPos[dgpspos_index].lon = lon*180/pi; // Longitude in degrees
			DGPSPos[dgpspos_index].alt = PVT->Alt - PVT->GeoidHeight; // Altitude above geoid in metres
			DGPSPos[dgpspos_index].n_sat = (int)(PVT->NrSV); // # Satellites
			if (DGPSPos[dgpspos_index].n_sat < 4) {
				pos_ok = 0;
			}
			DGPSPos[dgpspos_index].direction = PVT->Cog; //true track/course over ground in degrees (0 to 359.9)
			DGPSPos[dgpspos_index].speed = (PVT->Vn+PVT->Ve)*60*60/1000;// speed over ground in km/hr (0 to 999.9)
			DGPSPos[dgpspos_index].climb = PVT->Vu; // vertical velocity in m/s (-999.9 to 999.9)
			
			if (pos_ok)
					dgpspos_index = INC_INDEX(dgpspos_index);
		}
		/* Attitude */
		else if  (((VoidBlock_t*)SBFBlock)->ID == SBFID_ATTEULER) {
			AttitudeEulerBlock_t* ATTEULER = (AttitudeEulerBlock_t*) SBFBlock;
			printf("%-2i %13.1f %14.5f %14.5f %14.5f"
				" %3u %3u %3u 0 0 0 0 0 0\n",
				-3,
				ATTEULER->WNc*86400.0*7.0+ATTEULER->TOW/1000.0,
				ATTEULER->Heading,
				ATTEULER->Pitch,
				ATTEULER->Roll,
				(unsigned int)(ATTEULER->Error),
				ATTEULER->Mode,
				(unsigned int)(ATTEULER->NrSV)
			    );
			DGPSAtt[dgpsatt_index].az = ATTEULER->Heading;
			DGPSAtt[dgpsatt_index].pitch = ATTEULER->Pitch;
			DGPSAtt[dgpsatt_index].roll = ATTEULER->Roll;
				
			DGPSAtt[dgpsatt_index].att_ok = 1;
				
			dgpsatt_index = INC_INDEX(dgpsatt_index);
		}

//	switch (((VoidBlock_t*)SBFBlock)->ID) {
//
//			/*Time*/	
//			case SBFID_RECEIVERTIME :
//				ReceiverTimeBlock_t* RXTIME = (ReceiverTimeBlock_t*) SBFBlock;
//				ts.tm_year=RXTIME->UTCYear;
//				ts.tm_mon=RXTIME->UTCMonth;
//				ts.tm_mday=RXTIME->UTCDay;
//				ts.tm_hour=RXTIME->UTCHour;
//				ts.tm_min=RXTIME->UTCMin;
//				ts.tm_sec=RXTIME->UTCSec;
//				ts.tm_isdst = 0;
//				ts.tm_year += 100; //converts from year in 2-digit UTC
//				ts.tm_mon--; // Jan is 1 in UTC, 0 in Unix time
//				
//				DGPSTime = mktime(&ts) - timezone + LEAP_SECONDS;				
//				
//				break;
//
//			/*Position and Velocity*/
//			case SBFID_PVTGEODETIC:
//				PVTGeodeticBlock_t* PVT = (PVTGeodeticBlock_t*)SBFBlock;
//				pos_ok = 1;
//				lat = PVT->Lat; // Latitude in radians
//				DGPSPos[dgpspos_index].lat = lat*180/pi; // Latitude in degrees
//				lon = PVT->Lon; // Longitude in radians
//				DGPSPos[dgpspos_index].lon = lon*180/pi; // Longitude in degrees
//				DGPSPos[dgpspos_index].alt = PVT->Alt - PVT->GeoidHeight; // Altitude above geoid in metres
//				DGPSPos[dgpspos_index].n_sat = (int)(PVT->NrSV); // # Satellites
//				if (DGPSPos[dgpspos_index].n_sat < 4) {
//					pos_ok = 0;
//				}
//				DGPSPos[dgpspos_index].direction = PVT->Cog; //true track/course over ground in degrees (0 to 359.9)
//				DGPSPos[dgpspos_index].speed = (PVT->Vn+PVT->Ve)*60*60/1000;// speed over ground in km/hr (0 to 999.9)
//				DGPSPos[dgpspos_index].climb = PVT->Vu; // vertical velocity in m/s (-999.9 to 999.9)
//				
//				if (pos_ok)
//					dgpspos_index = INC_INDEX(dgpspos_index);
//								
//				break;
//
//			/*Attitude*/	
//			case SBFID_ATTEULER :
//				AttitudeEulerBlock_t* ATTEULER = (AttitudeEulerBlock_t*) SBFBlock;
//				DGPSAtt[dgpsatt_index].az = ATTEULER->Heading;
//				DGPSAtt[dgpsatt_index].pitch = ATTEULER->Pitch;
//				DGPSAtt[dgpsatt_index].roll = ATTEULER->Roll;
//				
//				DGPSAtt[dgpsatt_index].att_ok = 1;
//				
//				dgpsatt_index = INC_INDEX(dgpsatt_index);
//				
//				break;
//								
//			default:
//				break;
//				
//		}
	}
	
	//printf("\nDGPS: closing fd\n\n");
	close(fd);
	
	return;
		
}
