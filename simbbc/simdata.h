/* simbbc: kernel driver simulating the BLAST Bus and controller
 *
 * simbbc is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * simbbc is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with bbc_pci; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Written by Enzo Pascale, Sept. 2 2009
 * Thoroughly hacked by University of Toronto, 2011
 */


void InitBBCData(void);
void InitAuxData(void);
void HandleFrameLogic(void);
int  GetFrameNextWord(unsigned* out_data);  //out_data should be unsigned[2]
void WriteToFrame(unsigned addr, unsigned data);
unsigned int GetFrameCount(void);

/* select experiment to simulate */
#define BLASTPOL
//#define BLAST

/* use to enable digital output with parallel port. 0 to disable */
#define PARALLEL_BASE 0
//#define PARALLEL_BASE 0x378



#ifdef BLAST 

#define ACS0  23
#define ACS1   1

#define CAM0_PULSE    BBC_NODE(ACS0) | BBC_CH(1)  | BBC_READ
#define CAM1_PULSE    BBC_NODE(ACS0) | BBC_CH(2)  | BBC_READ
#define CAM0_TRIGGER  BBC_NODE(ACS0) | BBC_CH(53) | BBC_WRITE
#define CAM1_TRIGGER  BBC_NODE(ACS0) | BBC_CH(54) | BBC_WRITE
#define GYRO1	      BBC_NODE(ACS1) | BBC_CH(59) | BBC_READ
#define GYRO2	      BBC_NODE(ACS1) | BBC_CH(50) | BBC_READ
#define GYRO3	      BBC_NODE(ACS1) | BBC_CH(56) | BBC_READ

#define GYRO_DATA       32768

#define BBC_SYNC_CH   BBC_CH(56) | BBC_WRITE
#define BBC_STAT_CH   BBC_CH(57) | BBC_READ

#endif

#ifdef BLASTPOL
#define ACS2_D   5

#define CAM_PULSES    BBC_NODE(ACS2_D) | BBC_CH(50) | BBC_READ
#define CAM0_TRIGGER  BBC_NODE(ACS2_D) | BBC_CH(11) | BBC_WRITE
#define CAM1_TRIGGER  BBC_NODE(ACS2_D) | BBC_CH(12) | BBC_WRITE
#define GYRO1	      BBC_NODE(ACS2_D) | BBC_CH(12) | BBC_READ
#define GYRO2	      BBC_NODE(ACS2_D) | BBC_CH(13) | BBC_READ
#define GYRO3	      BBC_NODE(ACS2_D) | BBC_CH(14) | BBC_READ

#define GYRO_DATA       32768

#define BBC_SYNC_CH   BBC_CH(63) | BBC_WRITE
#define BBC_STAT_CH   BBC_CH(63) | BBC_READ

#endif
