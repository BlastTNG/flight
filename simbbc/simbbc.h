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
 */

#define BLASTPOL
// #define BLAST

#define TIME_STEP (HZ/100)

#ifdef BLAST 

#define ACS0  23  /* ACS0  node */
#define ACS1   1  /* ACS1  node */
#define LOOP1 17  /* LOOP1 node */


#define CAM0_PULSE_NODE ACS0
#define CAM1_PULSE_NODE ACS0
#define CAM0_TRIGGER_NODE ACS0
#define CAM1_TRIGGER_NODE ACS0
#define GYRO1_NODE        ACS1        
#define GYRO2_NODE        ACS1        
#define GYRO3_NODE        ACS1        

#define CAM0_TRIGGER_CH      1
#define CAM1_TRIGGER_CH      2
#define CAM0_PULSE_CH       53
#define CAM1_PULSE_CH       54
#define GYRO1_CH            59
#define GYRO2_CH            50
#define GYRO3_CH            56

#define GYRO1_DATA       32768
#define GYRO2_DATA       32768
#define GYRO3_DATA       32768

#endif

#ifdef BLASTPOL
#define ACS1_C   0, 0  
#define ACS1_D   1, 0  
#define ACS1_A1  2, 0  
#define ACS1_T1  3, 0  
#define ACS2_C   4, 0
#define ACS2_D   5, 0
#define ACS2_A1  6, 0
#define TMP1    50, 0   //all channels formerly on ACS0
#define TMP2    51, 0   //all channels formerly on ACS1
#define TMP3    52, 0   //all channels formerly on ACS2
#define TMP4    53, 0   //all channels formerly on ACS3
#define TMP5    54, 0   //all channels formerly on CRYO
#define TMP6    55, 0   //all channels formerly on BIAS


#define CAM0_PULSE_NODE ACS2_D
#define CAM1_PULSE_NODE ACS2_D
#define CAM0_TRIGGER_NODE ACS2_D
#define CAM1_TRIGGER_NODE ACS2_D
#define GYRO1_NODE        TMP2        
#define GYRO2_NODE        TMP2        
#define GYRO3_NODE        TMP2        

#define CAM0_TRIGGER_CH     11
#define CAM1_TRIGGER_CH     12
#define CAM0_PULSE_CH       53
#define CAM1_PULSE_CH       54
#define GYRO1_CH            59
#define GYRO2_CH            50
#define GYRO3_CH            56

#define GYRO1_DATA       32768
#define GYRO2_DATA       32768
#define GYRO3_DATA       32768

#endif
