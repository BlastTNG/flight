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

