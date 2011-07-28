/* calibrate.h: field calibrations for the BLASTBus (formerly in channels.h)
 *
 * This software is copyright (C) 2011 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef CALIBRATE_H
#define CALIBRATE_H

/* Fix things up so we can include this in C++ applications */
#ifdef __cplusplus
extern "C" {
#endif

/* BLASTus frame sample rate (TODO this will change!) */
#define SR (100.16)

/* Gains and offsets for ideal analog cards: cal = (counts + B)*M */
#define M_32PRE (10.24/2147483648.0)
#define B_32PRE	(-2147483648.0)
#define M_16PRE (10.24/32768.0)
#define B_16PRE (-32768.0)
#define M_16T	(4.096E6/2.2E3/32768.0/8.0)  //factor of 8 from maximizing range
#define B_16T	(0.0)

// Conversion factors for raw 32 bit digital gyro chanels
// put any correction to these directly in tx_struct.c
// these should only appear in tx_struct.c
#define DGY32_TO_DPS (60.0E-6 * 4.0/256.0)
#define DPS_TO_DGY32 (1.0/DGY32_TO_DPS)
#define DGY32_OFFSET (32768.0*65536.0)

#define H2LI (4294967296.0/24.0)
#define LI2H (1.0/H2LI)
#define DEG2LI (4294967296.0/360.0)
#define LI2DEG (1.0/DEG2LI)
#define RAD2LI (4294967296.0/2/M_PI)
#define DEG2I (65536.0/360.0)
#define I2DEG (1.0/DEG2I)
#define RAD2I (65536.0/2/M_PI)
#define H2I (65536.0/24.0)
#define I2H (1.0/H2I)
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

// Conversion factors for the rotated/calibrated gyros
// (GY_IFEL, GY_IFYAW, GY_IFROLL).
// Any correction to these belongs in ACS1, not here
#define DPS_TO_GY16 1000.0      // 1 gyro bit == 0.001 dps 
#define GY16_TO_DPS (1.0/DPS_TO_GY16)
#define GY16_OFFSET 32768.0

#ifdef __cplusplus
}
#endif

#endif
