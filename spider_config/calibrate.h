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

/* BLASTBus ADC sample rate. Always clock rate / 384.0 */
// Use ACSData.adc_rate instead from now on to account for variable clock rate
//#define ADC_SR (4.0e6/384.0)

/* BLASTBus frame sample rate (ADC_SR/104.0) */
// Use ACSData.bbc_rate instead from now on
//#define SR (100.16)

/* Gains and offsets for ideal analog cards: cal = (counts + B)*M */
/* 16- and 32-bit channels with preamps conversion to Volts */
#define M_32PRE (10.24/2147483648.0)
#define B_32PRE	(-2147483648.0)
#define M_16PRE (10.24/32768.0)
#define B_16PRE (-32768.0)
/* Bare analog (no preamp) conversion to Volts. Used by some IF thermometers */
#define M_16B (4.096/32768.0)
#define B_16B (-32768.0)
//NB: temperature cal just to volts now. Use a LUT to convert to degC
//can get 2x more range if impossible -'ve values discarded
/* bare thermometer conversion to Volts. No negative values allowed */
#define M_16T (4.096/32768.0/2.0)
#define B_16T (0.0)
/* AD590 calibrations. To Kelvin (NB: used to have /8 not /2) */
#define M_16_AD590	(M_16T/2.2E-3)
#define B_16_AD590	(B_16T)

#define M_DAC (5.0/32768.0)
#define B_DAC (-32768.0)

/* offset of encoder.  Reset if encoder has been unmounted. */
/* This is the elevation at which the encoder wraps around */
//#define ENC_RAW_EL_OFFSET (75.3) /* Updated 09-DEC_2010 by lmf */
/* to get proper wrapping in KST, the encoder elevation type should be
 * 'u' for 135 <= ENC_EL_RAW_OFFSET < 315 and 's' otherwise */
//#define ENC_ELEV_TYPE 's'

#define ENC_TABLE_OFFSET 0.0 

//#define ROX_C2V   (5.43736e-07/256.0)
//#define ROX_OFFSET (-1.1403)

#define STAGE_X_THROW 78500
#define STAGE_Y_THROW 78250

//#define ACTENC_TO_UM 1.05833333333 /* mm/enc.counts = 24000 counts/inch */
//#define ACTENC_OFFSET 1000000 /* this number should be arbitrarily larger than
                                 //the maximum throw */

//ideal calibrations NB: LVDT and ENC have different signs
//#define LVDT63_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
//#define LVDT64_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
//#define LVDT65_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
//LVDT flat fielding sjb: 28 Nov 2010
//#define LVDT63_ZERO  64389  /* in encoder counts */
//#define LVDT64_ZERO  65240  /* in encoder counts */
//#define LVDT65_ZERO  65536  /* in encoder counts */

/* Lock position is nominally at 40 degrees.
 * This is the offset to the true lock positions. This number is
 * relative to the elevation encoder reading, NOT true elevation
 */
#define LOCK_OFFSET (0.0) /* TODO update this */
#define LOCK_POSITION (40.0 + LOCK_OFFSET)

/* Thermal model numbers, from EP */
//#define T_PRIMARY_FOCUS   296.15 /* = 23C */
//#define T_SECONDARY_FOCUS 296.15 /* = 23C */
//#define POSITION_FOCUS     33333 /* absolute counts */

#ifndef CAM_WIDTH
#define CAM_WIDTH 1530.0  //should always be the larger dimension
#endif

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
#define VEL2I (65536.0/20.0)
#define I2VEL (1.0/VEL2I)

// Conversion factors for the rotated/calibrated gyros
// (GY_IFEL, GY_IFYAW, GY_IFROLL).
// Any correction to these belongs in ACS1, not here
#define DPS_TO_GY16 1000.0      // 1 gyro bit == 0.001 dps 
#define GY16_TO_DPS (1.0/DPS_TO_GY16)
#define GY16_OFFSET 32768.0

// Conversion factors for raw 32 bit digital gyro chanels
// put any correction to these directly in tx_struct.c
// these should only appear in tx_struct.c
#define DGY32_TO_DPS (60.0E-6 * 4.0/256.0)
#define DPS_TO_DGY32 (1.0/DGY32_TO_DPS)
#define DGY32_OFFSET (32768.0*65536.0)

// Conversion factors for the Pivot motor control loop.
// Used in motors.c and tx_struct.c
#define PIV_I_TO_DAC 3255.029 
#define DAC_TO_PIV_I 1.0/PIV_I_TO_DAC

//#define PIV_DAC_OFF (-1)*102 // 31mV Analog Input voltage offset
                             // as measured by the pivot controller
//#define PIV_DEAD_BAND 162.75 // 50mV*3.255029 DAC cts/mV

#define PIV_DAC_OFF 0.0   // value is set in controller firmware instead
#define PIV_DEAD_BAND 0.0 // value is set in controller firmware instead

	/* only zero wrt each other, not absolute */
//#define AD590_CALIB_PRIMARY_1   -0.7285
//#define AD590_CALIB_PRIMARY_2    0.7285
//#define AD590_CALIB_SECONDARY_1  0.143
//#define AD590_CALIB_SECONDARY_2 -0.143

/* zero point (in counts) of i_el */
#define I_EL_ZERO 32638

//new magnetomter
#define MAGZ_M (1.0)
#define MAGZ_B (0.0)

#define ENC1_OFFSET (-11.1798095703125) // encoder 1 to bore-sight offset angle
#define ENC2_OFFSET (265.1599294140625)  // encoder 2 to bore-sight offset angle
//#define GYBOX_OFFSET -69.8  // gyro box to bore-sight offset angle (fixed)




#ifdef __cplusplus
}
#endif

#endif
