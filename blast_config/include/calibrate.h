
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

/* BLASTBus frame sample rate */
#define SR (100)

/**
 * Scaling factors for each motor.  These are hard-wired based on the encoder/resolver
 */
#define RW_ENCODER_COUNTS (1 << 21)
#define RW_COUNTS_PER_REV (1 << 13)
#define PIV_RESOLVER_COUNTS (1 << 14)

#define EL_LOAD_ENCODER_COUNTS (1 << 26) /* This is the External, absolute encoder mounted on the inner frame */
#define EL_LOAD_COUNTS_PER_REV (1 << 26)
//#define EL_MOTOR_ENCODER_COUNTS 655360 /* This is (1 << 19) / 0.8 to correct for the gearbox */
#define EL_MOTOR_ENCODER_COUNTS (1 << 19) /* No gearbox */
#define EL_MOTOR_COUNTS_PER_REV (1 << 19)

#define RW_ENCODER_SCALING (360.0 / RW_ENCODER_COUNTS)
#define EL_MOTOR_ENCODER_SCALING ((-1.0)*360.0 / EL_MOTOR_ENCODER_COUNTS)
#define EL_LOAD_ENCODER_SCALING (360.0 / EL_LOAD_ENCODER_COUNTS)
#define PIV_RESOLVER_SCALING (360.0 / PIV_RESOLVER_COUNTS)
#define EL_MOTOR_CURRENT_SCALING (-1.0) /* So that current > 0 -> El increase */

/* Gains and offsets for ideal analog cards: cal = (counts + B)*M */
#define M_32PRE (10.24/2147483648.0)
#define B_32PRE	(-2147483648.0)
#define M_16PRE (10.24/32768.0)
#define B_16PRE (-32768.0)

/* Gains and offsets for the labjack AIN channels: cal = (counts + B)*M */
#define M_16LJAIN (10.8/32768.0)
#define B_16LJAIN (-10.8)

/* Gains and offsets for the roaches: */
#define M_32LOFREQ (0.00001)
#define B_32LOFREQ (750)
#define M_16LOFREQ (0.001)
#define B_16LOFREQ (750)
#define M_16RFREQ (100.0/32768) // kHz
#define B_16RFREQ (-100.0)
#define M_32RFREQ (100000.0/(1 << 31)) // kHz
#define B_32RFREQ (-100000.0)
#define M_16R_DB (50.0/65536) // kHz
#define B_16R_DB (0.0)

/* bare thermometer conversion to Volts. No negative values allowed */
#define M_16T (4.096/32768.0/2.0)
#define B_16T (0.0)
/* AD590 calibrations. To Kelvin */
#define M_16_AD590	(M_16T/2.2E-3)
#define B_16_AD590	(B_16T)

/* offset of encoder.  Reset if encoder has been unmounted. */
/* This is the elevation at which the encoder wraps around */
#define ENC_RAW_EL_OFFSET (291.84) //PCA 11-May-2017
                                   /* Note this is referenced relative to lock pin hole 0*/

/* to get proper wrapping in KST, the encoder elevation type should be
 * 'u' for 135 <= ENC_EL_RAW_OFFSET < 315 and 's' otherwise */
#define ENC_ELEV_TYPE 'u'

#define ROX_C2V   (5.43736e-07/256.0)
#define ROX_OFFSET (-1.1403)

#define STAGE_X_THROW 78500
#define STAGE_Y_THROW 78250

#define ACTENC_TO_UM 1.05833333333 /* mm/enc.counts = 24000 counts/inch */
//#define ACTENC_OFFSET 1000000 /* this number should be arbitrarily larger than
                                 //the maximum throw */

//ideal calibrations NB: LVDT and ENC have different signs
#define LVDT63_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
#define LVDT64_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
#define LVDT65_ADC_TO_ENC -0.75 /* adc counts to encoder counts */
//LVDT flat fielding sjb: 28 Nov 2010
#define LVDT63_ZERO  64389  /* in encoder counts */
#define LVDT64_ZERO  65240  /* in encoder counts */
#define LVDT65_ZERO  65536  /* in encoder counts */

/* Thermal model numbers, from EP */
#define T_PRIMARY_FOCUS   296.15 /* = 23C */
#define T_SECONDARY_FOCUS 296.15 /* = 23C */
#define POSITION_FOCUS     33333 /* absolute counts */

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
#define VEL2I (65536.0/10.0)
#define I2VEL (1.0/VEL2I)

/* Cryo preamp channel Voltage calibration */
/* Measured by Tristan @ Penn, September 29 2009 */
#define CRYO_A2_M ( 4.805248E-9)
#define CRYO_A2_B (-1.032198E1 )
/* Cryo Diode Voltage Calibration */
/* Modified by Ian Summer 2017 */
#define CRYO_D_M ( 10.8/32768)
#define CRYO_D_B (-10.8)
/* M3 was not measured (spider cable broken) so is an estimate */
#define CRYO_M3_M            (1.1319609e-05)
#define CRYO_M3_B            (-24293.822)
/* Current ranges +/-15 Amps*/
#define CUR15_M (15/32768.0)
#define CUR15_B (-15.0)

#define CURLOOP_CONV (50.0/8.0) /* 50 Amps = 8.0V */
/* Current sensor ranges +/-67.5 Amps*/
#define CURLOOP_D_M (10.8/32768.0*50.0/8.0)
#define CURLOOP_D_B (-67.5)


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

#define PIV_DAC_OFF (-1)*102 // 31mV Analog Input voltage offset
                             // as measured by the pivot controller
#define PIV_DEAD_BAND 162.75 // 50mV*3.255029 DAC cts/mV

/* zero point (in counts) of i_el */
#define I_EL_ZERO 32638

/**
 * UEI internal monitoring channels
 */
#define UEI_VOLT_M (0.000000149*23.1)   /// 149 nV * 23.1V scaler
#define UEI_VOLT_B (-8388608.0 * UEI_VOLT_M)           /// 2^23 (24-bit ADC)

#define UEI_TEMP_M (0.000000149*339)    /// 149 nV * 339V scaler
#define UEI_TEMP_B (-8388608.0 * UEI_TEMP_M)          /// 2^23 (24-bit ADC)

#define UEI_INTVOLT_M (0.000000149*45.3)   /// 149 nV * 45.3V scaler
#define UEI_INTVOLT_B (-8388608.0 * UEI_INTVOLT_M)          /// 2^23 (24-bit ADC)

#define UEI_CURRENT_M (0.000000149*12)     /// 149 nV * 12V scaler
#define UEI_CURRENT_B (-8388608.0 * UEI_CURRENT_M)          /// 2^23 (24-bit ADC)

#define LOCKIN_C2V (6.90336327e-7)
#define LOCKIN_OFFSET (-5.78715355)

#ifdef __cplusplus
}
#endif

#endif
