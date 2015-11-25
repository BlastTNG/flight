
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
#define EL_MOTOR_ENCODER_COUNTS 655360 /* This is (1 << 19) / 0.8 to correct for the gearbox */
#define EL_MOTOR_COUNTS_PER_REV (1 << 13)

#define RW_ENCODER_SCALING (360.0 / RW_ENCODER_COUNTS)
#define EL_MOTOR_ENCODER_SCALING (360.0 / EL_MOTOR_ENCODER_COUNTS)
#define EL_LOAD_ENCODER_SCALING (360.0 / EL_LOAD_ENCODER_COUNTS)
#define PIV_RESOLVER_SCALING (360.0 / PIV_RESOLVER_COUNTS)

/* Gains and offsets for ideal analog cards: cal = (counts + B)*M */
#define M_32PRE (10.24/2147483648.0)
#define B_32PRE	(-2147483648.0)
#define M_16PRE (10.24/32768.0)
#define B_16PRE (-32768.0)
/* bare thermometer conversion to Volts. No negative values allowed */
#define M_16T (4.096/32768.0/2.0)
#define B_16T (0.0)
/* AD590 calibrations. To Kelvin */
#define M_16_AD590	(M_16T/2.2E-3)
#define B_16_AD590	(B_16T)

/* offset of encoder.  Reset if encoder has been unmounted. */
/* This is the elevation at which the encoder wraps around */
#define ENC_RAW_EL_OFFSET (255.0) /* Updated 29-NOV-2012 by nng */
                                   /* Note this is referenced relative to the gyro beam*/

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
/* Modified by Jeff @ Penn, October 6 2009 */
#define CRYO_D_M ( 4.8023774e-09)
#define CRYO_D_B (-1.0317770e+01)
/* Cryo ROX resistance calibrations */
/* Measured May 12 2010 */
#define CRYO_HE3_FRIDGE_M    (1.1535865e-05)
#define CRYO_HE3_FRIDGE_B    (-24819.549)
#define CRYO_HE4_POT_M       (1.2435943e-05)
#define CRYO_HE4_POT_B       (-26763.858)
#define CRYO_OPTBOX_FILT_M   (1.2072812e-05)
#define CRYO_OPTBOX_FILT_B   (-25972.241)
#define CRYO_HWPR_M          (1.1940140e-05)
#define CRYO_HWPR_B          (-25727.908)
#define CRYO_300MK_STRAP_M   (1.1863976e-05)
#define CRYO_300MK_STRAP_B   (-25594.994)
#define CRYO_HORN_500_M      (1.1861167e-05)
#define CRYO_HORN_500_B      (-25533.556)
#define CRYO_HORN_350_M      (1.1676174e-05)
#define CRYO_HORN_350_B      (-25115.738)
#define CRYO_HORN_250_M      (1.2107397e-05)
#define CRYO_HORN_250_B      (-26091.788)
#define CRYO_M5_M            (1.1516911e-05)
#define CRYO_M5_B            (-24789.511)
#define CRYO_M4_M            (1.2124184e-05)
#define CRYO_M4_B            (-26136.965)
/* M3 was not measured (spider cable broken) so is an estimate */
#define CRYO_M3_M            (1.1319609e-05)
#define CRYO_M3_B            (-24293.822)

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

// convert mag readings to sine and cosine
// calibrated in Palestine, July 11, 2010
// Best fit to mag_x and mag_y
// y = -3000*sin(x-19)+33050 : mag_x
// y = 3000*cos(x-19)+33310 : mag_y
// x is dgps theta in degrees.
/*#define MAGX_M (-1.0/3000.0)
#define MAGX_B (33850/3000.0)
#define MAGY_M (-1.0/3100.0)
#define MAGY_B (32150.0/3100.0)
#define MAGZ_M (1.0)
#define MAGZ_B (0.0)*/
// convert mag readings to sine and cosine
// calibrated in Palestine, July 11, 2010
// Best fit to mag_x and mag_y
// y = -3000*sin(x-19)+33050 : mag_x
// y = 3000*cos(x-19)+33310 : mag_y
// x is dgps theta in degrees.
// The defines for x and y are no longer used.
//#define MAGX_M (-1.0/1290.0)
//#define MAGX_B (33500/1290.0)
//#define MAGY_M (-1.0/1290.0)
//#define MAGY_B (33400.0/1290.0)
#define MAGZ_M (-1/1290.0)
#define MAGZ_B (32768.0)

#define FAST_MAG

#define LOCKIN_C2V (6.90336327e-7)
#define LOCKIN_OFFSET (-5.78715355)

#ifdef __cplusplus
}
#endif

#endif
