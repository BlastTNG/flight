/* channels.h: contains channel list specific prototypes and definitions
 *
 * This software is copyright (C) 2002-2005 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */


#ifndef CHANNELS_H
#define CHANNELS_H

#include <stdio.h>
#include <time.h>

/* Fix things up so we can include this in C++ applications */
#ifdef __cplusplus
extern "C" {
#endif

/* Gains and offsets for ideal analog cards: cal = (counts + B)*M */
#define M_16PRE (10.24/32768.0)
#define B_16PRE (-32768.0)
#define M_16T	(4.096E6/2.2E3/32768.0/8.0)  //factor of 8 from maximizing range
#define B_16T	(0.0)

  /* FAST_PER_SLOW is the number of fast samples for each slow one */
#define FAST_PER_SLOW   20
#define NOT_MULTIPLEXED (FAST_PER_SLOW)
#define DISCARD_WORD    (FAST_PER_SLOW + 1)

#define SR (100.16)
  /* Number of DAS bolometer cards to include in the frame.  The maximum number
   * of cards is 12 */
#define DAS_CARDS 12

#define DAS_CHS 24
#define DAS_START 16  //motherboard node of first DAS card

#define N_FAST_BOLOS (DAS_CARDS * (DAS_CHS + DAS_CHS / 2))

  /* number of channels below the first slow channel */
#define SLOW_OFFSET 4

  /* offset of encoder.  Reset if encoder has been unmounted. */
  /* This is the elevation at which the encoder wraps around */
#define ENC_RAW_EL_OFFSET (75.1) /* Updated 03-JUL_2010 by lmf */
  /* to get proper wrapping in KST, the encoder elevation type should be
   * 'u' for 135 <= ENC_EL_RAW_OFFSET < 315 and 's' otherwise */
#define ENC_ELEV_TYPE 's'

  /* Bolo calibrations per Tristan and Matt for new DAS 2010-01-21 */
#define LOCKIN_C2V (6.90336327e-7)
#define LOCKIN_OFFSET (-5.78715355)

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
#define LVDT63_ZERO  65536  /* in encoder counts */
#define LVDT64_ZERO  65536  /* in encoder counts */
#define LVDT65_ZERO  65536  /* in encoder counts */

#define FIELD_LEN 20
#define UNITS_LEN 48

#ifndef CAM_WIDTH
#define CAM_WIDTH 1530.0  //should always be the larger dimension
#endif

  // Max Slew Veto
#define VETO_MAX 60000

  struct ChannelStruct {
    char field[FIELD_LEN]; /* name of channel for FileFormats and CalSpecs */
    char rw;        /* 'r' = read, 'w' = write */
    char node;       /* BlastBus node: 0 to 63 */
    char bus;        /* Bus number: 0 to 1 */
    char addr;       /* BlastBus address: 0 to 63 */
    float m_c2e;    /* Conversion from counts to enginering units is */
    float b_e2e;    /*   e = c * m_c2e + b_e2e */
    char type;      /* 's' = short, signed o'u' = unsigned short 'i' = 'S'
                       = signed 32 bit int, 'U' = unsigned 32 bit int */
    char quantity[UNITS_LEN]; /* eg, "Temperature" or "Angular Velocity" */
    char units[UNITS_LEN]; /* eg, "K" or "^o/s" */
  };

  struct NiosStruct {
    unsigned int niosAddr;
    unsigned int bbcAddr;
    unsigned char fast;
    unsigned char wide;
    unsigned char bus;
    const char* field;
    /* for the slow dl */
    float m;
    float b;
  };

  struct BiPhaseStruct {
    unsigned int channel;
    unsigned int index;
  };

  extern struct NiosStruct* NiosLookup;
  extern struct BiPhaseStruct* BiPhaseLookup;
  extern struct ChannelStruct** SlowChList;
  extern struct ChannelStruct* FastChList;

  extern unsigned short ccWideFast;
  extern unsigned short ccNarrowFast;
  extern unsigned short ccWideSlow;
  extern unsigned short ccNarrowSlow;
  extern unsigned short ccFast;
  extern unsigned short ccDecom;
  extern unsigned short ccDerived;
  extern unsigned short ccTotal;

  extern unsigned short BoloBaseIndex;

#if (defined __MCP__ || defined __DECOMD__)
  extern unsigned short BiPhaseFrameWords;
  extern unsigned short BiPhaseFrameSize;
#endif
  extern unsigned short DiskFrameWords;
  extern unsigned short DiskFrameSize;
  extern unsigned short slowsPerBi0Frame;
  extern unsigned short TxFrameWords[2];
  extern unsigned short TxFrameSize[2];
  extern unsigned int NiosSpares[FAST_PER_SLOW * 2];
  extern unsigned int BBCSpares[FAST_PER_SLOW * 2];

  void MakeAddressLookups(void);
  void FPrintDerived(int);
  struct NiosStruct* GetNiosAddr(const char*);
  inline struct BiPhaseStruct* GetBiPhaseAddr(const char*);
  inline struct BiPhaseStruct* ExtractBiPhaseAddr(struct NiosStruct*);
  void ReadSpecificationFile(FILE*);
  void WriteFormatFile(int, time_t, unsigned long);
  char* FieldToLower(char*);
  char* FieldToUpper(char*);

/* reserved node numbers */
#define SPARE      62
#define SPECIAL    63
#define EOC_MARKER -1

#define END_OF_CHANNELS {"", 'x', EOC_MARKER, -1, 0, 0}

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

/* AD590 calibrations from Marco, Nov 2006 */
#define AD590_CALIB_INFRAME_1    1.2250
#define AD590_CALIB_INFRAME_2    1.8125
#define AD590_CALIB_INFRAME_3    0.0188
#define AD590_CALIB_INFRAME_4    1.5125
#define AD590_CALIB_PRIMARY_2   -0.3750
#define AD590_CALIB_STRUT_1     -0.8437
#define AD590_CALIB_PRIMARY_1   -1.5937
#define AD590_CALIB_SECONDARY_1 -1.6250
#define AD590_CALIB_SECONDARY_2  0.1563
#define AD590_CALIB_STRUT_2     -0.8502
#define AD590_CALIB_PUSH_PLATE   0.4500
#define AD590_CALIB_ACT_MOTOR   -0.4375
#define AD590_CALIB_REC         -1.4822

/* zero point (in counts) of i_el */
#define I_EL_ZERO 32638

// convert mag readings to sine and cosine
// calibrated in Palestine, July 11, 2010
// Best fit to mag_x and mag_y
// y = -3000*sin(x-19)+33050 : mag_x
// y = 3000*cos(x-19)+33310 : mag_y
// x is dgps theta in degrees.
#define MAGX_M (-1.0/3000.0)
#define MAGX_B (33050/3000.0)
#define MAGY_M (1.0/3000.0)
#define MAGY_B (33310.0/3000.0)
#define MAGZ_M (1.0)
#define MAGZ_B (0.0)
  
#ifdef __cplusplus

}
#endif

#endif
