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

  /* FAST_PER_SLOW is the number of fast samples for each slow one */
#define FAST_PER_SLOW   20
#define NOT_MULTIPLEXED (FAST_PER_SLOW)
#define DISCARD_WORD    (FAST_PER_SLOW + 1)

  /* Number of DAS bolometer cards to include in the frame.  The maximum number
   * of cards is 11 */
#define DAS_CARDS 12

#define DAS_CHS 24

#define N_FAST_BOLOS (DAS_CARDS * (DAS_CHS + DAS_CHS / 2))

  /* number of channels below the first slow channel */
#define SLOW_OFFSET 4

  /* offset of encoder.  Reset if encoder has been unmounted. */
  /* This is the elevation at which the encoder wraps around */
#define ENC_ELEV_OFFSET 20.27
  /* to get proper wrapping in KST, the encoder elevation type should be
   * 'u' for 135 <= ENC_ELEV_OFFSET < 315 and 's' otherwise */
#define ENC_ELEV_TYPE 's'

#define LOCKIN_C2V (5.43736e-07)
#define LOCKIN_OFFSET (-1.1403)

#define ROX_C2V   (LOCKIN_C2V/256.0)
#define ROX_OFFSET LOCKIN_OFFSET

#define FIELD_LEN 20

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
  };

  struct NiosStruct {
    unsigned int niosAddr;
    unsigned int bbcAddr;
    unsigned char fast;
    unsigned char wide;
    unsigned char bus;
    const char* field;
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

#define T_JFET_M (-2.860308e-09*65536.0)
#define T_JFET_B (1.232735e+01)

#define T_LHE_M (-2.859373e-09*65536.0)
#define T_LHE_B (1.232225e+01)

#define T_CHARCOAL_M (-2.865098e-09*65536.0)
#define T_CHARCOAL_B (1.235145e+01)

// Conversion factors for the rotated/calibrated gyros
// (GYRO1, GYRO2, GYRO3).  Any correction to this belongs
// in ACS1, not here
#define DPS_TO_GY16 1092.8128
#define GY16_TO_DPS (1.0/DPS_TO_GY16)
#define GY16_OFFSET 32768.0

// Conversion factors for raw 32 bit analog gyro chanels
// put any correction to these directly in tx_struct.c
// these should only appear in tx_struct.c
#define DPS_TO_AGY32 (1092.8128*65536.0)
#define AGY32_TO_DPS (1.0/DPS_TO_AGY32)
#define AGY32_OFFSET (25600.0*65536.0)

// Conversion factors for raw 32 bit digital gyro chanels
// put any correction to these directly in tx_struct.c
// these should only appear in tx_struct.c
#define DGY32_TO_DPS (60.0E-6 * 4.0/256.0)
#define DPS_TO_DGY32 (1.0/DGY32_TO_DPS)
#define DGY32_OFFSET (32768.0*65536.0)

/* Gyrobox thermometer conversion -- this is the 32bit AD590 conversion */
#define TGYBOX_B (I2T_B)
#define TGYBOX_M (-9.5367431641e-08)

/* Gondola thermometry conversions -- this is the 16bit AD590 conversion */
#define I2T_M (-0.00625)
#define I2T_B (136.45)

#ifdef __cplusplus
}
#endif

#endif
