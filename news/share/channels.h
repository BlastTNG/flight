/* channels.h: contains channel list specific prototypes and definitions
 *
 * This software is copyright (C) 2002-2004 University of Toronto
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
  void WriteFormatFile(int, time_t);
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

#define T_JFET_M (-2.860308e-09*65536)
#define T_JFET_B (1.232735e+01)

  /* conversions between dps and a to d units for each gyro */
#define DPS_TO_ADU1 (1092.8128/1.0407)
#define ADU1_TO_DPS (1.0/DPS_TO_ADU1)
#define DPS_TO_ADU2 (1092.8128/1.036)
#define ADU2_TO_DPS (1.0/DPS_TO_ADU2)
#define DPS_TO_ADU3 (1092.8128/1.036)
#define ADU3_TO_DPS (1.0/DPS_TO_ADU3)

  /* Approximate gyro offsets - to get us close */
  /* If these numbers are changed, they must be changed on the ACS1
   * DSP as well */
#define GYRO1_OFFSET 25795.0
#define GYRO2_OFFSET 25535.0
#define GYRO3_OFFSET 25600.0

#ifdef __cplusplus
}
#endif

#endif
