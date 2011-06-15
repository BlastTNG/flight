/* channels.h: contains channel list specific prototypes and definitions
 *
 * This software is copyright (C) 2002-2011 University of Toronto
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

#define SR (100.16)

/* Number of DAS bolometer cards to include in the frame.  The maximum number
 * of cards is 12. Making 0 will disable bolometer channels
 * Unless specified, will assume 0. Blast should compile with -DDAS_CARDS=12 */
#ifndef DAS_CARDS
#define DAS_CARDS 0
#endif

#define DAS_CHS 24
#define DAS_START 16  //motherboard node of first DAS card
#define N_FAST_BOLOS (DAS_CARDS * (DAS_CHS + DAS_CHS / 2))

/* number of channels below the first slow channel */
#define SLOW_OFFSET 4

/* NB this calibration can't go it calibrate.h because BOLO channels need it */
/* Bolo calibrations per Tristan and Matt for new DAS 2010-01-21 */
#define LOCKIN_C2V (6.90336327e-7)
#define LOCKIN_OFFSET (-5.78715355)

#define FIELD_LEN 20
#define UNITS_LEN 48

  struct ChannelStruct {
    char field[FIELD_LEN]; /* name of channel for FileFormats and CalSpecs */
    char rw;        /* 'r' = read, 'w' = write */
    char node;      /* BlastBus node: 0 to 63 */
    char bus;       /* Bus number: 0 to 1 */
    char addr;      /* BlastBus address: 0 to 63 */
    double m_c2e;   /* Conversion from counts to enginering units is */
    double b_e2e;   /*   e = c * m_c2e + b_e2e */
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
    double m;
    double b;
  };

  struct BiPhaseStruct {
    unsigned int channel;
    unsigned int index;
    struct NiosStruct* nios;  //access wide, fast, m, b for ReadData
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

#ifdef __cplusplus

}
#endif

#endif
