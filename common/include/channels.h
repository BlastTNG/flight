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

/* The maximum number of DAS cards allowed */
#define MAX_DAS_CARDS 12

/* MWG: DAS = Data Acquisition Cards. The maximum number is 12.
 * Making DAS_CARDS 0 will disable bolometer channels;
 * Blast compiles with DAS_CARDS at 12 */

/* As of 2013-04-01, this is only the default value; a caller can change
 * this at runtime by modifying the das_cards variable before doing anything
 * else with this library */

#ifndef DAS_CARDS
#define DAS_CARDS 0
#endif

#if DAS_CARDS > MAX_DAS_CARDS
#error "Too many DAS cards."
#endif

#define DAS_CHS 24 /* number of DAS channels per DAS card */
#define DAS_START 16  //motherboard node of first DAS card
#define MAX_FAST_BOLOS (MAX_DAS_CARDS * (DAS_CHS + DAS_CHS / 2))


  extern uint16_t  das_cards; /* number of DAS cards -- defaults to
                                      DAS_CARDS */

/* number of channels below the first slow channel */
#define SLOW_OFFSET 4

/* NB this calibration can't go it calibrate.h because BOLO channels need it */
/* Bolo calibrations per Tristan and Matt for new DAS 2010-01-21 */
#define LOCKIN_C2V (6.90336327e-7)
#define LOCKIN_OFFSET (-5.78715355)

#define FIELD_LEN 32 // Was 20, but conflicted with DEF from channels_tng
#define UNITS_LEN 48

#pragma pack(push)
#pragma pack(4)
  struct ChannelStruct {
    char field[FIELD_LEN]; /* name of channel for FileFormats and CalSpecs */
    char rw;        /* 'r' = read, 'w' = write */
    char node;      /* BlastBus node: 0 to 63 */
    char bus;       /* Bus number: 0 to 1 */
    char addr;      /* BlastBus address: 0 to 63 */
    double m_c2e;   /* Conversion from counts to enginering units is */
    double b_e2e;   /*   e = c * m_c2e + b_e2e */
    char type;      /* 's' = short, signed o'u' = uint16_t  'i' = 'S'
                       = signed 32 bit int, 'U' = unsigned 32 bit int */
    char quantity[UNITS_LEN]; /* eg, "Temperature" or "Angular Velocity" */
    char units[UNITS_LEN]; /* eg, "K" or "^o/s" */
  };
#pragma pack(pop)

  struct NiosStruct {
    unsigned int niosAddr;
    unsigned int bbcAddr;
    unsigned char fast;
    unsigned char wide;
    unsigned char sign;
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

  extern uint16_t  ccWideFast;
  extern uint16_t  ccNarrowFast;
  extern uint16_t  ccWideSlow;
  extern uint16_t  ccNarrowSlow;
  extern uint16_t  ccFast;
  extern uint16_t  ccDecom;
  extern uint16_t  ccDerived;
  extern uint16_t  ccTotal;

  extern uint16_t  BoloBaseIndex;

#if (defined __MCP__ || defined __DECOMD__)
  extern uint16_t  BiPhaseFrameWords;
  extern uint16_t  BiPhaseFrameSize;
#endif
  extern uint16_t  DiskFrameWords;
  extern uint16_t  DiskFrameSize;
  extern uint16_t  slowsPerBi0Frame;
  extern uint16_t  TxFrameWords[2];
  extern uint16_t  TxFrameSize[2];
  extern unsigned int NiosSpares[FAST_PER_SLOW * 2];
  extern unsigned int BBCSpares[FAST_PER_SLOW * 2];

  void MakeAddressLookups(const char* dump_name); //arg is path to Nios.map dump
  void FPrintDerived(int);
  struct NiosStruct* GetNiosAddr(const char*);
  inline struct BiPhaseStruct* GetBiPhaseAddr(const char*);
  inline struct BiPhaseStruct* ExtractBiPhaseAddr(struct NiosStruct*);
  void ReadSpecificationFile(FILE*);
  void WriteFormatFile(int, time_t, unsigned long);
  char* FieldToLower(char*);
  char* FieldToUpper(char*);

#if (defined __DEFILE__ || defined __BLASTD__)
#include "derived.h"
  extern struct ChannelStruct* WideSlowChannels;
  extern struct ChannelStruct* SlowChannels;
  extern struct ChannelStruct* WideFastChannels;
  extern struct ChannelStruct* FastChannels;
  extern struct ChannelStruct* DecomChannels;
  extern union DerivedUnion* DerivedChannels;
#endif

/* reserved node numbers */
#define SPARE      62
#define SPECIAL    63
#define EOC_MARKER -1

#define END_OF_CHANNELS {"", 'x', EOC_MARKER, -1, 0, 0, 0, 'x', "", ""}

#ifdef __cplusplus

}
#endif

#endif
