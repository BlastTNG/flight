/* channels.c: contains routines for manipulating the BLAST channel lists
 *
 * This software is copyright (C) 2002-2010 University of Toronto
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* Author's Apology:
 * ================
 *
 * This code is purposely tricky.  It was writen to be robust in what it does,
 * with no thought to legibility, or ease of maintenance.  It's basically two
 * complementary programs rolled into one in order to prevent them from
 * diverging.  The two programs are broadly delimited by the INPUTTER define.
 * INPUTTERs, such as defile or interloquendi read framefiles.  Non-INPUTTERs,
 * sunch as decomd or mcp write them.
 *
 * Non-INPUTTERs get their data format from the tx_struct, which has to be
 * compiled into the program.  INPUTTERs get their data format from the .spec
 * file which the Non-INPUTTERs have writen.  In a nutshell, having to deal with
 * these two possibilities is what the bulk of the voodoo in this code is about.
 *
 * Unlike some other parts of the BLAST flight code, the dirfile writer, for
 * example, this code does what it set out to do: it provides a common interface
 * to the channel structures for both readers and writers of framefiles.  As
 * such, it should be fine as it is, unless major feature changes are needed.
 * In the four programs I've listed above you should have sufficient example to
 * understand how these functions work.  The only way a rewrite could make
 * things simpler would be if you split the two program classes (INPUTTER/non-
 * INPUTTER) into separate manifestations, something which would degrade this
 * library's robustness, since it would lead to divergence between the two
 * unless great diligence was practiced to prevent this.
 *
 * If you're looking for something to re-write, try the dirfile writer.
 *
 * Cheers,
 * -dvw
 */

#include <ctype.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include "channels.h"
#include "derived.h"

/* BUOS/BLAMM functions */
#include "blast.h"

/* defile and blastd are inputters */
#if (defined __DEFILE__ || defined __BLASTD__)
#  define INPUTTER
#endif

/* inputters need to know about frameread, outputters need to know about NIOS */
#ifdef INPUTTER
#  include "frameread.h"
#else
#  include "bbc_pci.h"
#endif

/* Be more verbose if we're running mcp */
#ifdef __MCP__
# define VERBOSE
#endif

#ifndef INPUTTER
extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];
extern struct ChannelStruct DecomChannels[];
extern union DerivedUnion DerivedChannels[];
#else
struct ChannelStruct* WideSlowChannels;
struct ChannelStruct* SlowChannels;
struct ChannelStruct* WideFastChannels;
struct ChannelStruct* FastChannels;
struct ChannelStruct* DecomChannels;
union DerivedUnion* DerivedChannels;
#endif

unsigned short ccWideFast;
unsigned short ccNarrowFast;
unsigned short ccWideSlow;
unsigned short ccNarrowSlow;
unsigned short ccSlow;
unsigned short ccFast;
unsigned short ccNoBolos;
unsigned short ccTotal;
unsigned short ccDecom;
unsigned short ccDerived;

unsigned short BiPhaseFrameWords;
unsigned short BiPhaseFrameSize;
unsigned short DiskFrameWords;
unsigned short DiskFrameSize;
unsigned short TxFrameWords[2];
unsigned short TxFrameSize[2];
unsigned short slowsPerBi0Frame;
unsigned short slowCount[2] = {0, 0};
unsigned short slowsPerBusFrame[2] = {0, 0};
unsigned short fastsPerBusFrame[2] = {SLOW_OFFSET, 1};

unsigned short das_cards = DAS_CARDS;

#ifndef INPUTTER
unsigned int NiosSpares[FAST_PER_SLOW * 2];
unsigned int BBCSpares[FAST_PER_SLOW * 2];
struct NiosStruct* NiosLookup;
struct BiPhaseStruct *BiPhaseLookup;
#else
struct ChannelStruct **SlowChList;
struct ChannelStruct *FastChList;
#endif

#define BOLO_BUS  0    /* bus on which the bolometers live */
unsigned int boloIndex[MAX_DAS_CARDS][DAS_CHS][2];
unsigned short BoloBaseIndex;
static struct ChannelStruct BoloChannels[MAX_FAST_BOLOS];

#define SPEC_VERSION 13
#define STRINGIFY2(x) #x
#define STRINGIFY(x) STRINGIFY2(x)
#ifndef INPUTTER
#  define FREADORWRITE fwrite
#  define SPECIFICATIONFILEFUNXION WriteSpecificationFile
#else
#  define FREADORWRITE fread
#  define SPECIFICATIONFILEFUNXION ReadSpecificationFile
#endif
void SPECIFICATIONFILEFUNXION(FILE* fp)
{
#ifdef INPUTTER
  int version = 0;
  int n_fast_bolos;
#endif
  char versionMagic[6] = "DFI" STRINGIFY(SPEC_VERSION);

  if (FREADORWRITE(&versionMagic, 6, 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));

#ifdef INPUTTER
  /* check spec file version */
  if (versionMagic[0] != 'D' || versionMagic[1] != 'F'
      || versionMagic[2] != 'I')
    bputs(fatal, "Spec file too old: version magic not found.\n"
        "To read this file, you will need defile version 2.1\n");
  else {
    version = atoi(&versionMagic[3]);
    if (version != SPEC_VERSION)
      blast_fatal("Unsupported Spec file version: %i (wanted %i).  "
          "To read this file you will need a different version of defile.\n",
          version, SPEC_VERSION);
  }
#endif

  if (FREADORWRITE(&das_cards, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));

#ifdef INPUTTER
  n_fast_bolos = das_cards * (DAS_CHS + DAS_CHS / 2);
#endif

  if (FREADORWRITE(&ccWideSlow, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(&ccNarrowSlow, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(&ccWideFast, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(&ccNarrowFast, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(&ccDecom, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(&ccDerived, sizeof(unsigned short), 1, fp) < 1)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));

#ifdef INPUTTER
  int bus, i;

  slowsPerBusFrame[0] = slowsPerBusFrame[1] = 0;

  /* Reallocate channel lists, if we're reading them */
  WideSlowChannels = reballoc(fatal, WideSlowChannels,
      ccWideSlow * sizeof(struct ChannelStruct));

  SlowChannels = reballoc(fatal, SlowChannels,
      ccNarrowSlow * sizeof(struct ChannelStruct));

  WideFastChannels = reballoc(fatal, WideFastChannels,
      ccWideFast * sizeof(struct ChannelStruct));

  FastChannels = reballoc(fatal, FastChannels,
      ccNarrowFast * sizeof(struct ChannelStruct));

  if (ccDecom > 0)
    DecomChannels = reballoc(fatal, DecomChannels,
        ccDecom * sizeof(struct ChannelStruct));

  DerivedChannels = reballoc(fatal, DerivedChannels,
      ccDerived * sizeof(union DerivedUnion));

  ccSlow = ccNarrowSlow + ccWideSlow;
  ccFast = ccNarrowFast + ccWideFast + n_fast_bolos + ccDecom;
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;
  ccTotal = ccFast + ccSlow;
#endif

  if (FREADORWRITE(WideSlowChannels,
        sizeof(struct ChannelStruct), ccWideSlow, fp) < ccWideSlow)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(SlowChannels,
        sizeof(struct ChannelStruct), ccNarrowSlow, fp) < ccNarrowSlow)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(WideFastChannels,
        sizeof(struct ChannelStruct), ccWideFast, fp) < ccWideFast)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (FREADORWRITE(FastChannels,
        sizeof(struct ChannelStruct), ccNarrowFast, fp) < ccNarrowFast)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));
  if (ccDecom > 0) {
    if (FREADORWRITE(DecomChannels,
          sizeof(struct ChannelStruct), ccDecom, fp) < ccDecom)
      blast_err("FREADORWRITE failed with code %d", ferror(fp));
  }
  if (FREADORWRITE(DerivedChannels,
        sizeof(union DerivedUnion), ccDerived, fp) < ccDerived)
    blast_err("FREADORWRITE failed with code %d", ferror(fp));

#ifdef INPUTTER
  /* Calculate slowsPerBi0Frame */
  for (i = 0; i < ccWideSlow; ++i)
    slowsPerBusFrame[(int)WideSlowChannels[i].bus] += 2;

  for (i = 0; i < ccNarrowSlow; ++i)
    slowsPerBusFrame[(int)SlowChannels[i].bus]++;

  for (bus = 0; bus < 2; ++bus) {
    slowsPerBusFrame[bus] = 1 + (slowsPerBusFrame[bus] - 1) / FAST_PER_SLOW;
    TxFrameWords[bus] = fastsPerBusFrame[bus] + slowsPerBusFrame[bus];
    TxFrameSize[bus] = TxFrameWords[bus] * 4;
  }

  slowsPerBi0Frame = slowsPerBusFrame[0] + slowsPerBusFrame[1];
  DiskFrameWords = SLOW_OFFSET + ccFast + slowsPerBi0Frame + ccWideFast;
  DiskFrameSize = 2 * DiskFrameWords;

#elif defined VERBOSE
  bputs(info, "Channels: Wrote version " STRINGIFY(SPEC_VERSION)
      " specification file.\n");
#endif
}

/************************************************************************/
/*                                                                      */
/*    MakeBoloTable: create the bolometer channel table                 */
/*                                                                      */
/************************************************************************/
static void MakeBoloTable(void) {
  int i, j, index = 0;
  int bolo_node = DAS_START;
  struct ChannelStruct channel = {
    "", 'r', 3, BOLO_BUS, 0, LOCKIN_C2V, LOCKIN_OFFSET, 'u'
  };

#ifdef VERBOSE
  blast_info("Channels: Generating Bolometer Channel Table.\n");
#endif

  for (i = 0; i < das_cards; ++i) {
    if (bolo_node%4 == 0) bolo_node++;  //skip motherboard common nodes
    channel.node = bolo_node++;
    channel.rw = 'r';
    for (j = 0; j < DAS_CHS; j += 2) {
      /* lsw channel at j */
      channel.addr = j;
      sprintf(channel.field, "n%02ic%02ilo", channel.node, j);
      boloIndex[i][j][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* msw at j and j+1 */
      channel.addr = DAS_CHS + (j >> 1);
      sprintf(channel.field, "n%02ic%02ihi", channel.node, j);
      boloIndex[i][j + 1][1] = boloIndex[i][j][1] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* lsw channel at j+1 */
      channel.addr = j+1;
      sprintf(channel.field, "n%02ic%02ilo", channel.node, j+1);
      boloIndex[i][j + 1][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
    }
  }
}

#ifndef INPUTTER
static struct NiosStruct SetNiosData(const struct ChannelStruct *channel,
    int addr, int fast, int wide)
{
  struct NiosStruct NiosData;
  NiosData.field = channel->field;
  NiosData.fast = fast;
  NiosData.wide = wide;
  NiosData.sign = (channel->type == 'u' || channel->type == 'U') ? 0 : 1;
  NiosData.m = channel->m_c2e;
  NiosData.b = channel->b_e2e;
  NiosData.bus = channel->bus;
  NiosData.bbcAddr = (channel->rw =='r' ? BBC_READ : BBC_WRITE)
    | BBC_NODE(channel->node) | BBC_CH(channel->addr);
  NiosData.niosAddr = channel->bus ? BBCPCI_WFRAME2_ADD(addr)
    : BBCPCI_WFRAME1_ADD(addr);

  return NiosData;
}

/* DumpNiosFrame - writes the constructed nios frame to the map file */
static void DumpNiosFrame(const char* fname)
{
  int bus, m, i, j, n, addr, m0addr;
  FILE* map;
  struct NiosStruct* ReverseMap[2][64][64];
  memset(ReverseMap, 0, 2 * 64 * 64 * sizeof(struct NiosStruct*));

  if ((map = fopen(fname, "w")) == NULL)
    return;

  for (bus = 0; bus < 2; ++bus) {
    fprintf(map, "Bus %i Map:\n", bus);
    for (m = 0; m < FAST_PER_SLOW; ++m) {
      for (i = 0; i < TxFrameWords[bus]; ++i) {
        addr = i + bus * BBCPCI_MAX_FRAME_SIZE + m * TxFrameWords[bus];
        m0addr = i + bus * BBCPCI_MAX_FRAME_SIZE;
        n = 0;
        fprintf(map, "%06x ", addr);
        if (i == 0) {
          if (bus) {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(4);
            fprintf(map, "%08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          } else {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(0);
            fprintf(map, "%08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          }
        } else if (i == 1 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(1);
          fprintf(map, "%08x [%2i %3i (%04x)] F FASTSAMP (lsb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 2 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(2);
          fprintf(map, "%08x [%2i %3i (%04x)] F FASTSAMP (msb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 3 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(3);
          fprintf(map, "%08x [%2i %3i (%04x)] M Multiplex Index = %i", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr), m);
        } else {
          for (j = 0; j < ccTotal; ++j)
            if (NiosLookup[j].niosAddr == addr) {
              n++;
              fprintf(map, "%08x [%2i %3i (%04x)] %c %s",
                  NiosLookup[j].bbcAddr,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].index,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].channel,
                  BI0_MAGIC(NiosLookup[j].bbcAddr),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
              ReverseMap[(NiosLookup[j].bbcAddr & BBC_READ) ? 1 : 0]
                [GET_NODE(NiosLookup[j].bbcAddr)]
                [GET_CH(NiosLookup[j].bbcAddr)] = &NiosLookup[j];
              if (NiosLookup[j].wide)
                fprintf(map, " (lsb)");
            } else if (NiosLookup[j].niosAddr == addr - 1 &&
                NiosLookup[j].wide) {
              n++;
              fprintf(map, "%08x [%2i %3i (%04x)] %c %s (msb)",
                  BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].index,
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].channel,
                  BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr)),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
            } else if (NiosLookup[j].fast && NiosLookup[j].niosAddr == m0addr) {
              n++;
              fprintf(map, "%08x [%2i %3i (%04x)] %c %s",
                  NiosLookup[j].bbcAddr,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].index,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].channel,
                  BI0_MAGIC(NiosLookup[j].bbcAddr),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
              if (NiosLookup[j].wide)
                fprintf(map, " (lsb)");
            } else if (NiosLookup[j].fast && NiosLookup[j].niosAddr
                == m0addr - 1 && NiosLookup[j].wide) {
              n++;
              fprintf(map, "%08x [%2i %3i (%04x)] %c %s (msb)",
                  BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].index,
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].channel,
                  BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr)),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
            }
        }
        for (j = 0; j < 2 * FAST_PER_SLOW; ++j) {
          if (NiosSpares[j] == addr) {
            n++;
            fprintf(map, "%08x [%2i %3i (%04x)] X Spare%i", BBCSpares[j],
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].index,
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].channel,
                BI0_MAGIC(BBCSpares[j]), j);
            ReverseMap[0][GET_NODE(BBCSpares[j])][GET_CH(BBCSpares[j])]
              = (struct NiosStruct*)(-(long)j - 100);
          }
        }
        if (n == 0) {
          fprintf(map, "**UNASSIGNED**\n");
          fclose(map);
          blast_fatal("Channels: FATAL: unassigned address in Nios Address"
              " Table.  Consult Nios Map.\n");
        } else if (n != 1) {
          fprintf(map, "**COLLISION**\n");
          fclose(map);
          blast_fatal("Channels: FATAL: collision in Nios Address Table "
              "assignment. Consult Nios Map.\n");
        }
        fprintf(map, "\n");
      }
    }
    fprintf(map, "\n");
  }

  fprintf(map, "BBC Map:\n");
  bus = 0;
  for (i = 0; i < 64; ++i) {
    for (j = 0; j < 64; ++j) {
      if (i == SPECIAL && j <= 4) {
        if (j == 0)
          fprintf(map, "%02i %02i Y %-32s", i, j, "Frame Sync 0");
        else if (j == 1)
          fprintf(map, "%02i %02i F %-32s", i, j, "(lsb) FASTSAMP");
        else if (j == 2)
          fprintf(map, "%02i %02i F %-32s", i, j, "(msb) FASTSAMP");
        else if (j == 3)
          fprintf(map, "%02i %02i F %-32s", i, j, "Multiplex Index");
        else if (j == 4)
          fprintf(map, "%02i %02i Y %-32s", i, j, "Frame Sync 1");

        if (ReverseMap[1][i][j]) {
          if (ReverseMap[1][i][j]->wide) {
            fprintf(map, "%02i %02i %c (lsb) %s", i, j,
                ReverseMap[1][i][j]->fast ? 'f' : 's',
                ReverseMap[1][i][j]->field);
            bus |= 0x2;
          } else
            fprintf(map, "%02i %02i %c %s", i, j, ReverseMap[1][i][j]->fast
                ? 'f' : 's', ReverseMap[1][i][j]->field);
        } else if (bus & 0x2) {
          fprintf(map, "%02i %02i %c (msb) %s", i, j, ReverseMap[1][i][j
              - 1]->fast ? 'f' : 's', ReverseMap[1][i][j - 1]->field);
          bus &= 0x1;
        } else {
          fprintf(map, "%02i %02i - -", i, j);
        }
        fprintf(map, "\n");
      } else if (ReverseMap[0][i][j] || ReverseMap[1][i][j]) {
        if ((long)ReverseMap[0][i][j] <= -100) {
          fprintf(map, "%02i %02i s Spare %02li%-24s", i, j,
              -(long)ReverseMap[0][i][j] - 100, "");
        } else if (ReverseMap[0][i][j]) {
          if (ReverseMap[0][i][j]->wide) {
            fprintf(map, "%02i %02i %c (lsb) %-26s", i, j,
                ReverseMap[0][i][j]->fast ? 'f' : 's',
                ReverseMap[0][i][j]->field);
            bus |= 0x1;
          } else
            fprintf(map, "%02i %02i %c %-32s", i, j, ReverseMap[0][i][j]->fast
                ? 'f' : 's', ReverseMap[0][i][j]->field);
        } else if (bus & 0x1) {
          fprintf(map, "%02i %02i %c (msb) %-26s", i, j, ReverseMap[0][i][j
              - 1]->fast ? 'f' : 's', ReverseMap[0][i][j - 1]->field);
          bus &= 0x2;
        } else
          fprintf(map, "%02i %02i - %-32s", i, j, "-");

        if (ReverseMap[1][i][j]) {
          if (ReverseMap[1][i][j]->wide) {
            fprintf(map, "%02i %02i %c (lsb) %s", i, j,
                ReverseMap[1][i][j]->fast ? 'f' : 's',
                ReverseMap[1][i][j]->field);
            bus |= 0x2;
          } else
            fprintf(map, "%02i %02i %c %s", i, j, ReverseMap[1][i][j]->fast
                ? 'f' : 's', ReverseMap[1][i][j]->field);
        } else if (bus & 0x2) {
          fprintf(map, "%02i %02i %c (msb) %s", i, j, ReverseMap[1][i][j
              - 1]->fast ? 'f' : 's', ReverseMap[1][i][j - 1]->field);
          bus &= 0x1;
        } else {
          fprintf(map, "%02i %02i - -", i, j);
        }
        fprintf(map, "\n");
      } else if (bus) {
        if (bus & 0x1) {
          fprintf(map, "%02i %02i %c (msb) %-26s", i, j, ReverseMap[0][i][j
              - 1]->fast ? 'f' : 's', ReverseMap[0][i][j - 1]->field);
          bus &= 0x2;
        } else
          fprintf(map, "%02i %02i - %-32s", i, j, "-");

        if (bus & 0x2) {
          fprintf(map, "%02i %02i %c (msb) %-19s", i, j, ReverseMap[1][i][j
              - 1]->fast ? 'f' : 's', ReverseMap[1][i][j - 1]->field);
          bus &= 0x1;
        } else
          fprintf(map, "%02i %02i - -", i, j);
        fprintf(map, "\n");
      } else
        fprintf(map, "%02i %02i - %-32s%02i %02i - -\n", i, j, "-", i, j);
    }
    fprintf(map, "\n");
  }

  fclose(map);
#ifdef VERBOSE
  blast_info("Channels: Wrote %s.\n", fname);
#endif
}

static int GetChannelByName(char names[4096][FIELD_LEN], int nn, char* field)
{
  int i;

  for (i = 0; i < nn; ++i) {
    if (strcmp(names[i], field) == 0)
      return i;
  }

  return -1;
}

/* Checks BBC Addresses to see if multiple fields are occupying the same
 * place or namespace */
static void BBCAddressCheck(char names[4096][FIELD_LEN], int nn,
    char* fields[64][64], char* name, int node, int addr)
{
  if (fields[node][addr])
    blast_fatal("Channels: FATAL: Conflicting BBC address found for %s and "
        "%s (node %i channel %i)\n", fields[node][addr], name, node, addr);

  if (nn != -1) {
    if (GetChannelByName(names, nn, name) != -1)
      blast_fatal("Channels: Namespace Collision: Duplicate channel name "
          "%s found\n", name);
    strcpy(names[nn], name);
    strcpy(names[nn + 1], FieldToUpper(name));
  }

  fields[node][addr] = name;
}

/* DoSanityChecks - run various sanity checks on the channel tables.  Also
 * compute useful parameters */
static void DoSanityChecks(void)
{
  const int n_fast_bolos = das_cards * (DAS_CHS + DAS_CHS / 2);
  int i, j, nn = 0;
  char* fields[2][64][64];
  char names[4096][FIELD_LEN];

//#ifdef VERBOSE
//  blast_info("Channels: Running Sanity Checks on Channel Lists.\n");
//#endif

  for (i = 0; i < 64; ++i)
    for (j = 0; j < 64; ++j)
      fields[0][i][j] = fields[1][i][j] = NULL;

  for (i = 0; WideSlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[(int)WideSlowChannels[i].bus] += 2;
    if (WideSlowChannels[i].type != 'U' && WideSlowChannels[i].type != 'S'
        && WideSlowChannels[i].type != 'i')
      blast_fatal("Channels: FATAL: Error in Wide Slow Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideSlowChannels[i].field, WideSlowChannels[i].type);

    BBCAddressCheck(names, nn, fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr);
    BBCAddressCheck(names, -1, fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr + 1);
    nn += 2;
  }
  ccWideSlow = i;

  for (i = 0; SlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[(int)SlowChannels[i].bus]++;
    if (SlowChannels[i].type != 'u' && SlowChannels[i].type != 's')
      blast_fatal("Channels: Error in Slow Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          SlowChannels[i].field, SlowChannels[i].type);

    BBCAddressCheck(names, nn, fields[SlowChannels[i].rw == 'r'],
        SlowChannels[i].field, SlowChannels[i].node, SlowChannels[i].addr);
    nn += 2;
  }
  ccNarrowSlow = i;
  ccSlow = ccNarrowSlow + ccWideSlow;

  for (i = 0; WideFastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)WideFastChannels[i].bus] += 2;
    if (WideFastChannels[i].type != 'U' && WideFastChannels[i].type != 'S'
        && WideFastChannels[i].type != 'i')
      blast_fatal("Channels: FATAL: Error in Wide Fast Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideFastChannels[i].field, WideFastChannels[i].type);

    BBCAddressCheck(names, nn, fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr);
    BBCAddressCheck(names, -1, fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr + 1);
    nn += 2;
  }
  ccWideFast = i;

  for (i = 0; FastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)FastChannels[i].bus]++;
    if (FastChannels[i].type != 'u' && FastChannels[i].type != 's')
      blast_fatal("Channels: Error in Fast Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          FastChannels[i].field, FastChannels[i].type);

    BBCAddressCheck(names, nn, fields[FastChannels[i].rw == 'r'],
        FastChannels[i].field, FastChannels[i].node, FastChannels[i].addr);
    nn += 2;
  }
  ccNarrowFast = i;

  for (i = 0; i < n_fast_bolos; ++i) {
    fastsPerBusFrame[BOLO_BUS]++;
    BBCAddressCheck(names, nn, fields[1], BoloChannels[i].field,
        BoloChannels[i].node, BoloChannels[i].addr);
    nn += 2;
  }
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;

#ifdef __DECOMD__
  for (i = 0; DecomChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)DecomChannels[i].bus]++;
    if (DecomChannels[i].type != 'u' && DecomChannels[i].type != 's')
      blast_fatal("Channels: Error in Decom Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          DecomChannels[i].field, DecomChannels[i].type);

    BBCAddressCheck(names, nn, fields[DecomChannels[i].rw == 'r'],
        DecomChannels[i].field, DecomChannels[i].node, DecomChannels[i].addr);
    nn += 2;
  }
  ccDecom = i;
#else
  ccDecom = 0;
#endif

  ccFast = ccWideFast + ccNarrowFast + n_fast_bolos + ccDecom;
  ccTotal = ccFast + ccSlow;

  /* Calculate slowsPerBi0Frame */
  for (i = 0; i < 2; ++ i) {
    slowsPerBusFrame[i] = 1 + (slowCount[i] - 1) / FAST_PER_SLOW;
    TxFrameWords[i] = fastsPerBusFrame[i] + slowsPerBusFrame[i];
    TxFrameSize[i] = TxFrameWords[i] * 4;
  }

  slowsPerBi0Frame = slowsPerBusFrame[0] + slowsPerBusFrame[1];

  DiskFrameWords = SLOW_OFFSET + ccFast + slowsPerBi0Frame + ccWideFast;
  DiskFrameSize = DiskFrameWords * 2;
  BiPhaseFrameWords = DiskFrameWords - ccDecom;
  BiPhaseFrameSize = BiPhaseFrameWords * 2;

  for (i = 0; DerivedChannels[i].comment.type != DERIVED_EOC_MARKER; ++i)
    switch (DerivedChannels[i].comment.type) {
      case 'b': /* bitfield */
        if (GetChannelByName(names, nn, DerivedChannels[i].bitfield.source)
            == -1)
          blast_fatal("Channels: Bitfield source %s not found.",
              DerivedChannels[i].bitfield.source);

        for (j = 0; j < 16; ++j) {
          if (DerivedChannels[i].bitfield.field[j][0] && GetChannelByName(names,
                nn, DerivedChannels[i].bitfield.field[j]) != -1)
            blast_fatal("Channels: Namespace Collision: Duplicate channel "
                "name %s found in derived channels",
                DerivedChannels[i].bitfield.field[j]);
          strcpy(names[nn++], DerivedChannels[i].bitfield.field[j]);
        }
        break;
      case '2': /* lincom2 -- one extra check from lincom */
      case '/': /* divide  -- ditto */
      case '*': /* multiply-- ditto */
      case 'x': /* mplex */
        if (GetChannelByName(names, nn, DerivedChannels[i].lincom2.source2)
            == -1)
          blast_fatal("Channels: Derived channel source %s not found.",
              DerivedChannels[i].lincom2.source2);

        /* FALLTHROUGH */
      case 'w': /* bitword -- same checks as lincom */
      case 't': /* linterp -- same checks as lincom */
      case 'p': /* phase   -- same checks as lincom */
      case 'r': /* recip   -- same checks as lincom */
      case 'c': /* lincom */
        if (GetChannelByName(names, nn, DerivedChannels[i].lincom.source) == -1)
          //bolometer channel exception.
          //TODO (BLAST-Pol OK) bolo channels should be searched
          if (das_cards > 0 && ((DerivedChannels[i].lincom.source[0] != 'n' &&
                  DerivedChannels[i].lincom.source[0] != 'N') ||
                (DerivedChannels[i].lincom.source[3] != 'c' &&
                 DerivedChannels[i].lincom.source[3] != 'C') ||
                !isdigit(DerivedChannels[i].lincom.source[1]) ||
                !isdigit(DerivedChannels[i].lincom.source[2]) ||
                !isdigit(DerivedChannels[i].lincom.source[4]) ||
                !isdigit(DerivedChannels[i].lincom.source[5]) ||
                DerivedChannels[i].lincom.source[6] != '\0'))
          {
            blast_fatal("Channels: Derived channel source %s not found.",
                DerivedChannels[i].lincom.source);
          }

        if (GetChannelByName(names, nn, DerivedChannels[i].lincom.field) != -1)
          blast_fatal("Channels: Namespace Collision: Duplicate channel "
              "name %s found in derived channels",
              DerivedChannels[i].lincom.field);
        strcpy(names[nn++], DerivedChannels[i].lincom.field);

        /* FALLTHROUGH */
      case '#': /* comment -- they always pass the check */
        break;
      case 'u': /* Units metadata */
        if (GetChannelByName(names, nn, DerivedChannels[i].units.source) == -1)
          blast_fatal("Channels: Derived channel source %s not found.",
              DerivedChannels[i].units.source);
        //TODO (BLAST-Pol OK) add checks for metadata collisions?
        break;
      default:
        blast_fatal("Channels: FATAL: Unrecognised Derived Channel Type "
            "`%c'\n", DerivedChannels[i].comment.type);
    }
  ccDerived = i;

#ifdef VERBOSE
//  blast_info("Channels: All Checks Passed.\n");
  blast_info("Channels: Number of Derived Channel Records: %i\n", ccDerived);
  blast_info("Channels: Slow Channels Per Biphase Frame: %i\n",
      slowsPerBi0Frame);
  blast_info("Channels: Fast Channels Per Biphase Frame: %i\n",
      ccFast + SLOW_OFFSET);
  blast_info("Channels: Slow Channels Per Tx Frame: %i / %i\n",
      slowsPerBusFrame[0], slowsPerBusFrame[1]);
  blast_info("Channels: Fast Channels Per Tx Frame: %i / %i\n",
      fastsPerBusFrame[0], fastsPerBusFrame[1]);
#endif

  for (i = 0; i < 2; ++i) {
#ifdef __DECOMD__
//in decomd, add extra space for decom channels
#ifdef VERBOSE
    blast_info("Channels: BBC Bus %i: Frame Bytes: %4i  Allowed: %4i "
        "(%.2f%% full)\n", i, 4 * TxFrameWords[i], 4 * BBC_FRAME_SIZE,
        100. * TxFrameWords[i] / (BBC_FRAME_SIZE+3));
#endif
    if (TxFrameWords[i] > BBC_FRAME_SIZE+3)
      blast_fatal("Channels: FATAL: BBC Bus %i frame too big.\n", i);
#else  //__DECOMD__
//outside decomd, enforce unaltered limits
#ifdef VERBOSE
    blast_info("Channels: BBC Bus %i: Frame Bytes: %4i  Allowed: %4i "
        "(%.2f%% full)\n", i, 4 * TxFrameWords[i], 4 * BBC_FRAME_SIZE,
        100. * TxFrameWords[i] / BBC_FRAME_SIZE);
#endif
    if (TxFrameWords[i] > BBC_FRAME_SIZE)
      blast_fatal("Channels: FATAL: BBC Bus %i frame too big.\n", i);
#endif  //__DECOMD__
  }

#ifdef VERBOSE
  blast_info("Channels:  BiPhase : Frame Bytes: %4i  Allowed: %4i "
      "(%.2f%% full)\n", 2 * BiPhaseFrameWords, 2 * BI0_FRAME_SIZE,
      100. * BiPhaseFrameWords / BI0_FRAME_SIZE);
#endif
  if (BiPhaseFrameWords > BI0_FRAME_SIZE)
    blast_fatal("Channels: FATAL: Biphase frame too big.\n");
}
#endif

/* MakeAddressLookups - fills the nios and biphase address lookup tables */
void MakeAddressLookups(const char* dump_name)
{
  const int n_fast_bolos = das_cards * (DAS_CHS + DAS_CHS / 2);
  int i, mplex, bus, spare_count;
  int slowIndex[2][FAST_PER_SLOW];

#ifdef INPUTTER
  struct ChannelStruct EmptyChannel = {"", 'w', -1, -1, -1, 1, 1};
#endif

  if (das_cards > 0)
    MakeBoloTable();

#ifndef INPUTTER
  DoSanityChecks();
#endif

//#ifdef VERBOSE
//  blast_info("Channels: Generating Address Lookup Tables\n");
//#endif

  unsigned int BiPhaseAddr;
  unsigned int addr[2] = {
    SLOW_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
  };

  const int slowTop[2] = {
#ifndef INPUTTER
    SLOW_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
#else
      slowsPerBusFrame[0],
    slowsPerBusFrame[1]
#endif
  };

#ifndef INPUTTER
  BiPhaseAddr = SLOW_OFFSET + slowsPerBi0Frame;
  /* allocate the Nios address table */
  NiosLookup = balloc(tfatal, ccTotal * sizeof(struct NiosStruct));

  /* allocate the BiPhase address table */
  BiPhaseLookup = balloc(tfatal, BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct));

  /* fill BiPhase Lookup with invalid data */
  memset(BiPhaseLookup, 0xff, BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct));
#else
  BiPhaseAddr = 0;
  /* allocate the Defile address tables */
  FastChList = balloc(fatal, (ccFast + ccWideFast)
      * sizeof(struct ChannelStruct));

  SlowChList = balloc(fatal, slowsPerBi0Frame * sizeof(struct ChannelStruct*));

  for (i = 0; i < slowsPerBi0Frame; ++i)
    SlowChList[i] = balloc(fatal, FAST_PER_SLOW * sizeof(struct ChannelStruct));
#endif

  /* initialise slow channels */
  for (i = 0; i < FAST_PER_SLOW; ++i) {
#ifndef INPUTTER
    slowIndex[0][i] = SLOW_OFFSET;
    slowIndex[1][i] = 1;
#else
    slowIndex[0][i] = 0;
    slowIndex[1][i] = 0;
#endif
  }

#ifndef INPUTTER
  for (i = 0; i < FAST_PER_SLOW * 2; ++i)
    NiosSpares[i] = -1;
#endif

  /* populate slow channels */
  for (i = 0; i < ccWideSlow; ++i) {
    bus = WideSlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] + 1 >= slowTop[bus])
      if (++mplex >= FAST_PER_SLOW)
      {
        blast_err("failing on index: %d of %d", i, ccWideSlow-1);
        blast_fatal("Channels: FATAL: Ran out of subframes while trying to "
            "insert wide slow channel %s\n", WideSlowChannels[i].field);
      }

#ifndef INPUTTER
    NiosLookup[i] = SetNiosData(&WideSlowChannels[i], mplex * TxFrameWords[bus]
        + slowIndex[bus][mplex], 0, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + SLOW_OFFSET - 1)
      - SLOW_OFFSET;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].index
      = mplex;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].channel
      = slowIndex[bus][mplex] + 1 + bus * (slowsPerBusFrame[0] + SLOW_OFFSET
          - 1) - SLOW_OFFSET;
#else
    SlowChList[slowIndex[bus][mplex] + bus * slowsPerBusFrame[0]][mplex]
      = WideSlowChannels[i];
    SlowChList[slowIndex[bus][mplex] + 1 + bus * slowsPerBusFrame[0]][mplex]
      = EmptyChannel;
#endif

    slowIndex[bus][mplex] += 2;
  }

  for (i = 0; i < ccNarrowSlow; ++i) {
    bus = SlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] >= slowTop[bus])
      if (++mplex >= FAST_PER_SLOW)
        blast_fatal("Channels: FATAL: Ran out of subframes while trying to "
            "insert slow channel %s\n", SlowChannels[i].field);

#ifndef INPUTTER
    NiosLookup[i + ccWideSlow] = SetNiosData(&SlowChannels[i],
        mplex * TxFrameWords[bus] + slowIndex[bus][mplex], 0, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + SLOW_OFFSET - 1)
      - SLOW_OFFSET;
#else
    SlowChList[slowIndex[bus][mplex] + bus * slowsPerBusFrame[0]][mplex]
      = SlowChannels[i];
#endif

    slowIndex[bus][mplex]++;
  }

  /* Fill Up remaining Slow Channels with spares */
  spare_count = 0;
  for (bus = 0; bus < 2; ++bus)
    for (mplex = 0; mplex < FAST_PER_SLOW; ++mplex)
      while (slowIndex[bus][mplex] < slowTop[bus]) {
#ifndef INPUTTER
        BBCSpares[spare_count] = BBC_WRITE | BBC_NODE(SPARE) |
          BBC_CH(spare_count);
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].index = mplex;
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].channel
          = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0]
              + SLOW_OFFSET - 1) - SLOW_OFFSET;
        i = mplex * TxFrameWords[bus] + slowIndex[bus][mplex];
        NiosSpares[spare_count] = bus ? BBCPCI_WFRAME2_ADD(i)
          : BBCPCI_WFRAME1_ADD(i);
#else
        SlowChList[slowIndex[bus][mplex] + bus * slowsPerBusFrame[0]][mplex]
          = EmptyChannel;
#endif
        slowIndex[bus][mplex]++;
        spare_count++;
      }

#ifdef VERBOSE
  blast_info("Channels: Added %i spare slow channels.\n", spare_count);
#endif

  for (i = 0; i < ccNarrowFast; ++i) {
#ifndef INPUTTER
    NiosLookup[i + ccSlow + ccWideFast] = SetNiosData(&FastChannels[i],
        addr[(int)FastChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow + ccWideFast].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow
        + ccWideFast].bbcAddr)].channel = BiPhaseAddr++;
#else
    FastChList[BiPhaseAddr++] = FastChannels[i];
#endif

    addr[(int)FastChannels[i].bus]++;
  }

  for (i = 0; i < ccWideFast; ++i) {
#ifndef INPUTTER
    NiosLookup[i + ccSlow] = SetNiosData(&WideFastChannels[i],
        addr[(int)WideFastChannels[i].bus], 1, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].channel
      = BiPhaseAddr++;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].index = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].channel = BiPhaseAddr++;
#else
    FastChList[BiPhaseAddr++] = WideFastChannels[i];
    FastChList[BiPhaseAddr++] = EmptyChannel;
#endif

    addr[(int)WideFastChannels[i].bus] += 2;
  }

  if (das_cards > 0) {
#ifdef INPUTTER
    /* save the location of the first bolometer in the frame so that defile
     * can calculate the offsets properly when it goes to reconstruct the
     * bolometers */
    BoloBaseIndex = BiPhaseAddr + slowsPerBi0Frame + SLOW_OFFSET;
#endif

    for (i = 0; i < n_fast_bolos; ++i) {
#ifndef INPUTTER
      NiosLookup[i + ccNoBolos] = SetNiosData(&BoloChannels[i],
          addr[(int)BoloChannels[i].bus], 1, 0);

      BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].index
        = NOT_MULTIPLEXED;
      BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].channel
        = BiPhaseAddr++;
#else
      FastChList[BiPhaseAddr++] = BoloChannels[i];
#endif

      addr[(int)BoloChannels[i].bus]++;
    }
  }

  for (i = 0; i < ccDecom; ++i) {
#ifndef INPUTTER
    NiosLookup[i + ccNoBolos + n_fast_bolos] = SetNiosData(&DecomChannels[i],
        addr[(int)DecomChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos
        + n_fast_bolos].bbcAddr)].index = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos
        + n_fast_bolos].bbcAddr)].channel = BiPhaseAddr++;
#else
    FastChList[BiPhaseAddr++] = DecomChannels[i];
#endif

    addr[(int)DecomChannels[i].bus]++;
  }

#ifndef INPUTTER
  /* Add the channels that aren't in the channel list to the Biphase lookup */

  /* Bus 0 Frame Sync */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].channel = 0;

  /* FASTSAMP msb */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].channel = 1;

  /* FASTSAMP lsb */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].channel = 2;

  /* Bus 1 Frame Sync */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].channel = 3;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(4)) >> 16].index = DISCARD_WORD;

  if (dump_name) DumpNiosFrame(dump_name);
#endif
}

/* If you need to access these, trust me, buddy, you aren't an inputter */
#ifndef INPUTTER
inline struct BiPhaseStruct* ExtractBiPhaseAddr(struct NiosStruct* niosAddr)
{
  struct BiPhaseStruct* bps = &BiPhaseLookup[BI0_MAGIC(niosAddr->bbcAddr)];
  bps->nios = niosAddr;
  return bps;
}

//MWG: This used to be inline; problems compiling encouraged me to simplify.
struct BiPhaseStruct* GetBiPhaseAddr(const char* field)
{
  struct NiosStruct* niosAddr = GetNiosAddr(field);
  struct BiPhaseStruct* bps = &BiPhaseLookup[BI0_MAGIC(niosAddr->bbcAddr)];
  bps->nios = niosAddr;
  return bps;
}

/************************************************************************/
/*                                                                      */
/*    GetNiosAddr                                                       */
/*                                                                      */
/************************************************************************/
struct NiosStruct* GetNiosAddr(const char* field) {
  int i;

  for (i = 0; i < ccTotal; ++i)
    if (strcmp(NiosLookup[i].field, field) == 0)
      return &NiosLookup[i];

  blast_fatal("Channels: Nios Lookup for channel %s failed.\n", field);

  return NULL;
}
#endif

/* NB: This function is non-reentrant */
char* FieldToLower(char* s)
{
  int i;
  static char ls[FIELD_LEN];

  for (i = 0; i < FIELD_LEN; i++)
    ls[i] = tolower(s[i]);

  return(ls);
}

/* NB: This function is non-reentrant */
char* FieldToUpper(char* s)
{
  int i;
  static char us[FIELD_LEN];

  for (i = 0; i < FIELD_LEN; i++)
    us[i] = toupper(s[i]);

  return(us);
}

void WriteFormatFile(int fd, time_t start_time, unsigned long offset)
{
  char field[FIELD_LEN];
  int bolo_node;
  char line[1024];
  int i, j;

  sprintf(line, "FASTSAMP         RAW    U %i\n", FAST_PER_SLOW);
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");


#ifdef __DEFILE__
  sprintf(line, "\n## DEFILE FRAME RECONSTRUCTION FLAGS:\n"
      "DEFILE_FLAGS     RAW   u %i\n\n"
      "# Defile removed one or more zeroed frames before this fast frame\n"
      "DEFILE_ZEROED    BIT  DEFILE_FLAGS 0\n"
      "# Defile corrected what looked like a mangled index in this fast frame\n"
      "DEFILE_MANGLED   BIT  DEFILE_FLAGS 1\n"
      "# Defile inserted this fast frame -- it's recycled old data\n"
      "DEFILE_INSERTED  BIT  DEFILE_FLAGS 2\n"
      "# Defile found a single frame error in the stream and replaced it with "
      "old datax\n"
      "DEFILE_SINGLE    BIT  DEFILE_FLAGS 3\n", FAST_PER_SLOW);
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");
#endif

  strcpy(line, "\n## SLOW CHANNELS:\n");
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");

  for (i = 0; i < ccNarrowSlow; ++i) {
    snprintf(line, 1024,
        "%-16s RAW    %c 1\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
        FieldToLower(SlowChannels[i].field), SlowChannels[i].type,
        FieldToUpper(SlowChannels[i].field),
        FieldToLower(SlowChannels[i].field), SlowChannels[i].m_c2e,
        SlowChannels[i].b_e2e);
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
    if (strlen(SlowChannels[i].quantity)>0) {
      snprintf(line, 1024, "%s/quantity STRING %s\n",
          FieldToUpper(SlowChannels[i].field), SlowChannels[i].quantity);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
    if (strlen(SlowChannels[i].units)>0) {
      snprintf(line, 1024, "%s/units STRING %s\n",
          FieldToUpper(SlowChannels[i].field), SlowChannels[i].units);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
  }

  for (i = 0; i < ccWideSlow; ++i) {
    snprintf(line, 1024,
        "%-16s RAW    %c 1\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
        FieldToLower(WideSlowChannels[i].field), WideSlowChannels[i].type,
        FieldToUpper(WideSlowChannels[i].field),
        FieldToLower(WideSlowChannels[i].field), WideSlowChannels[i].m_c2e,
        WideSlowChannels[i].b_e2e);
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
    if (strlen(WideSlowChannels[i].quantity)>0) {
      snprintf(line, 1024, "%s/quantity STRING %s\n",
          FieldToUpper(WideSlowChannels[i].field),
          WideSlowChannels[i].quantity);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
    if (strlen(WideSlowChannels[i].units)>0) {
      snprintf(line, 1024, "%s/units STRING %s\n",
          FieldToUpper(WideSlowChannels[i].field), WideSlowChannels[i].units);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
  }

  strcpy(line, "\n## FAST CHANNELS:\n");
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");

  for (i = 0; i < ccNarrowFast; i++) {
    snprintf(line, 1024,
        "%-16s RAW    %c %d\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
        FieldToLower(FastChannels[i].field), FastChannels[i].type,
        FAST_PER_SLOW, FieldToUpper(FastChannels[i].field),
        FieldToLower(FastChannels[i].field), FastChannels[i].m_c2e,
        FastChannels[i].b_e2e);
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
    if (strlen(FastChannels[i].quantity)>0) {
      snprintf(line, 1024, "%s/quantity STRING %s\n",
          FieldToUpper(FastChannels[i].field), FastChannels[i].quantity);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
    if (strlen(FastChannels[i].units)>0) {
      snprintf(line, 1024, "%s/units STRING %s\n",
          FieldToUpper(FastChannels[i].field), FastChannels[i].units);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
  }

  for (i = 0; i < ccWideFast; i++) {
    snprintf(line, 1024,
        "%-16s RAW    %c %d\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
        FieldToLower(WideFastChannels[i].field), WideFastChannels[i].type,
        FAST_PER_SLOW, FieldToUpper(WideFastChannels[i].field),
        FieldToLower(WideFastChannels[i].field), WideFastChannels[i].m_c2e,
        WideFastChannels[i].b_e2e);
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
    if (strlen(WideFastChannels[i].quantity)>0) {
      snprintf(line, 1024, "%s/quantity STRING %s\n",
          FieldToUpper(WideFastChannels[i].field),
          WideFastChannels[i].quantity);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
    if (strlen(WideFastChannels[i].units)>0) {
      snprintf(line, 1024, "%s/units STRING %s\n",
          FieldToUpper(WideFastChannels[i].field), WideFastChannels[i].units);
      if (write(fd, line, strlen(line)) < 0)
        berror(err, "Error writing to format file\n");
    }
  }

  for (i = 0; i < ccDecom; i++) {
    snprintf(line, 1024,
        "%-16s RAW    %c %d\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
        FieldToLower(DecomChannels[i].field), DecomChannels[i].type,
        FAST_PER_SLOW, FieldToUpper(DecomChannels[i].field),
        FieldToLower(DecomChannels[i].field), DecomChannels[i].m_c2e,
        DecomChannels[i].b_e2e);
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
  }

  if (das_cards > 0) {
    /* bolo channels */
    strcpy(line, "\n## BOLOMETER:\n");
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");

    bolo_node = DAS_START;
    for (i = 0; i < das_cards; i++) {
      bolo_node++;
      if (bolo_node%4 == 0) bolo_node++;
      for (j = 0; j < DAS_CHS; j++) {
        sprintf(field, "n%02dc%02d", bolo_node, j);
        snprintf(line, 1024,
            "%-16s RAW    U %d\n%-16s LINCOM 1 %-16s %.12e %.12e\n",
            FieldToLower(field), FAST_PER_SLOW, FieldToUpper(field),
            FieldToLower(field), LOCKIN_C2V, LOCKIN_OFFSET);
        if (write(fd, line, strlen(line)) < 0)
          berror(err, "Error writing to format file\n");
      }
    }
  }

  /* derived channels */
  strcpy(line, "\n## DERIVED CHANNELS:\n");
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");

  for (i = 0; i < ccDerived; ++i) {
    switch (DerivedChannels[i].comment.type) {
      case 'b': /* bitfield */
        snprintf(line, 1024, "\n# %s BITFIELD:\n",
            DerivedChannels[i].bitfield.source);
        for (j = 0; j < 16; ++j)
          if (DerivedChannels[i].bitfield.field[j][0]) {
            if (write(fd, line, strlen(line)) < 0)
              berror(err, "Error writing to format file\n");
            snprintf(line, 1024, "%-16s BIT %-16s %i\n",
                DerivedChannels[i].bitfield.field[j],
                DerivedChannels[i].bitfield.source, j);
          }
        break;
      case 'x': /* mplex */
        snprintf(line, 1024, "%-16s MPLEX %-16s %-16s %i %i\n",
            DerivedChannels[i].mplex.field, DerivedChannels[i].mplex.source,
            DerivedChannels[i].mplex.index, DerivedChannels[i].mplex.value,
            DerivedChannels[i].mplex.max);
        break;
      case 'c': /* lincom */
        snprintf(line, 1024, "%-16s LINCOM 1 %-16s %.12e %.12e\n",
            DerivedChannels[i].lincom.field, DerivedChannels[i].lincom.source,
            DerivedChannels[i].lincom.m_c2e, DerivedChannels[i].lincom.b_e2e);
        break;
      case '2': /* lincom2 */
        snprintf(line, 1024,
            "%-16s LINCOM 2 %-16s %.12e %.12e %-16s %.12e %.12e\n",
            DerivedChannels[i].lincom2.field, DerivedChannels[i].lincom2.source,
            DerivedChannels[i].lincom2.m_c2e, DerivedChannels[i].lincom2.b_e2e,
            DerivedChannels[i].lincom2.source2,
            DerivedChannels[i].lincom2.m2_c2e,
            DerivedChannels[i].lincom2.b2_e2e);
        break;
      case 't': /* linterp */
        snprintf(line, 1024, "%-16s LINTERP %-16s %s\n",
            DerivedChannels[i].linterp.field, DerivedChannels[i].linterp.source,
            DerivedChannels[i].linterp.lut);
        break;
      case 'w': /* bitword */
        snprintf(line, 1024, "%-16s BIT %-16s %i %i\n",
            DerivedChannels[i].bitword.field, DerivedChannels[i].bitword.source,
            DerivedChannels[i].bitword.offset,
            DerivedChannels[i].bitword.length);
        break;
      case '#': /* comment */
        snprintf(line, 1024, "\n# %s\n", DerivedChannels[i].comment.text);
        break;
      case 'u': /* units channel */
        snprintf(line, 1024, "%s/units STRING %s\n%s/quantity STRING %s\n",
            DerivedChannels[i].units.source, DerivedChannels[i].units.units,
            DerivedChannels[i].units.source, DerivedChannels[i].units.quantity);
        break;
      case 'p': /* phase */
        snprintf(line, 1024, "%-16s PHASE %-16s %i\n",
            DerivedChannels[i].phase.field, DerivedChannels[i].phase.source,
            DerivedChannels[i].phase.shift);
        break;
      case 'r': /* reciprocal */
        snprintf(line, 1024, "%-16s RECIP %-16s %.12e\n",
            DerivedChannels[i].recip.field, DerivedChannels[i].recip.source,
            DerivedChannels[i].recip.dividend);
        break;
      case '/': /* divide */
        snprintf(line, 1024, "%-16s DIVIDE %-16s %-16s\n",
            DerivedChannels[i].math.field, DerivedChannels[i].math.source,
            DerivedChannels[i].math.source2);
        break;
      case '*': /* multiply */
        snprintf(line, 1024, "%-16s MULTIPLY %-16s %-16s\n",
            DerivedChannels[i].math.field, DerivedChannels[i].math.source,
            DerivedChannels[i].math.source2);
        break;
    }
    if (write(fd, line, strlen(line)) < 0)
      berror(err, "Error writing to format file\n");
  }

  snprintf(line, 1024, "\n# Nice CPU Values\n"
      "TIME_SEC_DIRFILE  LINCOM  1       time 1 -%lu\n"
      "TIME_HOUR_DIRFILE LINCOM 1       TIME_SEC_DIRFILE 0.000277777 0\n"
      "TIME_DAY_DIRFILE  LINCOM  1      TIME_SEC_DIRFILE 1.15741E-5  0\n",
      start_time
    );
  if (write(fd, line, strlen(line)) < 0)
    berror(err, "Error writing to format file\n");
}
