/* channels.c: contains routines for manipulating the BLAST channel lists
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

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "channels.h"

/* bputs functions */
#include "blast.h"

/* defile and blastd are inputters */
#if (defined __DEFILE__ || defined __BLASTD__)
#  define INPUTTER
#endif

#ifdef INPUTTER
#  include "frameread.h"
#else
#  include "bbc_pci.h"
#endif

/* Be more verbose if we're running mcp */
#ifdef __MCP__
# define VERBOSE
#endif

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

#ifndef INPUTTER
extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];
extern struct ChannelStruct DecomChannels[];
#else
struct ChannelStruct* WideSlowChannels;
struct ChannelStruct* SlowChannels;
struct ChannelStruct* WideFastChannels;
struct ChannelStruct* FastChannels;
struct ChannelStruct* DecomChannels;
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

unsigned short BoloBaseIndex;

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

#ifndef INPUTTER
unsigned int NiosSpares[FAST_PER_SLOW * 2];
unsigned int BBCSpares[FAST_PER_SLOW * 2];
struct NiosStruct* NiosLookup;
struct BiPhaseStruct *BiPhaseLookup;
#else
struct ChannelStruct **SlowChList;
struct ChannelStruct *FastChList;
#endif

/* bus on which the bolometers are */
#define BOLO_BUS  1

struct ChannelStruct BoloChannels[N_FAST_BOLOS];

#define SPEC_VERSION "10"
#ifndef INPUTTER
#  define FREADORWRITE fwrite
#  define SPECIFICATIONFILEFUNXION WriteSpecificationFile
#else
#  define FREADORWRITE fread
#  define SPECIFICATIONFILEFUNXION ReadSpecificationFile
#endif
void SPECIFICATIONFILEFUNXION(FILE* fp)
{
  char versionMagic[6] = "DFI" SPEC_VERSION;

  FREADORWRITE(&versionMagic, 6, 1, fp);

#ifdef INPUTTER
  /* check spec file version */
  if (versionMagic[0] != 'D' || versionMagic[1] != 'F'
      || versionMagic[2] != 'I')
    berror(fatal, "Spec file too old: version magic not found.\n"
        "To read this file, you will need defile version 2.1\n");
  else {
    int version = atoi(&versionMagic[3]);
    if (version != 10)
      berror(fatal, "Unsupported Spec file version: %i.  Cannot continue.\n",
          version);
  }
#endif

  FREADORWRITE(&ccWideSlow, sizeof(unsigned short), 1, fp);
  FREADORWRITE(&ccNarrowSlow, sizeof(unsigned short), 1, fp);
  FREADORWRITE(&ccWideFast, sizeof(unsigned short), 1, fp);
  FREADORWRITE(&ccNarrowFast, sizeof(unsigned short), 1, fp);
  FREADORWRITE(&ccDecom, sizeof(unsigned short), 1, fp);

#ifdef INPUTTER
  int bus, i;

  slowsPerBusFrame[0] = slowsPerBusFrame[1] = 0;

  /* Reallocate channel lists, if we're reading them */
  if ((WideSlowChannels = realloc(WideSlowChannels,
          ccWideSlow * sizeof(struct ChannelStruct))) == NULL)
    berror(fatal, "unable to allocate heap");

  if ((SlowChannels = realloc(SlowChannels,
          ccNarrowSlow * sizeof(struct ChannelStruct))) == NULL)
    berror(fatal, "unable to allocate heap");

  if ((WideFastChannels = realloc(WideFastChannels,
          ccWideFast * sizeof(struct ChannelStruct))) == NULL)
    berror(fatal, "unable to allocate heap");

  if ((FastChannels = realloc(FastChannels,
          ccNarrowFast * sizeof(struct ChannelStruct))) == NULL)
    berror(fatal, "unable to allocate heap");

  if (ccDecom > 0)
    if ((DecomChannels = realloc(DecomChannels,
            ccDecom * sizeof(struct ChannelStruct))) == NULL)
      berror(fatal, "unable to allocate heap");

  ccSlow = ccNarrowSlow + ccWideSlow;
  ccFast = ccNarrowFast + ccWideFast + N_FAST_BOLOS + ccDecom;
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;
  ccTotal = ccFast + ccSlow;
#endif

  FREADORWRITE(WideSlowChannels, sizeof(struct ChannelStruct), ccWideSlow, fp);
  FREADORWRITE(SlowChannels, sizeof(struct ChannelStruct), ccNarrowSlow, fp);
  FREADORWRITE(WideFastChannels, sizeof(struct ChannelStruct), ccWideFast, fp);
  FREADORWRITE(FastChannels, sizeof(struct ChannelStruct), ccNarrowFast, fp);
  if (ccDecom > 0)
    FREADORWRITE(DecomChannels, sizeof(struct ChannelStruct), ccDecom, fp);

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

  bprintf(info, "Slow Channels per BiPhase Frame: %i\n",
      slowsPerBi0Frame);

#elif defined VERBOSE
  bputs(info, "Wrote version " SPEC_VERSION " specification file.\n");
#endif
}

/************************************************************************/
/*                                                                      */
/*    MakeBoloTable: create the bolometer channel table                 */
/*                                                                      */
/************************************************************************/
void MakeBoloTable(void) {
  int i, j, index = 0;
  struct ChannelStruct channel = {
    "", 'r', 3, BOLO_BUS, 0, 1.19209e-7, -2097152.0, 'u'
  };

#ifdef VERBOSE
  bprintf(info, "Generating Bolometer Channel Table.\n");
#endif

  for (i = 0; i < DAS_CARDS; ++i) {
    channel.node = i + 5;
    channel.rw = 'r';
    for (j = 0; j < DAS_CHS; j += 2) {
      /* lsw channel at j */
      channel.addr = j;
      sprintf(channel.field, "n%ic%ilo", channel.node, j);
      boloIndex[i][j][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* msw at j and j+1 */
      channel.addr = DAS_CHS + (j >> 1);
      sprintf(channel.field, "n%ic%ihi", channel.node, j);
      boloIndex[i][j + 1][1] = boloIndex[i][j][1] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* lsw channel at j+1 */
      channel.addr = j+1;
      sprintf(channel.field, "n%ic%ilo", channel.node, j+1);
      boloIndex[i][j + 1][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
    }
  }
}

#ifndef INPUTTER
struct NiosStruct SetNiosData(const struct ChannelStruct *channel, int addr,
    int fast, int wide)
{
  struct NiosStruct NiosData;
  NiosData.field = channel->field;
  NiosData.fast = fast;
  NiosData.wide = wide;
  NiosData.bus = channel->bus;
  NiosData.bbcAddr = (channel->rw =='r' ? BBC_READ : BBC_WRITE)
    | BBC_NODE(channel->node) | BBC_CH(channel->addr);
  NiosData.niosAddr = channel->bus ? BBCPCI_WFRAME2_ADD(addr)
    : BBCPCI_WFRAME1_ADD(addr);

  return NiosData;
}

/* DumpNiosFrame - writes the constructed nios frame to the map file */
void DumpNiosFrame(void)
{
  int bus, m, i, j, n, addr, m0addr;
  FILE* map;
  struct NiosStruct* ReverseMap[2][64][64];
  memset(ReverseMap, 0, 2 * 64 * 64 * sizeof(struct NiosStruct*));

  if ((map = fopen("/data/etc/Nios.map", "w")) == NULL)
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
              = (struct NiosStruct*)(-j - 100);
          }
        }
        if (n == 0) {
          fprintf(map, "**UNASSIGNED**\n");
          fclose(map);
          bprintf(fatal, "FATAL: unassigned address in Nios Address Table."
              " Consult Nios Map.\n");
        } else if (n != 1) {
          fprintf(map, "**COLLISION**\n");
          fclose(map);
          bprintf(fatal, "FATAL: collision in Nios Address Table "
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
        if ((int)ReverseMap[0][i][j] <= -100) {
          fprintf(map, "%02i %02i s Spare %02i%-24s", i, j,
              -(int)ReverseMap[0][i][j] - 100, "");
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
  bprintf(info, "Wrote /data/etc/Nios.map.\n");
#endif
}
#endif

/* Checks BBC Addresses to see if multiple fields are occupying the same
 * place */
void BBCAddressCheck(char** names, int nn, char* fields[64][64], char* name,
    int node, int addr)
{
  int i;

  if (fields[node][addr])
    bprintf(fatal, "FATAL: Conflicting BBC address found for %s and %s"
        " (node %i channel %i)\n", fields[node][addr], name, node, addr);

  if (nn != -1) {
    for (i = 0; i < nn; ++i)
      if (strcmp(names[i], name) == 0)
        bprintf(fatal, "FATAL: Duplicate channel name %s found\n", name);
    names[nn] = name;
  }

  fields[node][addr] = name;
}

#ifndef INPUTTER
/* DoSanityChecks - run various sanity checks on the channel tables.  Also
 * compute useful parameters */
void DoSanityChecks(void)
{
  int i, j, nn = 0;
  char* fields[2][64][64];
  char* names[64 * 64];

#ifdef VERBOSE
  bprintf(info, "Running Sanity Checks on Channel Lists.\n");
#endif

  for (i = 0; i < 64; ++i)
    for (j = 0; j < 64; ++j)
      fields[0][i][j] = fields[1][i][j] = NULL;

  for (i = 0; WideSlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[(int)WideSlowChannels[i].bus] += 2;
    if (WideSlowChannels[i].type != 'U' && WideSlowChannels[i].type != 'S'
        && WideSlowChannels[i].type != 'i')
      bprintf(fatal, "FATAL: Error in Wide Slow Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideSlowChannels[i].field, WideSlowChannels[i].type);

    BBCAddressCheck(names, nn++, fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr);
    BBCAddressCheck(names, -1, fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr + 1);
  }
  ccWideSlow = i;

  for (i = 0; SlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[(int)SlowChannels[i].bus]++;
    if (SlowChannels[i].type != 'u' && SlowChannels[i].type != 's')
      bprintf(fatal, "Error in Slow Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          SlowChannels[i].field, SlowChannels[i].type);

    BBCAddressCheck(names, nn++, fields[SlowChannels[i].rw == 'r'],
        SlowChannels[i].field, SlowChannels[i].node, SlowChannels[i].addr);
  }
  ccNarrowSlow = i;
  ccSlow = ccNarrowSlow + ccWideSlow;

  for (i = 0; WideFastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)WideFastChannels[i].bus] += 2;
    if (WideFastChannels[i].type != 'U' && WideFastChannels[i].type != 'S'
        && WideFastChannels[i].type != 'i')
      bprintf(fatal, "FATAL: Error in Wide Fast Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideFastChannels[i].field, WideFastChannels[i].type);

    BBCAddressCheck(names, nn++, fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr);
    BBCAddressCheck(names, -1, fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr + 1);
  }
  ccWideFast = i;

  for (i = 0; FastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)FastChannels[i].bus]++;
    if (FastChannels[i].type != 'u' && FastChannels[i].type != 's')
      bprintf(fatal, "Error in Fast Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          FastChannels[i].field, FastChannels[i].type);

    BBCAddressCheck(names, nn++, fields[FastChannels[i].rw == 'r'],
        FastChannels[i].field, FastChannels[i].node, FastChannels[i].addr);
  }
  ccNarrowFast = i;

  for (i = 0; i < N_FAST_BOLOS; ++i) {
    fastsPerBusFrame[BOLO_BUS]++;
    BBCAddressCheck(names, nn++, fields[1], BoloChannels[i].field,
        BoloChannels[i].node, BoloChannels[i].addr);
  }
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;

#ifdef __DECOMD__
  for (i = 0; DecomChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[(int)DecomChannels[i].bus]++;
    if (DecomChannels[i].type != 'u' && DecomChannels[i].type != 's') {
      printf("Error in Decom Channel List:\n"
          "    %s does not have a valid type (%c)\n",
          DecomChannels[i].field, DecomChannels[i].type);
      exit(1);
    }

    BBCAddressCheck(names, nn++, fields[DecomChannels[i].rw == 'r'],
        DecomChannels[i].field, DecomChannels[i].node, DecomChannels[i].addr);
  }
  ccDecom = i;
#else
  ccDecom = 0;
#endif

  ccFast = ccWideFast + ccNarrowFast + N_FAST_BOLOS + ccDecom;
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

#ifdef VERBOSE
  bprintf(info, "All Checks Passed.\n");
  bprintf(info, "Slow Channels Per Biphase Frame: %i\n", slowsPerBi0Frame);
  bprintf(info, "Fast Channels Per Biphase Frame: %i\n", ccFast + SLOW_OFFSET);
  bprintf(info, "Slow Channels Per Tx Frame: %i / %i\n", slowsPerBusFrame[0],
      slowsPerBusFrame[1]);
  bprintf(info, "Fast Channels Per Tx Frame: %i / %i\n", fastsPerBusFrame[0],
      fastsPerBusFrame[1]);
#endif

  for (i = 0; i < 2; ++i) {
#ifdef VERBOSE
    bprintf(info, "BBC Bus %i: Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n",
        i, 4 * TxFrameWords[i], 4 * BBC_FRAME_SIZE,
        100. * TxFrameWords[i] / BBC_FRAME_SIZE);
#endif
    if (TxFrameWords[i] > BBC_FRAME_SIZE)
      bprintf(info, "FATAL: BBC Bus %i frame too big.\n", i);
  }

#ifdef VERBOSE
  bprintf(info, " BiPhase : Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n",
      2 * BiPhaseFrameWords, 2 * BI0_FRAME_SIZE,
      100. * BiPhaseFrameWords / BI0_FRAME_SIZE);
#endif
  if (BiPhaseFrameWords > BI0_FRAME_SIZE)
    bprintf(info, "FATAL: Biphase frame too big.\n");
}
#endif

/* MakeAddressLookups - fills the nios and biphase address lookup tables */
void MakeAddressLookups(void)
{
  int i, mplex, bus, spare_count;
  int slowIndex[2][FAST_PER_SLOW];

#ifdef INPUTTER
  struct ChannelStruct EmptyChannel = {"", 'w', -1, -1, -1, 1, 1};
#endif

  MakeBoloTable();

#ifndef INPUTTER
  DoSanityChecks();
#endif

#ifdef VERBOSE
  bprintf(info, "Generating Address Lookup Tables\n");
#endif

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
  if ((NiosLookup = malloc(ccTotal * sizeof(struct NiosStruct)))
      == NULL)
    bprintf(tfatal, "Unable to malloc Nios Address Lookup Table.\n");

  /* allocate the BiPhase address table */
  if ((BiPhaseLookup = malloc(BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct)))
      == NULL)
    bprintf(tfatal, "Unable to malloc Biphase Address Lookup Table.\n");

  /* fill BiPhase Lookup with invalid data */
  memset(BiPhaseLookup, 0xff, BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct));
#else
  BiPhaseAddr = 0;
  /* allocate the Defile address tables */
  if ((FastChList = malloc((ccFast + ccWideFast)
          * sizeof(struct ChannelStruct)))
      == NULL)
    berror(fatal, "Unable to malloc heap");

  if ((SlowChList = malloc(slowsPerBi0Frame * sizeof(struct ChannelStruct*)))
      == NULL)
    berror(fatal, "Unable to malloc heap");

  for (i = 0; i < slowsPerBi0Frame; ++i)
    if ((SlowChList[i] = malloc(FAST_PER_SLOW * sizeof(struct ChannelStruct)))
        == NULL)
      berror(fatal, "Unable to malloc heap");
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
        bprintf(fatal, "FATAL: Ran out of subframes while trying to "
            "insert wide slow channel %s\n", WideSlowChannels[i].field);

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
        bprintf(fatal, "FATAL: Ran out of subframes while trying to "
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
              + SLOW_OFFSET - 1);
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
  bprintf(info, "Added %i spare slow channels.\n", spare_count);
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

#ifdef INPUTTER
  /* save the location of the first bolometer in the frame so that defile
   * can calculate the offsets properly when it goes to reconstruct the
   * bolometers */
  BoloBaseIndex = BiPhaseAddr + slowsPerBi0Frame + SLOW_OFFSET;
#endif

  for (i = 0; i < N_FAST_BOLOS; ++i) {
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

  for (i = 0; i < ccDecom; ++i) {
#ifndef INPUTTER
    NiosLookup[i + ccNoBolos + N_FAST_BOLOS] = SetNiosData(&DecomChannels[i],
        addr[(int)DecomChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos
        + N_FAST_BOLOS].bbcAddr)].index = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos
        + N_FAST_BOLOS].bbcAddr)].channel = BiPhaseAddr++;
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

  DumpNiosFrame();
#endif
}

#ifndef INPUTTER
inline struct BiPhaseStruct* ExtractBiPhaseAddr(struct NiosStruct* niosAddr)
{
  return &BiPhaseLookup[BI0_MAGIC(niosAddr->bbcAddr)];
}

inline struct BiPhaseStruct* GetBiPhaseAddr(const char* field)
{
  return &BiPhaseLookup[BI0_MAGIC((GetNiosAddr(field))->bbcAddr)];
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

  bprintf(fatal, "Nios Lookup for channel %s failed.\n", field);

  return NULL;
}
#endif

void FPrintDerived(FILE *fp) {
  fprintf(fp,
      "P_X_H         LINCOM 1 p_x_deg 0.0003662109375 0\n"
      "### Sensor Veto ###\n"
      "SUN_VETO         BIT sensor_veto 0\n"
      "ISC_VETO         BIT sensor_veto 1\n"
      "ELENC_VETO       BIT sensor_veto 2\n"
      "MAG_VETO         BIT sensor_veto 3\n"
      "GPS_VETO         BIT sensor_veto 4\n"
      "ELCLIN_VETO      BIT sensor_veto 5\n"
      "IS_SCHED         BIT sensor_veto 6\n"
      "### ISC State Field ###\n"
      "ISC_SAVE_IMAGES  BIT isc_state 0\n"
      "ISC_PAUSE        BIT isc_state 1\n"
      "ISC_ABORT        BIT isc_state 2\n"
      "ISC_AUTOFOCUS    BIT isc_state 3\n"
      "ISC_BRIGHT_STAR  BIT isc_state 4\n"
      "ISC_SHUTDOWN     BIT isc_state 5\n"
      "ISC_PULSE        BIT isc_bits  1\n"
      "### Bias Generator Bitfield ###\n"
      "BIAS_IS_DC       BIT biasin 1\n"
      "BIAS_CLK_IS_INT  BIT biasin 2\n"
      "BIAS_IS_INT      BIT biasin 3\n"
    "### Cryo State Bitfield ###\n"
    "HE_LEV_SENS      BIT cryostate 0\n"
    "CHARC_HEATER     BIT cryostate 1\n"
    "COLDP_HEATER     BIT cryostate 2\n"
    "CALIBRATOR       BIT cryostate 3\n"
    "POT_VALVE        BIT cryostate 4\n"
    "POT_DIREC        BIT cryostate 5\n"
    "LHE_VALVE        BIT cryostate 6\n"
    "LHE_DIREC        BIT cryostate 7\n"
    "### Cryo Valve Limit Switches ###\n"
    "POT_IS_CLOSED    BIT cryoin 0\n"
    "POT_IS_OPEN      BIT cryoin 1\n"
    "POT_STATE        LINCOM 2 POT_IS_CLOSED 1 0 POT_IS_OPEN 1 0\n"
    "LHE_IS_CLOSED    BIT cryoin 2\n"
    "LHE_IS_OPEN      BIT cryoin 3\n"
    "LHE_STATE        LINCOM 2 LHE_IS_CLOSED 1 0 LHE_IS_OPEN 1 0\n"
    "### Cryo Table Lookups ###\n"
    "# Diodes\n"
    "T_charcoal       LINTERP  T_CHARCOAL   /data/etc/dt600.txt\n"
    "T_lhe            LINTERP  T_LHE        /data/etc/dt600.txt\n"
    "T_ln2            LINTERP  T_LN2        /data/etc/dt600.txt\n"
    "T_heatswitch     LINTERP  T_HEATSWITCH /data/etc/dt600.txt\n"
    "T_jfet           LINTERP  T_JFET       /data/etc/dt600.txt\n"
    "T_vcs_filt       LINTERP  T_VCS_FILT   /data/etc/dt600.txt\n"
    "T_ln2_filt       LINTERP  T_LN2_FILT   /data/etc/dt600.txt\n"
    "T_lhe_filt       LINTERP  T_LHE_FILT   /data/etc/dt600.txt\n"
    "T_he4pot_d       LINTERP  T_HE4POT_D   /data/etc/dt600.txt\n"
    "T_vcs_fet        LINTERP  T_VCS_FET    /data/etc/dt600.txt\n"
    "# GRTs (ROX)\n"
    "#T_he3fridge	LINTERP	 T_HE3FRIDGE  /data/etc/rox102a.txt\n"
    "#T_he4pot        LINTERP  T_HE4POT     /data/etc/rox102a.txt\n"
    "#T_horn_500      LINTERP  T_HORN_500   /data/etc/rox102a.txt\n"
    "#T_base_500      LINTERP  T_BASE_500   /data/etc/rox102a.txt\n"
    "#T_base_250      LINTERP  T_BASE_250   /data/etc/rox102a.txt\n"
    "# Level Sensor\n"
    "HE4_LITRE        LINTERP  HE4_LEV      /data/etc/he4_litre.txt\n"
    "HE4_PERCENT      LINTERP  HE4_LEV      /data/etc/he4_percent.txt\n"
    "# Control Bits\n"
    "# To Be Filled In At A Later Date\n"
    "# TEMPORARY GRT Calibration\n"
    "#Res_He3	       LINCOM  1       N10C0   1.9416E04       -8.4574\n"
    "#Res_horn_500   LINCOM  1       N10C1   1.9416E04       -8.4574\n"
    "#Res_pot        LINCOM  1       N10C2   1.9416E04       -8.4574\n"
    "#Res_base_500   LINCOM  1       N10C4   1.9416E04       -8.4574\n"
    "#Res_ring_250   LINCOM  1       N10C7   1.9416E04       -8.4574\n"
    "#T_he3fridge      LINTERP N7C3         /data/etc/rox102a3.txt\n"
    "#T_he4pot         LINTERP N7C22        /data/etc/rox102a22.txt\n"
    "#T_m3             LINTERP N7C7         /data/etc/rox102a7.txt\n"
    "#T_m4             LINTERP N7C4         /data/etc/rox102a4.txt \n"
    "#T_m5             LINTERP N7C5         /data/etc/rox102a5.txt\n"
    "#T_optbox_filt    LINTERP N7C23        /data/etc/rox102a23.txt\n"
    "#T_300mk_strap    LINTERP N7C20        /data/etc/rox102a20.txt\n"
    "#T_horn_250       LINTERP N7C6         /data/etc/rox102a6.txt\n"
    "#T_horn_350       LINTERP N7C19        /data/etc/rox102a19.txt\n"
    "#T_horn_500       LINTERP N7C21        /data/etc/rox102a21.txt\n"


    "T_he3fridge      LINTERP T_HE3FRIDGE   /data/etc/rox102a3.txt\n"
    "T_he4pot         LINTERP T_HE4POT      /data/etc/rox102a22.txt\n"
    "T_m3             LINTERP T_M3          /data/etc/rox102a7.txt\n"
    "T_m4             LINTERP T_M4          /data/etc/rox102a4.txt \n"
    "T_m5             LINTERP T_M5          /data/etc/rox102a5.txt\n"
    "T_optbox_filt    LINTERP T_OPTBOX_FILT /data/etc/rox102a23.txt\n"
    "T_300mk_strap    LINTERP T_300MK_STRAP /data/etc/rox102a20.txt\n"
    "T_horn_250       LINTERP T_HORN_250    /data/etc/rox102a6.txt\n"
    "T_horn_350       LINTERP T_HORN_350    /data/etc/rox102a19.txt\n"
    "T_horn_500       LINTERP T_HORN_500    /data/etc/rox102a21.txt\n"

    "#\n"
    "Clin_Elev LINTERP clin_elev /data/etc/clin_elev.lut\n"
    "# Nice CPU Values\n"
    "CPU_SEC LINCOM  1       cpu_time        1       -%lu\n"
    "CPU_MIN LINCOM  1       CPU_SEC 0.016666666 0\n"
    "CPU_HOUR LINCOM 1       CPU_SEC 0.000277777 0\n"
    "CPU_DAY LINCOM  1       CPU_SEC 1.15741E-5  0\n"
    "CPU_WEEK LINCOM 1       CPU_SEC 1.65344E-6  0\n"
    "CPU_MONTH LINCOM 1      CPU_SEC 3.85803E-7  0\n"
    "CPU_YEAR LINCOM 1       CPU_SEC 3.17099E-8  0\n", time(NULL)
    );
}
