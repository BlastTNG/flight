#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include "tx_struct.h"
#include "bbc_pci.h"

/* if compiling MCP load the real mprintf function prototypes, otherwise, just
 * make up a fake one */
#ifdef __MCP__
#  include "mcp.h"
#else
#  define mprintf(x, ...) \
     do {  /* encase in a do {} while(0) loop to properly swallow the ; */ \
       printf(__VA_ARGS__); \
       if (strcmp(#x, "MCP_FATAL") == 0) \
         exit(1); \
     } while (0)
#endif

unsigned int boloIndex[DAS_CARDS][DAS_CHS][2];

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];

unsigned short ccWideFast;
unsigned short ccNarrowFast;
unsigned short ccWideSlow;
unsigned short ccNarrowSlow;
unsigned short ccSlow;
unsigned short ccFast;
unsigned short ccNoBolos;
unsigned short ccTotal;

unsigned short BiPhaseFrameWords;
unsigned short BiPhaseFrameSize;
unsigned short TxFrameWords[2];
unsigned short TxFrameSize[2];
unsigned short slowsPerBi0Frame;
unsigned short slowCount[2] = {0, 0};
unsigned short slowsPerBusFrame[2];
unsigned short fastsPerBusFrame[2] = {FAST_OFFSET, 1};

struct NiosStruct* NiosLookup;
unsigned int NiosSpares[FAST_PER_SLOW * 2];
unsigned int BBCSpares[FAST_PER_SLOW * 2];
struct BiPhaseStruct *BiPhaseLookup;

/* bus on which the bolometers are */
#define BOLO_BUS  1

struct ChannelStruct BoloChannels[N_FAST_BOLOS];

void WriteSpecificationFile(FILE* fp)
{
  fwrite(&ccWideSlow, sizeof(unsigned short), 1, fp);
  fwrite(&ccNarrowSlow, sizeof(unsigned short), 1, fp);
  fwrite(&ccWideFast, sizeof(unsigned short), 1, fp);
  fwrite(&ccNarrowFast, sizeof(unsigned short), 1, fp);

  fwrite(WideSlowChannels, sizeof(struct ChannelStruct), ccWideSlow, fp);
  fwrite(SlowChannels, sizeof(struct ChannelStruct), ccNarrowSlow, fp);
  fwrite(WideFastChannels, sizeof(struct ChannelStruct), ccWideFast, fp);
  fwrite(FastChannels, sizeof(struct ChannelStruct), ccNarrowFast, fp);

  mprintf(MCP_INFO, "Wrote specification file.\n");
}

/************************************************************************/
/*                                                                      */
/*    MakeBoloTable: create the bolometer channel table                 */
/*                                                                      */
/************************************************************************/
void MakeBoloTable(void) {
  int i, j, index = 0;;
  struct ChannelStruct channel = {
    "", 'r', 3, BOLO_BUS, 0, 1.19209e-7, -2097152.0, 'u'
  };

  mprintf(MCP_INFO, "Generating Bolometer Channel Table.\n");

  for (i = 0; i < DAS_CARDS; ++i) {
    channel.node = i + 5;
    channel.rw = 'r';
    for (j = 0; j < DAS_CHS; j += 2) {
      /* lsw channel at j */
      channel.addr = j;
      sprintf(channel.field, "n%ic%ilo", channel.node, j); /* ignored */
      boloIndex[i][j][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* msw at j and j+1 */
      channel.addr = DAS_CHS + (j >> 1);
      sprintf(channel.field, "n%ic%ihi", channel.node, j); /* ignored */
      boloIndex[i][j + 1][1] = boloIndex[i][j][1] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
      /* lsw channel at j+1 */
      channel.addr = j+1;
      sprintf(channel.field, "n%ic%ilo", channel.node, j+1); /* ignored */
      boloIndex[i][j + 1][0] = index;
      memcpy(&BoloChannels[index++], &channel, sizeof(channel));
    }
  }
}

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

  if ((map = fopen("/data/etc/Nios.map", "w")) == NULL)
    return;

  for (bus = 0; bus < 2; ++bus) {
    fprintf(map, "Bus %i Map:\n", bus);
    for (m = 0; m < FAST_PER_SLOW; ++m) {
      for (i = 0; i < TxFrameWords[bus]; ++i) {
        addr = i + bus * BBCPCI_MAX_FRAME_SIZE + m * TxFrameWords[bus];
        m0addr = i + bus * BBCPCI_MAX_FRAME_SIZE;
        n = 0;
        fprintf(map, "%06x", addr);
        if (i == 0) {
          if (bus) {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(4);
            fprintf(map, " %08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          } else {
            n++;
            addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(0);
            fprintf(map, " %08x [%2i %3i (%04x)] Y Frame Sync", addr,
                BiPhaseLookup[BI0_MAGIC(addr)].index,
                BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
          }
        } else if (i == 1 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(1);
          fprintf(map, " %08x [%2i %3i (%04x)] F FASTSAMP (lsb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 2 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(2);
          fprintf(map, " %08x [%2i %3i (%04x)] F FASTSAMP (msb)", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr));
        } else if (i == 3 && bus == 0) {
          n++;
          addr = BBC_WRITE | BBC_NODE(SPECIAL) | BBC_CH(3);
          fprintf(map, " %08x [%2i %3i (%04x)] M Multiplex Index = %i", addr,
              BiPhaseLookup[BI0_MAGIC(addr)].index,
              BiPhaseLookup[BI0_MAGIC(addr)].channel, BI0_MAGIC(addr), m);
        } else {
          for (j = 0; j < ccTotal; ++j)
            if (NiosLookup[j].niosAddr == addr) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s",
                  NiosLookup[j].bbcAddr,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].index,
                  BiPhaseLookup[BI0_MAGIC(NiosLookup[j].bbcAddr)].channel,
                  BI0_MAGIC(NiosLookup[j].bbcAddr),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
              if (NiosLookup[j].wide)
                fprintf(map, " (lsb)");
            } else if (NiosLookup[j].niosAddr == addr - 1 &&
                NiosLookup[j].wide) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s (msb)",
                  BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr),
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].index,
                  BiPhaseLookup[BI0_MAGIC(
                    BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr))].channel,
                  BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[j].bbcAddr)),
                  NiosLookup[j].fast ? 'f' : 's', NiosLookup[j].field);
            } else if (NiosLookup[j].fast && NiosLookup[j].niosAddr == m0addr) {
              n++;
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s",
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
              fprintf(map, " %08x [%2i %3i (%04x)] %c %s (msb)",
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
            fprintf(map, " %08x [%2i %3i (%04x)] X Spare%i", BBCSpares[j],
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].index,
                BiPhaseLookup[BI0_MAGIC(BBCSpares[j])].channel,
                BI0_MAGIC(BBCSpares[j]), j);
          }
        }
        fprintf(map, "\n");
        if (n == 0) {
          fclose(map);
          mprintf(MCP_FATAL, "FATAL: unassigned address in Nios Address Table."
              " Consult Nios Map.\n");
        } else if (n != 1) {
          fclose(map);
          mprintf(MCP_FATAL, "FATAL: collision in Nios Address Table "
              "assignment. Consult Nios Map.\n");
        }
      }
    }
    fprintf(map, "\n");
  }

  mprintf(MCP_INFO, "Wrote /data/etc/Nios.map.\n");
  fclose(map);
}

/* Checks BBC Addresses to see if multiple fields are occupying the same
 * place */
void BBCAddressCheck(char* fields[64][64], char* name, int node, int addr)
{
  if (fields[node][addr])
    mprintf(MCP_FATAL, "FATAL: Conflicting BBC address found for %s and %s"
        " (node %i channel %i)\n", fields[node][addr], name, node, addr);

  fields[node][addr] = name;
}

/* DoSanityChecks - run various sanity checks on the channel tables.  Also
 * compute useful parameters */
void DoSanityChecks(void)
{
  int i, j;
  char* fields[2][64][64];

  mprintf(MCP_INFO, "Running Sanity Checks on Channel Lists.\n");

  for (i = 0; i < 64; ++i)
    for (j = 0; j < 64; ++j)
      fields[0][i][j] = fields[1][i][j] = NULL;

  for (i = 0; WideSlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[WideSlowChannels[i].bus] += 2;
    if (WideSlowChannels[i].type != 'U' && WideSlowChannels[i].type != 'S'
        && WideSlowChannels[i].type != 'i')
      mprintf(MCP_FATAL, "FATAL: Error in Wide Slow Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideSlowChannels[i].field, WideSlowChannels[i].type);

    BBCAddressCheck(fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr);
    BBCAddressCheck(fields[WideSlowChannels[i].rw == 'r'],
        WideSlowChannels[i].field, WideSlowChannels[i].node,
        WideSlowChannels[i].addr + 1);
  }
  ccWideSlow = i;

  for (i = 0; SlowChannels[i].node != EOC_MARKER; ++i) {
    slowCount[SlowChannels[i].bus]++;
    if (SlowChannels[i].type != 'u' && SlowChannels[i].type != 's')
      mprintf(MCP_FATAL, "Error in Slow Channel List:\n"
          "    %s does not have a valid slow type (%c)\n",
          SlowChannels[i].field, SlowChannels[i].type);

    BBCAddressCheck(fields[SlowChannels[i].rw == 'r'], SlowChannels[i].field,
        SlowChannels[i].node, SlowChannels[i].addr);
  }
  ccNarrowSlow = i;
  ccSlow = ccNarrowSlow + ccWideSlow;

  for (i = 0; WideFastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[WideFastChannels[i].bus] += 2;
    if (WideFastChannels[i].type != 'U' && WideFastChannels[i].type != 'S'
        && WideFastChannels[i].type != 'i')
      mprintf(MCP_FATAL, "FATAL: Error in Wide Fast Channel List:\n"
          "    %s does not have a valid wide type (%c)\n",
          WideFastChannels[i].field, WideFastChannels[i].type);

    BBCAddressCheck(fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr);
    BBCAddressCheck(fields[WideFastChannels[i].rw == 'r'],
        WideFastChannels[i].field, WideFastChannels[i].node,
        WideFastChannels[i].addr + 1);
  }
  ccWideFast = i;

  for (i = 0; FastChannels[i].node != EOC_MARKER; ++i) {
    fastsPerBusFrame[FastChannels[i].bus]++;
    if (FastChannels[i].type != 'u' && FastChannels[i].type != 's')
      mprintf(MCP_FATAL, "Error in Fast Channel List:\n"
          "    %s does not have a valid slow type (%c)\n",
          FastChannels[i].field, FastChannels[i].type);

    BBCAddressCheck(fields[FastChannels[i].rw == 'r'], FastChannels[i].field,
        FastChannels[i].node, FastChannels[i].addr);
  }
  ccNarrowFast = i;
  ccFast = ccWideFast + ccNarrowFast + N_FAST_BOLOS;
  ccNoBolos = ccSlow + ccWideFast + ccNarrowFast;
  ccTotal = ccFast + ccSlow;

  for (i = 0; i < N_FAST_BOLOS; ++i) {
    fastsPerBusFrame[BOLO_BUS]++;
    BBCAddressCheck(fields[1], FastChannels[i].field, BoloChannels[i].node,
        BoloChannels[i].addr);
  }

  /* Calculate slowsPerBi0Frame */
  for (i = 0; i < 2; ++ i) {
    slowsPerBusFrame[i] = 1 + (slowCount[i] - 1) / FAST_PER_SLOW;
    TxFrameWords[i] = fastsPerBusFrame[i] + slowsPerBusFrame[i];
    TxFrameSize[i] = TxFrameWords[i] * 4;
  }

  slowsPerBi0Frame = slowsPerBusFrame[0] + slowsPerBusFrame[1];


  mprintf(MCP_INFO, "All Checks Passed.\n");
  mprintf(MCP_INFO, "Slow Channels Per Biphase Frame: %i\n", slowsPerBi0Frame);
  mprintf(MCP_INFO, "Slow Channels Per Tx Frame: %i / %i\n",
      slowsPerBusFrame[0],  slowsPerBusFrame[1]);
  mprintf(MCP_INFO, "Fast Channels Per Biphase Frame: %i\n", ccFast
      + FAST_OFFSET);
  mprintf(MCP_INFO, "Fast Channels Per Tx Frame: %i / %i\n",
      fastsPerBusFrame[0],  fastsPerBusFrame[1]);

  BiPhaseFrameWords = FAST_OFFSET + ccFast + slowsPerBi0Frame + 1;
  BiPhaseFrameSize = 2 * BiPhaseFrameWords;
  for (i = 0; i < 2; ++i) {
    mprintf(MCP_INFO,
        "BBC Bus %i: Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n", i,
        4 * TxFrameWords[i], 4 * BBC_FRAME_SIZE,
        100. * TxFrameWords[i] / BBC_FRAME_SIZE);
    if (TxFrameWords[i] > BBC_FRAME_SIZE)
      mprintf(MCP_FATAL, "FATAL: BBC Bus %i frame too big.\n", i);
  }

  mprintf(MCP_INFO,
      " BiPhase : Frame Bytes: %4i  Allowed: %4i (%.2f%% full)\n",
      2 * BiPhaseFrameWords, 2 * BI0_FRAME_SIZE,
      100. * BiPhaseFrameWords / BI0_FRAME_SIZE);
  if (BiPhaseFrameWords > BI0_FRAME_SIZE)
    mprintf(MCP_FATAL, "FATAL: Biphase frame too big.\n");
}

/* MakeAddressLookups - fills the nios and biphase address lookup tables */
void MakeAddressLookups(void)
{
  int i, mplex, bus, spare_count;
  int slowIndex[2][FAST_PER_SLOW];

  MakeBoloTable();

  DoSanityChecks();

  mprintf(MCP_INFO, "Generating Address Lookup Tables\n");

  unsigned int BiPhaseAddr = FAST_OFFSET + slowsPerBi0Frame;
  unsigned int addr[2] = {
    FAST_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
  };

  int slowTop[2] = {
    FAST_OFFSET + slowsPerBusFrame[0],
    1 + slowsPerBusFrame[1]
  };

  /* allocate the Nios address table */
  if ((NiosLookup = malloc(ccTotal * sizeof(struct NiosStruct)))
      == NULL)
    mprintf(MCP_TFATAL, "Unable to malloc Nios Address Lookup Table.\n");

  /* allocate the BiPhase address table */
  if ((BiPhaseLookup = malloc(BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct)))
      == NULL)
    mprintf(MCP_TFATAL, "Unable to malloc Biphase Address Lookup Table.\n");

  /* fill BiPhase Lookup with invalid data */
  memset(BiPhaseLookup, 0xff, BI0_TABLE_SIZE * sizeof(struct BiPhaseStruct));

  /* initialise slow channels */
  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slowIndex[0][i] = FAST_OFFSET;
    slowIndex[1][i] = 1;
  }
  for (i = 0; i < FAST_PER_SLOW * 2; ++i)
    NiosSpares[i] = -1;

  /* populate slow channels */
  for (i = 0; i < ccWideSlow; ++i) {
    bus = WideSlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] + 1 >= slowTop[bus]) {
      if (++mplex >= FAST_PER_SLOW)
        mprintf(MCP_FATAL, "FATAL: Ran out of subframes while trying to "
            "insert wide slow channel %s\n", WideSlowChannels[i].field);
    }

    NiosLookup[i] = SetNiosData(&WideSlowChannels[i], mplex * TxFrameWords[bus]
        + slowIndex[bus][mplex], 0, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + FAST_OFFSET - 1);
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].index
      = mplex;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i].bbcAddr))].channel
      = slowIndex[bus][mplex] + 1 + bus * (slowsPerBusFrame[0] + FAST_OFFSET
          - 1);

    slowIndex[bus][mplex] += 2;
  }

  for (i = 0; i < ccNarrowSlow; ++i) {
    bus = SlowChannels[i].bus;
    mplex = 0;
    while (slowIndex[bus][mplex] >= slowTop[bus]) {
      if (++mplex >= FAST_PER_SLOW) {
        mprintf(MCP_FATAL, "FATAL: Ran out of subframes while trying to "
            "insert slow channel %s\n", SlowChannels[i].field);
      }
    }

    NiosLookup[i + ccWideSlow] = SetNiosData(&SlowChannels[i],
        mplex * TxFrameWords[bus] + slowIndex[bus][mplex], 0, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].index = mplex;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccWideSlow].bbcAddr)].channel
      = slowIndex[bus][mplex] + bus * (slowsPerBusFrame[0] + FAST_OFFSET - 1);

    slowIndex[bus][mplex]++;
  }

  /* Fill Up remaining Slow Channels with spares */
  spare_count = 0;
  for (bus = 0; bus < 2; ++bus)
    for (mplex = 0; mplex < FAST_PER_SLOW; ++mplex)
      while (slowIndex[bus][mplex] < slowTop[bus]) {
        BBCSpares[spare_count] = BBC_WRITE | BBC_NODE(SPARE) |
          BBC_CH(spare_count);
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].index = mplex;
        BiPhaseLookup[BI0_MAGIC(BBCSpares[spare_count])].channel
          = slowIndex[bus][mplex] + bus * slowsPerBusFrame[0];
        i = mplex * TxFrameWords[bus] + slowIndex[bus][mplex];
        NiosSpares[spare_count++] = bus ? BBCPCI_WFRAME2_ADD(i)
          : BBCPCI_WFRAME1_ADD(i);
        slowIndex[bus][mplex]++;
      }

  mprintf(MCP_INFO, "Added %i spare slow channels.\n", spare_count);

  for (i = 0; i < ccNarrowFast; ++i) {
    NiosLookup[i + ccSlow + ccWideFast] = SetNiosData(&FastChannels[i],
        addr[FastChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow + ccWideFast].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow
        + ccWideFast].bbcAddr)].channel = BiPhaseAddr++;

    addr[FastChannels[i].bus]++;
  }

  for (i = 0; i < ccWideFast; ++i) {
    NiosLookup[i + ccSlow] = SetNiosData(&WideFastChannels[i],
        addr[WideFastChannels[i].bus], 1, 1);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccSlow].bbcAddr)].channel
      = BiPhaseAddr++;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].index = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(BBC_NEXT_CHANNEL(NiosLookup[i
          + ccSlow].bbcAddr))].channel = BiPhaseAddr++;

    addr[WideFastChannels[i].bus] += 2;
  }

  for (i = 0; i < N_FAST_BOLOS; ++i) {
    NiosLookup[i + ccNoBolos] = SetNiosData(&BoloChannels[i],
        addr[BoloChannels[i].bus], 1, 0);

    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].index
      = NOT_MULTIPLEXED;
    BiPhaseLookup[BI0_MAGIC(NiosLookup[i + ccNoBolos].bbcAddr)].channel
      = addr[BoloChannels[i].bus];

    addr[BoloChannels[i].bus]++;
  }

  /* Add the channels that aren't in the channel list to the Biphase lookup */
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(0)) >> 16].channel = 0;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(1)) >> 16].channel = 1;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(2)) >> 16].channel = 2;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].index = NOT_MULTIPLEXED;
  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(3)) >> 16].channel = 3;

  BiPhaseLookup[(BBC_NODE(SPECIAL) | BBC_CH(4)) >> 16].index = DISCARD_WORD;

  DumpNiosFrame();
}

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

  mprintf(MCP_FATAL, "Nios Lookup for channel %s failed.\n", field);

  return NULL;
}

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
    "T_opt_box_w	LINTERP	T_OPT_BOX_W	/data/etc/dt600.txt\n"
    "T_lhe_filt   LINTERP T_LHE_FILT    /data/etc/dt600.txt\n"
    "T_vcs	LINTERP	T_VCS	/data/etc/dt600.txt\n"
    "T_vcs_filt   LINTERP T_VCS_FILT    /data/etc/dt600.txt\n"
    "T_ln2   LINTERP T_LN2    /data/etc/dt600.txt\n"
    "T_ln2_filt   LINTERP T_LN2_FILT    /data/etc/dt600.txt\n"
    "T_charcoal   LINTERP T_CHARCOAL    /data/etc/dt600.txt\n"
    "T_optics_box   LINTERP T_OPTICS_BOX    /data/etc/dt600.txt\n"
    "T_jfet   LINTERP T_JFET    /data/etc/dt600.txt\n"
    "T_he4pot_d   LINTERP T_HE4POT_D    /data/etc/dt600.txt\n"
    "T_cold_plate   LINTERP T_COLD_PLATE   /data/etc/dt600.txt\n"
    "# GRTs (ROX)\n"
    "#T_he3fridge	LINTERP	T_HE3FRIDGE	/data/etc/rox102a.txt\n"
    "#T_he4pot    LINTERP T_HE4POT    /data/etc/rox102a.txt\n"
    "#T_horn_500    LINTERP T_HORN_500   /data/etc/rox102a.txt\n"
    "#T_base_500   LINTERP T_BASE_500   /data/etc/rox102a.txt\n"
    "#T_base_250    LINTERP T_BASE_250   /data/etc/rox102a.txt\n"
    "# Level Sensor\n"
    "he4_litre    LINTERP HE4_LEVEL   /data/etc/he4_level.txt\n"
    "# Control Bits\n"
    "# To Be Filled In At A Later Date\n"
    "# TEMPORARY GRT Calibration\n"
    "Res_He3	       LINCOM  1       N10C0   1.9416E04       -8.4574\n"
    "Res_horn_500   LINCOM  1       N10C1   1.9416E04       -8.4574\n"
    "Res_pot        LINCOM  1       N10C2   1.9416E04       -8.4574\n"
    "Res_base_500   LINCOM  1       N10C4   1.9416E04       -8.4574\n"
    "Res_ring_250   LINCOM  1       N10C7   1.9416E04       -8.4574\n"
    "T_he3fridge LINTERP N10C3        /data/etc/rox102a.txt\n"
    "T_horn_500  LINTERP N10C7       /data/etc/rox102a.txt\n"
    "T_he4pot    LINTERP N10C22       /data/etc/rox102a.txt\n"
    "T_base_500  LINTERP N10C4       /data/etc/rox102a.txt \n"
    "T_base_250  LINTERP N10C5       /data/etc/rox102a.txt\n"
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
