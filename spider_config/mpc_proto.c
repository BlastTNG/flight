/* MCP: the MPC communication protocol
 *
 * Copyright (C) 2012-2013, D. V. Wiebe
 * All rights reserved.
 *
 * This file is part of the BLAST flight code licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
#include "blast.h"
#include "command_common.h"
#include "mpc_proto.h"
#include "mce_counts.h"
#include "tes.h"

#include <string.h>

int mpc_cmd_rev = -1;
int16_t mpc_proto_rev = -1;

/* desecrate the C preprocessor to extract this file's SVN revision */

/* Fun fact: GCC's CPP allows $ in identifier names */
#define $Rev (0?0 /* eat a : here */
#define $    )

static const inline int mpc_proto_revision(void) { return $Rev$; };

#undef $Rev
#undef $
/* end preprocessor desecration */

/* initialisation routine: sets up the environment; returns non-zero on error */
int mpc_init(void)
{
  mpc_proto_rev = mpc_proto_revision();
  mpc_cmd_rev = command_list_serial_as_int();
  if (mpc_cmd_rev < 0) {
    bputs(err, "Unable to parse the command list revision.");
    return -1;
  }

  bprintf(info, "Protocol revision: %i/%i\n", mpc_proto_rev, mpc_cmd_rev);
  return 0;
}

/* compose a general-purpose data packet for transmission to PCM
 *
 * GP packet looks like:
 *
 * RRGMTTNN...
 *
 * where
 *
 * R = 16-bit protocol revision
 * G = 'G' indicating MCE gp packet
 * M = mce number
 * T = payload type
 * N = payload length (in WORDs)
 *
 * followed by payload data.
 */
size_t mpc_compose_gpdata(uint16_t type, uint16_t len, const uint16_t *data,
    int nmce, char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'G';
  buffer[3] = nmce;
  memcpy(buffer + 4, &type, sizeof(type));
  memcpy(buffer + 6, &len, sizeof(len));
  memcpy(buffer + 8, data, sizeof(uint16_t) * len);
  return 8 + len * sizeof(uint16_t);
}

/* compose a MCE array synopsis packet for transmission to PCM
 *
 * MCE array synopsis packet looks like:
 *
 * RRQMss...
 *
 * where
 *
 * R = 16-bit protocol revision
 * Q = 'Q' indicating MCE array synopsis
 * M = mce number
 * s = stream of 8-bit statistics
 */
size_t mpc_compose_synop(const uint8_t *data, int nmce, char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'Q';
  buffer[3] = nmce;
  memcpy(buffer + 4, data, N_STAT_TYPES * NUM_ROW * NUM_COL);
  return 4 + N_STAT_TYPES * NUM_ROW * NUM_COL;
}

/* compose a MCE parameter packet for transmission to PCM
 * 
 * MCE status packet looks like:
 *
 * RRZM
 *
 * where
 *
 * R = 16-bit protocol revision
 * Z = 'Z' indicating MCE parameter data
 * M = mce number
 *
 * followed by the mce parameter data
 */
size_t mpc_compose_param(const uint32_t *param, int nmce, char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'Z';
  buffer[3] = nmce;
  memcpy(buffer + 4, param, sizeof(uint32_t) * N_MCE_STAT);
  return 4 + sizeof(uint32_t) * N_MCE_STAT;
}

/* compsoe a PCM notice packet for transmission to the mpc
 *
 * PCM notice packet looks like:
 *
 * RRND MVST FFFF FFFF BBBB BBBB LLln rddd d
 *
 * where
 *
 * R = 16-bit protocol revision
 * N = 'N' indicating notice packet
 * D = divisor
 * M = data_mode
 * V = squid_veto
 * S = super slow data request
 * T = turnaround flag
 * F = bolo_filt_freq (double)
 * B = bolo_filt_bw (double)
 * L = bolo_filt_len
 * l = row_len
 * n = num_rows
 * r = data_rate
 * d = 4 bytes of data_mode_bits
 */
size_t mpc_compose_notice(int divisor, int turnaround, int request_ssdata,
    int data_mode, int row_len, int num_rows, int data_rate, uint8_t squidveto,
    double bolo_filt_freq, double bolo_filt_bw, uint16_t bolo_filt_len,
    char data_mode_bits[2][2], char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'N';
  buffer[3] = divisor & 0xff;
  buffer[4] = data_mode;
  buffer[5] = squidveto;
  buffer[6] = request_ssdata ? 1 : 0;
  buffer[7] = turnaround ? 1 : 0;
  memcpy(buffer + 8, &bolo_filt_freq, sizeof(double));
  memcpy(buffer + 16, &bolo_filt_bw, sizeof(double));
  memcpy(buffer + 24, &bolo_filt_len, sizeof(uint16_t));
  buffer[26] = row_len;
  buffer[27] = num_rows;
  buffer[28] = data_rate;
  buffer[29] = data_mode_bits[0][0];
  buffer[30] = data_mode_bits[0][1];
  buffer[31] = data_mode_bits[1][0];
  buffer[32] = data_mode_bits[1][1];
  return 33;
}

/* compose a PCM request packet for transmission to PCM
 *
 * PCM request packet looks like:
 *
 * rrRMP
 *
 * where
 *
 * r = 16-bit protocol revision
 * R = 'R' indicating a PCM request packet
 * M = 8-bit MCE number
 * P = the power cycle flag
 */
size_t mpc_compose_pcmreq(int nmce, int power_cycle, char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'R';
  buffer[3] = nmce & 0xff;
  buffer[4] = power_cycle ? 1 : 0;
  return 5;
}

/* compose a TES data packet for transmission from the mpc
 *
 * data packet looks like:
 *
 * RRTMBBNN(FFFF...)(DD..)
 *
 * where
 *
 * R = 16-bit protocol revision
 * T = 'T' indicating a slow packet
 * M = 8-bit MCE number
 * B = 16-bit bset number
 * N = 16-bit frame count
 * F = "N" 32-bit framenumbers
 * D = "N" frames of 16-bit tes data in bset order
 */
size_t mpc_compose_tes(const uint16_t *data, const uint32_t *framenum,
    uint16_t bset_num, int16_t nf, int sync, int nmce, int ntes,
    const int16_t *tesind, char *buffer)
{
  int i, j;
  size_t len = 8 + (sizeof(uint16_t) * ntes + sizeof(uint32_t)) * nf;
  uint16_t *dbuf = (uint16_t *)(buffer + 8 + sizeof(uint32_t) * nf); 

  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'T'; /* tes data packet */
  buffer[3] = (nmce & 0xF) | (sync ? 0x80 : 0); /* mce number and sync bit */
  memcpy(buffer + 4, &bset_num, sizeof(bset_num)); /* BSET */
  memcpy(buffer + 6, &nf, sizeof(nf));             /* NUM_FRAMES */

  /* frame numbers */
  memcpy(buffer + 8, framenum, sizeof(uint32_t) * nf);

  /* append data */
  for (i = 0; i < nf; ++i)
    for (j = 0; j < ntes; ++j)
      dbuf[j + i * ntes] = data[tesind[j] + i * NUM_ROW * NUM_COL];

  return len;
}

/* compose a slow data packet for transfer from the mpc
 *
 * Slow data packet looks like:
 *
 * RRSM...
 *
 * where
 *
 * R = 16-bit protocol revision
 * S = 'S' indicating a slow packet
 * M = 8-bit MCE number
 *
 * followed by the packed slow data
 */
size_t mpc_compose_slow(const struct mpc_slow_data *dat, int mce, char *buffer)
{
  size_t len = sizeof(*dat);

  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'S'; /* slow data packet */
  buffer[3] = mce & 0xF; /* mce number */

  /* this will probably work... */
  memcpy(buffer + 4, dat, len);

  return len + 4;
}

/* compose an init packet for transfer from the mpc
 *
 * Init packet looks like:
 *
 * RRAM
 *
 * where
 *
 * R = 16-bit protocol revision
 * A = 'A' inidcating init packet
 * M = 8-bit mce number
 */
size_t mpc_compose_init(int mce, char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'A'; /* init ("awake") packet */
  buffer[3] = mce & 0xF; /* mce number */

  return 4;
}

/* compose an bset list for transfer to the mpc
 *
 * Fset packet looks like:
 *
 * RRFxNN1122334455...
 *
 * where
 *
 * R = 16-bit protocol revision
 * F = 'F' indicating bset packet
 * x is a padding byte
 * N = the bset_num
 * 11, 22, 33, ... are the 16-bit TES numbers
 */
size_t mpc_compose_bset(const int16_t *set, int set_len, uint16_t num,
    char *buffer)
{
  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'F'; /* bset packet */
  buffer[3] = 0; /* padding */
  memcpy(buffer + 4, &num, sizeof(num)); /* bset number + serial */
  memcpy(buffer + 6, set, set_len * 2);

  return 6 + set_len * 2;
}

/* compose a command for transfer to the mpc
 *
 * Command packet looks like:
 *
 * RRCTZZNN...
 *
 * where
 *
 * R = 16-bit protocol revision
 * C = 'C', indicating command packet
 * T = MCE number (or 8 if global single command)
 * Z = 16-bit command list revision
 * N = 16-bit command number
 *
 * followed by command parameters, if any.  Like the rest of the protocol all
 * numbers a little-endian */
size_t mpc_compose_command(struct ScheduleEvent *ev, char *buffer)
{
  size_t len = 2 + 1 + 2 + 2 + 1; /* 16-bit protocol revision + 'C' + 16-bit
                                     command list revision + 16-bit command
                                     number + 'm'/'s' */
  int16_t i16;
  int32_t i32;
  char *ptr;
  int i;

  memcpy(buffer, &mpc_proto_rev, sizeof(mpc_proto_rev));
  buffer[2] = 'C'; /* command packet */
  if (ev->is_multi)
    buffer[3] = (char)ev->ivalues[0];
  else 
    buffer[3] = 8;
  i16 = mpc_cmd_rev;
  memcpy(buffer + 4, &i16, sizeof(i16)); /* 16-bit command list revision */
  i16 = ev->command;
  memcpy(buffer + 6, &i16, sizeof(i16)); /* 16-bit command number */
  if (ev->is_multi) {
    ptr = buffer + 8;
    for (i = 1; i < mcommands[ev->t].numparams; ++i) {
      switch (mcommands[ev->t].params[i].type) {
        case 'i':
        case 'l':
          *(ptr++) = 'N'; /* 32-bit integer */
          i32 = (int32_t)(ev->ivalues[i]);
          memcpy(ptr, &i32, sizeof(i32));
          ptr += sizeof(i32);
          break;
        case 'f':
        case 'd':
          *(ptr++) = 'R'; /* 64-bit double */
          memcpy(ptr, ev->rvalues + i, sizeof(double));
          ptr += sizeof(double);
          break;
        case 's':
          *(ptr++) = 'T'; /* 32-byte NUL-terminated string */
          memcpy(ptr, ev->svalues + i, 32);
          ptr += 32;
          break;
      }
    }
    len = ptr - buffer;
  }

  return len;
}

/* perform routine checks on an incoming packet.  Returns the packet type, or
 * -1 on error */
int mpc_check_packet(size_t len, const char *data, const char *peer, int port)
{
  const int16_t pcm_proto = (len >= 2) ? *(const int16_t*)data : -1;
  const char *ptr = data + 2; /* skip proto rev */

  /* check protocol revision */
  if (mpc_proto_rev != pcm_proto) {
    bprintf(err, "Ignoring %i-byte packet with bad protocol revision %i from "
        "%s/%i\n", len, pcm_proto, peer, port);
    return -1;
  }

  /* check type -- there's a list of valid packet codes here */
  if (*ptr != 'A' &&
      *ptr != 'C' &&
      *ptr != 'F' &&
      *ptr != 'G' &&
      *ptr != 'N' &&
      *ptr != 'Q' &&
      *ptr != 'R' &&
      *ptr != 'S' &&
      *ptr != 'T' &&
      *ptr != 'Z')
  {
    bprintf(err, "Ignoring %i-byte packet of unknown type 0x%X from %s/%i\n",
        len, (unsigned char)*ptr, peer, port);
    return -1;
  }

  return *ptr;
}

/* decompose a command into the ScheduleEvent struct.  returns target MCE (0-6)
 * on success, -1 on error */
int mpc_decompose_command(struct ScheduleEvent *ev, int mce, size_t len,
    const char *data)
{
  const char *ptr = data + 3; /* skip proto revision and packet type */
  int16_t pcm_cmd_rev;
  int i = 1; /* parameter zero is never initialised */
  int target;

  /* check command type and command list revision */
  memset(ev, 0, sizeof(*ev));
  if (*ptr < 0 || *ptr == 7 || *ptr > 8) {
    bprintf(err, "Dropping Command packet with bad type 0x%x",
        (unsigned char)*ptr);
    return -1;
  }
  if (*ptr == 8) {
    ev->is_multi = 0;
    target = 0; /* single command are always global */
  } else {
    ev->is_multi = 1;
    target = *ptr;
  }
  ptr++;

  pcm_cmd_rev = *(int16_t*)ptr;
  if (pcm_cmd_rev != mpc_cmd_rev) {
    bprintf(err, "Dropping Command packet with bad command list revision %i",
        pcm_cmd_rev);
    return -1;
  }
  ptr += 2;

  /* no poing in decoding the rest of this if we don't have to */
  if (target && target != 1 + mce)
    return target;

  ev->command = *(int16_t*)(ptr);
  ptr += 2;

  bprintf(info, "Received command #%i (%s)", ev->command,
      CommandName(ev->is_multi, ev->command));

  if (ev->is_multi)
    while (ptr - data < len) {
      switch (*(ptr++)) {
        case 'N': /* 32-bit integer */
          ev->ivalues[i] = *(int32_t*)ptr;
          ptr += sizeof(int32_t);
          bprintf(info, "Param %02i: integer: %i", i, ev->ivalues[i]);
          break;
        case 'R':
          ev->rvalues[i] = *(double*)ptr;
          ptr += sizeof(double);
          bprintf(info, "Param %02i: float:   %g", i, ev->rvalues[i]);
          break;
        case 'T':
          memcpy(ev->svalues + i, ptr, 32);
          ptr += 32;
          bprintf(info, "Param %02i: string:  %s", i, ev->svalues[i]);
          break;
        default:
          bprintf(err, "Droping Command packet with bad parameter type: 0x%X",
              *(ptr - 1));
          return -1;
      }
      ++i;
    }

  return target;
}

/* decompose an bset packet into a bolometer list, taking care of filtering on
 * the mce number; array is allocated by the caller.
 */
int mpc_decompose_bset(uint16_t *bset_num, int16_t *set, int mce, size_t len,
    const char *data)
{
  int16_t *in = (int16_t*)(data + 6);
  uint16_t new_bset_num = *(uint16_t*)(data + 4);
  const int n = (len - 6) / 2;
  int i, m = 0;

  /* no change */
  if (new_bset_num == *bset_num)
    return -1;

  for (i = 0; i < n; ++i)
    if (TES_MCE(in[i]) == mce)
      set[m++] = TES_OFFSET(in[i]);

  bprintf(info, "New BSet 0x%04X with %i channels for MCE%i\n", new_bset_num, m,
      mce);

  *bset_num = new_bset_num;
  return m;
}

/* returns the mce number on success or -1 on error */
int mpc_decompose_init(size_t len, const char *data, const char *peer, int port)
{
  if (len != 4 || data[3] < 0 || data[3] > NUM_MCE)
    return -1;

  bprintf(info, "Pinged by %s/%i running MCE%i", peer, port, data[3]);
  return data[3];
}

/* decompose the slow data and stuff it into the right struct;
 * returns -1 on error and updates nuttin */
int mpc_decompose_slow(struct mpc_slow_data slow_dat[NUM_MCE][3],
    int mce_slow_index[NUM_MCE], size_t len, const char *data,
    const char *peer, int port)
{
  size_t dat_len = sizeof(slow_dat[0][0]);
  int index, nmce;

  if (dat_len + 4 != len) {
    bprintf(err, "Bad slow data packet (size %zu) from %s/%i",
        len, peer, port);
    return -1;
  }

  /* data[3] is the MCE number */
  nmce = data[3];
  if (nmce < 0 || nmce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in slow data packet from %s/%i",
        nmce, peer, port);
    return -1;
  }

  index = mce_slow_index[nmce];

  /* again, this will probably work... */
  memcpy(&slow_dat[nmce][index], data + 4, dat_len);

  /* increment write pointer */
  mce_slow_index[nmce] = (mce_slow_index[nmce] + 1) % 3;

  return nmce;
}

int mpc_decompose_tes(int *pb_size, uint32_t *frameno, uint16_t *tes_data,
    size_t len, const char *data, uint16_t bset_num, int set_len[NUM_MCE],
    int *bad_bset_count, const char *peer, int port)
{
  int16_t nf;
  /* check len */
  if (len < 12) {
    bprintf(err, "Bad data packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  /* get mce number */
  int mce = data[3];

  if ((mce & 0x7F) < 0 || (mce & 0x7F) >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in slow data packet from %s/%i", mce & 0x7F,
        peer, port);
    return -1;
  }

  /* check bset number */
  if (*(uint16_t*)(data + 4) != bset_num) {
    (*bad_bset_count)++;
    return -1;
  }

  /* get frame count */
  *pb_size = nf = *(uint16_t*)(data + 6);

  /* check len again */
  if (len < (set_len[mce & 0x7F] * sizeof(uint16_t) + sizeof(uint32_t)) * nf +
      8)
  {
    bprintf(err, "Bad data packet (size %zu) from %s/%i, MCE %i", len, peer,
        port, mce & 0x7F);
    return -1;
  }

  /* framenumbers */
  memcpy(frameno, data + 8, sizeof(uint32_t) * nf);

  /* copy tes data */
  memcpy(tes_data, data + 8 + sizeof(uint32_t) * nf,
      len - 8 - sizeof(uint32_t) * nf);

  return mce;
}

int mpc_decompose_pcmreq(int *power_cycle, size_t len, const char *data,
    const char *peer, int port)
{
  if (len != 5) {
    bprintf(err, "Bad PCM request packet (size %zu) from %s/%i", len, peer,
        port);
    return -1;
  }

  /* get mce number */
  int mce = data[3];
  if (mce < 0 || mce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in PCM request packet from %s/%i", mce, peer,
        port);
    return -1;
  }

  *power_cycle = data[4] ? 1 : 0;

  return mce;
}

int mpc_decompose_notice(int nmce, const char **data_mode_bits, int *turnaround,
    int *divisor, int *ssdata_req, int *data_mode, int *row_len, int *num_rows,
    int *data_rate, int *squidveto, double *bolo_filt_freq,
    double *bolo_filt_bw, uint16_t *bolo_filt_len, size_t len, const char *data,
    const char *peer, int port)
{
  if (len != 33) {
    bprintf(err, "Bad notice packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  *divisor = data[3];
  *data_mode = (int)data[4];
  *squidveto = (int)data[5];
  *ssdata_req = data[6];

  *turnaround = data[7];

  memcpy(bolo_filt_freq, data + 8, sizeof(double));
  memcpy(bolo_filt_bw, data + 16, sizeof(double));
  memcpy(bolo_filt_len, data + 24, sizeof(uint16_t));

  *row_len =   (int)data[26] * 2;
  *num_rows =  (int)data[27];
  *data_rate = (int)data[28];

  *data_mode_bits = data + 29;

  return 0;
}

int mpc_decompose_param(uint32_t *param, size_t len, const char *data,
    const char *peer, int port)
{
  if (len != 4 + sizeof(uint32_t) * N_MCE_STAT) {
    bprintf(err, "Bad parameter packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  /* data[3] is the MCE number */
  int nmce = data[3];
  if (nmce < 0 || nmce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in parameter packet from %s/%i", nmce, peer,
        port);
    return -1;
  }

  memcpy(param + nmce * N_MCE_STAT, data + 4, sizeof(uint32_t) * N_MCE_STAT);
  return 0;
}

int mpc_decompose_synop(uint8_t *synop, size_t len, const char *data,
    const char *peer, int port)
{
  if (len != 4 + N_STAT_TYPES * NUM_COL * NUM_ROW) {
    bprintf(err, "Bad synopsis packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  int nmce = data[3];
  if (nmce < 0 || nmce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in synopsis packet from %s/%i", nmce, peer,
        port);
    return -1;
  }

  memcpy(synop + nmce * N_STAT_TYPES * NUM_ROW * NUM_COL,
      data + 4, N_STAT_TYPES * NUM_ROW * NUM_COL);
  return 0;
}

ssize_t mpc_decompose_gpdata(uint16_t *serial, size_t len, const char *data,
    const char *peer, int port)
{
  uint16_t payload_len, type;
  if (len < 8) {
    bprintf(err, "Bad GP packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  int16_t nmce = data[3];
  if (nmce < 0 || nmce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in synopsis packet from %s/%i", nmce, peer,
        port);
    return -1;
  }

  memcpy(&type, data + 4, sizeof(type));
  memcpy(&payload_len, data + 6, sizeof(payload_len));

  /* check length again */
  if (len != payload_len * sizeof(uint16_t) + 8) {
    bprintf(err, "Bad GP packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  /* we serialise thusly: mce, type, length, data */
  memcpy(serial++, &nmce, sizeof(nmce));
  memcpy(serial++, &type, sizeof(type));
  memcpy(serial++, &payload_len, sizeof(nmce));
  memcpy(serial, data + 8, sizeof(uint16_t) * payload_len);

  /* return length in words (including type, length and mce#) */
  return payload_len + 3;
}
