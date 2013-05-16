/* MCP: the MPC communication protocol
 *
 * Copyright (c) 2012-2013, D. V. Wiebe
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * - Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * This software is provided by the copyright holders and contributors "as is"
 * and any express or implied warranties, including, but not limited to, the
 * implied warranties of merchantability and fitness for a particular purpose
 * are disclaimed. In no event shall the copyright holder or contributors be
 * liable for any direct, indirect, incidental, special, exemplary, or
 * consequential damages (including, but not limited to, procurement of
 * substitute goods or services; loss of use, data, or profits; or business
 * interruption) however caused and on any theory of liability, whether in
 * contract, strict liability, or tort (including negligence or otherwise)
 * arising in any way out of the use of this software, even if advised of the
 * possibility of such damage.
 */
#include "blast.h"
#include "command_common.h"
#include "mpc_proto.h"
#include "tes.h"

#include <string.h>

static int mpc_cmd_rev = -1, mpc_proto_rev = -1;

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
    bputs(err, "MPC: Unable to parse the command list revision.");
    return -1;
  }

  bprintf(info, "MPC: Protocol revision: %i/%i\n", mpc_proto_rev, mpc_cmd_rev);
  return 0;
}

/* compose a turnaround packet for transmission to the mpc
 *
 * turnaround packet looks like:
 *
 * RRtF
 *
 * where
 *
 * R = 16-bit protocol revision
 * t = 't' indicating a turnaround packet
 * F = the turnaround flag (either 0 or 1)
 */
size_t mpc_compose_turnaround(int flag, char *buffer)
{
  int16_t i16 = mpc_proto_rev;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
  buffer[2] = 't'; /* turnaround packet */
  buffer[3] = flag ? 1 : 0; /* turnaround */
  return 4;
}

/* compose a TES data packet for transmission from the mpc
 *
 * data packet looks like:
 *
 * RRTMBB00FFFF...
 *
 * where
 *
 * R = 16-bit protocol revision
 * T = 'T' indicating a slow packet
 * M = 8-bit MCE number
 * B = 16-bit bset number
 * 0 = padding
 * F = 32-bit framenumber
 *
 * followed by the tes data in bset order
 */
size_t mpc_compose_tes(const uint32_t *data, uint32_t framenum,
    uint16_t bset_num, int nmce, int ntes, const int16_t *tesind, char *buffer)
{
  size_t len = 12 + sizeof(uint32_t) * ntes;
  int16_t i16 = mpc_proto_rev;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
  buffer[2] = 'T'; /* tes data packet */
  buffer[3] = nmce & 0xF; /* mce number */
  memcpy(buffer + 4, &bset_num, sizeof(bset_num)); /* BSET */
  memcpy(buffer + 8, &framenum, sizeof(framenum)); /* FRAMENUM */

  /* append data */
  memcpy(buffer + 12, data, sizeof(uint32_t) * ntes);

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
  int16_t i16 = mpc_proto_rev;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
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
  int16_t i16 = mpc_proto_rev;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
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
  int16_t i16 = mpc_proto_rev;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
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
 * T = 'm' or 's' indicating single or multiword command
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
  int32_t i32;
  int16_t i16 = mpc_proto_rev;
  char *ptr;
  int i;

  memcpy(buffer, &i16, sizeof(i16)); /* 16-bit protocol revision */
  buffer[2] = 'C'; /* command packet */
  buffer[3] = ev->is_multi ? 'm' : 's'; /* multi/single command */
  i16 = mpc_cmd_rev;
  memcpy(buffer + 4, &i16, sizeof(i16)); /* 16-bit command list revision */
  i16 = ev->command;
  memcpy(buffer + 6, &i16, sizeof(i16)); /* 16-bit command number */
  if (ev->is_multi) {
    ptr = buffer + 8;
    for (i = 0; i < mcommands[ev->t].numparams; ++i) {
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
  if (*ptr != 'A' && *ptr != 'C' && *ptr != 'F' && *ptr != 'S' && *ptr != 'T'
      && *ptr != 't') {
    bprintf(err, "Ignoring %i-byte packet of unknown type 0x%X from %s/%i\n",
        len, (unsigned char)*ptr, peer, port);
    return -1;
  }

  return *ptr;
}

/* decompose a command into the ScheduleEvent struct.  returns non-zero on
 * error */
int mpc_decompose_command(struct ScheduleEvent *ev, size_t len,
    const char *data)
{
  const char *ptr = data + 3; /* skip proto revision and packet type */
  int16_t pcm_cmd_rev;
  int i = 0;

  /* check command type and command list revision */
  memset(ev, 0, sizeof(*ev));
  if (*ptr != 'm' && *ptr != 's') {
    bprintf(err, "Dropping Command packet with bad type 0x%x",
        (unsigned char)*ptr);
    return -1;
  }
  ev->is_multi = (*ptr == 'm') ? 1 : 0;
  ptr++;

  pcm_cmd_rev = *(int16_t*)ptr;
  if (pcm_cmd_rev != mpc_cmd_rev) {
    bprintf(err, "Dropping Command packet with bad command list revision %i",
        pcm_cmd_rev);
    return -1;
  }
  ptr += 2;

  ev->command = *(int16_t*)(ptr);
  ptr += 2;

  bprintf(info, "Received %sword command #%i (%s)", ev->is_multi ? "mutli" :
      "single ", ev->command, CommandName(ev->is_multi, ev->command));

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

  return 0;
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
  if (new_bset_num == *bset_num) {
    bprintf(info, "Ignoring rebroadcast BSet 0x%04X\n", new_bset_num);
    return -1;
  }

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
  if (nmce < 0 || nmce > NUM_MCE) {
    bprintf(err, "Unknown MCE %i in slow data packet from %s/%i",
        nmce, peer, port);
    return -1;
  }

  index = mce_slow_index[nmce];

  /* again, this will probably work... */
  memcpy(&slow_dat[nmce][index], data + 4, dat_len);

  /* increment write pointer */
  mce_slow_index[nmce] = (mce_slow_index[nmce] + 1) % 3;

  return 0;
}

int mpc_decompose_tes(uint32_t *tes_data, size_t len, const char *data,
    uint16_t bset_num, int set_len[NUM_MCE], int *bad_bset_count,
    const char *peer, int port)
{
  /* check len */
  if (len < 12) {
    bprintf(err, "Bad data packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  /* get mce number */
  int mce = data[3];
  if (mce < 0 || mce >= NUM_MCE) {
    bprintf(err, "Unknown MCE %i in slow data packet from %s/%i", mce, peer,
        port);
    return -1;
  }

  /* check len again */
  if (len < set_len[mce] * 4 + 12) {
    bprintf(err, "Bad data packet (size %zu) from %s/%i", len, peer, port);
    return -1;
  }

  /* check bset number */
  if (*(uint16_t*)(data + 4) != bset_num) {
    (*bad_bset_count)++;
    return -1;
  }

  /* copy tes data */
  memcpy(tes_data, data + 8, len - 8);

  return mce;
}

int mpc_decompose_turnaround(size_t len, const char *data, int old)
{
  if (len < 4)
    return old;

  bprintf(info, "%s turnaround", data[3] ? "Into" : "Out of");
  return data[3] ? 1 : 0;
}
