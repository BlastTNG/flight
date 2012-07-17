/* mcp: the master control program
 *
 * flcdataswap - swap data between FLCs using UDP packets
 *
 * This software is copyright (C) 2012 University of Toronto
 *
 * This file is part of mcp and pcm
 *
 * mcp and pcm are free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp and pcm are distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 *
 */

#ifndef FLC_DATA_SWAP_H
#define FLC_DATA_SWAP_H

#define FLC_DATA_PORT "17777"

struct flc_data {
  //TODO this string contet is for testing only
  char msg[128];
  /* things we want:
      time
      free disk space
      command count
      last command received
      schedule timeout
      possibly a CRC check
      */
};

//start send and receive thread. "other" should contain dot-and-numbers IP
void start_flc_data_swapper(const char *other);

//execute data swap. Overwrites d with data from other computer
struct flc_data *swap_flc_data(struct flc_data *d);

#endif  //FLC_DATA_SWAP_H


