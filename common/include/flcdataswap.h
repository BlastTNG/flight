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
  time_t time;
  unsigned short df;
  unsigned short t_cpu;
  time_t timeout;
  unsigned short last_command;
  unsigned short command_count;

  /* things we want:
      time  wide "time_x_flc"
      free disk space "disk_free" -> "df_x_flc"
      command count   "count_x_cmd"
      last command received "last_x_cmd"
      schedule timeout  "timeout" -> "timeout_x"
      cpu temperature "t_cpu_flc" -> "t_cpu_x_flc"
      possibly a CRC check
      */
};

//start send and receive threads. "other" should contain dot-and-numbers IP
void start_flc_data_swapper(const char *other);

//get pointer to output write buffer. Stable between calls to swap_flc_data()
struct flc_data *get_flc_out_data();

//execute data swap. Fills d with received data (and returns pointer to it)
struct flc_data *swap_flc_data(struct flc_data *d);

#endif  //FLC_DATA_SWAP_H


