/* 
 * store_data.h: 
 *
 * This software is copyright (C) 2013-2017 Seth Hillbrand
 *
 * This file is part of mcp, created for the BLASTPol Project.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: May 4th, 2017 by Laura Fissel
 */

#ifndef INCLUDE_STORE_DATA_H_
#define INCLUDE_STORE_DATA_H_

#define STORE_DATA_FRAMES_PER_FILE 300 // Store 5 minutes worth of data in one file.
#define MAX_NUM_FILENAME_CHARS 72
#endif /* INCLUDE_STORE_DATA_H_ */
typedef struct {
    bool    header_written;
    bool    have_warned;
    bool    init;
    uint32_t    crc;
    uint32_t mcp_framenum;
    uint32_t frames_stored;
    E_RATE rate;
    uint16_t nrate;
    char file_name[MAX_NUM_FILENAME_CHARS];
    char chlist_name[MAX_NUM_FILENAME_CHARS];
    char type[12];
    fileentry_t *fp;
    channel_t *mcp_framenum_addr;
    channel_header_t *channels_pkg;
} store_file_info_t;
void store_data_200hz(void);
void store_data_100hz(void);
void store_data_5hz(void);
void store_data_1hz(void);
