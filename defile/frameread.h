/* frameread: reads mcp-style framefiles
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * frameread is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * frameread is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with frameread; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#ifndef FRAMEREAD_H
#define FRAMEREAD_H

#include <linux/limits.h>

#define FR_DONE            0
#define FR_MORE_IN_FILE    1
#define FR_NEW_CHUNK       2
#define FR_CURFILE_CHANGED 3

#define GPB_LEN (PATH_MAX * 4)
#define FILENAME_LEN (PATH_MAX + NAME_MAX + 1)

typedef unsigned int chunkindex_t;

unsigned long GetFrameFileSize(
    const char*,
    int
    );

int GetNextChunk(
    char*,
    int
    );

void PathSplit_r(
    const char*,
    char*,
    char*
    );

int  ReconstructChannelLists(
    const char*,
    const char*
    );

long int SetStartChunk(
    long int,
    char*,
    int
    );

int StaticSourcePart(
    char*,
    const char*,
    chunkindex_t*,
    int
    );

int StreamToNextChunk(
    int,
    char*,
    int,
    int*,
    const char*,
    char*
    );

#endif
