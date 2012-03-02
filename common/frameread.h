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

#define FR_DONE            0  /* nothing left in file and not in persist mode */
#define FR_MORE_IN_FILE    1  /* current chunk has increased in size */
#define FR_NEW_CHUNK       2  /* current chunk done, new chunk found */
#define FR_CURFILE_CHANGED 3  /* no new chunk, curfile points to new file */

#define GPB_LEN (PATH_MAX * 4)
#define FILENAME_LEN (PATH_MAX + NAME_MAX + 1)

#ifdef HAVE_STDINT_H
#include <stdint.h>
typedef uint16_t chunkindex_t;
#elif SIZEOF_UNSIGNED == 4
typedef unsigned chunkindex_t;
#elif SIZEOF_UNSIGNED_SHORT == 4
typedef unsigned short chunkindex_t;
#elif SIZEOF_UNSIGNED_LONG == 4
typedef unsigned long chunkindex_t;
#else
typedef unsigned chunkindex_t;
#endif

unsigned long long GetFrameFileSize(
    const char*,
    int
    );

int GetNextChunk(
    char*,
    int
    );

char* GetSpecFile(
    char*,
    const char*,
    const char*
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
