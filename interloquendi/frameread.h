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

#define GPB_LEN (PATH_MAX * 4)
#define FILENAME_LEN (PATH_MAX + NAME_MAX + 1)

typedef unsigned int chunkindex_t;

int      GetNextChunk     ( char*    , int                               );
long int SetStartChunk    ( long int , char*       , int           );
int      StaticSourcePart ( char*    , const char* , chunkindex_t* , int );

#endif
