/*
 * diskmanager_tng.h
 *
 *  Created on: Mar 29, 2010
 *      Author: Seth Hillbrand
 *
 * This file is part of FCP, the EBEX flight control program
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * fcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * fcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with fcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifndef INCLUDE_DISKMANAGER_TNG_H
#define INCLUDE_DISKMANAGER_TNG_H

#include <stdint.h>
#include <sys/stat.h>
#include <ck_ht.h>

#include "file_buffer_tng.h"

/*
 *  fileentry structure is assigned to each file currently open on an disk manager volume.  Calls to operate
 * on files reference the index number of a fileentry in the #filepool
 */
typedef struct diskentry diskentry_t;
typedef struct fileentry fileentry_t;

typedef struct fileentry
{
    FILE                *fp;                /**< fp File pointer used to write data to the file */
    pthread_t           parent;             /**< parent pthread ID of the creating thread */
    char                *filename;          /**< filename base file name of the file being written/read */
    char                mode[4];            /**< mode One of "r", "w", "a" or with "+" appended */
    diskentry_t         *disk;              /**< disk Pointer to file's disk */
    filebuffer_t        buffer;             /**< buffer For file data caching in mcp */
    ck_ht_hash_t        filehash;           /**< filehash Hash of filename, used for hash table */
    uint32_t            is_closed;          /**< is_closed Set by the calling routine to make the entry for cleanup */
    int32_t             last_error;         /**< last_error In the event of disk error, this is set to errno */
} fileentry_t;


fileentry_t *file_open(const char*, const char *m_mode);
int file_close(fileentry_t*);
ssize_t file_write(fileentry_t*, const char*, size_t);
ssize_t file_read(fileentry_t*, void*, size_t, size_t);
int file_tell(fileentry_t*);
int file_printf(fileentry_t*, const char*, ...);
ssize_t file_write_struct(fileentry_t*, void*, size_t, size_t);
int file_access(const char*, int);
int file_stat(const char*, struct stat*);
int file_copy(const char*, const char*);
int file_get_path(fileentry_t*, char*);
int32_t get_current_disk_free_space();
const char * get_current_disk_mnt_point();
int16_t get_current_disk_index();

int check_disk_init();

void initialize_diskmanager();
void diskmanager_shutdown();
int make_local_symlink(char * filename);

#endif /* INCLUDE_DISKMANAGER_TNG_H */
