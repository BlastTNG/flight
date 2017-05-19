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

#include <time.h>
#include <pthread.h>
#include <sys/stat.h>
#include <limits.h>
#include <stdbool.h>
#include <ck_ht.h>

#include "file_buffer_tng.h"
#define HOME_DIR					"/data"
#define MNT_DIR_PREFIX				"mcp_hd"
#define NUM_USB_DISKS               8
#define DISK_MAX_NUMBER             NUM_USB_DISKS+1
#define DISK_MAX_FILES				100	/** Maximum number of concurrently open files */
#define DISK_MIN_FREE_SPACE			50  /** Minimum amount of free space in MB for a disk to be used */
#define DEFAULT_SYS_TYPE            "xfs"

#define FILE_BUFFER_SIZE			2*1024*1024 /*2 MB*/

extern int16_t SouthIAm; // Needed to determine which drives to mount.

typedef struct harddrive_info {
    char *uuid;
    char *mnt_point;
    char *filesystem;
    bool is_mounted;
    bool is_writing;
} harddrive_info_t;

//
// diskentry structure provides all information needed to utilize both local disks and AoE disks in the
// #diskpool
//
typedef struct diskentry
{
	bool			isUSB;				// isUSB true==USB Disk, false==Local disk
	bool			initialized;		// initialized means that the disk has been mounted and is ready for writing.
	bool			mounted;		    // is the disk mounted?
	bool			skip;		        // Set when a disk is identified as suspect.
	                                    // If set then skip unless there is no other choice.
	char			*uuid;				// Device UUID number
	char			*mnt_point;			// mnt_point Location where disk is mounted
	uint32_t		fail_count;			// fail_count Incremented each time disk does not respond to
										// update ping.  After MAX_NONRESPONSE fails
										// incrementing stops
	time_t			last_accessed;		// last_accessed Last time the disk was read or written (mounted)
	int32_t	 		free_space;			// free_space Free space on the device in MB;
    int16_t         index;              // disk index number in the diskpool disk array
} diskentry_t;

// diskpool structure keeps track of all available volumes and their last status.  This allows rapid
// reaction by the disk manager to mount a new volume and continue recording data
typedef struct diskpool
{
	diskentry_t	    *current_disk;	// current_disk Pointer to current primary disk
	diskentry_t	    local_disk;    // Pointer to the local disk
	diskentry_t     disk[NUM_USB_DISKS];
} diskpool_t;

//
// fileentry structure is assigned to each file currently open on an disk manager volume.  Calls to operate
// on files reference the index number of a fileentry in the #filepool
//
typedef struct fileentry
{
    FILE                *fp;                /**< fp File pointer used to write data to the file */
    pthread_t           parent;             /**< parent pthread ID of the creating thread */
    char                *filename;          /**< filename base file name of the file being written/read */
    char                mode[4];            /**< mode One of "r", "w", "a" or with "+" appended */
    diskentry_t         *disk;              /**< disk Pointer to an entry in the #diskpool where the file is
                                                currently being written */
    filebuffer_t        buffer;             /**< buffer The filebuffer structure provides a means of buffering
                                                 file writes to optimize the network throughput and ensure data
                                                 connectivity before writing */
    uint32_t            filehash;           /**< filehash Hash of filename, used for sl */
    uint32_t            is_closed;          /**< is_closed Set by the calling routine to make the entry for cleanup */
    int32_t             last_error;         /**< last_error In the event of disk error, this is set to errno */
} fileentry_t;


int diskpool_add_to_pool(diskentry_t *m_disk);
diskentry_t *diskpool_is_in_pool(const char *m_dev);

fileentry_t *file_open (const char *m_filename, const char *m_mode);
int file_close(fileentry_t *m_file);
ssize_t file_write (volatile fileentry_t *m_file, const char *m_data, size_t m_len);
ssize_t file_read (fileentry_t *m_file, void *m_data, size_t m_len, size_t m_num);
long int file_tell (fileentry_t *m_file);
int file_printf(fileentry_t *m_file, const char* m_fmt, ...);
ssize_t file_write_struct (volatile fileentry_t *m_file, void *m_struct, size_t m_size, size_t m_num);
int file_access(const char *m_filename, int m_permission);
int file_stat(const char *m_filename, struct stat *m_statbuf);
int file_copy(const char *m_src, const char *m_dest);
int file_get_path (fileentry_t *m_file, char *m_path);

bool initialize_diskmanager(int m_preferred_disk);
void diskmanager_shutdown();


#endif /* INCLUDE_DISKMANAGER_TNG_H */
