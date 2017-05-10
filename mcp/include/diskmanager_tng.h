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

#include "file_buffer_tng.h"
#define HOME_DIR					"/data"
#define MNT_DIR_PREFIX				"mcp_hd"
#define NUM_USB_DISKS               8
#define DISK_MAX_NUMBER             NUM_USB_DISKS+1
#define DISK_MAX_FILES				100	/** Maximum number of concurrently open files */
#define DISK_MIN_FREE_SPACE			50  /** Minimum amount of free space in MB for a disk to be used */
#define DEFAULT_SYS_TYPE            "xfs"

extern int16_t SouthIAm; // Needed to determine which drives to mount.

typedef struct harddrive_info {
    char *uuid;
    char *mnt_point;
    char *filesystem;
    bool is_mounted;
    bool is_writing;
} harddrive_info_t;

// Hardware IDs for the drives connected by USB
static const char drive_uuids_fc2[NUM_USB_DISKS][64] = {"ccbff6e7-8e51-49e4-a987-9ebf5644813e",
                                                     "674e5a19-eb93-4c05-b12c-6a50c03ca5c1",
                                                     "67e991c8-1e1e-4f77-84f1-9273c050e385",
                                                     "22804e9d-a3e1-4cf8-a5b2-ff2fcf22bc5e",
                                                     "94ac1984-a52b-4be6-afb7-cb8302d249e0",
                                                     "993e105e-1cbc-4913-abca-29540242c57e",
                                                     "6846dffc-cf41-447a-a576-4ab34cad7974",
                                                     "a52e5c25-8dbc-4e55-ae73-7c5f8b49968c"};
static const char drive_uuids_fc1[NUM_USB_DISKS][64] = {"",
                                                     "",
                                                     "",
                                                     "",
                                                     "",
                                                     "",
                                                     "",
                                                     ""};
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
	uint32_t		tag;				// tag corresponds to s_diskpool::tag
	int8_t			host;				// host 0 = FatherXMas; 1 = StNick; -1 = unclaimed
} diskentry_t;

// diskpool structure keeps track of all available volumes and their last status.  This allows rapid
// reaction by the disk manager to mount a new volume and continue recording data
typedef struct diskpool
{
	volatile uint32_t	tag;				// < tag Unique identifier to mark update cycle
											// Incremented at each update.  s_diskentry entries that
											// do not match tag are blacklisted
	diskentry_t	* volatile current_disk;	// current_disk Pointer to current primary disk
	diskentry_t		disk[DISK_MAX_NUMBER];	// disk Struct of all disks ever seen
} diskpool_t;

//
// fileentry structure is assigned to each file currently open on an disk manager volume.  Calls to operate
// on files reference the index number of a fileentry in the #filepool
//
typedef struct fileentry
{
	FILE				*fp;				// fp File pointer used to write data to the file
	pthread_t			parent;				// parent pthread ID of the creating thread
	pthread_rwlock_t	mutex;				// mutex Read/Write access control for file
	char				*filename;			// filename Filename without the mount point string
	bool				continued;			// continued Boolean. true indicates that the file continues one
											// that began on a different disk.  Filename is
											// suffixed with '.cont'
	diskentry_t * volatile disk;			// disk Pointer to an entry in the #diskpool where the file is
											// currently being written
	filebuffer_t		buffer;				// buffer The filebuffer structure provides a means of buffering
	 	 	 	 	 	 	 	 	 	 	// file writes to optimize the network throughput and ensure data
	 	 	 	 	 	 	 	 	 	 	// connectivity before writing
} fileentry_t;

//
// The filepool contains a pool of all open files in the diskmanager system.  The index assigned to a file will
// remain fixed over the lifetime of the file.
//
typedef struct filepool
{
//
// N.B. The disk error flags are held in the filepool because their effect is to halt disk writing for the
// entire pool.  It is highly conceivable that many files will attempt to signal an error at the same time, so
// keeping the mutex and signal with the file pool is logical.  This allows for expansion to multiple
// pools in a future experiment (possibly in a segregated network?).  There you would not want a single lock
// in the disk pool, but you would want the file pool that has an error to respond to a single signal
// (say that 10 times fast).
//
	pthread_mutex_t	write_error_mutex;		// write_error_mutex Mutex for disk error handling
	pthread_cond_t	write_error_signal;		// write_error_signal Signal to trigger disk fail-over handling
	volatile diskentry_t	*error_disk;	// error_disk Pointer to disk on which a write error occurred

	pthread_mutex_t	buffer_mutex;			// buffer_mutex Mutex for signaling that buffers should be flushed
	pthread_cond_t	buffer_signal;			// buffer_signal Write signal to flush buffers
	fileentry_t		file[DISK_MAX_FILES];	// file Array of files being used
} filepool_t;
void diskpool_update_last_time(int m_pos, time_t m_time);
void diskpool_update_disk_tag(int m_pos);
int diskpool_is_in_pool(const char *m_mnt_point);
int diskpool_add_to_pool(diskentry_t *m_disk);
void diskpool_update_free_space(int m_pos, int32_t m_space);
int drivepool_add_init_usb_info(int m_pos, int m_hdusb);
void diskpool_update_all(void);


void *diskmanager_thread(void *m_arg);


#endif /* INCLUDE_DISKMANAGER_TNG_H */
