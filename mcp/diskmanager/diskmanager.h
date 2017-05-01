/*
 * diskmanager.h
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

#ifndef DISKMANAGER_H_
#define DISKMANAGER_H_

#include <time.h>
#include <pthread.h>
#include <sys/stat.h>
#include <limits.h>
#include <stdbool.h>

#include "aoe_common.h"
#include "file_buffer.h"

#define	DISK_MAX_NUMBER				16	/** Maximum number of disks available */
#define DISK_NUM_PRESSURE_VESSELS	2	/** Set the number of pressure vessels in use */
#define DISK_MAX_NONRESPONSE		100	/** Increment limit for non-responding disks */
#define DISK_MAX_FILES				100	/** Maximum number of concurrently open files */
#define DISK_MIN_FREE_SPACE			50  /** Minimum amount of free space in MB for a disk to be used */
#define DISK_TIME_LIMIT				600 /** 10 minutes worth of seconds.  Defines period for which a disk is
											considered 'in use' after last access. */

#define FILE_BUFFER_SIZE			2*1024*1024 /*2 MB*/

#define DISK_BUFFER_MAX_WAIT		1 	/* Maximum time in seconds to wait before writing data.
										 * This is set low to facilitate testing with a low data rate.
										 * @TODO: Determine an optimal wait time (concurrent with buffer size)
									     */
#define DISK_AOE_POLL_TIME			10	/* Number of seconds between AoE disk polls */

#define HOME_DIR					"/data"
#define MNT_DIR_PREFIX				"fcp_"

#ifndef __bool_true_false_are_defined
#	define true 1
#	define false 0
#endif

#ifdef AOE_DEBUG
#	define aoe_dbg(...) ebex_dbg(...)
#else
#	define aoe_dbg(...)
#endif

/**
 * diskentry structure provides all information needed to utilize both local disks and AoE disks in the
 * #diskpool
 */
typedef struct diskentry
{
	uint16_t		major;				/**< major Major number of the AoE disk i.e. \#.x */
	uint8_t			minor;				/**< minor Minor number of the AoE disk i.e. x.\# */
	uint8_t			MAC[6];				/**< MAC Media Access Control address of the AoE disk */
	bool			isAoE;				/**< isAoE true==AoE Disk, false==Local disk */
	char			*dev;				/**< dev Disk \/dev entry.  Needed for non-AoE disks */
	char			*mnt_point;			/**< mnt_point Location where disk is mounted */
	uint32_t		fail_count;			/**< fail_count Incremented each time disk does not respond to
											update ping.  After MAX_NONRESPONSE fails
											incrementing stops*/
	time_t			last_accessed;		/**< last_accessed Last time the disk was read or written (mounted) */
	int32_t	 		free_space;			/**< free_space Free space on the device in MB; */
	uint32_t		tag;				/**< tag corresponds to s_diskpool::tag */
	int8_t			host;				/**< host 0 = FatherXMas; 1 = StNick; -1 = unclaimed */
} diskentry_t;

/**
 * diskpool structure keeps track of all available volumes and their last status.  This allows rapid
 * reaction by the disk manager to mount a new volume and continue recording data
 */
typedef struct diskpool
{
	volatile uint32_t	tag;				/**< tag Unique identifier to mark update cycle
												Incremented at each update.  s_diskentry entries that
												do not match tag are blacklisted */
	diskentry_t	* volatile current_disk;	/**< current_disk Pointer to current primary disk */
	diskentry_t		disk[DISK_MAX_NUMBER];	/**< disk Struct of all disks ever seen */
} diskpool_t;

/**
 * fileentry structure is assigned to each file currently open on an disk manager volume.  Calls to operate
 * on files reference the index number of a fileentry in the #filepool
 */
typedef struct fileentry
{
	FILE				*fp;				/**< fp File pointer used to write data to the file */
	pthread_t			parent;				/**< parent pthread ID of the creating thread */
	pthread_rwlock_t	mutex;				/**< mutex Read/Write access control for file */
	char				*filename;			/**< filename Filename without the mount point string */
	bool				continued;			/**< continued Boolean. true indicates that the file continues one
												that began on a different disk.  Filename is
												suffixed with '.cont' */
	diskentry_t * volatile disk;			/**< disk Pointer to an entry in the #diskpool where the file is
												currently being written */
	filebuffer_t		buffer;				/**< buffer The filebuffer structure provides a means of buffering
	 	 	 	 	 	 	 	 	 	 	 	 file writes to optimize the network throughput and ensure data
	 	 	 	 	 	 	 	 	 	 	 	 connectivity before writing */
} fileentry_t;

/**
 * The filepool contains a pool of all open files in the diskmanager system.  The index assigned to a file will
 * remain fixed over the lifetime of the file.
 */
typedef struct filepool
{
	/**
	 * N.B. The disk error flags are held in the filepool because their effect is to halt disk writing for the
	 * entire pool.  It is highly conceivable that many files will attempt to signal an error at the same time, so
	 * keeping the mutex and signal with the file pool is logical.  This allows for expansion to multiple
	 * pools in a future experiment (possibly in a segregated network?).  There you would not want a single lock
	 * in the disk pool, but you would want the file pool that has an error to respond to a single signal
	 * (say that 10 times fast).
	 */
	pthread_mutex_t	write_error_mutex;		/**< write_error_mutex Mutex for disk error handling */
	pthread_cond_t	write_error_signal;		/**< write_error_signal Signal to trigger disk fail-over handling */
	volatile diskentry_t	*error_disk;	/**< error_disk Pointer to disk on which a write error occurred */

	pthread_mutex_t	buffer_mutex;			/**< buffer_mutex Mutex for signaling that buffers should be flushed */
	pthread_cond_t	buffer_signal;			/**< buffer_signal Write signal to flush buffers */
	fileentry_t		file[DISK_MAX_FILES];	/**< file Array of files being used */
} filepool_t;

int diskpool_add_to_pool(diskentry_t *m_disk);
unsigned int diskpool_get_current_tag();
int diskpool_is_in_pool(const char *m_dev);

void diskpool_update_all();

/* @TODO: I would prefer filepool_flush_buffers to be static but is currently accessed by
 * logfile.c.  Fixing this will require implementing system-wide graceful
 * shutdown procedures.
 */
void filepool_flush_buffers();

int file_open (const char *m_filename, const char *m_mode);
int file_close (int m_fileid);
ssize_t file_write (int m_fileid, const char *m_data, size_t m_len);
ssize_t file_read (int m_fileid, void *m_data, size_t m_len, size_t m_num);
long int file_tell (int m_fileid);
int file_error (int m_fileid);
int file_get_path (int m_fd, char *m_path);
int file_printf(int m_fileid, const char* m_fmt, ...);
ssize_t file_write_struct (int m_fileid, void *m_struct, size_t m_size, size_t m_num);
int file_access(const char *m_filename, int m_permission);
int file_tail(int m_fileid, char **m_dest, size_t m_len);
int file_stat(const char *m_filename, struct stat *m_statbuf);
int file_copy(const char *m_src, const char *m_dest);

void *diskmanager_thread(void *m_arg);
void diskmanager_shutdown();

void *diskmanager_error_handle_thread(void *m_arg);

int diskpool_add_to_pool(diskentry_t *m_disk);

void diskpool_update_disk_tag(int m_pos);
void diskpool_update_last_time(int m_pos, time_t m_time);
void diskpool_update_host(int m_pos, int8_t m_host_id);
void diskpool_update_free_space(int m_pos, int32_t m_space);

int diskpool_dummy_mkdir (const char *m_dir);

#endif /* DISKMANAGER_H_ */
