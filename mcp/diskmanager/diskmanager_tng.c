/**
 * diskmanager.c
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

#include <blast.h>

#include <pthread.h>
#include <sys/statvfs.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/mount.h>
#include <dirent.h>
#include <mntent.h>
#include <limits.h>
#include <stdarg.h>
#include <sched.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <stdio.h>
#include <stddef.h>
#include <stdint.h>
#include <time.h>

#include "diskmanager_tng.h"
static int diskpool_verify_primary_disk();

static int file_change_disk(int m_fileid, diskentry_t *m_disk);
static int file_reopen_on_new_disk(int m_fileid, diskentry_t *m_disk);
static void filepool_handle_disk_error(volatile diskentry_t *m_disk);

static int file_open_local(const char*, const char*);
static void file_close_internal(int, bool);
static inline bool file_is_local(const char*);

static harddrive_info_t hd_info[NUM_USB_DISKS];
static diskpool_t s_diskpool;
static filepool_t s_filepool;

static const uint64_t diskman_timeout = 250000000;	// diskman_timeout 1/4 second in nano seconds

// When set to true, the diskmanager freespace thread will update the current drive's flash
static bool s_diskmanager_update_freespace_flag = false;

// When set to true, diskmanager will exit on next loop
static volatile bool s_diskmanager_exit = false;

// Initialized to False, procedures dependent on file access wait for true
static volatile bool s_ready = false;
static const float BUFFER_WRITE_PERCENTAGE = 0.5; // A buffer more than this percentage full
												  // triggers a disk write

/* Controls access to the diskpool */
static pthread_rwlock_t mutex_diskpool = PTHREAD_RWLOCK_INITIALIZER;
/* Controls access to the filepool */
static pthread_rwlock_t mutex_filepool = PTHREAD_RWLOCK_INITIALIZER;

static pthread_t 		diskspace_thread;		/**< flash_thread is the thread ID for updating disk flash entries */

#define DISK_BUFFER_MAX_WAIT		1 	// Maximum time in seconds to wait before writing data.
										// This is set low to facilitate testing with a low data rate.

// TODO(laura): Write something that actually checks whether the disks are alive.
// Maybe something using the equivalent of touch?
static int usb_is_disk_alive()
{
    // blast_info("Hope so!");
    return(1);
}

/**
 * This function exists to keep the #HOME_DIR directory clean.
 *
 * @details When mcp quits, it will leave a unique directory in #HOME_DIR.  It may also leave
 * a filesystem mounted there.  This function ensures that the crufty directories do not pile
 * up and that we don't have bunches of spurious mounts running around.  This function should
 * only be called once, on startup.
 */
static void diskmanager_clear_old_mounts()
{
	int				LINE_SIZE = 512;
	char			mount_line[LINE_SIZE];
    int i = 0;

	struct mntent	mountentry_s;
	FILE			*mtab_fp = NULL;

	DIR				*data_dirp = NULL;

	mtab_fp = setmntent("/proc/mounts", "r");
	if (mtab_fp == NULL) {
		blast_err("Could not open /proc/mounts");
		return;
	}
	BLAST_ZERO(mountentry_s);
	BLAST_ZERO(mount_line);

	while (getmntent_r(mtab_fp, &mountentry_s, mount_line, LINE_SIZE-1) != NULL) {
		/**
		 * We check that this is a directory we would like to unmount in two ways:
		 * - It must contain the prefix we use to identify the mounts we create
		 * - It must start with the HOME_DIR directory
		 */
		if ((strstr(mountentry_s.mnt_dir, MNT_DIR_PREFIX) != NULL)
			&& (strstr(mountentry_s.mnt_dir, HOME_DIR) == mountentry_s.mnt_dir)) {
			blast_info("Unmounting/Removing %s", mountentry_s.mnt_dir);
			if ((umount2(mountentry_s.mnt_dir, 0) == 0) && (rmdir(mountentry_s.mnt_dir) == 0)) {
				/* Since we are changing mounted volumes, close and re-open to ensure we don't miss anything */
				endmntent(mtab_fp);
				mtab_fp = setmntent("/proc/mounts", "r");
				blast_info("Successfully unmounted %s", mountentry_s.mnt_dir);
			} else {
				blast_info("Could not remove %s: %s", mountentry_s.mnt_dir, strerror(errno));
			}
		} else {
			blast_info("Keeping %s", mountentry_s.mnt_dir);
		}
	}

	endmntent(mtab_fp);

	data_dirp = opendir(HOME_DIR);
	if (data_dirp == NULL) {
		blast_fatal("Could not open HOME_DIR %s:%s", HOME_DIR, strerror(errno));
		return;
	}

    for (i = 0; i < NUM_USB_DISKS; i++) {
        if (rmdir(hd_info[i].mnt_point) == -1) {
            blast_info("Could not delete %s.", hd_info[i].mnt_point);
        } else {
            blast_info("Deleted %s.", hd_info[i].mnt_point);
        }
    }

	closedir(data_dirp);
}


// Called after a disk response.  Sets the last access time
//
// @param m_pos Disk position number to update
// @param m_time Disk last access time
void diskpool_update_last_time(int m_pos, time_t m_time)
{
	if (m_pos < DISK_MAX_NUMBER && m_pos >= 0) {
		s_diskpool.disk[m_pos].last_accessed = m_time;
		blast_info("Updated access time for diskpool disk index %i", m_pos);
    }
}

// Returns the free space in megabytes of the mounted filesystem
//
// Note: This function does not check that it is the root of a mounted filesystem.
// Thus, if you pass the mount point and the disk is not mounted, you get the free space
// of the underlying filesystem.  The calling function needs to be smart enough to do this.
//
// The positive return value can encode up to 2 petabytes thus overflow is not a problem.
//  Do not change the return value to int without making some allowance for free space >32GiB
//
// @param [in] m_disk Path to the mounted filesystem or any file residing on it.
// @return -1 on error, free space in megabytes on success
static int32_t diskmanager_freespace(const char *m_disk)
{
	struct statvfs 	statvfsbuf;
	int64_t retval = -1;

	if(statvfs(m_disk, &statvfsbuf) == -1) {
		blast_err("Failed to stat %s", m_disk);
		return retval;
	}
	retval = (statvfsbuf.f_bsize/1024) * (statvfsbuf.f_bavail/1024);
    // blast_info("Free space on disk %s is %li", m_disk, retval);
	return retval;
}

// @defgroup PoolMutexFunctions Convenience functions for the file pool and disk pool mutexes
// @{
static inline bool diskpool_readlock()
{
	struct timespec timeout;
	set_timeout(&timeout, 0, diskman_timeout);
	if (pthread_rwlock_timedrdlock(&mutex_diskpool, &timeout) != 0) {
		return false;
	}
	return true;
}

static inline bool diskpool_writelock()
{
	struct timespec timeout;
	set_timeout(&timeout, 0, diskman_timeout);
	if(pthread_rwlock_timedwrlock(&mutex_diskpool, &timeout) != 0) {
		return false;
	}
	return true;
}

static inline void diskpool_unlock()
{
    blast_info("diskpool unlocked.");
	pthread_rwlock_unlock(&mutex_diskpool);
}

static inline bool filepool_readlock()
{
	struct timespec timeout;
	set_timeout(&timeout, 0, diskman_timeout);
	if(pthread_rwlock_timedrdlock(&mutex_filepool, &timeout) != 0) {
		return false;
	}
	return true;
}

static inline bool filepool_writelock()
{
	struct timespec timeout;
	set_timeout(&timeout, 0, diskman_timeout);
	while(pthread_rwlock_timedwrlock(&mutex_filepool, &timeout) != 0) {
		return false;
	}
	return true;
}

static inline void filepool_unlock()
{
	pthread_rwlock_unlock(&mutex_filepool);
}

/**
 * @defgroup DiskInitializers Initialization functions for the disk pool and file pool
 * @{
 */
/**
 * Initialize the shared #s_filepool structure and associated mutexes
 * @return -1 on failure, 0 on success
 */
static int filepool_init()
{
	int i = 0;

	if (!filepool_writelock()) {
		blast_fatal("Could not initialize mutex_filepool");
		return -1;
	}

	if (pthread_mutex_init(&s_filepool.write_error_mutex, NULL) != 0) {
		blast_fatal("filepool_init : Could not initialize write_error_mutex");
		filepool_unlock();
		return -1;
	}

	if (pthread_cond_init(&s_filepool.write_error_signal, NULL) != 0) {
		blast_fatal("filepool_init : Could not initialize write_error_signal");
		filepool_unlock();
		return -1;
	}

	if (pthread_mutex_init(&s_filepool.buffer_mutex, NULL) != 0) {
		blast_fatal("filepool_init : Could not initialize buffer_mutex");
		filepool_unlock();
		return -1;
	}

	if (pthread_cond_init(&s_filepool.buffer_signal, NULL) != 0) {
		blast_fatal("filepool_init : Could not initialize buffer_signal");
		filepool_unlock();
		return -1;
	}

	memset(s_filepool.file, 0, sizeof(fileentry_t) * DISK_MAX_FILES);

	for (i = 0; i < DISK_MAX_FILES; i++) {
		if(pthread_rwlock_init(&(s_filepool.file[i].mutex), NULL) != 0) {
			blast_fatal("filepool_init : Could not initialize file %d mutex", i);
			filepool_unlock();
			return -1;
		}
	}
	filepool_unlock();
	return 0;
}


/**
 * Initialize the shared #s_diskpool structure.
 * @return -1 on failure, 0 on success
 */
static int diskpool_init()
{
	if (!diskpool_writelock()) {
		blast_fatal("diskpool_init : Could not initialize mutex_diskpool");
		return -1;
	}

	s_diskpool.current_disk = NULL;
	memset(s_diskpool.disk, 0, sizeof(diskentry_t) * DISK_MAX_NUMBER);

	diskpool_unlock();
	return 0;
}

// Checks whether a device has already been added to the diskpool
// @param m_dev NULL-terminated string giving the device name e.g. /dev/etherd/e0.1\0
// @return -1 if not in pool, diskpool position otherwise
int diskpool_is_in_pool(const char *m_mnt_point)
{
	int i = 0;

	for (i = 0; i < DISK_MAX_NUMBER; i++) {
		if((s_diskpool.disk[i].mnt_point) && (strcmp(s_diskpool.disk[i].mnt_point, m_mnt_point) == 0)) {
			blast_dbg("Found %s at position %d", m_mnt_point, i);
			return i;
		}
	}
	blast_dbg("Did not find %s in pool", m_mnt_point);
	return -1;
}

// Iterate through the #s_diskpool to find first entry with no access time.  This
// should correspond to an open entry.  The counter will increment from 0 through
// DISK_MAX_NUMBER -1
//
// @param m_disk Pointer to a populated #diskentry_t structure
// @return -1 on failure, 0 on success
int diskpool_add_to_pool(diskentry_t *m_disk)
{
	int i = -1;

	if (s_diskmanager_exit)
		return -1;

	blast_dbg("adding %s to pool", m_disk->mnt_point?m_disk->mnt_point:"unk");
	while (++i < DISK_MAX_NUMBER) {
		if (s_diskpool.disk[i].last_accessed == 0) {
			memcpy(&(s_diskpool.disk[i]), m_disk, sizeof(diskentry_t));
			blast_dbg("Added new disk at position %d", i);
			return 0;
		}
	}
	blast_err("No open spaces for new disk");
	return -1;
}

// Called after a disk response.  Sets the free space on disk in #s_diskpool.
// We avoid setting the current disk's freespace as we don't want to overwrite
// local values with old data received from AoE FLASH
//
// @param m_pos Disk position number to update
// @param m_freespace Disk free space in MB
void diskpool_update_free_space(int m_pos, int32_t m_space)
{
	if (m_pos < DISK_MAX_NUMBER
			&& m_pos >= 0
			&& (&s_diskpool.disk[m_pos] != s_diskpool.current_disk)) {
		s_diskpool.disk[m_pos].free_space = m_space;
		blast_info("Updating the disk space for disk index %i, to %i", m_pos, m_space);
	}
}

// Update the local disk entry in the pool
static void diskpool_update_local()
{
	diskentry_t disk;
	int position;
	char *local_dir = NULL;

	blast_tmp_sprintf(local_dir, "%s/local/", HOME_DIR);

	position = diskpool_is_in_pool("local");

	// If we get a valid (>-1) position back then we have already run this routine and we just update
	// the disk information
	if (position > -1) {
		diskpool_update_last_time(position, time(NULL));
		disk.free_space = diskmanager_freespace(local_dir);
		if (disk.free_space == -1) {
			blast_err("Error getting free space from local disk");
		} else {
			diskpool_update_free_space(position, disk.free_space);
		}
		return;
	}

	mkdir(HOME_DIR, ACCESSPERMS);
	if ((access(HOME_DIR, R_OK|W_OK|X_OK) == -1)
		&& chmod(HOME_DIR, ACCESSPERMS) == -1) {
		blast_fatal("Could not access directory %s", HOME_DIR);
		return;
	}
	mkdir(local_dir, ACCESSPERMS);
	if ((access(local_dir, R_OK|W_OK|X_OK) == -1)
		&& chmod(local_dir, ACCESSPERMS) == -1) {
		blast_fatal("Could not access local directory %s", local_dir);
		return;
	}

	disk.uuid = bstrdup(err, "local");
	disk.fail_count = 0;
	disk.mnt_point = bstrdup(err, local_dir);
	disk.free_space = diskmanager_freespace(disk.mnt_point);
	disk.isUSB = false;
	disk.last_accessed = time(NULL);
	if (diskpool_add_to_pool(&disk) == -1) {
		blast_fatal("Could not add local disk to pool");
	}
}

static int diskpool_mkdir_recursive(char *m_directory)
{
	char		current_path[PATH_MAX];
	char		*path_chunk = NULL;
	char		*strtok_ref = NULL;
	int			ret = 0;
	int         stat_return = 0;
	char        *path_ptr;
	size_t      path_chunk_len = 0;
	size_t      current_path_len = 0;

	struct stat dir_stat;
//    blast_info("Making directory %s", m_directory);
	/** Ensure that we have a NULL delimiter in our temp string */
	current_path[0]=0;
    path_ptr = current_path;  // We use this to keep track of where we should be writing in current_path;

	// Takes a chunk of the directory path at a time
	path_chunk = strtok_r(m_directory, "/", &strtok_ref);
	if (m_directory[0] == '/') {
		snprintf(path_ptr, sizeof("/"), "/");
		path_ptr++;
	}
	while (path_chunk != NULL) {
//      blast_info("current_path = %s, path_chunk = %s", current_path, path_chunk);
	    current_path_len = strlen(current_path);
	    path_chunk_len = strlen(path_chunk);
//      blast_info("current_path_len = %i, path_chunk_len = %i", (int) current_path_len, (int) path_chunk_len);
		if (current_path_len + path_chunk_len + 2 > PATH_MAX) {
			blast_err("Path too long");
			return -1;
		}
		snprintf(path_ptr, path_chunk_len + 1, path_chunk); // The +1 is for the '\0'
		path_ptr += path_chunk_len;
		snprintf(path_ptr, sizeof("/"), "/");
		path_ptr++;
	    current_path_len = strlen(current_path);
//      blast_info("current_path = %s, length = %i", current_path, (int) current_path_len);
        stat_return = stat(current_path, &dir_stat);
		if (stat_return != 0) {
		    blast_info("Making path %s", current_path);
			ret = mkdir(current_path, ACCESSPERMS);
			if (ret < 0) {
				blast_strerror("Could not make %s", current_path);
				return ret;
			}
		}
		path_chunk = strtok_r(NULL, "/", &strtok_ref);
	}
	return 0;
}

// Checks if the string specifies a fully justified pathname.  If path is fully-justified,
// we consider the file to be local
//
// @param [in] m_filename Character string for file
// @return true if #m_filename is local, false if #m_filename is relative
static inline bool file_is_local(const char *m_filename)
{
	return (m_filename && m_filename[0] == '/');
}

// Takes a fully justified path to a file and
// recursively makes the directory structure to hold the file
//
// @details Begins by extracting the first token delimited by a
// slash.  Since the slash is not included in the token, it appends a
// slash to the directory name.  Then, if we get another token, we know
// that we have at least 1 directory to make.  The directory is created
// and we loop.  The last token is the filename, which we don't want as
// a directory.  We check for disk and permission errors but allow for
// existence errors (we don't care if the directory is already created)
//
// @param [in] m_file Justified file or directory name.  If partially justified,
// 				the primary disk's mount point is prepended
// @param [in] m_file_appended true if #m_filename is suffixed with a filename
// 				false if #m_filename is a bare directory name
// @return -1 on error, 0 on success
static int diskpool_mkdir_file(const char *m_filename, bool m_file_appended)
{
	char	directory_name[PATH_MAX];
	size_t	len = 0;
    // blast_info("Attempting to make mount files %s", m_filename);
	if (m_filename == NULL) {
		blast_err("Passed NULL pointer");
		return -1;
	}

	if (m_file_appended) {
		len = (size_t)(strrchr(m_filename, '/') - m_filename);
	} else {
		len = strlen(m_filename);
	}

	/** len is used for the maximum number of characters inclusive of the \0 char, so increment
	 * by one from strlen's return
	 */
	if (++len > PATH_MAX) len = PATH_MAX;

	if (file_is_local(m_filename)) {
		snprintf(directory_name, len, "%s", m_filename);
	} else {
	    // blast_info("Waiting for diskpool_verify_primary_disk to return not -1.");
		while(diskpool_verify_primary_disk() == -1) {
			pthread_yield();
		}
		len += strlen(s_diskpool.current_disk->mnt_point) + 1; /* The additional character here is for the '/' */
		if (len > PATH_MAX) len = PATH_MAX;

		snprintf(directory_name, len+1, "%s/%s", s_diskpool.current_disk->mnt_point, m_filename);
	}

	return diskpool_mkdir_recursive(directory_name);
}

// Attempt to make a mount point
// @param [out] m_mntpoint char array with the desired name of the mountpoint.
// @return -1 on failure, 0 on success
static int diskpool_make_mount_point(char *m_mntpoint)
{
	time_t currenttime = time(NULL);
	if (m_mntpoint == NULL) {
		blast_err("Passed NULL pointer");
		return -1;
	}
//  snprintf(temp_mount, PATH_MAX, "%s/%s%lu", HOME_DIR, MNT_DIR_PREFIX,
//  						(unsigned long)currenttime);
	errno = 0;
	// First test: does the directory already exist?
	DIR* dir = opendir(m_mntpoint);
	if (dir) {
        blast_info("Mount directory %s already exists.", m_mntpoint);
        closedir(dir);
    } else if (ENOENT == errno) {
        blast_info("Mount directory %s does not exist.", m_mntpoint);
        if (diskpool_mkdir_file(m_mntpoint, false) == -1) {
		    blast_strerror("Could not create new directory");
		    return -1;
		}
	} else {
        /* opendir() failed for some other reason. */
        blast_err("Error attempting to open mountpoint directory %s.", m_mntpoint);
        return -1;
    }
	return 0;
}

// Mounts a device with a specified UUID and returns
//
// Assumes we know the file system of our disks
//
// @param [in] m_disk UUID for the harddisk
// @param [in] m_mntpoint Fully justified path to directory where block device will mounted
// @param [in] m_rwflag Bit field specifying read/write access
// @param [in] m_data Flags that can be set on a per-filesystem basis.  Set to NULL if none.
// @return -1 on failure, 0 on success
static int diskpool_mount_disk(char *m_uuid, char *m_mntpoint, char *m_sys, uint64_t m_rwflag, void *m_data)
{
	char    *m_disk = NULL;

	if (m_uuid == NULL) {
		blast_err("Attempted to pass a NULL UUID");
		return -1;
	}

	blast_tmp_sprintf(m_disk, "/dev/disk/by-uuid/%s", m_uuid);

	if (diskpool_make_mount_point(m_mntpoint) == -1) {
		blast_fatal("Could not create new directory");
		return -1;
	}

	errno = 0;
	blast_info("Mounting uuid %s at %s with filesystem type %s",
				m_disk, m_mntpoint, m_sys);
	if (mount(m_disk, m_mntpoint, DEFAULT_SYS_TYPE, m_rwflag, m_data) == -1) {
		blast_dbg("Could not mount %s at %s with filesystem type %s: %s",
					m_disk, m_mntpoint, DEFAULT_SYS_TYPE, strerror(errno));
				/**
				 * If the device is not found, or the
				 * superblock is invalid, try the next filesystem in #type
				 */
	} else {
        blast_info("Successfully mounted uuid %s at %s",
                   m_disk, m_mntpoint);
	}

	if (errno != 0) {
			blast_err("Could not mount %s at %s, error:%d",
					m_disk, m_mntpoint, errno);
			return -1;
	}

	return 0;
}

// Handles mounting an index number from the diskpool.  Updates last access time on success.  On failure,
// updates
// @param m_disknum
// @return true on success, false on failure
static bool diskpool_mount_disk_index(int m_disknum)
{
	bool retval = false;

	errno = 0;
	if(diskpool_mount_disk(s_diskpool.disk[m_disknum].uuid,
	                       s_diskpool.disk[m_disknum].mnt_point,
	                       DEFAULT_SYS_TYPE, 0, NULL) != -1) {
		diskpool_update_last_time(m_disknum, time(NULL));
		retval = true;
		s_diskpool.disk[m_disknum].mounted = true;
		if (!s_diskpool.disk[m_disknum].initialized) s_diskpool.disk[m_disknum].initialized = true;
		blast_info("Mounted disk %i at %s. ", m_disknum, s_diskpool.disk[m_disknum].mnt_point);
		s_diskpool.disk[m_disknum].free_space = diskmanager_freespace(s_diskpool.disk[m_disknum].mnt_point);
	} else {
		/**
		 * If we can't mount the disk, we want to emit an error and not try to mount the disk again
		 * for some time.  We achieve this by incrementing the local fail_count.  This will de-favor
		 * the disk during the current mount cycle but will be reset once the disk responds to pings
		 */
		blast_strerror("Couldn't mount %s, ind = %i. ", s_diskpool.disk[m_disknum].mnt_point, m_disknum);
		s_diskpool.disk[m_disknum].skip = true;
		blast_info("Setting skip = true for index %i", m_disknum);
		diskpool_update_last_time(m_disknum, time(NULL));
		s_diskpool.disk[m_disknum].fail_count++;
	}
	return retval;
}

// Try to find the best available USB disk.
static int diskpool_find_next_good_usb_disk()
{
	size_t i = 0;
	blast_info("Starting diskpool_find_next_good_usb_disk.");
    for (i = 1; i < DISK_MAX_NUMBER; i++) {
        if ((s_diskpool.disk[i].isUSB)
            && ((s_diskpool.disk[i].free_space > DISK_MIN_FREE_SPACE) || !(s_diskpool.disk[i].initialized))
            && !(s_diskpool.disk[i].skip)) {
            blast_info("Next best (not full) disk is %i at %s.", (int) i, s_diskpool.disk[i].mnt_point);

            // This is the disk we would like.
            return i;
        } else {
            blast_info("Rejecting disk index %i, isUSB = %i, initialized =%i, free_space = %i, skip = %i",
                       (int) i,
                       s_diskpool.disk[i].isUSB, s_diskpool.disk[i].initialized,
                       s_diskpool.disk[i].free_space, s_diskpool.disk[i].skip);
        }
    }
	return -1;
}
/**
 * Update the freespace counter for a mounted disk in both the structure and flash
 *
 * @param m_disk Pointer to the disk.  Passed as (void *) to allow threading
 *
 */

static void *diskpool_update_mounted_free_space(void *m_disk)
{
	diskentry_t *disk			= NULL;
	int64_t 	new_freespace 	= 0;

	if (m_disk == NULL) {
		blast_err("Passed NULL pointer");
		return NULL;
	}

	disk = (diskentry_t *)m_disk;
	new_freespace = disk->free_space;

	if (disk->mnt_point && disk->mounted) {
		new_freespace = diskmanager_freespace(disk->mnt_point);
	} else {
		blast_err("Disk not mounted");
		return NULL;
	}

	/**
	 * If df_check failed for some reason, we don't know if it is a disk error or network error and we don't
	 * really care as this routine exists to facilitate choosing the next disk to mount and when.  Other routines
	 * deal with read/write errors and will unmount the disk.  We are careful to keep the diskpool free_space variable
	 * valid as -1 may cause flip-flops when we next update the diskpool from flash.
	 */
	if (new_freespace == -1) {
		blast_err("Could not get free space on %s", disk->mnt_point);
		return NULL;
	}

	/* Don't send flash for the local disk or for AoE disks whose size has not changed by at least 1MB */
	if (disk->isUSB && (new_freespace != disk->free_space)) {
		disk->free_space = new_freespace;
		// blast_dbg("New freespace %ld MB on %s", (int64_t)disk->free_space, disk->mnt_point);
	}

	return NULL;
}

// Try to find any USB disk with space (can have skip == 1).
static int diskpool_find_any_usb_disk()
{
	size_t i = 0;
    blast_warn("This does nothing right now.");
    for (i = 1; i < DISK_MAX_NUMBER; i++) {
        if ((s_diskpool.disk[i].isUSB)
            && (s_diskpool.disk[i].free_space > DISK_MIN_FREE_SPACE)) {
            blast_info("Next best (not full) disk is %i at %s.", (int) i, s_diskpool.disk[i].mnt_point);
            return i;
        }
    }
	return -1;
}

// Mounts the next available disk for writing.  N.B. This function does not lock the pool mutex;
// this must be done by the calling function.
//
// @details We try to mount a new disk three times, in order of increasing desperation.
//  - First pass is diskpool_find_best_aoe_disk.  This has all constraints turned on.  First disk to
//  	pass all constraints is assigned.
//  - Second pass is diskpool_find_any_aoe_disk.  This lifts some constraints and uses fail-ordering
//  - Third pass is to use the local disk.
//  While this scheme is redundant in scanning the pool, it will succeed very quickly in all normal cases.
//  The speed of checking the normal cases means that, should it failover, the time to the second case is
//  extremely small.
// @return -1 on failure, positive value indicating pool index on success
static int diskpool_mount_new()
{
	int32_t best_disk = 0;
	int32_t count = 0;
	bool disk_mounted = false;

	blast_info("First try to mount one of our USB disks that has skip = 0");
	while (!disk_mounted && (count++ < DISK_MAX_NUMBER)) {
		best_disk = diskpool_find_next_good_usb_disk();
		if (best_disk == -1) {
			blast_info("No USB disks found!");
			break;
		}
		disk_mounted = diskpool_mount_disk_index(best_disk);
	}
	if (disk_mounted) {
	    blast_info("We have successfully mounted disk %i", best_disk);
    } else {
	    blast_info("2nd try: we will accept any USB disk (even if the skip keyword is set).");
    }
    count = 0;
	// 2nd try: we will accept any USB disk (even if the skip keyword is set)
	while (!disk_mounted && (count++ < DISK_MAX_NUMBER)) {
		best_disk = diskpool_find_any_usb_disk();
		if (best_disk == -1) {
			blast_info("No USB disks found!");
			break;
		}
		disk_mounted = diskpool_mount_disk_index(best_disk);
	}

	if (!disk_mounted) {
		// Our final fall-back is the local disk.  First check to see if there is enough space.  If not,
		// return a fatal error.
		blast_info("No USB mounted disks are available.  Using local disk");
		if (s_diskpool.disk[0].free_space > DISK_MIN_FREE_SPACE) {
			disk_mounted = true;
			best_disk = 0;
		} else {
			blast_fatal("The local disk only has %ldMB of space.  Our minimum is %dMB.  Quitting.",
					(int64_t) s_diskpool.disk[0].free_space, (int) DISK_MIN_FREE_SPACE);
			/// We force the exit here, even if the user has disabled exits.  No diskspace overrides all
			exit(1);
		}
	}

	if (disk_mounted) {
		blast_info("Disk %d mounted at %s", (int)best_disk,
				s_diskpool.disk[best_disk].mnt_point?s_diskpool.disk[best_disk].mnt_point:"unknown");
		diskpool_update_last_time(best_disk, time(NULL));
	}
	return best_disk;
}

// Unmounts the specified disk after updating the free space information and passing it to flash.
// This function should only be called in a thread as it will block while files are open on the
// disk.  Do not depend on this thread returning in anything close to a reasonable amount of
// time, so remember to detach it.
//
// @param [in,out] m_disk Disk to unmount
static void* diskpool_unmount(void *m_disk)
{
	diskentry_t *disk = (diskentry_t *) m_disk;

	if (!disk || !disk->isUSB || !disk->mnt_point) {
		blast_err("Tried to unmount invalid disk");
	} else {
		blast_info("Unmounting %s", disk->mnt_point);
		if (s_filepool.error_disk != disk) {
			diskpool_update_mounted_free_space(m_disk);
		}

		errno = 0;
		while ((umount2(disk->mnt_point, 0) == -1) && (errno == EBUSY)) {
			sleep(10);
		}
		rmdir(disk->mnt_point);

		BLAST_SAFE_FREE(disk->mnt_point);
		blast_info("Finished Unmounting %s", disk->mnt_point);
	}
	return NULL;
}

// Mounts a new disk and places it in the primary pointer.  This routine cannot fail or we
// will have no disks with which to write data.  The failsafe is in diskpool_mount_new() where
// if no USB disks are available it simply chooses the local disk.  This should always work.
static void diskpool_mount_primary()
{
	int mountpoint = -1;
	int i = 0;

	blast_info("Attempting to mount primary disk.");

	/** We require this function to not fail, so we will wait for the write lock */
	while (!diskpool_writelock()) pthread_yield();

	while (mountpoint == -1) {
		mountpoint = diskpool_mount_new();
        blast_info("diskpool_mount_new returned %i, i = %i", mountpoint, i);
		if (i++ > DISK_MAX_NUMBER) {
			blast_fatal("Could not mount primary disk");
			return;
		}
	}
	s_diskpool.current_disk = &(s_diskpool.disk[mountpoint]);
	blast_info("Current disk is now %i.  Unlocking the diskpool and returning.", mountpoint);
	diskpool_unlock();
}

// Move current disk to the previous slot, mount a new current disk and unmount the previous.
// The unmount proceeds lazily, so the kernel doesn't process until all files are closed.
static void diskpool_swap_disks() {
	pthread_t 			unmount_thread;
	diskentry_t			*previous_disk;

	previous_disk = s_diskpool.current_disk;
	blast_info("Calling diskpool_mount_primary.");
	diskpool_mount_primary();

	if (previous_disk->isUSB) {
		if (pthread_create(&unmount_thread, NULL, diskpool_unmount, previous_disk) != 0) {
			blast_err("Could not create unmount thread");
			return;
		}
		pthread_detach(unmount_thread);
	}
}

// Verifies that the primary disk is mounted and has enough free space.
// @return 0 on verified disk, -1 if disk error is being handled
static int diskpool_verify_primary_disk()
{
	int retval = -1;

	while (s_diskpool.current_disk == NULL) {
	    blast_info("Calling diskpool_mount_primary.");
		diskpool_mount_primary();
	}
	// blast_info("We now have a current disk!");
	if (s_filepool.error_disk != s_diskpool.current_disk) {
		retval = 0;
		if (s_diskpool.current_disk->free_space < DISK_MIN_FREE_SPACE) {
		    blast_info("Current disk has only %i bytes of free space.  Triggering a disk swap.",
		               s_diskpool.current_disk->free_space);
			diskpool_swap_disks();
		}
        // TODO(laura): Add something that checks that the disk is alive and can be written too.
	}
	// blast_info("Exiting diskpool_verify_primary_disk.");
	return retval;
}

// Add initial disk info for the USB drives.
int drivepool_add_init_usb_info(int m_pos, int m_hdusb)
{
    diskentry_t disk;
	disk.uuid = bstrdup(err, hd_info[m_hdusb].uuid);
	disk.fail_count = 0;
	disk.mnt_point = bstrdup(err, hd_info[m_hdusb].mnt_point);
    disk.free_space = 0;
    disk.mounted = 0;
    disk.skip = 0;
    disk.initialized = 0;
	disk.isUSB = true;
    disk.last_accessed = time(NULL);
	if (diskpool_add_to_pool(&disk) == -1) {
        blast_err("Could not add disk index %i (%s) to the diskpool.", m_pos, disk.mnt_point);
        return -1;
	} else {
	    blast_info("Added disk index %i (%s) to the diskpool.", m_pos, disk.mnt_point);
	}
	return 0;
}

// Initializes the UUID and mount point information for each of the known USB disks.
// Should only be called the first time diskpool_update_all is called.
// Returns the number of disk for which the UUID and mnt points were set.
static void drivepool_init_usb_info()
{
    int i = 0;
    int j = 0;
    for (i = 0; i < NUM_USB_DISKS; i++) {
        j = i + 1; // drive index in s_diskpool structure
        if (drivepool_add_init_usb_info(j, i) == -1) {
            blast_info("Failed to add disk num %i", i);
        }
    }
}

// Updates the entire #s_diskpool.
//
// @details This routine should provide the shared diskpool structure with the most
// up-to-date information about the disks available to the system.  We enclose the routine in
// a writelock as we don't want the pool to change while we are scanning it.
void diskpool_update_all()
{
    static int first_time = 1;
	// This routine is called periodically, so if we can't acquire the writelock, don't bother
	// doing anything in this loop, simply wait for the next call.  Of course, if we are shutting
	// down, we should exit also.
	if (s_diskmanager_exit || !diskpool_writelock())
		return;

    blast_info("Calling diskpool_update_local.");
	diskpool_update_local();

    if (first_time) {
        blast_info("Initializing diskpool USB information.");
        drivepool_init_usb_info();
    }
    blast_info("drivepool_init_usb_info returned. Next we call diskpool_find_next_good_usb_disk.");
	// Find the first disk that is available to write.
    if (diskpool_find_next_good_usb_disk() == -1) {
        blast_warn("Could not find a USB disk.  Using local-only mode");
    } else {
         blast_info("diskpool_find_next_good_usb_disk returned without error.  Next we try to unlock.");
    }
    diskpool_unlock();
}
// This function provides a wrapper for the failure mutex lock and signal logic.  It called whenever a disk
// failure is detected.  The write_err_mutex prevents the disk failure from being raised multiple times on the
// same disk
// @param [in] m_disk Specifies the pointer to the disk where an error was detected
static void diskpool_raise_error(volatile diskentry_t *m_disk)
{
	if (m_disk == NULL) {
		blast_err("Received a NULL pointer to disk");
	} else {
		if (pthread_mutex_trylock(&s_filepool.write_error_mutex) == 0) {
			blast_dbg("Could not raise error on disk %i, currently handling disk %i (%s)",
					m_disk->index, s_filepool.error_disk->index, s_filepool.error_disk->mnt_point);
		} else {
			/// If the error disk has not already been set, raise the signal
			if (!s_filepool.error_disk) {
				blast_warn("Sucessfully raising error on disk %i (%s)",
						m_disk->index, m_disk->mnt_point);
				s_filepool.error_disk = (diskentry_t*) m_disk;
				__sync_synchronize();
				pthread_cond_signal(&(s_filepool.write_error_signal));
			}

			pthread_mutex_unlock(&(s_filepool.write_error_mutex));
		}
	}
}

// Verifies that all disks with open files are responding.
// @return 0 on verified all disks, -1 on error being handled
static int diskpool_verify_disks_in_use()
{
	int 	i = 0;
	int		retval = 0;
//   blast_info("Starting diskpool_verify_disks_in_use");

	for (i = 0; i < DISK_MAX_FILES; i++) {
		/**
		 * Do not ping NULL disks, disks we have already seen or disks already being handled or
		 * disks belonging to files that are closed
		 */
		if ((s_filepool.file[i].disk == NULL)
				|| (s_filepool.file[i].fp == NULL)
				|| (s_filepool.file[i].disk == s_filepool.error_disk)) {
//            	blast_info("We don't ping NULL files.");
			continue;
        }
		/* Local disks don't get verified, so we simply continue */
		if (!s_filepool.file[i].disk->isUSB) {
            blast_info("Local disk, so we won't verify.");
			continue;
		}

		if (usb_is_disk_alive()) {
			// blast_info("The disk is alive.");
		} else {
			blast_info("Raising error on disk %i", s_diskpool.current_disk->index);
			diskpool_raise_error(s_diskpool.current_disk);
			retval = -1;
		}
	}

	return retval;
}

// Flush all file buffers currently open in #s_filepool that have any data.
// @details  The filebuffer_writeout function, which is called below, executes a system I/O
// command (fwrite).  If the AoE device is not responding, this can be extremely expensive
// as fwrite is atomic and blocking.  Comparatively, our AoE ping is cheap, so we execute it
// before every write.
void filepool_flush_buffers()
{
	int i = 0;

	/* s_ready is an initialization flag for diskmanager */
	while (!s_ready) {
		if (s_diskmanager_exit) return;
		sched_yield();
	}

	/* This should catch any network errors on disks before we try an expensive I/O write. */
	while (diskpool_verify_disks_in_use() != 0) {
		if (s_diskmanager_exit) return;
		sched_yield();
	}

	for (i = 0; i < DISK_MAX_FILES; i++) {
		if ((s_filepool.file[i].disk == NULL)
				|| (s_filepool.file[i].fp == NULL)
				|| (filebuffer_len(&(s_filepool.file[i].buffer)) <= 0)) {
			continue;
		}

		// After locking the mutex, we check again as we may have been delayed.  Locking the mutex is
		// a more expensive operation than checking NULL conditions
		pthread_rwlock_rdlock(&s_filepool.file[i].mutex);
		if ((filebuffer_len(&(s_filepool.file[i].buffer)) <= 0)
						|| (s_filepool.file[i].disk == NULL)
						|| (s_filepool.file[i].fp == NULL)) {
			pthread_rwlock_unlock(&s_filepool.file[i].mutex);
			continue;
		}

		if (filebuffer_writeout(&s_filepool.file[i].buffer, s_filepool.file[i].fp) == -1) {
			/**
			 * Verify that our file has not been closed under our feet
			 */
			if (s_filepool.file[i].disk) {
				blast_info("Raising disk error on");
//           	blast_info("Raising disk error on %s",
//      			(unsigned)s_filepool.file[i].disk->major, (unsigned)s_filepool.file[i].disk->minor);
          		diskpool_raise_error(s_filepool.file[i].disk);
			}
			break;
		}
		pthread_rwlock_unlock(&s_filepool.file[i].mutex);
	}
}

// Provide a method for cleanly shutting down and syncing disks with unwritten data.
//
void diskmanager_shutdown()
{
	s_diskmanager_exit = true;
	filepool_flush_buffers();
	fcloseall();
	s_ready = false;
}

// Closes and re-opens a file that was interrupted by disk error.  File will be re-opened on
// the primary disk.
// @param [in] m_fileid Index of the file to be transferred
// @return -1 on error, 0 on success
static int file_change_disk(int m_fileid, diskentry_t *m_disk)
{
	int retval = -1;
	struct timespec timeout;

	if ((m_fileid >= 0)
			&& (m_fileid < DISK_MAX_FILES)
			&& (m_disk != NULL)
			&& (m_disk->mounted)) {
		set_timeout(&timeout, 0, diskman_timeout);
		if(pthread_rwlock_timedwrlock(&(s_filepool.file[m_fileid].mutex), &timeout) != 0) {
			blast_err("Could not acquire mutex.  File %d not moved", m_fileid);
			return -1;
		}
		// We place the file pointer NULL check inside the mutex to make sure that we have
		// sole access to the file and it hasn't been closed by an errant thread.
		if (s_filepool.file[m_fileid].fp != NULL) {
			file_close_internal(m_fileid, true);
			retval = file_reopen_on_new_disk(m_fileid, m_disk);
		}
		pthread_rwlock_unlock(&(s_filepool.file[m_fileid].mutex));
	}
	return retval;
}

// Open a continuation of a file on a new disk whose writing was interrupted on the previous
// disk by an error.  No mutexes are set as the error handler should be in control of the pool.
//
// @param [in] m_fileid File index number.  The file pointer should already be closed.
// @param [in] m_disk Pointer to the new disk on which the file should be opened.
// @return -1 on error, 0 on success
static int file_reopen_on_new_disk(int m_fileid, diskentry_t *m_disk)
{
	FILE 	*fp 			= NULL;
	int 	retval 			= 0;
	char 	*filename		= NULL;
	char	*full_filename 	= NULL;

	if (s_diskmanager_exit)  {
		blast_tfatal("MCP shutting down.  Killing thread");
		return -1;
	}

	if ((m_fileid < 0) || (m_fileid >= DISK_MAX_FILES)) {
		blast_err("Passed invalid file id %d", m_fileid);
		return -1;
	}

	if (s_diskpool.current_disk == NULL) {
		blast_err("No current disk");
		return -1;
	}

	if (asprintf(&filename, "%s.cont", s_filepool.file[m_fileid].filename) <= 0) {
		blast_err("Could not allocate filename memory");
		return -1;
	}

	// We need the directory, so in case of failure, just yield processing to allow for error
	// recovery.  diskpool_mount_primary() will take care of system failure to allow rebooting.
	while (diskpool_mkdir_file(filename, true) != 0) {
		sched_yield();
	}

	blast_tmp_sprintf(full_filename, "%s/%s", s_diskpool.current_disk->mnt_point, filename);
	// Re-opening on a new disk means that we were writing when we ran out of space.  Thus we
	// should open the new file for write access as well.
	fp = fopen(full_filename, "w");

	if (fp == NULL) {
		// In case of an error opening the new file, we still want to update the associated
		// #fileentry_s data. Therefore, just set #retval and continue.
		// This will allow us to (hopefully) recover from the error by trying the next
		// disk when we write more data.
		// @TODO: figure out a way to gracefully die after an open attempt fails.  Perhaps we want
		// to setup a temporary file on the local disk that is copied later?
		blast_strerror("Could not open file %s", filename);
		retval = -1;
	}

	s_filepool.file[m_fileid].fp = fp;
	s_filepool.file[m_fileid].continued = true;
	free(s_filepool.file[m_fileid].filename);
	s_filepool.file[m_fileid].filename = filename;
	s_filepool.file[m_fileid].disk = m_disk;

	return retval;
}

/**
 * Deal with a disk that has given an error.
 *
 * @details Logic is this:  If the disk with an error is the primary disk, then mount a new one
 * and make it the primary.  Next, transfer all files open on the failed disk to the
 * primary disk, whether we mounted a new one or not.
 *
 * @param m_disk Pointer to the #diskentry_t that failed
 */

static void filepool_handle_disk_error(volatile diskentry_t *m_disk)
{
	int i = 0;

	blast_dbg("Beginning error handling");

	if (m_disk == s_diskpool.current_disk) {
		diskpool_mount_primary();
	}

	for (i = 0; i < DISK_MAX_FILES; i++) {
		if ((s_filepool.file[i].disk == m_disk)
				&& (file_change_disk(i, s_diskpool.current_disk) == -1)) {
			blast_err("Could not re-open %s on %s",
				      s_filepool.file[i].filename?s_filepool.file[i].filename:"unk",
			          s_diskpool.current_disk->mnt_point?s_diskpool.current_disk->mnt_point:"unk");
		}
	}
}

// Error handler thread for disk manager.
void *diskmanager_error_handle_thread(__attribute__((unused))void *m_arg)
{
	pthread_t 			unmount_thread;

	pthread_mutex_lock(&(s_filepool.write_error_mutex));

	while(!s_diskmanager_exit) {
		pthread_cond_wait(&(s_filepool.write_error_signal), &(s_filepool.write_error_mutex));

		/**
		 * The two cases where we don't want to do anything are:
		 * - If we are shutting down, then exit
		 * - If we received a spurious wakeup (they happen), then error_disk will not be set and we
		 * 		should just go back to sleep
		 */
		if (s_diskmanager_exit) break;
		if (s_filepool.error_disk == NULL) continue;

		blast_err("Caught disk error, attempting to handle.");
		filepool_handle_disk_error(s_filepool.error_disk);
		if (s_filepool.error_disk->isUSB) {
			pthread_create(&unmount_thread, NULL, diskpool_unmount, (void *) s_filepool.error_disk);
			pthread_detach(unmount_thread);
		}

		s_filepool.error_disk = NULL;
	}

	pthread_mutex_unlock(&s_filepool.write_error_mutex);
	blast_info("Exiting on shutdown");
	return NULL;
}

static void *diskpool_update_mounted_free_space_thread(void *m_arg __attribute__((unused)))
{
    blast_info("Starting thread to monitor harddrive free space.");
	do {
		if (!s_diskmanager_update_freespace_flag) continue;

		diskpool_update_mounted_free_space(s_diskpool.current_disk);
		s_diskmanager_update_freespace_flag = false;
	} while (!usleep(50000) && !s_diskmanager_exit);

	return NULL;
}
// Opens a new file on the current disk
//
// This function assumes the correct form of mnt_point (no trailing '/') and
// m_filename (starts with an alpha-numeric, no '/')
//
// @param [in] m_filename Name of the file to open, justified path from mount base
// @param [in] m_mode Access mode for the file to be opened (follows fopen(2) format)
// @return -1 on failure, positive value indicating new file index in pool on success
int file_open(const char *m_filename, const char *m_mode)
{
	FILE 	*fp = NULL;
	int 	i = 0;
	int 	retval = -1;
	char 	*filename;

	if (s_diskmanager_exit) {
		blast_tfatal("Diskmanager is exiting.  Killing thread");
		return -1;
	}

	/* Treat fully justified paths as local files */
	if (file_is_local(m_filename)) {
		return file_open_local(m_filename, m_mode);
	}

        /* s_ready is an initialization flag for diskmanager */
	while (!s_ready) pthread_yield();

	// Continuously try to make the directory, yielding the processor each time.  Failure will
	// trigger the disk error recovery process, so we give it time to work.
	while(diskpool_mkdir_file(m_filename, true) != 0) {
		pthread_yield();
	}

	blast_tmp_sprintf(filename, "%s/%s", s_diskpool.current_disk->mnt_point, m_filename);
	errno = 0;
	fp = fopen(filename, m_mode);

	if (fp == NULL) {
		blast_strerror("Could not open file %s", filename);
		blast_info("Raising disk error on disk %u (%s)",
		        (unsigned)s_diskpool.current_disk->index, s_diskpool.current_disk->mnt_point);
		diskpool_raise_error(s_diskpool.current_disk);

		return -1;
	}

	// We have successfully opened the file.  Now a number of things need to happen.
	// - Set the stream buffer to 0 as we do internal buffering (in addition to AoE network buffering)
	// - Create a new #s_fileentry in the #s_filepool
	// - Populate the #s_fileentry data with a s_diskentry, file descriptor and name
	// - Allocate and set a write buffer
	setvbuf(fp, NULL, _IONBF, 0);

	for (i = 0; i < DISK_MAX_FILES; i++) {
		if ((s_filepool.file[i].fp == NULL)
				&& (pthread_rwlock_trywrlock(&s_filepool.file[i].mutex) == 0)) {
			break;
		}
	}

	if (i == DISK_MAX_FILES) {
		blast_err("Could not open %s, Too many files", filename);
		fclose(fp);
		return -1;
	}

	retval = i;

	s_filepool.file[i].fp = fp;
	s_filepool.file[i].parent = pthread_self();
	s_filepool.file[i].continued = false;
	s_filepool.file[i].filename = strdup(m_filename);
	s_filepool.file[i].disk = s_diskpool.current_disk;


	/* @TODO: implement variable buffer size based on anticipated write activity */
	if (filebuffer_init(&(s_filepool.file[i].buffer), (unsigned int) FILE_BUFFER_SIZE) == -1) {
		blast_err("Could not allocate buffer space for %s", filename);
		free(s_filepool.file[i].filename);
		s_filepool.file[i].filename = NULL;
		fclose(fp);
		s_filepool.file[i].fp = NULL;
		retval = -1;
	}

	pthread_rwlock_unlock(&s_filepool.file[i].mutex);
	return retval;
}

// Opens a new file on the local disk
//
// This function assumes the correct form of m_filename
// -- fully justified (including all paths)
//
// @param [in] m_filename Name of the file to open, fully justified path
// @param [in] m_mode Access mode for the file to be opened (follows fopen(2) format)
// @return -1 on failure, positive value indicating new file index in pool on success
static int file_open_local(const char *m_filename, const char *m_mode)
{
	FILE *fp = NULL;
	int i = 0;
	int retval = -1;

	/* We ignore the return value here.  If diskpool_mkdir fails, then
	 * fopen will fail as well, providing the appropriate error message.
	 */
	diskpool_mkdir_file(m_filename, true);

	errno = 0;
	fp = fopen(m_filename, m_mode);

	if (fp == NULL) {
		blast_err("Could not open file %s, error %d", m_filename, errno);
		return -1;
	}

	/*
	 * We have successfully opened the file.  Now a number of things need to happen.
	 * - Create a new #s_fileentry in the #s_filepool
	 * - Populate the #s_fileentry data with a s_diskentry, file descriptor and name
	 * - Allocate and set a write buffer
	 */

	setvbuf(fp, NULL, _IONBF, 0);

	for (i = 0; i < DISK_MAX_FILES; i++) {
		if ((s_filepool.file[i].fp == NULL)
				&& (pthread_rwlock_tryrdlock(&s_filepool.file[i].mutex) == 0)) {
			break;
		}
	}

	if (i == DISK_MAX_FILES) {
		blast_err("Could not open %s, Too many files", m_filename);
		fclose(fp);
		return -1;
	}

	retval = i;

	s_filepool.file[i].fp = fp;
	s_filepool.file[i].parent = pthread_self();
	s_filepool.file[i].continued = false;
	s_filepool.file[i].filename = strdup(m_filename);

	s_filepool.file[i].disk = &(s_diskpool.disk[0]);

	/* @TODO: implement variable buffer size based on anticipated write activity */
	if (filebuffer_init(&(s_filepool.file[i].buffer), (unsigned int) FILE_BUFFER_SIZE) == -1) {
		blast_err("Could not allocate buffer space for %s", m_filename);
		fclose(fp);
		s_filepool.file[i].fp = NULL;
		free(s_filepool.file[i].filename);
		s_filepool.file[i].filename = NULL;
		retval = -1;
	}

	pthread_rwlock_unlock(&s_filepool.file[i].mutex);
	return retval;
}

// Closes a file in the #s_filepool
//
// This function does not lock the file mutex.  This is purposeful as you should control a file for
// writing before attempting to close it.  In other words
// don't call this function unless you have locked the mutex yourself.
//
// @param [in] m_fileid Unique identification number to find file in the pool
// @param [in] m_continuing Boolean to determine whether this file is being transferred to a new disk
static void file_close_internal(int m_fileid, bool m_continuing)
{
	if (s_diskmanager_exit)
	    return;

	if ((m_fileid < 0) || (m_fileid >= DISK_MAX_FILES)) {
		blast_err("Passed invalid file id %d", m_fileid);
		return;
	}

	/**
	 *  A file that needs to be continued on a new disk indicates that
		write error, so don't try to flush the buffers.  This also prevents
		a loop where the file write fails on close.
	*/

	if (!m_continuing) {
		filebuffer_writeout(&s_filepool.file[m_fileid].buffer, s_filepool.file[m_fileid].fp);
	}

	if (fclose(s_filepool.file[m_fileid].fp) == -1) {
		blast_err("Error closing file. Error: %d", errno);
	}

	s_filepool.file[m_fileid].fp = NULL;
	s_filepool.file[m_fileid].disk = NULL;

	if (!m_continuing) {
		if (s_filepool.file[m_fileid].filename) free(s_filepool.file[m_fileid].filename);
		s_filepool.file[m_fileid].filename = NULL;
		filebuffer_free(&s_filepool.file[m_fileid].buffer);
	}
}

// External interface to the file_close_internal procedure.  Locks the
// file mutex and closes the file.  If we cannot acquire the mutex, return -1, otherwise
// return 0.  On -1 return, the file remains open.
int file_close(int m_fd)
{
	struct timespec timeout;
	set_timeout(&timeout, 2, 0);
	if ((m_fd >= 0) && (m_fd <DISK_MAX_FILES)) {
		if (pthread_rwlock_timedwrlock(&s_filepool.file[m_fd].mutex, &timeout) != 0) {
			blast_strerror("Could not acquire write mutex for file id %d", m_fd);
			return -1;
		}
		if(s_filepool.file[m_fd].fp != NULL) {
			file_close_internal(m_fd, false);
		}
		pthread_rwlock_unlock(&s_filepool.file[m_fd].mutex);
	}
	return 0;
}

// Returns the position in the open file specified by #m_fileid
// @param [in] m_fileid Index of the file
// @return integer position of the current point in the file, -1 on failure
int64_t file_tell(int m_fileid)
{
	struct timespec timeout;
	int64_t position = -1;

	if (s_diskmanager_exit)
		return -1;

	while (!s_ready)
		pthread_yield();

	if ((m_fileid < 0) || (m_fileid >= DISK_MAX_FILES)) {
		blast_err("Passed invalid file id %d", m_fileid);
	} else {
		set_timeout(&timeout, 0, diskman_timeout);
		if (pthread_rwlock_timedrdlock(&s_filepool.file[m_fileid].mutex, &timeout) != 0) {
			blast_err("Could not acquire mutex for file id %d", m_fileid);
		} else {
			if (s_filepool.file[m_fileid].fp != NULL) {
				position = ftell(s_filepool.file[m_fileid].fp);
			}
			pthread_rwlock_unlock(&s_filepool.file[m_fileid].mutex);
		}
	}
	return position;
}

// Structured data writing wrapper.  Mimics format of fwrite(2) with the
// exception of the file index being the first parameter instead of the
// last.  This is purposeful to ensure we are calling the appropriate
// function
//
// @param [in] m_fileid File index in #s_filepool
// @param [in] m_struct pointer to data structure to write
// @param [in] m_size Size of a single element in structure
// @param [in] m_num Number of elements to write
// @return -1 on failure, positive number of elements written on success
ssize_t file_write_struct(int m_fileid, void *m_struct, size_t m_size, size_t m_num)
{
	size_t length = m_size * m_num;
	ssize_t retval;
	retval = file_write(m_fileid, (const char*)m_struct, length);

	if (retval > 0) retval /= m_size;

	return retval;
}

// Writes data to a file buffer.
//
// @details We want to allow for writing constant strings of uncalculated length
// to a file.  Thus we allow #m_len to be = 0.  This has two effects:
// First, it assumes that we are writing a string
// Second, it assumes that we don't want to write the terminating NULL character
// Thus, use #m_len=0 only when writing constant strings.  If you want the NULL
// character written, you must calculate the length yourself.
//
// @param [in] m_fileid Unique identification number to file in the pool
// @param [in] m_data Data to be written
// @param [in] m_len Number of characters to be written. If 0, m_data is passed
// 					to strlen() to determine length
// @return -1 on failure, positive number of bytes on success
ssize_t file_write(int m_fileid, const char *m_data, size_t m_len)
{
	ssize_t 	retval = 0;

	if (s_diskmanager_exit)
		return -1;

	while (!s_ready)
		pthread_yield();

	if (m_data == NULL) {
		blast_err("Tried to write NULL string to file");
		return -1;
	}

	if ((m_fileid < 0) || (m_fileid >= DISK_MAX_FILES)) {
		blast_err("Passed invalid file id %d", m_fileid);
		return -1;
	}

	// Allow m_len to be undefined as sometimes we might wish to write a string
	// whose length we can't or don't want to calculate.  This only works for
	// NULL-terminated strings.
	if (m_len == 0) {
		m_len = strlen(m_data);
	}
	/* Check again.  If there are no characters, don't write out. */
	if (m_len == 0) {
		return 0;
	}

	retval = filebuffer_append(&(s_filepool.file[m_fileid].buffer), m_data, m_len);

	while (retval == -1) {
		// If filebuffer_append is unable to allocate more memory to the cache, we
		// need to write data immediately, without waiting for the thread scheduling.
		// Thus, we lock the buffer_mutex against other writes and place the disk writing
		// in this thread.
		if (pthread_mutex_trylock(&(s_filepool.buffer_mutex)) == 0) {
			filepool_flush_buffers();
			pthread_mutex_unlock(&(s_filepool.buffer_mutex));
			retval = filebuffer_append(&(s_filepool.file[m_fileid].buffer), m_data, m_len);
		} else {
			/**
			 * We cannot append data to the buffer and cannot flush it to disk.
			 * Without data writing, we need to quit and reboot (using watchdog)
			 * Needless to say, this should never happen.
			 */
			blast_fatal("Could not reallocate memory for expanding the disk buffer");
			return -1;
		}
	}

	// If we have enough data in any buffer to justify a write and we can acquire the signal
	// mutex, then send a signal to the disk writer to flush.
	// @warning { The order of evaluation is important here.  pthread_mutex_trylock will not
	// be executed unless the first condition evaluates to true.  Do not change the statement
	// order!}
	//
	// If #s_filepool.buffer_mutex is already locked, that indicates that a write signal has
	// already been sent and the file writer is busy outputting data to disk.  The signal will
	// not be sent until the file writer unlocks the mutex indicating it is ready to write again.
	//
	// We don't need to lock the #mutex_filepool for writing here as we have #buffer_mutex
	if ((filebuffer_fraction_full(&(s_filepool.file[m_fileid].buffer)) > BUFFER_WRITE_PERCENTAGE)
			&& (pthread_mutex_trylock(&(s_filepool.buffer_mutex)) == 0)) {
		pthread_cond_signal(&(s_filepool.buffer_signal));
		pthread_mutex_unlock(&(s_filepool.buffer_mutex));
	}

	return retval;
}

// Writes formatted output to a file
// @param m_fileid File in pool to which we we want to write
// @param m_fmt printf-style string of const chars and formatting strings
// @param ... Additional paramaters as needed for the formatting string
// @return -1 on failure, 0 on success
int file_printf(int m_fileid, const char* m_fmt, ...)
{
	char *string;
	va_list argptr;

	while (!s_ready) {
		if (s_diskmanager_exit) return -1;
		pthread_yield();
	}

	va_start(argptr, m_fmt);
	blast_tmp_vsprintf(string, m_fmt, argptr);
	va_end(argptr);

	return file_write(m_fileid, string, 0);
}


// Read data from a file on disk
//
// @param [in] m_fileid Unique identification number to file in pool
// @param [out] m_data buffer in which to store output
// @param [in] m_len size in bytes of 1 element
// @param [in] m_num number of elements to read
// @return -1 on failure, positive number of elements read on success
ssize_t file_read(int m_fileid, void *m_data, size_t m_len, size_t m_num)
{
	ssize_t retval = -1;
	struct timespec timeout;

	if (s_diskmanager_exit)
		return -1;

	if ((m_data != NULL) && (m_fileid >= 0) && (m_fileid < DISK_MAX_FILES)) {
		set_timeout(&timeout, 0, diskman_timeout);
		if(pthread_rwlock_timedrdlock(&(s_filepool.file[m_fileid].mutex), &timeout) != 0) {
			blast_err("Could not acquire mutex for %d", m_fileid);
		} else {
			if (s_filepool.file[m_fileid].fp != NULL) {
				retval = (ssize_t) fread(m_data, m_len, m_num, s_filepool.file[m_fileid].fp);
			}
			pthread_rwlock_unlock(&(s_filepool.file[m_fileid].mutex));
		}
	}

	return retval;
}

// Returns the error indicator for the specified file
// @param [in] m_fileid File index in the pool
// @return 0 if no error, -1 if called on NULL descriptor, the standard error indicator otherwise
int file_error(int m_fileid) {
	struct timespec timeout;
	int retval = -1;

	if (s_diskmanager_exit)
		return -1;

	if (m_fileid >= 0 && m_fileid < DISK_MAX_FILES) {
		set_timeout(&timeout, 0, diskman_timeout);
		if (pthread_rwlock_timedrdlock(&s_filepool.file[m_fileid].mutex, &timeout) != 0) {
			blast_err("Could not acquire read mutex on %d", m_fileid);
		} else {
			if (s_filepool.file[m_fileid].fp != NULL) {
				retval = ferror(s_filepool.file[m_fileid].fp);
			}
			pthread_rwlock_unlock(&s_filepool.file[m_fileid].mutex);
		}
	}
	return retval;
}

// TODO(laura): Is this function even used?
// Returns the fully-justified path of a file in the file pool.  N.B. Using this function is BAD.
// Accessing the AoE disks directly using the kernel system may cause your program to hang for
// extended periods of time while the kernel module stares at its navel and tries to determine
// whether a subnet protocol should take minutes to respond.  Use the built-in file access routines.
//
// @param [in] m_fd The index of the file in the pool
// @param [out] m_path Pointer to a char array of minimum size #PATH_MAX
// @return -1 on failure, 0 on success
int file_get_path(int m_fd, char *m_path)
{
	if (s_diskmanager_exit)
		return -1;

	/* s_ready is an initialization flag for diskmanager */
	while (!s_ready)
		pthread_yield();

	if (m_path == NULL
			|| m_fd < 0
			|| m_fd >= DISK_MAX_FILES
			|| s_filepool.file[m_fd].fp == NULL
			|| s_filepool.file[m_fd].disk == NULL
			|| s_filepool.file[m_fd].filename == NULL
			|| s_filepool.file[m_fd].disk->mnt_point == NULL
			|| s_filepool.error_disk == s_filepool.file[m_fd].disk) {
		blast_err("Could not acquire pathname for specified file");
		return -1;
	}

	if (file_is_local(s_filepool.file[m_fd].filename)) {
		if (strlen(s_filepool.file[m_fd].filename) > PATH_MAX) {
			blast_err("Local path name too long");
			return -1;
		}
		strncpy(m_path, s_filepool.file[m_fd].filename, PATH_MAX);
		blast_dbg("Got local path %s", m_path);
		return 0;
	}

	if ((strlen(s_filepool.file[m_fd].disk->mnt_point) + strlen(s_filepool.file[m_fd].filename)) + 2 > PATH_MAX) {
		blast_err("Path name too long");
		return -1;
	}

	strncpy(m_path, s_filepool.file[m_fd].disk->mnt_point, PATH_MAX);
	strncat(m_path, "/", 1);
	strncat(m_path, s_filepool.file[m_fd].filename, PATH_MAX - strlen(m_path));
	blast_dbg("Got relative path %s", m_path);
	return 0;
}

// Main disk manager thread.  Handles writing from the buffer.  Starts the error handler
// thread to deal with disk errors.
void *diskmanager_thread(void *m_arg)
{
	pthread_t error_handler_thread;
	pthread_attr_t error_handler_attribute;
	int i = 0;

	struct timespec	flush_wait;

	/**
	 * We set the disk error handler to have realtime priority as we want to recover from
	 * disk errors without intervening disk writes.  The SCHED_FIFO specifies first-in, first-out
	 * scheduling, so the error handler will not be interrupted.
	 */
	pthread_attr_init(&error_handler_attribute);
	pthread_attr_setdetachstate(&error_handler_attribute, PTHREAD_CREATE_DETACHED);
	pthread_attr_setschedpolicy(&error_handler_attribute, SCHED_FIFO);

// Initialize the harddrive_info structure
    for (i = 0; i < NUM_USB_DISKS; i++) {
        if (SouthIAm) { // fc2
            asprintf(&hd_info[i].uuid, "%s", drive_uuids_fc2[i]);
        } else { // fc1
            asprintf(&hd_info[i].uuid, "%s", drive_uuids_fc1[i]);
        }
        asprintf(&hd_info[i].mnt_point, "%s/%s%d", HOME_DIR, MNT_DIR_PREFIX, i + 1);
    }
	diskmanager_clear_old_mounts();

    if (diskpool_init() == -1 || filepool_init() == -1) {
        blast_fatal("Could not initialize pools");
        pthread_exit(NULL);
    }

    pthread_create(&error_handler_thread, &error_handler_attribute,
        &diskmanager_error_handle_thread, NULL);
    blast_info("Calling diskpool_update_all.");
    diskpool_update_all();
    blast_info("Finished calling diskpool_update_all.");
	blast_info("Calling diskpool_verify_primary_disk.");
    diskpool_verify_primary_disk();
    blast_info("Finished calling diskpool_verify_primary_disk.");
    s_ready = true;

    pthread_create(&diskspace_thread, NULL,
                   &diskpool_update_mounted_free_space_thread, NULL);
    pthread_detach(diskspace_thread);

    pthread_mutex_lock(&(s_filepool.buffer_mutex));
    while (!s_diskmanager_exit) {
        clock_gettime(CLOCK_REALTIME, &flush_wait);
        flush_wait.tv_sec += DISK_BUFFER_MAX_WAIT;

        pthread_cond_timedwait(&(s_filepool.buffer_signal), &(s_filepool.buffer_mutex),
             &flush_wait);
        filepool_flush_buffers();
//
// 		/**
// 		 * Signal the update thread to push new disk information out over the net
// 		 */
        s_diskmanager_update_freespace_flag = true;
    }
    blast_info("Caught signal to exit.  Flushing disks.");
    s_ready = false;


	return NULL;
}


