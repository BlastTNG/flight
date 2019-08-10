/**
 * diskmanager.c
 *
 *  Created on: Mar 29, 2010
 *      Author: Seth Hillbrand
 *
 * This file is part of MCP, the EBEX flight control program
 *
 * This software is copyright (C) 2010 Columbia University
 *
 * MCP is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * MCP is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with MCP; if not, write to the Free Software Foundation, Inc.,
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
#include <stdbool.h>
#include <time.h>
#include <ck_ht.h>

#include "file_buffer_tng.h"
#include "diskmanager_tng.h"
#define HOME_DIR					"/data"
#define MNT_DIR_PREFIX				"mcp_hd"
#define NUM_USB_DISKS               16
#define DISK_MAX_NUMBER             NUM_USB_DISKS+1
#define DISK_MAX_FILES				100	/** Maximum number of concurrently open files */
#define DISK_MIN_FREE_SPACE			50  /** Minimum amount of free space in MB for a disk to be used */
#define DEFAULT_SYS_TYPE            "xfs"

#define FILE_BUFFER_SIZE			2*1024*1024 /*2 MB*/

extern int16_t SouthIAm; // Needed to determine which drives to mount.

/*
 * diskentry structure provides all information needed to utilize both local disks and USB disks in the
 * diskpool
 */
typedef struct diskentry
{
	bool			isUSB;				// isUSB true==USB Disk, false==Local disk
	bool			initialized;		// initialized means that the disk has been mounted and is ready for writing.
	bool			mounted;		    // is the disk mounted?
	bool			skip;		        // Set when a disk is identified as suspect.
	                                    // If set then skip unless there is no other choice.
	char			*dev;				// Device list id.  Should be '/dev/disk/by-uuid/XXXX' format
	char			*mnt_point;			// mnt_point Location where disk is mounted
	uint32_t		fail_count;			// fail_count Incremented each time disk does not respond to
										// update ping.  After MAX_NONRESPONSE fails
										// incrementing stops
	time_t			last_accessed;		// last_accessed Last time the disk was read or written (mounted)
	int32_t	 		free_space;			// free_space Free space on the device in MB;
    int16_t         index;              // disk index number in the diskpool disk array
} diskentry_t;

/*
 *  diskpool structure keeps track of all available volumes and their last status.  This allows rapid
 *  reaction by the disk manager to mount a new volume and continue recording data
 */

typedef struct diskpool
{
	diskentry_t	    *current_disk;	// current_disk Pointer to current primary disk
	diskentry_t	    local_disk;    // Pointer to the local disk
	diskentry_t     disk[NUM_USB_DISKS];
} diskpool_t;


// Hardware IDs for the drives connected by USB
static const char drive_uuids[NUM_USB_DISKS][64] = {
		    "ccbff6e7-8e51-49e4-a987-9ebf5644813e",
        "674e5a19-eb93-4c05-b12c-6a50c03ca5c1",
        "67e991c8-1e1e-4f77-84f1-9273c050e385",
        "22804e9d-a3e1-4cf8-a5b2-ff2fcf22bc5e",
        "94ac1984-a52b-4be6-afb7-cb8302d249e0",
        "993e105e-1cbc-4913-abca-29540242c57e",
        "6846dffc-cf41-447a-a576-4ab34cad7974",
        "a52e5c25-8dbc-4e55-ae73-7c5f8b49968c",
        "526a82ec-cf24-4047-8ae5-a23e761e3704", // sdb1
        "a175b5d4-3aed-4f2a-b801-4d79a558375d", // sdc1
        "01249958-4154-4af0-85df-eeebab5b9cf7", // sdd1
        "5d064d3a-ff6c-46f0-9308-80abb3177e43", // sde1
        "f841003d-53c5-454e-915b-9e477c2f085e", // sdf1
        "548fa9fd-b0c5-46e7-b80a-553d0dd01221", // sdg1
        "1506c53d-d16c-4063-a182-5d167fa968c7", // sdh1
        "f1fd4434-15b2-48aa-bead-8af2394bc1db"}; // sdi1

static int file_change_disk(fileentry_t*, diskentry_t*);
static int file_reopen_on_new_disk(fileentry_t*, diskentry_t*);
static void filepool_handle_disk_error(diskentry_t*);
static int file_open_internal(fileentry_t*);
static fileentry_t *file_open_local(const char*, const char*);
static void file_close_internal(fileentry_t*, bool);
static inline bool file_is_local(const char*);
static int diskpool_make_mount_point(char *m_mntpoint);
static int diskpool_mkdir_file(const char *m_filename, bool m_file_appended);

static diskpool_t s_diskpool = {0};
static ck_ht_t s_filepool;
static pthread_t diskman_thread;

// When set to true, diskmanager will exit on next loop
static volatile bool s_diskmanager_exit = false;

// Initialized to False, procedures dependent on file access wait for true
static volatile bool s_ready = false;

extern int16_t SouthIAm;
uint64_t total_bytes_written[2] = {0, 0};
static const char *total_bytes_log = "/data/etc/MCP_total_bytes";
static const char *tmp_bytes_log = "/data/etc/tmp_total_bytes";

static void *ht_malloc(size_t r) {
    return malloc(r);
}

static void ht_free(void *p, size_t b, bool r) {
    free(p);
}

static struct ck_malloc ALLOCATOR = { .malloc = ht_malloc, .free = ht_free };

// Check whether the disk is initialized (ready).  Returns 1 if so, otherwise 0.
int check_disk_init() {
    if (s_ready) {
        return 1;
    } else {
    	return 0;
    }
}
/**
 * Iterate through the #s_diskpool to find first entry with no access time.  This
 * should correspond to an open entry.  The counter will increment from 0 through
 * DISK_MAX_NUMBER -1
 *
 * @param m_disk Pointer to a populated #diskentry_t structure
 * @return false on failure, true on success
 */
int diskpool_add_to_pool(diskentry_t *m_disk) {
    if (s_diskmanager_exit)
        return false;

    if (!m_disk) {
        blast_err("Tried to add unnamed disk to the pool!");
        return false;
    }
    for (int i = 0; i < NUM_USB_DISKS; i++) {
        if (!s_diskpool.disk[i].dev) {
            memcpy(&(s_diskpool.disk[i]), m_disk, sizeof(diskentry_t));
            return true;
        }
    }
    blast_err("No open slots in disk pool for %s", m_disk->dev);
    return false;
}

// Add initial disk info for the USB drives.
static int diskpool_add_init_usb_info(const char *m_uuid, int m_pos) {
    diskentry_t disk;
    asprintf(&disk.dev, "/dev/disk/by-uuid/%s", m_uuid);
    asprintf(&disk.mnt_point, "%s/%s%d", HOME_DIR, MNT_DIR_PREFIX, m_pos);
    disk.fail_count = 0;
    disk.free_space = -1;
    disk.mounted = 0;
    disk.skip = 0;
    disk.initialized = 0;
    disk.isUSB = true;
    disk.last_accessed = time(NULL);
    disk.index = m_pos;
    if (diskpool_add_to_pool(&disk) == -1) {
        blast_err("Could not add disk index %i (%s) to the diskpool.", m_pos,
                disk.dev);
        return -1;
    } else {
        // blast_info("Added disk index %i (%s) to the diskpool.", m_pos,
        //         disk.dev);
    }
    return 0;
}

/**
 * Initializes the UUID and mount point information for each of the known USB disks.
 * Should only be called the first time diskpool_update_all is called.
 * Returns the number of disk for which the UUID and mnt points were set.
 */
static void drivepool_init_usb_info() {
    int i = 0;
    for (i = 0; i < NUM_USB_DISKS; i++) {
        if (strlen(drive_uuids[i]) > 0) {
            diskpool_add_init_usb_info(drive_uuids[i], i);
        }
    }
}

static void initialize_total_bytes(void) {
    FILE *fp;

    fp = fopen(total_bytes_log, "r");
    if (!fp) {
        fp = fopen(total_bytes_log, "w");
        if (fp)
            fclose(fp);
        return;
    }

    fread(&total_bytes_written[SouthIAm], sizeof(total_bytes_written[SouthIAm]),
            1, fp);
    fclose(fp);
}

static void increment_total_bytes(size_t m_bytes) {
    FILE *fp;

    total_bytes_written[SouthIAm] += m_bytes;

    fp = fopen(tmp_bytes_log, "w");
    if (fp) {
        fwrite(&tmp_bytes_log, sizeof(total_bytes_written[SouthIAm]), 1, fp);
        fclose(fp);
        rename(tmp_bytes_log, total_bytes_log);
    }
}

/**
 * Checks whether a device has already been added to the diskpool
 * @param m_uuid NULL-terminated string giving the device UUID
 * @return pointer to diskentry if extant, NULL otherwise
 */
diskentry_t *diskpool_is_in_pool(const char *m_uuid) {
    if (s_diskmanager_exit)
        return false;

    if (!m_uuid) {
        blast_err("Tried to add unnamed disk to the pool!");
        return false;
    }

    for (int i = 0; i < NUM_USB_DISKS; i++) {
        if (!s_diskpool.disk[i].dev)
            return NULL;
        if (!strncasecmp(m_uuid, s_diskpool.disk[i].dev, strlen(m_uuid))) {
            return &(s_diskpool.disk[i]);
        }
    }
    return NULL;
}


/**
 * Verifies that all disks with open files are responding.
 */
static bool diskpool_disk_is_available(diskentry_t *m_disk) {
    if (!m_disk)
        return false;
    return true;
}


/**
 * Unmounts the specified disk after updating the free space information and passing it to flash.
 * This function should only be called in a thread as it will block while files are open on the
 * disk.  Do not depend on this thread returning in anything close to a reasonable amount of
 * time, so remember to detach it.
 *
 * @param [in,out] m_disk Disk to unmount
 */

static void *diskpool_unmount(void *m_disk) {
    diskentry_t *disk = (diskentry_t *) m_disk;

    if (!disk || !disk->mnt_point) {
        blast_err("Tried to unmount invalid disk");
    } else {
        blast_info("Unmounting %s", disk->mnt_point);

        errno = 0;
        while ((umount2(disk->mnt_point, 0) == -1) && (errno == EBUSY)) {
            sleep(1);
        }
        rmdir(disk->mnt_point);

        BLAST_SAFE_FREE(disk->mnt_point);
    }
    return NULL;
}

/**
 * Checks if the device specified is currently mounted on the system.
 *
 * @param [in] m_dev Fully justified path to the block device e.g. /dev/sda1
 * @param [out] m_mount Fully justified path where the device is mounted.  Set to NULL if you don't care
 * @return false on error or not mounted, true on success.
 */
static bool diskmanager_dev_is_mounted(const char *m_dev, char *m_mount) {
    int LINE_SIZE = 512;
    char mount_line[LINE_SIZE];
    bool retval = false;

    struct mntent mountentry_s;
    FILE *mtab_fp = NULL;

    if (m_dev == NULL) {
        return false;
    }

    /* The local disk is always mounted, so exit quickly here */
    if (strncmp(m_dev, "local", 5) == 0) {
        return true;
    }

    mtab_fp = setmntent("/proc/mounts", "r");
    if (mtab_fp == NULL) {
        blast_strerror("Could not open /proc/mounts");
        return false;
    }
    while (getmntent_r(mtab_fp, &mountentry_s, mount_line, LINE_SIZE - 1)
            != NULL) {
        if (strcmp(m_dev, mount_line) == 0) {
            if (m_mount != NULL) {
                strncpy(m_mount, mountentry_s.mnt_dir, PATH_MAX);
            }
            retval = true;
            break;
        }
    }

    endmntent(mtab_fp);
    return retval;
}

/**
 * Returns the free space in megabytes of the mounted filesystem
 *
 * Note: This function does not check that it is the root of a mounted filesystem.
 * Thus, if you pass the mount point and the disk is not mounted, you get the free space
 * of the underlying filesystem.  The calling function needs to be smart enough to do this.
 *
 * The positive return value can encode up to 2 petabytes thus overflow is not a problem.
 *  Do not change the return value to int without making some allowance for free space >32GiB
 *
 * @param [in] m_disk Path to the mounted filesystem or any file residing on it.
 * @return -1 on error, free space in megabytes on success
 */
static int32_t diskmanager_freespace(const char *m_disk) {
    struct statvfs statvfsbuf;
    int32_t retval = -1;

    if (statvfs(m_disk, &statvfsbuf) == -1) {
        blast_strerror("Failed to stat %s", m_disk);
        return retval;
    }
    retval = (statvfsbuf.f_bsize / 1024) * (statvfsbuf.f_bavail / 1024);

    return retval;
}

/**
 * Update the freespace counter for a mounted disk in both the structure and flash
 *
 * @param m_disk Pointer to the disk.  Passed as (void *) to allow threading
 *
 */

static void *diskpool_update_mounted_free_space(void *m_disk) {
    diskentry_t *disk = NULL;
    int32_t new_freespace = 0;

    if (m_disk == NULL) {
        blast_err("Passed NULL pointer");
        return NULL;
    }

    disk = (diskentry_t*) m_disk;
    new_freespace = disk->free_space;

    if (disk->mnt_point && diskmanager_dev_is_mounted(disk->dev, NULL)) {
        new_freespace = diskmanager_freespace(disk->mnt_point);
    } else {
        blast_err("Disk not mounted");
        return NULL;
    }

    /**
     * If df_check failed for some reason, we don't know if it is a disk error or network error and we don't
     * really care as this routine exists to facilitate choosing the next disk to mount and when.  Other routines
     * deal with read/write errors and will unmount the disk.
     */
    if (new_freespace == -1) {
        blast_err("Could not get free space on %s", disk->dev);
        return NULL;
    }

    disk->free_space = new_freespace;
    return NULL;
}

/**
 * Primary disk search routine.  This routine will identify the first disk that satisfies all of our
 * criteria.  The criteria are given below in a large boolean expression and evaluated in increasing order
 * of time required.
 * @return positive index number of the best matching disk or -1 on failure
 */
static diskentry_t *diskpool_find_new_disk() {
    diskentry_t *disk = NULL;
    diskentry_t *best_disk = NULL;
    uint32_t best_fail = UINT32_MAX;

    for (int i = 0; i < NUM_USB_DISKS; i++) {
        disk = &s_diskpool.disk[i];
        if ((disk->dev) && ((disk->free_space == -1) ||
                (disk->free_space > DISK_MIN_FREE_SPACE))
                && (disk != s_diskpool.current_disk)) {
            if (!disk->fail_count) {
                best_disk = disk;
                break;
            }

            if (disk->fail_count < best_fail) {
                best_fail = disk->fail_count;
                best_disk = disk;
            }
        }
    }
    return best_disk;
}


/**
 * Mounts a block device and returns the newly created mount point.
 *
 * Loops through the following possible disk formats before exiting with failure:
 * ext3, ext4, xfs, jfs, reiser
 *
 * @param [in] m_disk Fully justified path to the block device e.g. /dev/sda1
 * @param [out] m_mntpoint Fully justified path to directory where block device has been mounted
 * @param [in] m_rwflag Bit field specifying read/write access
 * @param [in] m_data Flags that can be set on a per-filesystem basis.  Set to NULL if none.
 * @return -1 on failure, 0 on success
 *
 */
static int diskpool_mount_disk(diskentry_t *m_disk,
        uint32_t m_rwflag, void *m_data) {
    char type[3][16] = { "xfs", "ext4", "" };
    int i = 0;
    static bool first_time = true;

    if (m_disk == NULL) {
        blast_err("Attempted to pass a NULL device");
        return -1;
    }

    if (diskpool_make_mount_point(m_disk->mnt_point) == -1) {
        blast_fatal("Could not create new directory");
        return -1;
    }

    /**
     * We want to loop through this as many times as it takes to find the appropriate filesystem type.  This is
     * dependent on the number of types we have specified above.  This will always be larger than 1, so I format
     * the loop this way to allow i to be used as the proper index, counting from 0.
     */
    do {
        errno = 0;
        if (first_time) {
            blast_info("Mounting %s at %s with filesystem type %s", m_disk->dev,
                    m_disk->mnt_point, type[i]);
        }
        if (mount(m_disk->dev, m_disk->mnt_point, type[i], m_rwflag, m_data) == -1) {
            if (first_time) {
                blast_dbg("Could not mount %s at %s with filesystem type %s: %s",
                        m_disk->dev, m_disk->mnt_point, type[i], strerror(errno));
            }
            /**
             * If the device is not found, or the
             * superblock is invalid, try the next filesystem in #type
             */
            if (errno == ENODEV || errno == EINVAL) {
                usleep(10000);
                continue;
            }
        }
        break;
    } while (type[++i][0] != '\0');
    first_time = false;

    if (errno != 0) return -1;
    return 0;
}

/**
 * Handles mounting an index number from the diskpool.
 * Updates last access time on success.
 * @param m_disknum
 * @return true on success, false on failure
 */
static bool diskpool_mount_diskentry(diskentry_t *m_disk) {
    bool retval = false;
    char *options = "";

    errno = 0;
    if (diskmanager_dev_is_mounted(m_disk->dev, NULL)) {
        blast_info("Not using %s: Already mounted", m_disk->dev);
        m_disk->last_accessed = time(NULL);
        m_disk->fail_count++;
        return false;
    }

    if (diskpool_mount_disk(m_disk,
            MS_NOATIME | MS_NODIRATIME, options) != -1) {
        m_disk->last_accessed = time(NULL);
        retval = true;
    } else {
        /**
         * If we can't mount the disk, we want to emit an error and not try to mount the disk again
         * for some time.  We achieve this by incrementing the local fail_count.  This will de-favor
         * the disk during the current mount cycle but will be reset once the disk responds to pings
         */
        m_disk->last_accessed = time(NULL);
        m_disk->fail_count++;
    }
    return retval;
}

/**
 * Mounts the next available disk for writing.
 *
 * @details We try to mount a new disk three times, in order of increasing desperation.
 *  - First pass is diskpool_find_best_aoe_disk.  This has all constraints turned on.  First disk to
 *      pass all constraints is assigned.
 *  - Second pass is diskpool_find_any_aoe_disk.  This lifts some constraints and uses fail-ordering
 *  - Third pass is to use the local disk.
 *  While this scheme is redundant in scanning the pool, it will succeed very quickly in all normal cases.
 *  The speed of checking the normal cases means that, should it failover, the time to the second case is
 *  extremely small.
 * @return pointer to the mounted disk entry on success, NULL on failure
 */
static diskentry_t *diskpool_mount_new(void) {
    diskentry_t *best_disk = 0;
    int32_t count = 0;
    bool disk_mounted = false;
    bool have_warned = false;

    while (!disk_mounted) {
        count = 0;
        while (!disk_mounted && (count++ < NUM_USB_DISKS)) {
            best_disk = diskpool_find_new_disk();
            if (!best_disk) {
                if (!have_warned) {
                    blast_warn("No USB disks found, waiting...");
                    have_warned = true;
                }
                usleep(10000);
                break;
            }

            disk_mounted = diskpool_mount_diskentry(best_disk);
        }
        usleep(10000);
    }

    if (disk_mounted) {
        blast_info("Disk %s mounted at %s", best_disk->dev,
                best_disk->mnt_point);
        best_disk->last_accessed = time(NULL);
        diskpool_update_mounted_free_space(best_disk);
    }
    return best_disk;
}

/**
 * Mounts a new disk and places it in the primary pointer.  This routine cannot fail or we
 * will have no disks with which to write data.  The failsafe is in diskpool_mount_new() where
 * if no AoE are available it simply loops.  This should always work.
 */
static void diskpool_mount_primary() {
    s_diskpool.current_disk = diskpool_mount_new();
    if (!s_diskpool.current_disk) {
        blast_fatal("Could not mount primary disk");
        exit(1);
    }
    blast_info("New primary disk mounted (index = %u) at mount point %s",
               (s_diskpool.current_disk)->index, (s_diskpool.current_disk)->mnt_point);
}

/**
 * Make a new mount point for a disk
 * @param [out] m_mntpoint Pointer to a pointer to a char array.
 *          Will be allocated and receive the name of the newly created directory.
 * @return -1 on failure, 0 on success
 */
static int diskpool_make_mount_point(char *m_mntpoint) {
    if (m_mntpoint == NULL) {
        blast_err("Passed NULL pointer");
        return -1;
    }

    errno = 0;
    if (diskpool_mkdir_file(m_mntpoint, false) == -1) {
        blast_strerror("Could not create new directory");
        return -1;
    }
    return 0;
}

/**
 * Ensure that the referenced directory and all of its parent directories have been made
 * @param m_directory
 * @return
 */
static int diskpool_mkdir_recursive(char *m_directory) {
    char current_path[PATH_MAX];
    int offset = 0;
    char *path_chunk = NULL;
    char *strtok_ref = NULL;
    int ret = 0;
    struct stat dir_stat;

    /** Ensure that we have a NULL delimiter in our temp string */
    current_path[0] = 0;

    /**
     * Takes a chunk of the directory path at a time
     */
    path_chunk = strtok_r(m_directory, "/", &strtok_ref);
    if (m_directory[0] == '/') {
        offset = snprintf(current_path, PATH_MAX, "%s/", current_path);
    }
    while (path_chunk != NULL) {
        if (offset + strlen(path_chunk) + 2 > PATH_MAX) {
            blast_err("Path too long");
            return -1;
        }
        offset += snprintf(current_path + offset, PATH_MAX - offset, "%s/", path_chunk);
        if ((ret = stat(current_path, &dir_stat)) != 0) {
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
/**
 * Takes a fully justified path to a file and
 * recursively makes the directory structure to hold the file
 *
 * @details Begins by extracting the first token delimited by a
 * slash.  Since the slash is not included in the token, it appends a
 * slash to the directory name.  Then, if we get another token, we know
 * that we have at least 1 directory to make.  The directory is created
 * and we loop.  The last token is the filename, which we don't want as
 * a directory.  We check for disk and permission errors but allow for
 * existence errors (we don't care if the directory is already created)
 *
 * @param [in] m_file Justified file or directory name.  If partially justified,
 *              the primary disk's mount point is prepended
 * @param [in] m_file_appended true if #m_filename is suffixed with a filename
 *              false if #m_filename is a bare directory name
 * @return -1 on error, 0 on success
 */
static int diskpool_mkdir_file(const char *m_filename, bool m_file_appended) {
    char directory_name[PATH_MAX];
    size_t len = 0;

    if (m_filename == NULL) {
        blast_err("Passed NULL pointer");
        return -1;
    }

    if (m_file_appended) {
        len = (size_t) (strrchr(m_filename, '/') - m_filename);
    } else {
        len = strlen(m_filename);
    }

    /** len is used for the maximum number of characters inclusive of the \0 char, so increment
     * by one from strlen's return
     */
    if (++len > PATH_MAX)
        len = PATH_MAX;

    if (file_is_local(m_filename)) {
        snprintf(directory_name, len, "%s", m_filename);
    } else {
        len += strlen(s_diskpool.current_disk->mnt_point) + 1; /* The additional character here is for the '/' */
        if (len > PATH_MAX)
            len = PATH_MAX;

        snprintf(directory_name, len + 1, "%s/%s",
                s_diskpool.current_disk->mnt_point, m_filename);
    }
    // blast_dbg("Making %s", directory_name);
    return diskpool_mkdir_recursive(directory_name);
}


/**
 * This function ensures that all disks are unmounted before disk manager starts.
 */
static void diskmanager_clear_old_mounts()
{
    char mount_line[PATH_MAX] = { 0 };

    struct mntent mountentry_s;
    FILE *mtab_fp = NULL;

    mtab_fp = setmntent("/proc/mounts", "r");
    if (mtab_fp == NULL) {
        blast_strerror("Could not open /proc/mounts");
        return;
    }

    while (getmntent_r(mtab_fp, &mountentry_s, mount_line, PATH_MAX - 1) != NULL) {
        /**
         * We check that this is a directory we would like to unmount in two ways:
         * - It must contain the prefix we use to identify the mounts we create
         * - It must start with the HOME_DIR directory
         */
        if ((strstr(mountentry_s.mnt_dir, MNT_DIR_PREFIX) != NULL)
                && (strstr(mountentry_s.mnt_dir, HOME_DIR)
                        == mountentry_s.mnt_dir)) {
            blast_startup("Unmounting %s", mountentry_s.mnt_dir);
            if ((umount2(mountentry_s.mnt_dir, 0) == 0)
                    && (rmdir(mountentry_s.mnt_dir) == 0)) {
                /* Since we are changing mounted volumes, close and re-open to ensure we don't miss anything */
                endmntent(mtab_fp);
                mtab_fp = setmntent("/proc/mounts", "r");
            } else {
                blast_info("Could not remove %s: %s", mountentry_s.mnt_dir,
                        strerror(errno));
            }
        }
        usleep(10000);
    }
    endmntent(mtab_fp);
}

static void file_free_fileentry(fileentry_t *m_file) {
    if (m_file) {
        BLAST_SAFE_FREE(m_file->filename);
        BLAST_SAFE_FREE(m_file);
    }
}

/**
 * Closes and re-opens a file that was interrupted by disk error.  File will be re-opened on
 * the primary disk.
 * @param [in] m_fileid Index of the file to be transferred
 * @return -1 on error, 0 on success
 */
static int file_change_disk(fileentry_t *m_file, diskentry_t *m_disk) {
	blast_info("Attempting to change the disk for file %s", m_file->filename);
    int retval = -1;

    if ((m_file && m_disk) && (diskmanager_dev_is_mounted(m_disk->dev, NULL))) {
        if (m_file->fp)
            file_close_internal(m_file, true); // This does not remove the file from the filepool.
        m_file->last_error = 0;

        if (m_file->mode[0] != 'r')
            retval = file_reopen_on_new_disk(m_file, m_disk);
    }
    return retval;
}

/**
 * Checks if the string specifies a fully justified pathname.  If path is fully-justified,
 * we consider the file to be local
 *
 * @param [in] m_filename Character string for file
 * @return true if #m_filename is local, false if #m_filename is relative
 */
static inline bool file_is_local(const char *m_filename) {
    return (m_filename && m_filename[0] == '/');
}

/**
 * Open a continuation of a file on a new disk whose writing was interrupted on the previous
 * disk by an error.  No mutexes are set as the error handler should be in control of the pool.
 *
 * @param [in] m_fileid File index number.  The file pointer should already be closed.
 * @param [in] m_disk Pointer to the new disk on which the file should be opened.
 * @return -1 on error, 0 on success
 */

static int file_reopen_on_new_disk(fileentry_t *m_file, diskentry_t *m_disk) {
    FILE *fp = NULL;
    int retval = 0;
    char *filename = NULL;
    char *full_filename = NULL;
    char *substr = NULL;
    int name_len;
    int val = 1;
    int success = 0;
    ck_ht_entry_t ent;
    if (s_diskmanager_exit) {
        return -1;
    }

    if (s_diskpool.current_disk == NULL) {
        blast_err("No current disk");
        return -1;
    }

    name_len = strlen(m_file->filename);
    if ((substr = strstr(m_file->filename, "_cont_"))) {
        // If we've already continued, increment the counter and trim the basename
        val = (int)strtol(substr + 6, NULL, 10) + 1;
        name_len = substr - m_file->filename;
    }
    if (asprintf(&filename, "%.*s_cont_%04d", name_len, m_file->filename, val)<= 0) {
        blast_fatal("Could not allocate filename memory");
        return -1;
    }

    if (diskpool_mkdir_file(filename, true) != 0) {
        return -1;
    }
    // Remove old entry from the hashtable.
    ck_ht_entry_set(&ent, m_file->filehash, m_file->filename,
                    strlen(m_file->filename), m_file);
    if (!ck_ht_remove_spmc(&s_filepool, m_file->filehash, &ent)) {
        blast_err("Could not remove old entry %s from file hash table", m_file->filename);
    }

    blast_tmp_sprintf(full_filename, "%s/%s",
            s_diskpool.current_disk->mnt_point, filename);
    /* Re-opening on a new disk means that we were writing when we ran out of space.  Thus we
     * should open the new file for write access as well.
     */
    fp = fopen(full_filename, m_file->mode);

    if (fp == NULL) {
        blast_strerror("Could not open file %s", filename);
        m_file->last_error = errno;
        retval = -1;
    }

    m_file->fp = fp;
    BLAST_SAFE_FREE(m_file->filename);
    m_file->filename = filename;
    m_file->disk = m_disk;
    // Now we add back the entry into the filepool hashtable
    ck_ht_hash(&m_file->filehash, &s_filepool, m_file->filename,
                strlen(m_file->filename));
    ck_ht_entry_set(&ent, m_file->filehash, m_file->filename,
                strlen(m_file->filename), m_file);
    success = ck_ht_put_spmc(&s_filepool, m_file->filehash, &ent);
    if (!success) {
    	blast_info("Could not insert entry %s back into the hashtable.", m_file->filename);
    }
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
static void filepool_handle_disk_error(diskentry_t *m_disk) {
    fileentry_t *file;
    ck_ht_iterator_t iter = CK_HT_ITERATOR_INITIALIZER;
    ck_ht_entry_t *entry;
    pthread_t unmount_thread;

    blast_info("Beginning disk error handling for disk %u, free_space = %d, mnt_point = %s",
               m_disk->index, m_disk->free_space, m_disk->mnt_point);

    m_disk->fail_count++;
    if (m_disk == s_diskpool.current_disk) {
    	blast_info("Problem with current disk, attempting to mount a new disk");
        diskpool_mount_primary();
    }

    while (ck_ht_next(&s_filepool, &iter, &entry)) {
        file = (fileentry_t*) ck_ht_entry_value(entry);
        blast_info("Moving to next entry in the filepool hashtable. name = %s", file->filename);
        if ((file->disk == m_disk)
                && (file_change_disk(file, s_diskpool.current_disk) == -1)) {
            blast_err("Could not re-open %s of %s",
                    file->filename ? file->filename : "unk",
                    s_diskpool.current_disk->mnt_point ?
                            s_diskpool.current_disk->mnt_point : "unk");
        }
    }

    pthread_create(&unmount_thread, NULL, diskpool_unmount, m_disk);
    pthread_detach(unmount_thread);
}

/**
 * Checks if the file name specified exists on the current AoE disk.  If the path is fully-justified
 * then check for the file on local mount points, otherwise prepend the AoE mount point to the
 * primary disk
 *
 * @param [in] m_filename String specifying the filename
 * @param [in] m_permission Access permissions as given in access(2): either the bitwise
 *              OR of the flags R_OK, W_OK, X_OK, or the existence test F_OK.
 * @return 0 on success, -1 otherwise
 */
int file_access(const char *m_filename, int m_permission) {
    char *fullpath = NULL;

    if (s_diskmanager_exit) {
        blast_tfatal("MCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        sched_yield();

    if (m_filename == NULL || s_diskpool.current_disk == NULL) {
        return -1;
    }

    if (file_is_local(m_filename)) {
        blast_tmp_sprintf(fullpath, "%s", m_filename);
    } else {
        blast_tmp_sprintf(fullpath, "%s/%s", s_diskpool.current_disk->mnt_point,
                m_filename);
    }

    return access(fullpath, m_permission);
}

/**
 * Gets the statvfs structure for the specified file, either local or over AoE
 *
 * @param [in] m_filename String specifying the filename
 * @param [out] m_statbuf File statistics structure returned by stat(2)
 * @return -1 on error, 0 on success
 */
int file_stat(const char *m_filename, struct stat *m_statbuf) {
    char *fullpath = NULL;

    if (s_diskmanager_exit) {
        blast_tfatal("MCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        sched_yield();

    if ((m_statbuf == NULL) || m_filename == NULL
            || s_diskpool.current_disk == NULL) {
        return -1;
    }

    memset(m_statbuf, 0, sizeof(*m_statbuf));

    if (file_is_local(m_filename)) {
        blast_tmp_sprintf(fullpath, "%s", m_filename);
    } else {
        blast_tmp_sprintf(fullpath, "%s/%s", s_diskpool.current_disk->mnt_point,
                m_filename);
    }

    return stat(fullpath, m_statbuf);
}

/**
 * Removes and cleans up file entries based on their hash value
 */
static void filepool_close_file(fileentry_t *m_file) {
    ck_ht_entry_t ent;

    ck_ht_entry_set(&ent, m_file->filehash, m_file->filename,
            strlen(m_file->filename), m_file);
    if (!ck_ht_remove_spmc(&s_filepool, m_file->filehash, &ent)) {
        blast_err("Could not remove %s from file hash table", m_file->filename);
    }
    blast_info("Calling file_close_internal for %s", m_file->filename);
    file_close_internal(m_file, false);
    blast_info("Freeing the hashtag entry");
    file_free_fileentry(m_file);
}

/**
 * Flush all file buffers currently open in #s_filepool that have any data.
 *
 * @return 0 on success, negative errno of write error on failure
 */
static int filepool_flush_buffers(void) {
    fileentry_t *file = NULL;
    size_t length;
    ck_ht_iterator_t iter = CK_HT_ITERATOR_INITIALIZER;
    ck_ht_entry_t *entry;
	uint16_t filepool_counter = 0;

	// Move through the hashtable.
    while (ck_ht_next(&s_filepool, &iter, &entry)) {
		// blast_info("Moving to next entry in the filepool hashtable.");
        file = (fileentry_t*) ck_ht_entry_value(entry);
        diskentry_t *disk = (diskentry_t*) file->disk;
        file->last_error = 0;
        length = filebuffer_len(&(file->buffer));
        if (length > 0) {
            // blast_info("Attempting to open %s", file->filename);
            if (!file->fp && (file_open_internal(file) < 0)) {
                blast_err("Couldn't open %s: %s", file->filename,
                        strerror(file->last_error));
                break;
            }

            if (filebuffer_writeout(&(file->buffer), file->fp) < 0) {
                file->last_error = errno;
                blast_info("Error writing to file %s, %s on disk %u at mount point %s",
                           file->filename, strerror(errno), disk->index, disk->mnt_point);
                return -file->last_error;
            }
            increment_total_bytes(length);
        }

        if (file->is_closed) {
            blast_info("Closing the %uth entry in the hash table", filepool_counter);
            blast_info("Attempting to close file %s at mount point %s", file->filename, disk->mnt_point);
            filepool_close_file(file);
        }
        filepool_counter++;
    }
    return 0;
}

/**
 * Read data from a file on disk
 *
 * @param [in] m_fileid Unique identification number to file in pool
 * @param [out] m_data buffer in which to store output
 * @param [in] m_len size in bytes of 1 element
 * @param [in] m_num number of elements to read
 * @return -1 on failure, positive number of elements read on success
 *
 */
ssize_t file_read(fileentry_t *m_file, void *m_data, size_t m_len, size_t m_num) {
    ssize_t retval = -1;

    if (s_diskmanager_exit)
        return -1;

    if (m_data && m_file && m_file->fp && !m_file->is_closed) {
        retval = (ssize_t) fread(m_data, m_len, m_num, m_file->fp);
    }

    return retval;
}

/**
 * Opens a new file on the current disk
 *
 * This function assumes the correct form of mnt_point (no trailing '/') and
 * m_filename (starts with an alpha-numeric, no '/')
 *
 * @param [in] m_filename Name of the file to open, justified path from mount base
 * @param [in] m_mode Access mode for the file to be opened (follows fopen(2) format)
 * @return NULL on failure, pointer to the new fileentry_t on success
 *
 */
fileentry_t *file_open(const char *m_filename, const char *m_mode) {
    fileentry_t *retval = NULL;

    if (s_diskmanager_exit) {
        blast_tfatal("Diskmanager is exiting.  Killing thread");
        return NULL;
    }

    /* Treat fully justified paths as local files */
    if (file_is_local(m_filename)) {
        return file_open_local(m_filename, m_mode);
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();

    retval = calloc(1, sizeof(fileentry_t));
    retval->parent = pthread_self();
    retval->filename = strdup(m_filename);
    strncpy(retval->mode, m_mode, 4);

    if (m_mode[0] == 'r' || strstr(m_mode, "+")) {
        if (!diskpool_disk_is_available(s_diskpool.current_disk)
                || file_open_internal(retval) < 0) {
            file_free_fileentry(retval);
            return NULL;
        }
    }

    if (filebuffer_init(&(retval->buffer), (unsigned int) FILE_BUFFER_SIZE)
            == -1) {
        blast_fatal("Could not allocate buffer space for %s", m_filename);
        if (retval->fp)
            fclose(retval->fp);
        file_free_fileentry(retval);
        retval = NULL;
    }

    if (retval) {
        ck_ht_entry_t ent;
        bool success;
        ck_ht_hash(&retval->filehash, &s_filepool, retval->filename,
                strlen(retval->filename));
        ck_ht_entry_set(&ent, retval->filehash, retval->filename,
                strlen(retval->filename), retval);
        success = ck_ht_put_spmc(&s_filepool, retval->filehash, &ent);
        if (!success) {
            blast_err("File %s already open!", m_filename);
            file_free_fileentry(retval);
            retval = NULL;
        }
    }

    return retval;
}

/**
 * External interface to the file_close_internal procedure.  Marks the file for closing by
 * the main diskmanager routine
 */
int file_close(fileentry_t *m_file) {
    if (m_file) {
        m_file->is_closed = true;
    }
    return 0;
}

/**
 * Writes data to a file buffer.
 *
 * @details We want to allow for writing constant strings of uncalculated length
 * to a file.  Thus we allow #m_len to be = 0.  This has two effects:
 * First, it assumes that we are writing a string
 * Second, it assumes that we don't want to write the terminating NULL character
 * Thus, use #m_len=0 only when writing constant strings.  If you want the NULL
 * character written, you must calculate the length yourself.
 *
 * @param [in] m_fileid Unique identification number to file in the pool
 * @param [in] m_data Data to be written
 * @param [in] m_len Number of characters to be written. If 0, m_data is passed
 *                  to strlen() to determine length
 * @return -1 on failure, positive number of bytes on success
 */
ssize_t file_write(fileentry_t *m_file, const char *m_data, size_t m_len) {
    ssize_t retval = 0;

    while (!s_ready) {
        if (s_diskmanager_exit)
            return -1;
        pthread_yield();
    }

    if (m_data == NULL) {
        blast_err("Tried to write NULL string to file");
        return -1;
    }

    if (!m_file || m_file->is_closed) {
        blast_err("Passed invalid file");
        return -1;
    }

    /* Allow m_len to be undefined as sometimes we might wish to write a string
     * whose length we can't or don't want to calculate.  This only works for
     * NULL-terminated strings.
     */
    if (m_len == 0) {
        m_len = strlen(m_data);
    }
    /* Check again.  If there are no characters, don't write out. */
    if (m_len == 0) {
        return 0;
    }

    retval = filebuffer_append((filebuffer_t*) &(m_file->buffer), m_data,
            m_len);

    if (retval == -1) {
        /**
         * We cannot append data to the buffer and cannot flush it to disk.
         * Without data writing, we need to quit and reboot (using watchdog)
         * Needless to say, this should never happen.
         */
        blast_fatal("Could not allocate memory for expanding the disk buffer");
        return -1;
    }

    return retval;
}

/**
 * Copies a file using the disk manager system.  Can copy between arbitrary locations (on and off the disk system)
 * but is primarily useful for safely transferring files onto the AoE array
 *
 * @param [in] m_source Fully justified path to a local file
 * @param [in] m_dest Partially justified path to filename on AoE primary disk that will be created
 * @return -1 on error, 0 on success
 */
int file_copy(const char *m_source, const char *m_dest) {
    fileentry_t *source_fp = NULL;
    fileentry_t *dest_fp = NULL;
    const int FILE_CACHE_BLOCK = 64 * 1024;
    char cache[FILE_CACHE_BLOCK];
    ssize_t read_size = 0;
    int retval = 0;

    if (s_diskmanager_exit) {
        blast_tfatal("MCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();

    memset(cache, 0, FILE_CACHE_BLOCK);

    if ((m_source == NULL) || (m_dest == NULL))
        return -1;

    if (!s_diskpool.current_disk || !s_diskpool.current_disk->mnt_point) {
        blast_err("Current disk not mounted");
        return -1;
    }
    source_fp = file_open(m_source, "r");
    if (!source_fp) {
        blast_err("Could not open source file %s", m_source);
        return -1;
    }

    dest_fp = file_open(m_dest, "w");
    if (!dest_fp) {
        blast_err("Could not open destination file %s/%s",
                s_diskpool.current_disk->mnt_point, m_dest);
        file_close(source_fp);
        return -1;
    }

    while (!feof(source_fp->fp)) {
        read_size = file_read(source_fp, cache, 1, FILE_CACHE_BLOCK);
        if (read_size == -1) {
            blast_strerror("Reading file %s returned error.",
                    source_fp->filename);
            retval = -1;
            break;
        }
        if (file_write(dest_fp, cache, read_size) == -1) {
            blast_strerror("File write returned error writing to %s",
                    dest_fp->filename);
            retval = -1;
            break;
        }
    }

    file_close(source_fp);
    file_close(dest_fp);

    return retval;
}

int make_local_symlink(char * filename) {
    char * linkname = NULL;
    int i = 0;
    for (i = strlen(filename)-1; i >= 0; i--) {
      if (filename[i] == '/') break;
    }
    blast_tmp_sprintf(linkname, "/data/rawdir/%s", filename+i+1);
    unlink(linkname);

    return symlink(filename, linkname);
}
/**
 * Opens a pre-existing fileentry_t.
 * @param m_file
 * @return
 */
static int file_open_internal(fileentry_t *m_file) {
    char *filename;

    if (diskpool_mkdir_file(m_file->filename, true) != 0) {
        return -1;
    }

    blast_tmp_sprintf(filename, "%s/%s", s_diskpool.current_disk->mnt_point,
            m_file->filename);
    blast_info("Opening %s", filename);
    errno = 0;
    m_file->fp = fopen(filename, m_file->mode);
    m_file->disk = s_diskpool.current_disk;

    if (m_file->fp == NULL) {
        m_file->last_error = errno;
        blast_strerror("Could not open file %s", filename);
        return -1;
    }
    setvbuf(m_file->fp, NULL, _IONBF, 0);
    make_local_symlink(filename);

    return 0;
}

/**
 * Returns a pointer to the string for the current disk mount point
 */
const char * get_current_disk_mnt_point() {
    if (!s_diskpool.current_disk) return NULL;
    return s_diskpool.current_disk->mnt_point;
}

int32_t get_current_disk_free_space() {
    if (!s_diskpool.current_disk) return 0;
    return s_diskpool.current_disk->free_space;
}
int16_t get_current_disk_index() {
    if (!s_diskpool.current_disk) return 0;
    return s_diskpool.current_disk->index;
}

/**
 * Opens a new file on the local disk
 *
 * This function assumes the correct form of m_filename
 * -- fully justified (including all paths)
 *
 * @param [in] m_filename Name of the file to open, fully justified path
 * @param [in] m_mode Access mode for the file to be opened (follows fopen(2) format)
 * @return -1 on failure, positive value indicating new file index in pool on success
 *
 */

static fileentry_t *file_open_local(const char *m_filename, const char *m_mode) {
    FILE *fp = NULL;
    fileentry_t *retval = NULL;

    /* We ignore the return value here.  If diskpool_mkdir fails, then
     * fopen will fail as well, providing the appropriate error message.
     */
    diskpool_mkdir_file(m_filename, true);

    errno = 0;
    fp = fopen(m_filename, m_mode);

    if (fp == NULL) {
        blast_err("Could not open file %s, error %d", m_filename, errno);
        return NULL;
    }

    /*
     * We have successfully opened the file.  Now a number of things need to happen.
     * - Create a new #s_fileentry in the #s_filepool
     * - Populate the #s_fileentry data with a s_diskentry, file descriptor and name
     * - Allocate and set a write buffer
     */

    setvbuf(fp, NULL, _IONBF, 0);

    retval = calloc(1, sizeof(fileentry_t));
    retval->fp = fp;
    retval->parent = pthread_self();
    retval->filename = strdup(m_filename);
    retval->disk = &s_diskpool.local_disk;

    if (filebuffer_init(&(retval->buffer), (unsigned int) FILE_BUFFER_SIZE)
            == -1) {
        blast_fatal("Could not allocate buffer space for %s", m_filename);
        fclose(fp);
        file_free_fileentry(retval);
        retval = NULL;
    }

    if (retval) {
        ck_ht_entry_t ent;
        bool success;
        ck_ht_hash(&retval->filehash, &s_filepool, retval->filename,
                strlen(retval->filename));
        ck_ht_entry_set(&ent, retval->filehash, retval->filename,
                strlen(retval->filename), retval);
        success = ck_ht_put_spmc(&s_filepool, retval->filehash, &ent);
        if (!success) {
            blast_err("File %s already open!", m_filename);
            file_free_fileentry(retval);
            retval = NULL;
        }
    }

    return retval;
}

/**
 * Closes a file in the #s_filepool
 *
 * This function does not lock the file mutex.  This is purposeful as you should control a file for
 * writing before attempting to close it.  In other words
 * don't call this function unless you have locked the mutex yourself.
 *
 * @param [in] m_fileid Unique identification number to find file in the pool
 * @param [in] m_continuing Boolean to determine whether this file is being transferred to a new disk
 *
 */

static void file_close_internal(fileentry_t *m_file, bool m_continuing) {
    if (s_diskmanager_exit)
        return;

    if (!m_file) {
        blast_err("Passed invalid file");
        return;
    }

    if (m_file->fp && (fclose(m_file->fp) == -1)) {
        blast_strerror("Error closing file. Error: %d", errno);
        m_file->last_error = errno;
    }

    m_file->fp = NULL;
    m_file->disk = NULL;

    if (!m_continuing) {
        BLAST_SAFE_FREE(m_file->filename);
        filebuffer_free(&m_file->buffer);
    }
}

/**
 * Returns the position in the open file specified by #m_fileid
 * @param [in] m_fileid Index of the file
 * @return integer position of the current point in the file, -1 on failure
 */
int file_tell(fileentry_t *m_file) {
    int position = -1;

    if (s_diskmanager_exit)
        return -1;

    while (!s_ready)
        pthread_yield();

    if (!m_file || !m_file->fp || m_file->is_closed) {
        blast_err("Passed invalid file");
    } else {
        position = ftell(m_file->fp);
    }
    return position;
}

/**
 * Structured data writing wrapper.  Mimics format of fwrite(2) with the
 * exception of the file index being the first parameter instead of the
 * last.  This is purposeful to ensure we are calling the appropriate
 * function
 *
 * @param [in] m_file File pointer
 * @param [in] m_struct pointer to data structure to write
 * @param [in] m_size Size of a single element in structure
 * @param [in] m_num Number of elements to write
 * @return -1 on failure, positive number of elements written on success
 */

ssize_t file_write_struct(fileentry_t *m_file, void *m_struct, size_t m_size,
        size_t m_num) {
    size_t length = m_size * m_num;
    ssize_t retval;
    retval = file_write(m_file, (const char*) m_struct, length);

    if (retval > 0)
        retval /= m_size;

    return retval;
}

/**
 * Writes formatted output to a file
 * @param m_fileid File in pool to which we we want to write
 * @param m_fmt printf-style string of const chars and formatting strings
 * @param ... Additional paramaters as needed for the formatting string
 * @return -1 on failure, 0 on success
 */

int file_printf(fileentry_t *m_file, const char* m_fmt, ...) {
    char *string;
    va_list argptr;

    va_start(argptr, m_fmt);
    blast_tmp_vsprintf(string, m_fmt, argptr);
    va_end(argptr);

    return file_write(m_file, string, 0);
}

/**
 * Returns the fully-justified path of a file in the file pool.  N.B. Using this function is BAD.
 * Accessing the AoE disks directly using the kernel system may cause your program to hang for
 * extended periods of time while the kernel module stares at its navel and tries to determine
 * whether a subnet protocol should take minutes to respond.  Use the built-in file access routines.
 *
 * @param [in] m_fd The index of the file in the pool
 * @param [out] m_path Pointer to a char array of minimum size #PATH_MAX
 * @return -1 on failure, 0 on success
 */
int file_get_path(fileentry_t *m_file, char *m_path) {
    diskentry_t *disk;
    if (s_diskmanager_exit)
        return -1;

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();

    if (file_is_local(m_file->filename)) {
        if (strlen(m_file->filename) > PATH_MAX) {
            blast_err("Local path name too long");
            return -1;
        }
        strncpy(m_path, m_file->filename, PATH_MAX);
        return 0;
    }

    if (!m_file->disk || !m_file->disk->mnt_point) {
        blast_err("File has no current disk");
        disk = s_diskpool.current_disk;
        if (!disk || !disk->mnt_point)
            return -1;
    } else {
        disk = m_file->disk;
    }

    if ((strlen(disk->mnt_point) + strlen(m_file->filename)) + 2 > PATH_MAX) {
        blast_err("Path name too long");
        return -1;
    }

    snprintf(m_path, PATH_MAX - 1, "%s/%s", disk->mnt_point, m_file->filename);

    return 0;
}


/**
 * Main disk manager thread.  Handles writing from the buffer.  Starts the error handler
 * thread to deal with disk errors.
 */
static void *diskmanager(void *m_arg __attribute__((unused))) {
    int retval;
    while (!s_diskmanager_exit) {
        sleep(1);

        while ((retval = filepool_flush_buffers()) < 0) {
            blast_warn("Got an error writing to %s",
                    s_diskpool.current_disk->dev);
            filepool_handle_disk_error(s_diskpool.current_disk);
        }

        diskpool_update_mounted_free_space(s_diskpool.current_disk);

        if (s_diskpool.current_disk->free_space < DISK_MIN_FREE_SPACE) {
            filepool_handle_disk_error(s_diskpool.current_disk);
            continue;
        }

        if (!s_ready) {
	          s_ready = true;
	          blast_info("Set s_ready to true.");
        }
    }
    blast_info("Caught signal to exit.  Flushing disks.");
    s_ready = false;

    return NULL;
}


void initialize_diskmanager(void) {
    ck_ht_init(&s_filepool, CK_HT_MODE_BYTESTRING, NULL, &ALLOCATOR, 128,
            BLAST_MAGIC32);

    blast_info("Beginning initialize_dismanager.");
    diskmanager_clear_old_mounts();
    drivepool_init_usb_info();
    diskpool_mount_primary();
    initialize_total_bytes();

    // s_ready = true;
    // blast_info("Set s_ready to true.");

    pthread_create(&diskman_thread, NULL, diskmanager, NULL);
    pthread_detach(diskman_thread);
}

/**
 * Provide a method for cleanly shutting down and syncing disks with unwritten data.
 */
void diskmanager_shutdown() {
    s_diskmanager_exit = true;
    filepool_flush_buffers();
    fcloseall();
    s_ready = false;
}

