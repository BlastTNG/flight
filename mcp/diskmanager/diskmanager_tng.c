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
#include <ck_ht.h>

#include "diskmanager_tng.h"
// Hardware IDs for the drives connected by USB
static const char drive_uuids[2][NUM_USB_DISKS][64] = {{"",
        "",
        "",
        "",
        "",
        "",
        "",
        ""},
        {"ccbff6e7-8e51-49e4-a987-9ebf5644813e",
        "674e5a19-eb93-4c05-b12c-6a50c03ca5c1",
        "67e991c8-1e1e-4f77-84f1-9273c050e385",
        "22804e9d-a3e1-4cf8-a5b2-ff2fcf22bc5e",
        "94ac1984-a52b-4be6-afb7-cb8302d249e0",
        "993e105e-1cbc-4913-abca-29540242c57e",
        "6846dffc-cf41-447a-a576-4ab34cad7974",
        "a52e5c25-8dbc-4e55-ae73-7c5f8b49968c"}};

static int diskpool_verify_primary_disk();

static int file_change_disk(int m_fileid, diskentry_t *m_disk);
static int file_reopen_on_new_disk(int m_fileid, diskentry_t *m_disk);
static void filepool_handle_disk_error(volatile diskentry_t *m_disk);

static int file_open_local(const char*, const char*);
static void file_close_internal(int, bool);
static inline bool file_is_local(const char*);

static harddrive_info_t hd_info[NUM_USB_DISKS];
static diskpool_t s_diskpool = {0};
static diskentry_t s_localdisk = {0};
static ck_ht_t *s_filepool;
static pthread_t diskman_thread;

static const uint64_t diskman_timeout = 250000000;	// diskman_timeout 1/4 second in nano seconds

// When set to true, the diskmanager freespace thread will update the current drive's flash
static bool s_diskmanager_update_freespace_flag = false;

// When set to true, diskmanager will exit on next loop
static volatile bool s_diskmanager_exit = false;

// Initialized to False, procedures dependent on file access wait for true
static volatile bool s_ready = false;

extern int16_t SouthIAm;
uint64_t total_bytes_written[2] = {0,0};
static const char *total_bytes_log = "/data/etc/fcp_total_bytes";
static const char *tmp_bytes_log = "/data/etc/tmp_total_bytes";

// TODO(laura): Write something that actually checks whether the disks are alive.
// Maybe something using the equivalent of touch?
static int usb_is_disk_alive()
{
    // blast_info("Hope so!");
    return(1);
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
static void initialize_total_bytes(void)
{
    FILE *fp;

    fp = fopen(total_bytes_log, "r");
    if (!fp)
    {
        fp = fopen(total_bytes_log, "w");
        if (fp) fclose(fp);
        return;
    }

    fread(&total_bytes_written[SouthIAm], sizeof(total_bytes_written[SouthIAm]), 1, fp);
    fclose(fp);
}

static void increment_total_bytes(size_t m_bytes)
{
    FILE *fp;

    total_bytes_written[SouthIAm] += m_bytes;

    fp = fopen(tmp_bytes_log, "w");
    if (fp)
    {
        fwrite(&tmp_bytes_log, sizeof(total_bytes_written[SouthIAm]), 1, fp);
        fclose(fp);
        rename(tmp_bytes_log, total_bytes_log);
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
int diskpool_add_to_pool(diskentry_t *m_disk)
{
    ck_ht_hash_t h;
    ck_ht_entry_t entry;

    if (s_diskmanager_exit) return false;

    if (!m_disk) {
        blast_err("Tried to add unnamed disk to the pool!");
        return false;
    }
    for (int i = 0; i < NUM_USB_DISKS; i++) {
        if (!s_diskpool.disk[i].uuid) {
            memcpy(&(s_diskpool.disk[i]), m_disk, sizeof(diskentry_t));
            return true;
        }
    }
    blast_err("No open slots in disk pool for %s", m_disk->uuid);
    return false;
}



/**
 * Checks whether a device has already been added to the diskpool
 * @param m_uuid NULL-terminated string giving the device UUID
 * @return pointer to diskentry if extant, NULL otherwise
 */
diskentry_t *diskpool_is_in_pool(const char *m_uuid)
{
    ck_ht_hash_t h;
    ck_ht_entry_t entry;

    if (s_diskmanager_exit) return false;

    if (!m_uuid) {
        blast_err("Tried to add unnamed disk to the pool!");
        return false;
    }

    for (int i = 0; i < NUM_USB_DISKS; i++) {
        if (!s_diskpool.disk[i].uuid) return NULL;
        if (!strncasecmp(m_uuid, s_diskpool.disk[i].uuid, strlen(m_uuid))) {
            return &(s_diskpool.disk[i]);
        }
    }
    return NULL;
}

/**
 * Update the local disk entry in the pool
 */
static void diskpool_update_local()
{
    diskentry_t *disk;
    diskentry_t *extant_disk;
    ck_ht_hash_t h;
    char *local_dir = NULL;

    blast_tmp_sprintf(local_dir,"%s/local/",HOME_DIR);

    disk = diskpool_is_in_pool(localdev);

    /**
     * If we get a valid entry back then we have already run this routine and we just update
     * the disk information
     */
    if (disk)
    {
        disk->last_accessed = time(NULL);
        disk->free_space = diskmanager_freespace(local_dir);
        return;
    }

    h = blast_hash_key(localdev, strlen(localdev));
    diskpool_mkdir_recursive(local_dir);
    disk = balloc(fatal, sizeof(*disk));
    e_memset(disk, 0, sizeof(*disk));

    disk->dev = bstrdup(err,localdev);
    disk->fail_count = 0;
    disk->mnt_point = bstrdup(err, local_dir);
    disk->free_space = diskmanager_freespace(disk->mnt_point);
    disk->last_accessed = time(NULL);

    if ((extant_disk = blast_sl_add(s_diskpool.disk_sl, disk_hash, disk, false)))
    {
        blast_err("Local disk already added!");
        EBEX_SAFE_FREE(disk->dev);
        EBEX_SAFE_FREE(disk->mnt_point);
        EBEX_SAFE_FREE(disk);
        disk = extant_disk;
    }

    s_diskpool.local_disk = disk;
}

/**
 * Verifies that all disks with open files are responding.
 */
static bool diskpool_disk_is_available(diskentry_t *m_disk)
{

    if (!m_disk) return false;

    if (!m_disk->isAoE) return true;

    if (aoe_is_disk_alive(m_disk->major, m_disk->minor, m_disk->MAC))
        return true;

    return false;
}


/**
 * Unmounts the specified disk after updating the free space information and passing it to flash.
 * This function should only be called in a thread as it will block while files are open on the
 * disk.  Do not depend on this thread returning in anything close to a reasonable amount of
 * time, so remember to detach it.
 *
 * @param [in,out] m_disk Disk to unmount
 */

static void *diskpool_unmount(void *m_disk)
{
    diskentry_t *disk = (diskentry_t *) m_disk;

    if (!disk || !disk->isAoE || !disk->mnt_point)
    {
        blast_err("Tried to unmount invalid disk");
    }
    else
    {
        aoe_dbg("Unmounting %s", disk->mnt_point);

        errno = 0;
        while ((umount2(disk->mnt_point,0) == -1) && (errno == EBUSY))
        {
            sleep(1);
        }
        rmdir(disk->mnt_point);

        EBEX_SAFE_FREE(disk->mnt_point);
    }
    return NULL;
}

/**
 * Update the freespace counter for a mounted disk in both the structure and flash
 *
 * @param m_disk Pointer to the disk.  Passed as (void *) to allow threading
 *
 */

static void
*diskpool_update_mounted_free_space(void *m_disk)
{
    diskentry_t *disk           = NULL;
    long int    new_freespace   = 0;
    int         retval          = 0;

    if (m_disk == NULL)
    {
        blast_err("Passed NULL pointer");
        return NULL;
    }

    disk = (diskentry_t *)m_disk;
    new_freespace = disk->free_space;

    if (disk->mnt_point && diskmanager_dev_is_mounted(disk->dev, NULL))
        new_freespace = diskmanager_freespace(disk->mnt_point);
    else
    {
        blast_err("Disk not mounted");
        return NULL;
    }

    /**
     * If df_check failed for some reason, we don't know if it is a disk error or network error and we don't
     * really care as this routine exists to facilitate choosing the next disk to mount and when.  Other routines
     * deal with read/write errors and will unmount the disk.
     */
    if (new_freespace == -1)
    {
        blast_err("Could not get free space on %s", disk->dev);
        return NULL;
    }

    /* Don't send flash for the local disk or for AoE disks whose size has not changed by at least 1MB */
    if (disk->isAoE
            && ((new_freespace != disk->free_space)
            || (disk->host != s_preferred_major)))
    {
        disk->free_space = new_freespace;
        aoe_dbg("Sending new freespace %ld MB to %u:%u at %s",
                    (long int)disk->free_space, (unsigned)disk->major, (unsigned)disk->minor, disk->mnt_point);
        retval = aoe_update_flash(disk->major, disk->minor, disk->MAC, disk->free_space);
    }

    if (retval)
    {
        blast_err("Could not update freespace on %u:%u",
                (unsigned)disk->major,(unsigned)disk->minor);
    }


    return NULL;
}

/**
 * Primary AoE disk search routine.  This routine will identify the first disk that satisfies all of our
 * criteria.  The criteria are given below in a large boolean expression and evaluated in increasing order
 * of time required.  aoe_is_disk_alive should always be the last condition.
 * @return positive index number of the best matching disk or -1 on failure
 */
static diskentry_t *diskpool_find_best_aoe_disk(void)
{
    diskentry_t *disk = NULL;
    diskentry_t *best_disk = NULL;
    blast_gc_thread_state_t *state = NULL;
    blast_sl_node_t *node;
    time_t bitcheck = 1;

    node = blast_sl_get_first_node(s_diskpool.disk_sl, &state);
    while (node)
    {
        disk = node->data;
        if ((disk->isAoE)
                && (disk->free_space > DISK_MIN_FREE_SPACE)
                && (disk != s_diskpool.current_disk)
                && (disk->fail_count == 0)
                && (disk->major == s_preferred_major)
                && (aoe_is_disk_alive(disk->major,disk->minor, disk->MAC)))
        {
            best_disk = disk;
            if (time(NULL) & bitcheck) break;
            bitcheck <<= 1;
        }

        node = blast_sl_get_next_node(s_diskpool.disk_sl, node, &state);
    }
    if (node)
    {
        blast_gc_critical_exit(state);
    }

    return best_disk;
}

/**
 * Secondary AoE disk search routine.  Compared with #diskpool_find_best_aoe_disk, this routine loosens
 * the following restrictions:
 * - Disk can have a non-zero fail count
 * @return positive index number of the best matching disk or -1 on failure
 */
static diskentry_t *diskpool_find_any_aoe_disk()
{
    diskentry_t *disk = NULL;
    diskentry_t *best_disk = NULL;
    blast_gc_thread_state_t *state = NULL;
    blast_sl_node_t *node;
    uint32_t best_fail = UINT32_MAX;

    node = blast_sl_get_first_node(s_diskpool.disk_sl, &state);
    while (node)
    {
        disk = node->data;
        if ((disk->isAoE)
                && (disk->free_space > DISK_MIN_FREE_SPACE)
                && (disk != s_diskpool.current_disk)
                && (disk->major == s_preferred_major)
                && (aoe_is_disk_alive(disk->major, disk->minor, disk->MAC)))
        {
            if (!disk->fail_count)
            {
                best_disk = disk;
                break;
            }
            else
            {
                if (disk->fail_count < best_fail)
                {
                    best_fail = disk->fail_count;
                    best_disk = disk;
                }
            }
        }

        node = blast_sl_get_next_node(s_diskpool.disk_sl, node, &state);
    }
    if (node)
    {
        blast_gc_critical_exit(state);
    }

    return best_disk;

}

/**
 * Handles mounting an index number from the diskpool.  Updates last access time on success.  On failure,
 * updates
 * @param m_disknum
 * @return true on success, false on failure
 */
static bool diskpool_mount_diskentry (diskentry_t *m_disk)
{
    bool retval = false;
    char *options = ""; //"logbufs=8,logbsize=256k";

    errno = 0;
    if (diskmanager_dev_is_mounted(m_disk->dev, NULL))
    {
        blast_info("Not using %s: Already mounted", m_disk->dev);
        m_disk->last_accessed = time(NULL);
        m_disk->fail_count++;
        return false;
    }

    if(diskpool_mount_disk(m_disk->dev,&m_disk->mnt_point,MS_NOATIME|MS_NODIRATIME,options) != -1)
    {
        m_disk->host = s_preferred_major;
        m_disk->last_accessed = time(NULL);
        retval = true;
    }
    else
    {
        /**
         * If we can't mount the disk, we want to emit an error and not try to mount the disk again
         * for some time.  We achieve this by incrementing the local fail_count.  This will de-favor
         * the disk during the current mount cycle but will be reset once the disk responds to pings
         */
        blast_strerror("Couldn't mount %s", m_disk->dev);
        m_disk->last_accessed = time(NULL);
        m_disk->fail_count++;
        aoe_update_flash(m_disk->major, m_disk->minor,
                         m_disk->MAC, m_disk->free_space);
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
static diskentry_t *diskpool_mount_new(void)
{
    diskentry_t *best_aoe = 0;
    int32_t count = 0;
    bool disk_mounted = false;

    while (!disk_mounted)
    {
        count = 0;

        /**
         * The first pass should only check the preferred pressure vessel, thus we only allow looping over the
         * number of disks in 1 pressure vessel
         */
        while (!disk_mounted && (count++ < DISK_MAX_NUMBER/DISK_NUM_PRESSURE_VESSELS))
        {
            best_aoe = diskpool_find_best_aoe_disk();
            if (!best_aoe)
            {
                blast_warn("No AoE disks found in first pass");
                break;
            }

            disk_mounted = diskpool_mount_diskentry(best_aoe);
        }

        /**
         * We want to be more loose with the count variable in our second iteration as we now allow looping through
         * all disks in all pressure vessels.
         */
        count = 0;
        while (!disk_mounted && (count++ < DISK_MAX_NUMBER))
        {
            best_aoe = diskpool_find_any_aoe_disk();
            if (!best_aoe)
            {
                blast_err("No AoE disks found in second pass");
                sleep(1);
                aoe_scan_for_disks();
                break;
            }

            disk_mounted = diskpool_mount_diskentry(best_aoe);
        }
    }

    if (disk_mounted)
    {
        blast_info("Disk %s mounted at %s", best_aoe->dev, best_aoe->mnt_point);
        best_aoe->last_accessed = time(NULL);
        diskpool_update_mounted_free_space(best_aoe);
    }

    return best_aoe;

}

/**
 * Mounts a new disk and places it in the primary pointer.  This routine cannot fail or we
 * will have no disks with which to write data.  The failsafe is in diskpool_mount_new() where
 * if no AoE are available it simply chooses the local disk.  This should always work.
 */
static void diskpool_mount_primary()
{
    s_diskpool.current_disk = diskpool_mount_new();

    if (!s_diskpool.current_disk)
    {
        blast_fatal("Could not mount primary disk");
        exit(1);
    }
}

/**
 * Make a new mount point for a disk
 * @param [out] m_mntpoint Pointer to a pointer to a char array.
 *          Will be allocated and receive the name of the newly created directory.
 * @return -1 on failure, 0 on success
 */
static int diskpool_make_mount_point (char **m_mntpoint)
{
    char temp_mount[PATH_MAX];
    time_t currenttime = time(NULL);
    if (m_mntpoint == NULL)
    {
        blast_err("Passed NULL pointer");
        return -1;
    }
    snprintf(temp_mount, PATH_MAX, "%s/%s%lu", HOME_DIR, MNT_DIR_PREFIX,
                            (unsigned long)currenttime);

    errno=0;
    if (diskpool_mkdir_file(temp_mount, false) == -1)
    {
        blast_strerror("Could not create new directory");
        return -1;
    }

    *m_mntpoint = bstrdup(loglevel_err, temp_mount);
    if (*m_mntpoint == NULL) return -1;

    return 0;
}

/**
 * Ensure that the referenced directory and all of its parent directories have been made
 * @param m_directory
 * @return
 */
static int diskpool_mkdir_recursive (char *m_directory)
{
    char        current_path[PATH_MAX];
    char        *path_chunk = NULL;
    char        *strtok_ref = NULL;
    int         ret = 0;
    struct stat dir_stat;

    /** Ensure that we have a NULL delimiter in our temp string */
    current_path[0]=0;

    /**
     * Takes a chunk of the directory path at a time
     */
    path_chunk = strtok_r(m_directory, "/", &strtok_ref);
    if (m_directory[0] == '/')
    {
        strcat(current_path, "/");
    }
    while (path_chunk != NULL)
    {
        if (strlen(current_path) + strlen(path_chunk) + 2 > PATH_MAX)
        {
            blast_err("Path too long");
            return -1;
        }
        strcat(current_path, path_chunk);
        strcat(current_path, "/");
        if (stat(current_path, &dir_stat) != 0)
        {
            ret = mkdir(current_path, ACCESSPERMS);
            if (ret < 0)
            {
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
static int diskpool_mkdir_file(const char *m_filename, bool m_file_appended)
{

    char    directory_name[PATH_MAX];
    size_t  len = 0;

    if (m_filename == NULL)
    {
        blast_err("Passed NULL pointer");
        return -1;
    }

    if (m_file_appended)
    {
        len = (size_t)(strrchr(m_filename, '/') - m_filename);
    }
    else
    {
        len = strlen(m_filename);
    }

    /** len is used for the maximum number of characters inclusive of the \0 char, so increment
     * by one from strlen's return
     */
    if (++len > PATH_MAX) len = PATH_MAX;

    if (file_is_local(m_filename))
    {
        snprintf(directory_name, len, "%s", m_filename);
    }
    else
    {
        len += strlen(s_diskpool.current_disk->mnt_point) + 1; /* The additional character here is for the '/' */
        if (len > PATH_MAX) len = PATH_MAX;

        snprintf(directory_name,len+1,"%s/%s",s_diskpool.current_disk->mnt_point,m_filename);
    }


    return diskpool_mkdir_recursive(directory_name);

}

static void file_free_fileentry(fileentry_t *m_file)
{
    if (m_file)
    {
        EBEX_SAFE_FREE(m_file->filename);
        EBEX_SAFE_FREE(m_file);
    }
}

/**
 * Closes and re-opens a file that was interrupted by disk error.  File will be re-opened on
 * the primary disk.
 * @param [in] m_fileid Index of the file to be transferred
 * @return -1 on error, 0 on success
 */
static int file_change_disk(fileentry_t *m_file, diskentry_t *m_disk)
{
    int retval = -1;

    if ( (m_file && m_disk)
            && (diskmanager_dev_is_mounted(m_disk->dev, NULL)))
    {
        if (m_file->fp) file_close_internal(m_file,true);
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
static inline bool
file_is_local(const char *m_filename)
{
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

static int file_reopen_on_new_disk(fileentry_t *m_file, diskentry_t *m_disk)
{
    FILE    *fp             = NULL;
    int     retval          = 0;
    char    *filename       = NULL;
    char    *full_filename  = NULL;

    if (s_diskmanager_exit)
    {
        return -1;
    }

    if (s_diskpool.current_disk == NULL)
    {
        blast_err("No current disk");
        return -1;
    }

    if (asprintf(&filename,"%s.cont",m_file->filename) <= 0)
    {
        blast_fatal("Could not allocate filename memory");
        return -1;
    }

    if (diskpool_mkdir_file(filename,true) != 0)
    {
        return -1;
    }

    blast_tmp_sprintf(full_filename, "%s/%s", s_diskpool.current_disk->mnt_point, filename);
    /* Re-opening on a new disk means that we were writing when we ran out of space.  Thus we
     * should open the new file for write access as well.
     */
    fp = fopen(full_filename, m_file->mode);

    if (fp == NULL)
    {
        blast_strerror("Could not open file %s", filename);
        m_file->last_error = errno;
        retval = -1;
    }

    m_file->fp = fp;
    EBEX_SAFE_FREE(m_file->filename);
    m_file->filename = filename;
    m_file->disk = m_disk;

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

static void filepool_handle_disk_error(diskentry_t *m_disk)
{
    blast_gc_thread_state_t *state;
    blast_sl_node_t *node;
    fileentry_t *file;
    pthread_t unmount_thread;

    aoe_dbg("Beginning error handling");

    m_disk->fail_count++;
    if (m_disk == s_diskpool.current_disk)
    {
        diskpool_mount_primary();
    }

    node = blast_sl_get_first_node(filepool, &state);
    while (node)
    {
        file = node->data;
        if ((file->disk == m_disk)
                && (file_change_disk(file, s_diskpool.current_disk) == -1))
        {
            blast_err("Could not re-open %s of %s", file->filename?file->filename:"unk",
                    s_diskpool.current_disk->mnt_point?s_diskpool.current_disk->mnt_point:"unk");
        }
        node = blast_sl_get_next_node(filepool, node, &state);
    }

    /// After changing disks, update the pointer for interloquendi
    framefile_update_curfile();

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
int file_access(const char *m_filename, int m_permission)
{
    char *fullpath = NULL;

    if (s_diskmanager_exit)
    {
        blast_tfatal("FCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        sched_yield();

    if (m_filename == NULL || s_diskpool.current_disk == NULL)
    {
        return -1;
    }

    if (file_is_local(m_filename))
    {
        blast_tmp_sprintf(fullpath, "%s", m_filename);
    }
    else
    {
        if (s_diskpool.current_disk->isAoE &&
                !aoe_is_disk_alive(s_diskpool.current_disk->major,
                        s_diskpool.current_disk->minor, s_diskpool.current_disk->MAC))
        {
            return -1;
        }
        blast_tmp_sprintf(fullpath, "%s/%s",s_diskpool.current_disk->mnt_point,m_filename);
    }

    return access(fullpath,m_permission);
}

/**
 * Gets the statvfs structure for the specified file, either local or over AoE
 *
 * @param [in] m_filename String specifying the filename
 * @param [out] m_statbuf File statistics structure returned by stat(2)
 * @return -1 on error, 0 on success
 */
int file_stat(const char *m_filename, struct stat *m_statbuf)
{
    char *fullpath = NULL;

    if (s_diskmanager_exit)
    {
        blast_tfatal("FCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        sched_yield();

    if ((m_statbuf == NULL)
            || m_filename == NULL
            || s_diskpool.current_disk == NULL)
    {
        return -1;
    }

    e_memset(m_statbuf,0,sizeof(*m_statbuf));

    if (file_is_local(m_filename))
    {
        blast_tmp_sprintf(fullpath, "%s", m_filename);
    }
    else
    {
        if (s_diskpool.current_disk->isAoE &&
                !aoe_is_disk_alive(s_diskpool.current_disk->major, s_diskpool.current_disk->minor, s_diskpool.current_disk->MAC))
        {
            return -1;
        }
        blast_tmp_sprintf(fullpath,"%s/%s",s_diskpool.current_disk->mnt_point,m_filename);
    }

    return stat(fullpath, m_statbuf);
}

/**
 * Removes and cleans up file entries based on their hash value
 */
static void filepool_close_file(hash_t m_hash)
{
    fileentry_t *file;

    file = blast_sl_remove(filepool, m_hash);
    if (file)
    {
        file_close_internal(file, false);
        file_free_fileentry(file);
    }
}

/**
 * Flush all file buffers currently open in #s_filepool that have any data.
 *
 * @return 0 on success, negative errno of write error on failure
 */
static int filepool_flush_buffers(void)
{
    blast_gc_thread_state_t *state;
    blast_sl_node_t *node;
    fileentry_t *file;
    size_t length;

    node = blast_sl_get_first_node(filepool, &state);
    while (node)
    {
        file = node->data;

        file->last_error = 0;
        length = filebuffer_len(&(file->buffer));
        if ( length > 0)
        {
            if (!file->fp && (file_open_internal(file) < 0))
            {
                blast_err("Couldn't open %s: %s", file->filename, strerror(file->last_error));
                break;
            }

            if (filebuffer_writeout(&(file->buffer), file->fp) < 0)
            {
                file->last_error = errno;
                break;
            }
            increment_total_bytes(length);
        }

        if (file->is_closed)
        {
            filepool_close_file(file->filehash);
        }

        node = blast_sl_get_next_node(filepool, node, &state);
    }

    if (node)
    {
        blast_gc_critical_exit(state);
        return -file->last_error;
    }

    return 0;
}

/**
 * Copies a file using the disk manager system.  Can copy between arbitrary locations (on and off the disk system)
 * but is primarily useful for safely transferring files onto the AoE array
 *
 * @param [in] m_source Fully justified path to a local file
 * @param [in] m_dest Partially justified path to filename on AoE primary disk that will be created
 * @return -1 on error, 0 on success
 */
int file_copy (const char *m_source, const char *m_dest)
{
    fileentry_t *source_fp = NULL;
    fileentry_t *dest_fp = NULL;
    const int FILE_CACHE_BLOCK = 64*1024;
    char cache[FILE_CACHE_BLOCK];
    ssize_t read_size = 0;
    int retval = 0;

    if (s_diskmanager_exit)
    {
        blast_tfatal("FCP shutting down.  Killing thread");
        return -1;
    }

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();

    memset(cache,0,FILE_CACHE_BLOCK);

    if ((m_source == NULL)
            ||(m_dest==NULL))
    {
        return -1;
    }

    if (!s_diskpool.current_disk || !s_diskpool.current_disk->mnt_point)
    {
        blast_err("Current disk not mounted");
        return -1;
    }
    source_fp = file_open(m_source,"r");
    if (!source_fp)
    {
        blast_err("Could not open source file %s", m_source);
        return -1;
    }

    dest_fp = file_open(m_dest,"w");
    if (!dest_fp)
    {
        blast_err("Could not open destination file %s/%s",
                s_diskpool.current_disk->mnt_point,m_dest);
        file_close(source_fp);
        return -1;
    }

    while (!feof(source_fp->fp))
    {
        read_size = file_read(source_fp, cache,1,FILE_CACHE_BLOCK);
        if (read_size == -1)
        {
            blast_strerror("Reading file %s returned error.",
                        source_fp->filename);
            retval = -1;
            break;

        }
        if (file_write(dest_fp,cache,read_size) == -1)
        {
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

/**
 * Opens a pre-existing fileentry_t.
 * @param m_file
 * @return
 */
static int file_open_internal(fileentry_t *m_file)
{
    char        *filename;

    if( diskpool_mkdir_file(m_file->filename,true) != 0)
    {
        return -1;
    }

    blast_tmp_sprintf(filename, "%s/%s",s_diskpool.current_disk->mnt_point,m_file->filename);
    errno = 0;
    m_file->fp = fopen(filename, m_file->mode);
    m_file->disk = s_diskpool.current_disk;

    if (m_file->fp == NULL)
    {
        m_file->last_error = errno;
        blast_strerror("Could not open file %s", filename);
        return -1;
    }
    setvbuf(m_file->fp, NULL, _IONBF, 0);

    return 0;
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

fileentry_t *file_open (const char *m_filename, const char *m_mode)
{
    fileentry_t *retval = NULL;
    fileentry_t *extant_val;

    if (s_diskmanager_exit)
    {
        blast_tfatal("Diskmanager is exiting.  Killing thread");
        return NULL;
    }

    /* Treat fully justified paths as local files */
    if (file_is_local(m_filename))
    {
        return file_open_local(m_filename, m_mode);
    }

        /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();

    retval = balloc(loglevel_fatal, sizeof(fileentry_t));
    e_memset(retval, 0, sizeof(*retval));

    retval->parent = pthread_self();
    retval->filename = strdup(m_filename);
    strncpy(retval->mode, m_mode, 4);

    if (m_mode[0] == 'r' || strstr(m_mode, "+"))
    {
        if (!diskpool_disk_is_available(s_diskpool.current_disk) || file_open_internal(retval) < 0)
        {
            file_free_fileentry(retval);
            return NULL;
        }
    }

    if (filebuffer_init(&(retval->buffer), (unsigned int) FILE_BUFFER_SIZE) == -1)
    {
        blast_fatal("Could not allocate buffer space for %s", m_filename);
        if (retval->fp) fclose(retval->fp);
        file_free_fileentry(retval);
        retval = NULL;
    }

    if (retval)
    {
        retval->filehash = blast_hash_key(m_filename, strlen(m_filename));
        extant_val = blast_sl_add(filepool, retval->filehash, retval, false);
        if (extant_val)
        {
            blast_err("File %s already open!", m_filename);
            file_free_fileentry(retval);
            retval = NULL;
        }
    }

    return retval;
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

static fileentry_t *file_open_local (const char *m_filename, const char *m_mode)
{
    FILE *fp = NULL;
    fileentry_t *retval = NULL;
    fileentry_t *extant_val = NULL;

    /* We ignore the return value here.  If diskpool_mkdir fails, then
     * fopen will fail as well, providing the appropriate error message.
     */
    diskpool_mkdir_file(m_filename,true);

    errno = 0;
    fp = fopen(m_filename, m_mode);

    if (fp == NULL)
    {
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


    retval = balloc(loglevel_fatal, sizeof(fileentry_t));
    e_memset(retval, 0, sizeof(*retval));

    retval->fp = fp;
    retval->parent = pthread_self();
    retval->filename = strdup(m_filename);
    retval->disk = s_diskpool.local_disk;


    if (filebuffer_init(&(retval->buffer), (unsigned int) FILE_BUFFER_SIZE) == -1)
    {
        blast_fatal("Could not allocate buffer space for %s", m_filename);
        fclose(fp);
        file_free_fileentry(retval);
        retval = NULL;
    }

    if (retval)
    {
        retval->filehash = blast_hash_key(m_filename, strlen(m_filename));
        extant_val = blast_sl_add(filepool, retval->filehash, retval, false);
        if (extant_val)
        {
            blast_err("File %s already open!", m_filename);
            fclose(fp);
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

static void file_close_internal (fileentry_t *m_file, bool m_continuing)
{

    if (s_diskmanager_exit)
        return;

    if (!m_file)
    {
        blast_err("Passed invalid file");
        return;
    }

    if (m_file->fp && (fclose(m_file->fp) == -1))
    {
        blast_strerror("Error closing file. Error: %d", errno);
        m_file->last_error = errno;
    }

    m_file->fp = NULL;
    m_file->disk = NULL;

    if (!m_continuing)
    {
        EBEX_SAFE_FREE(m_file->filename);
        filebuffer_free(&m_file->buffer);
    }

}

/**
 * External interface to the file_close_internal procedure.  Marks the file for closing by
 * the main diskmanager routine
 */
int file_close(fileentry_t *m_file)
{
    if (m_file)
    {
        m_file->is_closed = true;

        /**
         * If we do not have any unwritten data in the file, we can fully remove it
         * from the skip list without
         */
        if (filebuffer_len(&(m_file->buffer)) == 0)
        {
            filepool_close_file(m_file->filehash);
        }
    }
    return 0;
}

/**
 * Returns the position in the open file specified by #m_fileid
 * @param [in] m_fileid Index of the file
 * @return integer position of the current point in the file, -1 on failure
 */
long int file_tell (fileentry_t *m_file)
{
    long int position = -1;

    if (s_diskmanager_exit)
        return -1;

    while (!s_ready)
        pthread_yield();

    if (!m_file || !m_file->fp || m_file->is_closed)
    {
        blast_err("Passed invalid file");
    }
    else
    {
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

ssize_t file_write_struct (volatile fileentry_t *m_file, void *m_struct, size_t m_size, size_t m_num)
{
    size_t length = m_size * m_num;
    ssize_t retval;
    retval = file_write(m_file, (const char*)m_struct, length);

    if (retval > 0) retval /= m_size;

    return retval;
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
ssize_t file_write (volatile fileentry_t *m_file, const char *m_data, size_t m_len)
{
    ssize_t     retval = 0;

    while (!s_ready)
    {
        if (s_diskmanager_exit) return -1;
        pthread_yield();
    }

    if (m_data == NULL)
    {
        blast_err("Tried to write NULL string to file");
        return -1;
    }

    if (!m_file || m_file->is_closed)
    {
        blast_err("Passed invalid file");
        return -1;
    }

    /* Allow m_len to be undefined as sometimes we might wish to write a string
     * whose length we can't or don't want to calculate.  This only works for
     * NULL-terminated strings.
     */
    if (m_len == 0)
    {
        m_len = strlen(m_data);
    }
    /* Check again.  If there are no characters, don't write out. */
    if (m_len == 0)
    {
        return 0;
    }

    retval = filebuffer_append((filebuffer_t*)&(m_file->buffer),m_data,m_len);

    if (retval == -1)
    {

        /**
         * We cannot append data to the buffer and cannot flush it to disk.
         * Without data writing, we need to quit and reboot (using watchdog)
         * Needless to say, this should never happen.
         */
        blast_fatal("Could not reallocate memory for expanding the disk buffer");
        return -1;
    }

    return retval;
}

/**
 * Writes formatted output to a file
 * @param m_fileid File in pool to which we we want to write
 * @param m_fmt printf-style string of const chars and formatting strings
 * @param ... Additional paramaters as needed for the formatting string
 * @return -1 on failure, 0 on success
 */

int file_printf(fileentry_t *m_file, const char* m_fmt, ...)
{
    char *string;
    va_list argptr;

    va_start(argptr, m_fmt);
    blast_tmp_vsprintf(string, m_fmt, argptr);
    va_end(argptr);

    return file_write(m_file, string,0);
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
ssize_t file_read (fileentry_t *m_file, void *m_data, size_t m_len, size_t m_num)
{
    ssize_t retval = -1;

    if (s_diskmanager_exit)
        return -1;

    if (m_data && m_file && m_file->fp && !m_file->is_closed)
    {
        retval = (ssize_t) fread(m_data, m_len, m_num, m_file->fp);
    }

    return retval;

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
int file_get_path (fileentry_t *m_file, char *m_path)
{
    diskentry_t *disk;
    if (s_diskmanager_exit)
        return -1;

    /* s_ready is an initialization flag for diskmanager */
    while (!s_ready)
        pthread_yield();


    if (file_is_local(m_file->filename))
    {
        if (strlen(m_file->filename) > PATH_MAX)
        {
            blast_err("Local path name too long");
            return -1;
        }
        strncpy(m_path,m_file->filename, PATH_MAX);
        aoe_dbg("Got local path %s", m_path);
        return 0;
    }

    if (!m_file->disk || !m_file->disk->mnt_point)
    {
        blast_err("File has no current disk");
        disk = s_diskpool.current_disk;
        if (!disk || !disk->mnt_point)
            return -1;
    }
    else
    {
        disk = m_file->disk;
    }

    if ((strlen(disk->mnt_point) + strlen(m_file->filename)) + 2 > PATH_MAX)
    {
        blast_err("Path name too long");
        return -1;
    }

    snprintf(m_path, PATH_MAX - 1, "%s/%s", disk->mnt_point, m_file->filename);

    aoe_dbg("Got relative path %s", m_path);
    return 0;
}

bool initialize_diskmanager(int m_preferred_disk)
{

    filepool = blast_sl_new(8);

    diskmanager_clear_old_mounts();

    diskpool_update_local();
    aoe_scan_for_disks();
    diskpool_mount_primary();
    initialize_total_bytes();

    s_ready = true;

    pthread_create(&diskman_thread, NULL, diskmanager, NULL);
    pthread_detach(diskman_thread);

    return true;
}

/**
 * Main disk manager thread.  Handles writing from the buffer.  Starts the error handler
 * thread to deal with disk errors.
 */
static void *diskmanager(void *m_arg __attribute__((unused)))
{
    int retval;


    while(!s_diskmanager_exit)
    {
        while ((retval = filepool_flush_buffers()) < 0)
        {
            blast_warn("Got an error writing to %s", s_diskpool.current_disk->uuid);
            filepool_handle_disk_error(s_diskpool.current_disk);
        }

        diskpool_update_mounted_free_space(s_diskpool.current_disk);

        if (s_diskpool.current_disk->free_space < DISK_MIN_FREE_SPACE)
        {
            filepool_handle_disk_error(s_diskpool.current_disk);
        }

        sleep(1);
    }
    blast_info("Caught signal to exit.  Flushing disks.");
    s_ready = false;


    return NULL;
}

/**
 * Provide a method for cleanly shutting down and syncing disks with unwritten data.
 */
void
diskmanager_shutdown()
{

    s_diskmanager_exit = true;
    filepool_flush_buffers();
    fcloseall();
    s_ready = false;
    diskpool_unmount(s_diskpool.current_disk);
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
static int32_t diskmanager_freespace(const char *m_disk)
{
    struct statvfs  statvfsbuf;
    long int retval = -1;

    if(statvfs(m_disk, &statvfsbuf) == -1)
    {
        blast_strerror("Failed to stat %s",m_disk);
        return retval;
    }
    retval = (statvfsbuf.f_bsize/1024) * (statvfsbuf.f_bavail/1024);

    return retval;
}

/**
 * Checks if the device specified is currently mounted on the system.
 *
 * @param [in] m_dev Fully justified path to the block device e.g. /dev/sda1
 * @param [out] m_mount Fully justified path where the device is mounted.  Set to NULL if you don't care
 * @return false on error or not mounted, true on success.
 */

static bool diskmanager_dev_is_mounted(const char *m_dev, char *m_mount)
{
    int             LINE_SIZE = 512;
    char            mount_line[LINE_SIZE];
    bool            retval = false;

    struct mntent   mountentry_s;
    FILE            *mtab_fp = NULL;

    if (m_dev == NULL)
    {
        return false;
    }

    /* The local disk is always mounted, so exit quickly here */
    if (strncmp(m_dev,"local",5) == 0)
    {
        return true;
    }

    mtab_fp = setmntent("/proc/mounts","r");
    if (mtab_fp == NULL){
        blast_strerror("Could not open /proc/mounts");
        return false;
    }
    while (getmntent_r(mtab_fp,&mountentry_s,mount_line,LINE_SIZE-1) != NULL)
    {
        if (strcmp(m_dev,mount_line)==0)
        {
            if (m_mount != NULL)
            {
                strncpy(m_mount,mountentry_s.mnt_dir, PATH_MAX);
            }
            retval = true;
            break;
        }
    }

    endmntent(mtab_fp);
    return retval;
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

static int diskpool_mount_disk(char *m_disk, char **m_mntpoint, unsigned long m_rwflag, void *m_data)
{
    char    type[3][16] = {"xfs","ext4",""};
    int     i           = 0;

    if (m_disk == NULL)
    {
        blast_err("Attempted to pass a NULL device");
        return -1;
    }

    if (diskpool_make_mount_point(m_mntpoint) == -1)
    {
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
        blast_info("Mounting %s at %s with filesystem type %s",
                m_disk, *m_mntpoint, type[i]);
        if (mount(m_disk, *m_mntpoint, type[i], m_rwflag, m_data) == -1) {
                aoe_dbg("Could not mount %s at %s with filesystem type %s: %s",
                                    m_disk, *m_mntpoint, type[i], strerror(errno));
                /**
                 * If the device is not found, or the
                 * superblock is invalid, try the next filesystem in #type
                 */
                if (errno == ENODEV || errno == EINVAL) continue;
        }

        break;

    } while (type[++i][0] != '\0');

    if (errno != 0) {
            blast_err("Could not mount %s at %s, error:%d",
                    m_disk, *m_mntpoint, errno);
            return -1;
    }

    return 0;
}

/**
 * This function exists to keep the #HOME_DIR directory clean.
 *
 * @details When fcp quits, it will leave a unique directory in #HOME_DIR.  It may also leave
 * a filesystem mounted there.  This function ensures that the crufty directories do not pile
 * up and that we don't have bunches of spurious mounts running around.  This function should
 * only be called once, on startup.
 */
static void diskmanager_clear_old_mounts()
{
    int             LINE_SIZE = 512;
    char            mount_line[LINE_SIZE];

    struct mntent   mountentry_s;
    FILE            *mtab_fp = NULL;

    struct dirent   *dir_entry_p = NULL;
    DIR             *data_dirp = NULL;

    mtab_fp = setmntent("/proc/mounts","r");
    if (mtab_fp == NULL){
        blast_strerror("Could not open /proc/mounts");
        return;
    }
    EBEX_ZERO(mountentry_s);
    EBEX_ZERO(mount_line);

    while (getmntent_r(mtab_fp,&mountentry_s,mount_line,LINE_SIZE-1) != NULL)
    {
        /**
         * We check that this is a directory we would like to unmount in two ways:
         * - It must contain the prefix we use to identify the mounts we create
         * - It must start with the HOME_DIR directory
         */
        if ((strstr(mountentry_s.mnt_dir, MNT_DIR_PREFIX)!=NULL)
            && (strstr(mountentry_s.mnt_dir, HOME_DIR) == mountentry_s.mnt_dir))
        {
            aoe_dbg("Unmounting/Removing %s", mountentry_s.mnt_dir);
            if ((umount2(mountentry_s.mnt_dir,0) == 0) && (rmdir(mountentry_s.mnt_dir) == 0))
            {
                /* Since we are changing mounted volumes, close and re-open to ensure we don't miss anything */
                endmntent(mtab_fp);
                mtab_fp = setmntent("/proc/mounts","r");
            }
            else
            {
                aoe_dbg("Could not remove %s: %s", mountentry_s.mnt_dir, strerror(errno));
            }
        }
        else
        {
            aoe_dbg("Keeping %s", mountentry_s.mnt_dir);
        }
    }

    endmntent(mtab_fp);

    data_dirp = opendir(HOME_DIR);
    if (data_dirp == NULL)
    {
        blast_fatal("Could not open HOME_DIR %s:%s",HOME_DIR, strerror(errno));
        return;
    }

    while ((dir_entry_p = readdir(data_dirp)))
    {
        if ((strstr(dir_entry_p->d_name, MNT_DIR_PREFIX)!= NULL))
        {
            snprintf(mount_line, LINE_SIZE, "%s/%s", HOME_DIR, dir_entry_p->d_name);
            aoe_dbg("clear_old_mounts : Deleting %s", mount_line);
            if (rmdir(mount_line) == -1)
            {
                blast_strerror("Could not delete %s", mount_line);
            }
        }
        else
        {
            aoe_dbg("clear_old_mounts : Not Deleting %s/%s", HOME_DIR, dir_entry_p->d_name);
        }
    }

    closedir(data_dirp);

    return;
}

