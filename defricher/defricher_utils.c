/* 
 * defricher_utils.c: 
 *
 * This software is copyright (C) 2013-2014 Seth Hillbrand
 *
 * This file is part of defricher, created for the BLASTPol Project.
 *
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * History:
 * Created on: Apr 6, 2015 by Seth Hillbrand
 */


#include <stdio.h>
#include <stdint.h>
#include <inttypes.h>
#include <stdbool.h>
#include <limits.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <string.h>
#include <unistd.h>

#include <blast.h>
#include "defricher_utils.h"

/**
 * Ensure that the referenced directory and all of its parent directories have been made
 * @param m_directory
 * @return
 */
static int defricher_mkdir_recursive (char *m_directory)
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
            defricher_err("Path too long");
            return -1;
        }
        strcat(current_path, path_chunk);
        strcat(current_path, "/");
        if (stat(current_path, &dir_stat) != 0)
        {
            ret = mkdir(current_path, ACCESSPERMS);
            if (ret < 0)
            {
                defricher_strerr("Could not make %s", current_path);
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
int defricher_mkdir_file(const char *m_filename, bool m_file_appended)
{

    char    directory_name[PATH_MAX];
    size_t  len = 0;

    if (m_filename == NULL)
    {
        defricher_err("Passed NULL pointer");
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

    if (*m_filename == '/')
    {
        snprintf(directory_name, len, "%s", m_filename);
    }
    else
    {
        len += strlen(get_current_dir_name()) + 1; /* The additional character here is for the '/' */
        if (len > PATH_MAX) len = PATH_MAX;

        snprintf(directory_name,len+1,"%s/%s",get_current_dir_name(),m_filename);
    }


    return defricher_mkdir_recursive(directory_name);

}
