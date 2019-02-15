#include <unistd.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <fcntl.h>
#include <errno.h>
#include <syslog.h>
#include <signal.h>
#include <libgen.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <pthread.h>

#include "groundhog_funcs.h"

int verbose = 1;
int system_idled = 0;
sigset_t signals;
struct LinklistState ll_state[MAX_NUM_LINKLIST_FILES+1] = {{0}};

struct LinklistState * groundhog_ll_state(uint32_t serial) {
  int i;
  for (i=0; i<MAX_NUM_LINKLIST_FILES; i++) {
    if (!ll_state[i].serial || (serial == ll_state[i].serial)) break;
  }
  ll_state[i].serial = serial;
  return &ll_state[i];
}

void clean_up(void) {
    unlink("/var/run/groundhog.pid");
    // closelog();
}

void daemonize()
{
	int pid;
	FILE* stream;

	if ((pid = fork()) != 0) {
		if (pid == -1) {
			groundhog_fatal("unable to fork to background\n");
		}
		if ((stream = fopen("/var/run/groundhog.pid", "w")) == NULL) {
			groundhog_fatal("unable to write PID to disk\n");
		} else {
			fprintf(stream, "%i\n", pid);
			fflush(stream);
			fclose(stream);
		}
		// closelog();
		groundhog_info("PID = %i\n", pid);
		exit(0);
	}
	atexit(clean_up);

	/* Daemonise */
	chdir("/");
	freopen("/dev/null", "r", stdin);
	freopen(GROUNDHOG_LOG, "a", stdout);
	setvbuf(stdout,NULL,_IONBF,0);
	freopen("/dev/null", "w", stderr);
	setsid();
}
// -------------------------------------- //
// --- groundhog_check_for_fileblocks --- //
// -------------------------------------- //
/* This function returns non-zero when a file download linklist has been 
 * received.
 * 
 * ll - the linklist for the received data
 * ll_name - the name of the file download linklist
 * 
 */
int groundhog_check_for_fileblocks(linklist_t * ll, char * ll_name) {
  return (!strcmp(ll->name, ll_name));
}

// ------------------------------------//
// --- groundhog_unpack_fileblocks --- //
// ------------------------------------//
/* This function unpacks telemetry data that has been downlinked as file blocks,
 * which are packets that comprise parts of a larger file to be assembled by
 * groundhog. If file data is in the telemetry stream, this function will loop
 * through the buffer extracting parts of the files and writing them to disk.
 * 
 * ll - the linklist for the received data
 * transmit_size - the size of the buffer received
 * buffer - the buffer received with packed linklist file blocks
 *
 */
int groundhog_unpack_fileblocks(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                linklist_rawfile_t ** ll_rawfile) {
  static uint8_t * dummy_buffer = NULL;
  if (!dummy_buffer) dummy_buffer = calloc(1, ll->superframe->size);

	unsigned int bytes_unpacked = 0;
	while ((bytes_unpacked+ll->blk_size) <= transmit_size) {
		// write the rawfile to disk 
		if (ll_rawfile) {
      groundhog_process_and_write(ll, ll->blk_size, compbuffer+bytes_unpacked, NULL, "FILEBLOCKS", 
                                  NULL, ll_rawfile, !(*ll_rawfile) ? GROUNDHOG_OPEN_NEW_RAWFILE : 0);
    }
#ifdef GROUNDHOG_FILEBLOCKS_EXTRACT_TO_DISK
		// unpack and extract to disk
		decompress_linklist(dummy_buffer, ll, compbuffer+bytes_unpacked);
#endif
		bytes_unpacked += ll->blk_size;
	}
	return (ll->blocks[0].i*100/ll->blocks[0].n);
}

void groundhog_make_symlink_name(char *fname, char *symname) {
  sprintf(fname, "%s/%s_live", archive_dir, symname);
}

// ------------------------------------//
// --- groundhog_process_and_write --- //
// ------------------------------------//
/* This function unpacks telemetry data that has been downlinked as linklist data.
 * Data will be written to a linklist rawfile with automatic handling of allframe
 * data.
 * 
 * ll - the linklist for the received data
 * transmit_size  - the size of the buffer received
 * compbuffer     - the buffer received with linklist data
 * local_allframe - a pointer to the allframe data
 *                    - if NULL, allframe data will not be written or extracted
 * filename_str   - the prefix for the name of the rawfile to be opened.
 *                    - a timestamp will be appended to this file name
 *                    - if NULL, a linklist rawfile will not be created
 * disp_str       - in verbose mode, a modifier for the string printed to stdout
 *                    - if NULL, no string will be printed to stdout
 * ll_rawfile     - a pointer to the linklist rawfile pointer
 *                    - is updated when a new file is created
 * flags          - special flags controlling linklist rawfile handling
 *                    - if GROUNDHOG_OPEN_NEW_RAWFILE, a new linklist rawfile
 *                      will be opened
 *                    - if GROUNDHOG_REUSE_VALID_RAWFILE, the last rawfile used
 *                      that matches the filename_str will be re-opened only if
 *                      GROUNDHOG_OPEN_NEW_RAWFILE is specified
 *
 */
int groundhog_process_and_write(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags) {
  // process the linklist and write the data to disk
  int af = read_allframe(NULL, ll->superframe, compbuffer);
  int retval = 0;

  if ((flags & GROUNDHOG_OPEN_NEW_RAWFILE) && filename_str) {
	  *ll_rawfile = groundhog_open_rawfile(*ll_rawfile, ll, filename_str, flags);
  }

  // catch null rawfile
  if (!(*ll_rawfile)) {
    return retval;
  }

  if (af > 0) { // an allframe was received
    if (verbose && disp_str) groundhog_info("[%s] Received an allframe :)\n", disp_str);
    if (local_allframe) memcpy(local_allframe, compbuffer, ll->superframe->allframe_size);

    if (*ll_rawfile) retval = tell_linklist_rawfile(*ll_rawfile);
    else retval = 0;
  } else if (af == 0) { // just a regular frame (< 0 indicates problem reading allframe)
    if (verbose && disp_str) groundhog_info("[%s] Received linklist \"%s\"\n", disp_str, ll->name);

    // check for consistency in transmit size with linklist bulk size
    if (transmit_size > ll->blk_size) {
      groundhog_warn("groundhog_process_and_write: Packet size mismatch blk_size=%d, transmit_size=%d\n", ll->blk_size, transmit_size);
      transmit_size = ll->blk_size;
    }

    // write the linklist data to disk
    if (*ll_rawfile) {
      if (local_allframe) {
        memcpy(compbuffer+ll->blk_size, local_allframe, ll->superframe->allframe_size);
      }
      write_linklist_rawfile(*ll_rawfile, compbuffer);
      flush_linklist_rawfile(*ll_rawfile);

      retval = tell_linklist_rawfile(*ll_rawfile);
    }
  }
  return retval;
}

linklist_rawfile_t * groundhog_open_rawfile(linklist_rawfile_t * ll_rawfile, linklist_t *ll, char * symname, int flags) {
  if (ll_rawfile) {
    close_and_free_linklist_rawfile(ll_rawfile);
    ll_rawfile = NULL;
  } 
  char sname[LINKLIST_MAX_FILENAME_SIZE];
  char filename[LINKLIST_MAX_FILENAME_SIZE];
  int newfile = 1;

	groundhog_make_symlink_name(sname, symname);

  if (flags & GROUNDHOG_REUSE_VALID_RAWFILE) { // option to reuse file, so check symlink
    // do a search for the linklist format
		strcat(sname, LINKLIST_FORMAT_EXT);

    // if path exists, check if serials match
		if (realpath(sname, filename)) { 
			uint32_t serial = read_linklist_formatfile_comment(filename, LINKLIST_FILE_SERIAL_IND, "%x");
			if (serial == *(uint32_t *) ll->serial) {
        // serials match, so don't need to make a new file
				newfile = 0;
			} 
		} 
    // undo strcat on sname 
    char * ext = strstr(sname, LINKLIST_FORMAT_EXT);
    if (ext) ext[0] = '\0';
  }

  if (newfile) {
    // make a new filename
    make_linklist_rawfile_name(ll, filename);
    groundhog_info("Opening new file \"%s\"\n", filename);
  } else {
    // remove the file extension from the existing filename
    char * ext = strstr(filename, LINKLIST_FORMAT_EXT);
    if (ext) ext[0] = '\0';
    groundhog_info("Opening existing file \"%s\"\n", filename);
  }
  
  // open the rawfile and create symlinks
  ll_rawfile = open_linklist_rawfile(filename, ll);
  create_rawfile_symlinks(ll_rawfile, sname);

  // generate the calspecs file
  sprintf(sname, "%s" CALSPECS_FORMAT_EXT, filename);
  groundhog_write_calspecs(sname);

  return ll_rawfile;
}
