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
    }
    else {
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
 * 
 */
int groundhog_check_for_fileblocks(linklist_t * ll) {
  return (!strcmp(ll->name, FILE_LINKLIST));
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
 * buffer - the buffer received
 *
 */
int groundhog_unpack_fileblocks(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer) {
  static uint8_t * dummy_buffer = NULL;
  if (!dummy_buffer) dummy_buffer = calloc(1, ll->superframe->size);

  // unpack and extract to disk
  if (groundhog_check_for_fileblocks(ll)) {
    unsigned int bytes_unpacked = 0;
    while ((bytes_unpacked+ll->blk_size) <= transmit_size) {
      decompress_linklist(dummy_buffer, ll, compbuffer+bytes_unpacked);
      bytes_unpacked += ll->blk_size;
      usleep(1000);
    }
    return (ll->blocks[0].i*100/ll->blocks[0].n);
  }
  return 0;
}

int groundhog_process_and_write(linklist_t * ll, unsigned int transmit_size, uint8_t * compbuffer,
                                uint8_t * local_allframe, char * filename_str, char * disp_str,
                                linklist_rawfile_t ** ll_rawfile, unsigned int flags) {
  // process the linklist and write the data to disk
  int af = read_allframe(NULL, ll->superframe, compbuffer);
  int retval = -1;
  if (af > 0) { // an allframe was received
    if (verbose) groundhog_info("[%s] Received an allframe :)\n", disp_str);
    memcpy(local_allframe, compbuffer, ll->superframe->allframe_size);

    if (*ll_rawfile) retval = tell_linklist_rawfile(*ll_rawfile);
    else retval = 0;
  } else if (af == 0) { // just a regular frame (< 0 indicates problem reading allframe)
    if (flags && GROUNDHOG_OPEN_NEW_RAWFILE) {
      *ll_rawfile = groundhog_open_new_rawfile(*ll_rawfile, ll, filename_str);
    }
    if (verbose) groundhog_info("[%s] Received linklist \"%s\"\n", disp_str, ll->name);

    // check for consistency in transmit size with linklist bulk size
    if (transmit_size > ll->blk_size) {
      groundhog_warn("Packet size mismatch blk_size=%d, transmit_size=%d\n", ll->blk_size, transmit_size);
      transmit_size = ll->blk_size;
    }

    // write the linklist data to disk
    if (*ll_rawfile) {
      memcpy(compbuffer+ll->blk_size, local_allframe, ll->superframe->allframe_size);
      write_linklist_rawfile(*ll_rawfile, compbuffer);
      flush_linklist_rawfile(*ll_rawfile);

      retval = tell_linklist_rawfile(*ll_rawfile);
    }
  }
  return retval;
}

linklist_rawfile_t * groundhog_open_new_rawfile(linklist_rawfile_t * ll_rawfile, linklist_t * ll, char * symname) {
  if (ll_rawfile) {
    close_and_free_linklist_rawfile(ll_rawfile);
    ll_rawfile = NULL;
  } 
  char filename[128];
  make_linklist_rawfile_name(ll, filename);
  ll_rawfile = open_linklist_rawfile(filename, ll);

  char fname[128];
  sprintf(fname, "%s/%s_live", archive_dir, symname);
  create_rawfile_symlinks(ll_rawfile, fname);

  sprintf(fname, "%s" CALSPECS_FORMAT_EXT, filename);
  groundhog_write_calspecs(fname);

  return ll_rawfile;
}
