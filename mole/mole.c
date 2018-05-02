/* ---------------------------------------------------------------------
 * ------------------------------- MOLE --------------------------------
 * ---------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL)
 * Version 2 or higher.
 *
 * ------------------------- Description -------------------------------
 * Mole is a client program that requests frame data from a bittlm 
 * server on a given network. Upon receipt, the framefile packets are 
 * converted to the DIR file format on the local machine. Additionally,
 * image data (full frame images, thumbnails, science images) with the 
 * framefile are parsed and assembled for viewing either via an external
 * interface (i.e. .jpg and .ppm files) or via the BIS format (viewable
 * in KST). Mole also stores a raw version of the framefile packets to 
 * the local disk with a corresponding format file. 
 * 
 * The converted DIR file is available via /data/etc/mole.lnk and the 
 * directory of cached DIR files, format files, framefiles is located at
 * /data/rawdir/.
 * 
 * For convenience, symbolic links to the latest full frame images (saved
 * as .jpg) are generated in /data/etc every time a new image is received.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: June 24, 2014
 *
 * Copyright 2017 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 * 26/08/17 - made mole independent of local telemlist.bit and linklists;
 * the telemlist and linklist for a specified link is requested from the
 * bittlm server and parsed locally so that mole is always up-to-date
 *
 */

#include <fcntl.h>
#include <stdlib.h>
#include <stdio.h>

#include <errno.h>
#include <netdb.h>
#include <signal.h>
#include <string.h>
#include <time.h>
#include <unistd.h>
#include <inttypes.h>
#include <getopt.h>
#include <termios.h>

#include <sys/socket.h>
#include <sys/syslog.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <dirent.h>

#include <netinet/in.h>
#include <arpa/inet.h>

#include <pthread.h> // threads
#include <openssl/md5.h>

#include <linklist.h>
#include <linklist_compress.h>
#include <linklist_writer.h>
#include <linklist_connect.h>

static linklist_tcpconn_t tcpconn = {"cacofonix"};
char dirfile_name[128] = "test.DIR";
char symname[128] = "/data/rawdir/LIVE";

int main(int argc, char *argv[]) {
  // mode selection
  int server_mode = 0;
  int client_mode = 1;
  unsigned int flags = TCPCONN_FILE_RAW | TCPCONN_RESOLVE_NAME;

  // initialization variables
  uint32_t req_serial = 0;
  unsigned int req_framenum = 0;
  unsigned int req_init_framenum = 0;
  int req_blksize = 0;

  // received data variables
  uint8_t * recv_buffer = NULL;
  uint8_t recv_header[TCP_PACKET_HEADER_SIZE] = {0};
  unsigned int buffer_size = 0;
  unsigned int recv_size = 0;

  // superframe and linklist 
  superframe_t * superframe = NULL;
  linklist_t * linklist = NULL;
  linklist_dirfile_t * ll_dirfile = NULL;
  linklist_rawfile_t * ll_rawfile = NULL;

  int i;
  for (i = 1; i < argc; i++) {
    if (argv[i][0] == '@') { // custom target
      strcpy(tcpconn.ip, argv[i]+1);
    } else if (strcmp(argv[i], "-s") == 0) { // server mode
      server_mode = 1;
    } else if (strcmp(argv[i], "-o") == 0) { // specify an output file
      strcpy(dirfile_name, argv[++i]);
    } else if (strcmp(argv[i], "-nc") == 0) { // no client mode
      client_mode = 0;
    } else {
      printf("Unrecognized option \"%s\"\n", argv[i]);
      exit(1);
    }
  }

  pthread_t server_thread;
  if (server_mode) {
    // start the server to accept clients
    pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL); 
  }

  if (client_mode) {
    char linklistname[64] = {0};
    user_file_select(&tcpconn, linklistname);

    req_serial = sync_with_server(&tcpconn, linklistname, flags, &superframe, &linklist);
    req_init_framenum = initialize_client_connection(&tcpconn, req_serial);
    req_blksize = linklist->blk_size;
    if (linklist->flags & LL_INCLUDE_ALLFRAME) req_blksize += superframe->allframe_size;    

    printf("Client initialized with serial 0x%.4x and %d frames\n", req_serial, req_init_framenum);

    // open linklist dirfile
    ll_dirfile = open_linklist_dirfile(linklist, dirfile_name);  

    // open linklist rawfile
    char filename[128] = {0};
    sprintf(filename, "%s/%s", archive_dir, linklistname);
    ll_rawfile = open_linklist_rawfile(linklist, filename); 
    create_rawfile_symlinks(ll_rawfile, symname);

    while (1) {
      if (buffer_size < req_blksize) {
        buffer_size = req_blksize;
        recv_buffer = realloc(recv_buffer, buffer_size);
      }
      recv_size = retrieve_data(&tcpconn, req_framenum, req_blksize, recv_buffer, recv_header);
      write_linklist_dirfile(ll_dirfile, recv_buffer);
      write_linklist_rawfile(ll_rawfile, recv_buffer);

      printf("Received frame %d (size %d)\n", req_framenum, recv_size);
  
/* 
      int i;
      for (i = 0; i < recv_size; i++) {
        if (i % 32 == 0) printf("\n%.4d: ", i/32);
        printf("0x%.2x ", recv_buffer[i]);
      }
      printf("\n");
*/

      memset(recv_buffer, 0, buffer_size);
      memset(recv_header, 0, TCP_PACKET_HEADER_SIZE);
      req_framenum++;
    }
  }

  if (server_mode) pthread_join(server_thread, NULL);
}






