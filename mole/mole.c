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

linklist_tcpconn_t tcpconn = {{0}};

int main(int argc, char *argv[]) {
  // mode selection
  int server_mode = 1;
  int client_mode = 1;
  unsigned int flags = TCPCONN_FILE_RAW;

  // initialization variables
  uint32_t req_serial = 0;
  unsigned int req_framenum = 0;
  unsigned int req_init_framenum = 0;
  int req_blksize = 0;

  // received data variables
  uint8_t * recv_buffer = NULL;
  unsigned int buffer_size = 0;
  unsigned int recv_size = 0;

  // superframe and linklist 
  superframe_t * superframe = NULL;
  linklist_t * linklist = NULL;

  int i;
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-ns") == 0) { // no server mode
      server_mode = 0;
    } else if (strcmp(argv[i], "-nc") == 0) { // no client mode
      client_mode = 0;
    }
  }

  pthread_t server_thread;
  if (server_mode) {
    // start the server to accept clients
    pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL); 
  }

  if (client_mode) {
    sprintf(tcpconn.ip, "cacofonix");
    char linklistname[64] = {0};
    user_file_select(&tcpconn, linklistname);

    req_serial = sync_with_server(&tcpconn, linklistname, flags, &superframe, &linklist);
    req_init_framenum = initialize_client_connection(&tcpconn, req_serial);
    req_blksize = linklist->blk_size;
    if (linklist->flags & LL_INCLUDE_ALLFRAME) req_blksize += superframe->allframe_size;    

    printf("Client initialized with serial 0x%.4x and framenum %d\n", req_serial, req_init_framenum);
    printf("Binary file blksize is %d, linklist blksize is %d\n", req_blksize, linklist->blk_size);

    while (req_framenum < req_init_framenum) {
      if (buffer_size < req_blksize) {
        buffer_size = req_blksize;
        recv_buffer = realloc(recv_buffer, buffer_size);
      }
      printf("Requesting frame %d\n", req_framenum);  

      recv_size = retrieve_data(&tcpconn, req_framenum, req_blksize, recv_buffer);

      printf("Received frame %d (size %d)\n", req_framenum, recv_size);

      memset(recv_buffer, 0, buffer_size);
      req_framenum++;
    }
  }

  if (server_mode) pthread_join(server_thread, NULL);
}






