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

void USAGE(void) {
  
  printf("\n\nMole is a generic linklist client/server that converts raw "
      "binary data to dirfiles.\n\n"
      "Usage: mole [OPTION]...\n\n"
      " -nc                Don't run a client.\n"
      "                    Default is to run a client.\n"
      " -s                 Run a server for other mole clients to connect to.\n"
      "                    Default is to not run a server.\n"
      " -w X               Start acquiring data X frames from the current index.\n"
      "                    Default is 20.\n"
      " -v                 Verbose mode.\n"
      "                    \n"
      " @host              Connect to a specific host.\n"
      " --archive-dir dir  Save raw binary files to specified directory.\n"
      "                    Only valid if backing up binary data locally.\n"
      "                    Default is /data/rawdir.\n"
      " --backup           Backup binary data in the archive directory.\n"
      "                    Default is to not backup data.\n"
      " --loopback         Have mole extract its own binary files.\n"
      " --little-end       Force mole to interpret data as little endian.\n"
      " --mole-dir dir     Set the directory in which dirfiles will be stored.\n"
      "                    Default is /data/mole.\n"
      " --no-backup        Do not backup data in the archive directory.\n"
      "                    This is the default.\n"
      " --no-check         Ignore checksum values when processing data.\n"  
      "\n");

    exit(0);
}

void print_display(char * text, unsigned int recv_framenum) {
	static char spin[] = "/-\\|";
  static unsigned int s = 0;
  static unsigned int prev_framenum = 0;
  static unsigned int idle_count = 0;

	char arrow[13] = "------------";

  if (recv_framenum == prev_framenum) {
    idle_count++;
  } else {
    idle_count = 0;
  }
  prev_framenum = recv_framenum;

  if (idle_count < 5) {
    s = (s+1)%4;
  }

	arrow[s%12] = '>';
	arrow[(s+4)%12] = '>';
	arrow[(s+8)%12] = '>';
	arrow[10] = '\0';
	printf("%c Frame %d %s %s", spin[s], recv_framenum, arrow, text);
	printf("\r");
	fflush(stdout); 
}

static linklist_tcpconn_t tcpconn = {"cacofonix"};
char mole_dir[128] = "/data/mole";
char symdir_name[128] = "/data/etc/mole.lnk";
char symraw_name[128] = "/data/rawdir/LIVE";

int main(int argc, char *argv[]) {
  // mode selection
  int server_mode = 0;
  int client_mode = 1;
  unsigned int flags = TCPCONN_FILE_RAW | TCPCONN_RESOLVE_NAME;
  unsigned int rewind = 20;
  unsigned int ll_flags = LL_USE_BIG_ENDIAN; // this is the default for telemetry
  int bin_backup = 0;

  // configure the TCP connection
  tcpconn.flag |= TCPCONN_LOOP;

  // initialization variables
  uint32_t req_serial = 0;
  unsigned int req_framenum = 0;
  unsigned int req_init_framenum = 0;

  // received data variables
  uint8_t * recv_buffer = NULL;
  uint8_t recv_header[TCP_PACKET_HEADER_SIZE] = {0};
  unsigned int buffer_size = 0;
  int64_t recv_framenum = 0;
  uint16_t recv_flags = 0;
  int resync = 1;

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
    } else if (strcmp(argv[i], "-nc") == 0) { // no client mode
      client_mode = 0;
    } else if (strcmp(argv[i], "-w") == 0) { // rewind value 
      rewind = atoi(argv[++i]);
    } else if (strcmp(argv[i], "-v") == 0) { // verbose mode
      ll_flags |= LL_VERBOSE;
    } else if (strcmp(argv[i], "--backup") == 0) { // write binary backup files
      bin_backup = 1;
    } else if (strcmp(argv[i], "--archive-dir") == 0) { // set the archive directory
      strcpy(archive_dir, argv[++i]);
    } else if (strcmp(argv[i], "--mole-dir") == 0) { // set the mole directory dirfiles
      strcpy(mole_dir, argv[++i]);
    } else if (strcmp(argv[i], "--no-backup") == 0) { // don't write binary backup files
      bin_backup = 0;
    } else if (strcmp(argv[i], "--no-check") == 0) { // no checksum 
      ll_flags |= LL_IGNORE_CHECKSUM;
    } else if (strcmp(argv[i], "--little-end") == 0) { // force little endian
      ll_flags &= ~LL_USE_BIG_ENDIAN;
    } else if (strcmp(argv[i], "--loopback") == 0) { // loopback mode
      bin_backup = 0;
      strcpy(tcpconn.ip, "localhost");
      server_mode = 1;
    } else if (strcmp(argv[i], "--help") == 0) { // view usage
      USAGE();
    } else {
      printf("Unrecognized option \"%s\"\n", argv[i]);
      USAGE();
    }
  }

  pthread_t server_thread;
  if (server_mode) {
    // start the server to accept clients
    pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL); 
  }

  if (client_mode) {
    char linklistname[64] = {0};
    char selectname[64] = {0};
    char filename[128] = {0};
    user_file_select(&tcpconn, selectname);

    while (1) {
      // display
      print_display(linklistname, recv_framenum);

      // the file on the server has switched, so resync 
      if (resync) {
        // sync with the server and get the initial framenum
				req_serial = sync_with_server(&tcpconn, selectname, linklistname, flags, &superframe, &linklist);
				req_init_framenum = initialize_client_connection(&tcpconn, req_serial);

				printf("Client initialized with serial 0x%.4x and %d frames\n", req_serial, req_init_framenum);

				// open linklist dirfile
				sprintf(filename, "%s/%s", mole_dir, linklistname);
        if (ll_dirfile) close_and_free_linklist_dirfile(ll_dirfile);
				ll_dirfile = open_linklist_dirfile_opt(filename, linklist, ll_flags);
				unlink(symdir_name);
				symlink(filename, symdir_name);  

        // open the linklist rawfile
				sprintf(filename, "%s/%s", archive_dir, linklistname);
        if (ll_rawfile) close_and_free_linklist_rawfile(ll_rawfile);
				ll_rawfile = open_linklist_rawfile_opt(filename, linklist, (bin_backup) ? 0 : LL_RAWFILE_DUMMY);
        if (bin_backup) {
				  create_rawfile_symlinks(ll_rawfile, symraw_name);
        }

				// set the first framenum request
				req_framenum = (req_init_framenum > rewind) ? req_init_framenum-rewind : 0;
				req_framenum = MAX(req_framenum, tell_linklist_rawfile(ll_rawfile)); 

        resync = 0;
        continue; 
      }

      // check size of the buffer and enlarge if necessary
      if (buffer_size < ll_rawfile->framesize) {
        buffer_size = ll_rawfile->framesize;
        recv_buffer = realloc(recv_buffer, buffer_size);
      }
  
      // send data request a rawfile has been opened 
			recv_flags = 0;
			recv_framenum = request_data(&tcpconn, req_framenum, &recv_flags);
			if ((recv_flags & TCPCONN_FILE_RESET) || (recv_framenum < 0)) { 
				linklist_err("Data request failed\n");
				resync = 1;
        continue;
			}

      // there is no data on the server
      if (recv_flags & TCPCONN_NO_DATA) {
        usleep(50000);
        continue;
      } 

      // get the data from the server
			if (retrieve_data(&tcpconn, recv_buffer, ll_rawfile->framesize) < 0) {
				linklist_err("Data retrieve failed\n");
        resync = 1;
        continue;
      }

      /* all flags are cleared at this point */

			// write the dirfile
      if (ll_dirfile) {
			  seek_linklist_dirfile(ll_dirfile, recv_framenum);
			  write_linklist_dirfile(ll_dirfile, recv_buffer);
      }

			// write the rawfile
      if (bin_backup && ll_rawfile) {
			  seek_linklist_rawfile(ll_rawfile, recv_framenum);
			  write_linklist_rawfile(ll_rawfile, recv_buffer);
      }

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






