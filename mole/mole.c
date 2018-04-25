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

// opens a dialog to select file by name from the server
void user_file_select(linklist_tcpconn_t * tc, char *linklistname)
{
  char name[1024][64] = {{0}};
  int numlink = request_server_archive_list(tc,name);

  int i,j;

  printf("\nSelect archive file:\n\n");

  int n = numlink/3;
  int width = 0;
  for (i=0;i<n;i++) if (strlen(name[i]) > width) width = strlen(name[i]);
  width += 6;

  for (i=0;i<n;i++) {
    if (name[i][0]) printf("%.2d: %s",i,name[i]);
    for (j = strlen(name[i])+4; j < 32; j++) printf(" ");
    if (name[i+n][0]) printf("%.2d: %s",i+n,name[i+n]);
    for (j = strlen(name[i+n])+4; j < 32; j++) printf(" ");
    if (name[i+n+n][0]) printf("%.2d: %s",i+n+n,name[i+n+n]);
    printf("\n");
  }

  while (1) {
    char ta[10];
    printf("\nFile number: ");
    fscanf(stdin,"%s",ta);
    int cn = atoi(ta);
    if ((cn >= 0) && (cn < numlink)) {
      strcpy(linklistname, name[cn]);
      break;
    }
    printf("\nInvalid selection\n");
  }

  printf("Archive file \"%s\" selected\n",linklistname);
}

int main(int argc, char *argv[]) {
  // start the server to accept clients
  pthread_t server_thread;
  pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL); 

  sprintf(tcpconn.ip, "cacofonix");
  char linklistname[64] = {0};
  user_file_select(&tcpconn, linklistname);
}






