/* -----------------------------------------------------------------------
 * -------------------------- TCP Communications -------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL).
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This is TCP network setup and configuration for communications with
 * ground station links. Contained are the functions used to transfer
 * frame files over the network along with the format files and
 * linklists required to parse the received data. The main programs that
 * use this code are mole and bittlm.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: August 29, 2017
 *
 * Copyright 2017 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
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

#include "linklist.h"
#include "linklist_connect.h"

#define DATAETC "/data/etc"
#define MAX_CONNECT_TRIES 5

#ifndef MIN 
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifdef __cplusplus
extern "C" {
#endif

char archive_dir[128] = "/data/rawdir";
uint16_t theday = 0;
uint16_t themonth = 0;
uint16_t theyear = 0;

/* packet header (12 bytes)
 * ----------------
 * [0-3] = unique sender recver serial
 * [4-7] = frame number
 * [8-9] = packet number
 * [10-11] = total number of packets
 */

void writeTCPHeader(uint8_t * header, uint32_t serial, uint32_t frame_num, uint16_t i_pkt, uint16_t n_packets) {
  int j;

  // build header
  *((uint32_t *) (header+0)) = serial;
  *((uint32_t *) (header+4)) = frame_num;
  *((uint16_t *) (header+8)) = i_pkt;
  *((uint16_t *) (header+10)) = n_packets;
}

void readTCPHeader(uint8_t * header, uint32_t **ser, uint32_t **frame_num, uint16_t **i_pkt, uint16_t **n_pkt) {
  int j;

  // extract header
  *ser = (uint32_t *) (header+0);
  *frame_num = (uint32_t *) (header+4);
  *i_pkt = (uint16_t *) (header+8);
  *n_pkt = (uint16_t *) (header+10);
}

static int one (const struct dirent *unused)
{
  if (unused) return 1;
  return 1;
}

void *connection_handler(void *arg)
{
  //Get the socket descriptor
  int sock = *((int *) arg);
  *(int *) arg = -1; // unlock server thread so new clients can be accepted

  struct TCPCONN tc = {{0}};
  tc.fd = sock;

  int read_size;
  char client_message[TCP_PACKET_HEADER_SIZE];
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};
  uint32_t *req_frame_num;
  uint32_t *req_serial;
  uint16_t *req_i;
  uint16_t *req_n;
  uint8_t *buffer = NULL;
  unsigned int buffersize = 0;

  int client_on = 1;
  unsigned int writesize = 0;
  unsigned int frame_lag = 0; 

  FILE * clientbufferfile = NULL;
  char linklist_name[128] = {0};

  char archive_filename[128] = {0};
  char archive_format_filename[128] = {0};
  linklist_t archive_ll = {0};
  unsigned int archive_framenum = 0;

	struct dirent **dir;

  printf("::SERVER:: Accept CLIENT %d\n",sock);

  while (client_on) {
    // recv header request from client
    read_size = recv(sock, client_message, TCP_PACKET_HEADER_SIZE, 0);

    if (read_size <= 0) {
      printf("::CLIENT %d:: connection dropped\n",sock);
      client_on = 0;
      break;
    }
    readTCPHeader((uint8_t *) client_message, &req_serial, &req_frame_num, &req_i, &req_n);

    // process type of request
    if (*req_serial == SERVER_LL_REQ) { // client requesting superframe/linklist format file to be sent
      // recv name: *req_frame_num is number of characters
      char req_name[64] = {0};
      read_size = recv(sock, req_name, MIN(*req_frame_num, 63), 0);
    
      if (read_size <= 0) {
        printf("::CLIENT %d:: unable to receive linklist name\n",sock);
        client_on = 0;
        break;
      }

			printf("::CLIENT %d:: request for file transfer\n", sock);        

			// strip name of slashes
			int i;
			for (i = strlen(req_name)-1; i >= 0; i--) {
				if (req_name[i] == '/') {
					req_name[i] = 0;
					break;
				}
			}
			i++;

			// build filename in data backup directory
			char filename[128] = {0};
			sprintf(filename, "%s/%s", archive_dir, req_name+i);
			if (send_client_file(&tc, filename, SERVER_ARCHIVE_REQ) < 0) {
				client_on = 0;
				break;
			}
    } else if (*req_serial == SERVER_ARCHIVE_LIST_REQ) { // client requesting list of archived files
      printf("::CLIENT %d:: request for archive file list\n", sock);

      int n = scandir(archive_dir, &dir, one, alphasort);
      int i;
      int ifile = 0;
      int nfile = 0;

      if (n < 0) {
        printf("::CLIENT %d:: unable to open archive directory\n", sock);
        client_on = 0;
        break;
      }

      // load archive names
      for (i = 0; i < n; i++) {
        int pos;
        for (pos = 0; pos < strlen(dir[i]->d_name); pos++) {
          if (dir[i]->d_name[pos] == '.') break;
        }

        // look for linklist binary files
        if (strncmp(dir[i]->d_name+pos, LINKLIST_EXT, strlen(LINKLIST_EXT)) == 0) {
          char tempc[128];
          dir[i]->d_name[pos] = 0; // clear the suffix

          // check if corresponding superframe format and linklist format files exist
          sprintf(tempc, "%s/%s" LINKLIST_FORMAT_EXT, archive_dir, dir[i]->d_name);
          if (access(tempc, F_OK) != 0) {
            dir[i]->d_name[0] = 0;
            continue;
          }
          sprintf(tempc,"%s/%s" SUPERFRAME_FORMAT_EXT, archive_dir, dir[i]->d_name);
          if (access(tempc,F_OK) != 0) {
            dir[i]->d_name[0] = 0;
            continue;
          }

          // strip the forward slashes          
          int j;
          for (j=strlen(dir[i]->d_name)-1;j>=0;j--) {
            if (dir[i]->d_name[j] == '/') break;
          }
          strcpy(tempc,dir[i]->d_name+j+1); // copy the abbr. name
          strcpy(dir[i]->d_name,tempc); // copy it back          
 
          nfile++;
        } else {
          dir[i]->d_name[0] = 0;
        }
      }

      ifile = 0;
      for (i = 0; i < n; i++) {
        if (strlen(dir[i]->d_name) <= 0) continue;

        // respond with header for list transfer
        // format: SERVER_ARCHIVE_LIST_REQ, num chars, file index, total # of files
        writeTCPHeader(header, SERVER_ARCHIVE_LIST_REQ, strlen(dir[i]->d_name)+1, ifile, nfile);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
          printf("::CLIENT %d:: unable to send archive name %s\n", sock, dir[i]->d_name);
          client_on = 0;
        }
        // send name
        if (send(sock, dir[i]->d_name, strlen(dir[i]->d_name)+1, 0) <= 0) {
          printf("::CLIENT %d:: unable to send archive name %s\n", sock, dir[i]->d_name);
          client_on = 0;
        }
        ifile++;
      }

    } else if (*req_serial == SERVER_SET_LL_NAME_REQ) { // client wants to set the linklist name
      // recv name: *req_frame_num is number of characters
      char req_name[64] = {0};
      read_size = recv(sock, req_name, MIN(*req_frame_num, 63), 0);

      if (read_size <= 0) {
        printf("::CLIENT %d:: unable to receive and set linklist name\n", sock);
        client_on = 0;
        break;
      }

      // strip name of slashes
      int j;
      for (j = strlen(req_name)-1; j >= 0; j--) {
        if (req_name[j] == '/') { 
          req_name[j] = 0;
          break;
        }
      }
      j++;

      // this name should include the linklist binary file extension (LINKLIST_EXT)
      strcpy(linklist_name, req_name+j);
      printf("::CLIENT %d:: linklist name set to \"%s\"\n", sock, linklist_name);
    } else if (*req_serial == SERVER_ARCHIVE_REQ) { // client requesting archived data
      if (archive_framenum == 0) { // assume archive file has not yet been loaded
        // check if linklist_name has been loaded from format file and linklist file request
        if (strlen(linklist_name) == 0) {
          printf("::CLIENT %d:: no archive file requested\n", sock);
          client_on = 0;
          break;
        }

        // write data filename for archive file
        sprintf(archive_filename,"%s/%s", archive_dir, linklist_name);

        // get the extensionless name
        int j;
        for (j = 0; j < strlen(linklist_name); j++) {
          if (linklist_name[j] == '.') break;
        }
        // create path to the linklist format file
        strncpy(archive_format_filename, linklist_name, j);
        strcat(archive_format_filename, LINKLIST_FORMAT_EXT);

        // load dummy linklist that corresponds to archive file
        int formatfile_blksize = read_linklist_formatfile_size(archive_filename);
        if (formatfile_blksize < 0) {
          printf("::CLIENT %d:: unable to find file \"%s\"\n", sock, archive_filename);
          client_on = 0;
          break;
        }
        archive_ll.blk_size = formatfile_blksize;
        *((uint32_t *) archive_ll.serial) = SERVER_ARCHIVE_REQ;

        // get framenumber from the file
        if (clientbufferfile) fclose(clientbufferfile);
        clientbufferfile = NULL;
        if ((clientbufferfile = fopen(archive_filename, "rb")) == NULL) {
          printf("::CLIENT %d:: unable to open archive file \"%s\"\n", sock, archive_filename);
          client_on = 0;
          break;
        }
        fseek(clientbufferfile, 0, SEEK_END);
      }

      // update the archived file framenumber
      archive_framenum = MAX(ftell(clientbufferfile)/archive_ll.blk_size-1,0);
      writesize = archive_ll.blk_size;

      // open data file if not yet opened
      if (!clientbufferfile) {
        clientbufferfile = fopen(archive_filename,"r");    
        printf("::CLIENT %d:: opening file \"%s\"\n", sock, archive_filename);
      }
      if (!clientbufferfile) {
        printf("::CLIENT %d:: bufferfile %s unreadable\n", sock, archive_filename);
        client_on = 0;
        break;
      }

      // handle the type of data block request 
      if (*req_frame_num == 0) { // requesting a frame number initialization
        // initialize variables for live data block transfer
        printf("::CLIENT %d:: request for initialization\n",sock);

        // respond with header for live data
        // format: serial, initialization framenm, day, year*12+month
        writeTCPHeader(header, *((uint32_t *) archive_ll.serial), archive_framenum-frame_lag, theday, theyear*12+themonth);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
          printf("::CLIENT %d:: unable to send initialization message\n",sock);
          client_on = 0;
        }
        memset(header,0,TCP_PACKET_HEADER_SIZE);

        printf("::CLIENT %d:: initialized with serial 0x%x\n", sock, *((uint32_t *) archive_ll.serial));

      } else { // requesting data block
        // wait for the requested frame num to become available
        if ((archive_framenum-((int) frame_lag)) < (*((int*) req_frame_num))) {
          continue;
        } 

				// read the file
				if (fseek(clientbufferfile, (*req_frame_num)*writesize, SEEK_SET) != 0) {
					printf("::CLIENT %d:: fseek failed\n", sock);
					client_on = 0;
					break;
				} 
        if (buffersize < writesize) {
          if (!(buffer = realloc(buffer, writesize))) {
            printf("::CLIENt %d:: cannot allocate buffer\n", sock);
            client_on = 0; 
            break;
          }
          buffersize = writesize;
        }
				fread(buffer, 1, writesize, clientbufferfile);

        // respond with header for live data
        // format: serial, frame number, day, year*12+month
        writeTCPHeader(header, *((uint32_t *) archive_ll.serial), *req_frame_num, theday, theyear*12+themonth);
        if (send(sock, header, TCP_PACKET_HEADER_SIZE, MSG_MORE) <= 0) {
          printf("::CLIENT %d:: unable to send header\n", sock);
          client_on = 0;
          break;
        }
        // send the data 
        if (send(sock, buffer, writesize, 0) <= 0) {
          printf("::CLIENT %d:: unable to send data\n",sock);
          break;
        }
        memset(buffer, 0, buffersize);
      }
    }
  }

  printf("::SERVER:: closed connection to CLIENT %d\n",sock);

  // cleanup
  if (clientbufferfile) 
  {
    fclose(clientbufferfile);
    clientbufferfile = NULL;
  }
  if (buffer) free(buffer);
  close(sock);

  return 0;
}

void server_thread(void * arg)
{
  int client_sock, c;
  struct sockaddr_in server , client;

  // arg points to the port number
  int theport = *(int *) arg;

  //Create socket
  int socket_desc = socket(AF_INET , SOCK_STREAM , 0);
  if (socket_desc == -1) {
    perror("socket could not create server socket");
    exit(4);
  }

  //Prepare the sockaddr_in structure
  server.sin_family = AF_INET;
  server.sin_addr.s_addr = INADDR_ANY;
  server.sin_port = htons(theport);

  int tru = 1;
  setsockopt(socket_desc, SOL_SOCKET, SO_REUSEADDR, &tru, sizeof(int));

  //Bind
  if (bind(socket_desc, (struct sockaddr *) &server , sizeof(server)) < 0) {
    //print the error message
    perror("bind failed. Error");
    return;
  }

  //Listen
  listen(socket_desc , 3);


  //Accept and incoming connection
  printf("::SERVER:: Waiting for incoming connections...\n");
  c = sizeof(struct sockaddr_in);
  pthread_t thread_id;

  while ((client_sock = accept(socket_desc, (struct sockaddr *)&client, (socklen_t*)&c))) {
    if (client_sock < 0) {
      printf("Accept client failed\n");
    }
  
    if (pthread_create(&thread_id, NULL, connection_handler, (void*) &client_sock) < 0) {
      perror("Could not create client thread\n");
    }
    
    // prevent race condition with fast accept of multple clients
    while (client_sock != -1) usleep(10000);
  }
}

// blocks until establishes a connection with the host at tc->ip
// return the file descriptor, which is also at tc->fd
int connect_tcp(struct TCPCONN * tc)
{
  // start tcp client 
  struct sockaddr_in server_info;
  struct hostent *he;
  int socket_fd = 0;
  int numtries = 0;

  int connected = 0;
  while (!connected) {
    if ((he = gethostbyname(tc->ip))==NULL) {
      printf("Cannot get the host name \"%s\"\n",tc->ip);
      numtries = MAX_CONNECT_TRIES;
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return socket_fd;
      }
      goto RETRY;
    }

    if (!(tc->flag & TCPCONN_NOLOOP)) printf("Connecting to %s...\n",tc->ip);
    connected = 1; 
    if ((socket_fd = socket(AF_INET, SOCK_STREAM, 0)) == -1) {
      fprintf(stderr, "Socket Failure!!\n");
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return 0;
      }
      goto RETRY;
    }

    int tru = 1;
    setsockopt(socket_fd,SOL_SOCKET,SO_REUSEADDR,&tru,sizeof(int));

    memset(&server_info, 0, sizeof(server_info));
    server_info.sin_family = AF_INET;
    server_info.sin_port = htons(CLIENT_TELEM_PORT);
    server_info.sin_addr = *((struct in_addr *)he->h_addr);

    if (connect(socket_fd, (struct sockaddr *)&server_info, sizeof(struct sockaddr))<0) {
      connected = 0;
      close(socket_fd);
      socket_fd = 0;
      if (tc->flag & TCPCONN_NOLOOP) { // non blocking mode
        return socket_fd;
      }
      printf("Connection refused...trying again\n");
      goto RETRY;
    }
    break;

    RETRY : 
    sleep(2);
    numtries++;

    // try a new host if not in multi mole mode
    if ((numtries >= MAX_CONNECT_TRIES) && !(tc->flag & TCPCONN_LOOP)) {
      printf("\nTry a different host: ");
      fscanf(stdin,"%s",tc->ip);
      if (tc->ip[strlen(tc->ip)-1] == '\n') tc->ip[strlen(tc->ip)-1] = 0;
      numtries = 0;
    }
  }

  tc->fd = socket_fd; // set the file desc

  return socket_fd;
}

// closes the tcp connection and sets the file descriptor to 0
int close_connection(struct TCPCONN * tc)
{
  if (tc->fd != 0) close(tc->fd);
  tc->fd = 0;
  return tc->fd;
}

void send_client_error(struct TCPCONN * tc)
{
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};

  writeTCPHeader(header,0x1badfeed, 0, 0, 1);
  send(tc->fd, header, TCP_PACKET_HEADER_SIZE, 0);
}

// send a file to the client
int send_client_file(struct TCPCONN * tc, char * filename, uint32_t serial)
{
  uint8_t header[TCP_PACKET_HEADER_SIZE] = {0};
  uint8_t buffer[1024] = {0};  

  // open the requested file
  FILE * req_file = fopen(filename, "r");
  if (req_file == 0) {
    printf("::CLIENT %d:: could not open file \"%s\"\n",tc->fd,filename);
    send_client_error(tc);
    return 0;
  }

  // get file size
  fseek(req_file,0,SEEK_END);
  unsigned int req_filesize = ftell(req_file);
  fseek(req_file, 0, SEEK_SET);

  // respond with header for file transfer
  // format: SERVER_LL_REQ, total file size [bytes], 4 byte linklist serial
  writeTCPHeader(header, SERVER_LL_REQ, req_filesize, (serial & 0xffff), (serial & 0xffff0000)>>16);
  if (send(tc->fd, header, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
    printf("::CLIENT %d:: unable to send file transfer initialization message\n", tc->fd);
    return -1;
  }
  memset(header, 0, TCP_PACKET_HEADER_SIZE);

  // send raw linklist file
  unsigned int bytes_transferred = 0;
  unsigned int loc = 0, col = 0;
  while (bytes_transferred < req_filesize) {
    loc = MIN(1024, req_filesize-bytes_transferred);
    col = fread(buffer, 1, loc, req_file);
    if (loc != col) {
      printf("::CLIENT %d:: file read error\n", tc->fd);
      return 0;
    } else {
      send(tc->fd, buffer, col, 0);
      bytes_transferred += col;
      memset(buffer, 0, col);
    }

  }

  fclose(req_file);

  printf("::CLIENT %d:: file transfer complete (%d bytes)\n", tc->fd, req_filesize);

  return 1;
}


// requests a file from the server
// file is saved at DATAETC/filename
uint32_t request_server_file(struct TCPCONN * tc, char * filename, unsigned int flags)
{
  int sock = tc->fd;
  uint8_t buffer[1024] = {0};
  uint8_t msg[TCP_PACKET_HEADER_SIZE] = {0};

  uint32_t *recv_ser, *recv_filesize;
  uint16_t *recv_i, *recv_n;
  uint32_t ll_serial;

  char savedname[256] = {0};
  sprintf(savedname,"%s/%s",DATAETC,filename);

  //printf("Requesting file \"%s\" from server...\n",filename);

  // send request for file by name
  writeTCPHeader(buffer,SERVER_LL_REQ,strlen(filename),flags,0);
  if (send(sock,buffer,TCP_PACKET_HEADER_SIZE,0) <= 0) {
    printf("request_server_file: failed to send file request message\n");
    return 0;
  }
  if (send(sock,filename,strlen(filename),0) <= 0) {
    printf("request_server_file: failed to send filename\n");
    return 0;
  }

  // get file info before receiving
  if (recv(sock, msg, TCP_PACKET_HEADER_SIZE, MSG_WAITALL) <= 0) {
    printf("request_server_file: failed to recv file stats\n");
    return 0;
  }
  readTCPHeader(msg, &recv_ser, &recv_filesize, &recv_i, &recv_n);
  ll_serial = ((*recv_n)<<16) + (*recv_i);

  if (*recv_ser != SERVER_LL_REQ) { // handle error messages
    printf("request_server_file: file \"%s\" not on server\n",filename);
    return *recv_ser;
  }
  //printf("0x%x 0x%x %d\n",*recv_ser, ll_serial, *recv_filesize);

  // receive the file and write it to disk
  FILE * f = fopen(savedname,"wb");

  unsigned int bytes_recvd = 0;
  unsigned int loc = 0;
  while (bytes_recvd < *recv_filesize) {
    loc = MIN(1024,(*recv_filesize)-bytes_recvd);
    if (recv(sock, buffer, loc, MSG_WAITALL) <= 0) {
      printf("request_server_file: failed to recv file block\n");
      return 0;
    }
    fwrite(buffer,1,loc,f);
    bytes_recvd += loc;
    memset(buffer,0,loc);
  }
  fflush(f);
  fclose(f);

  //printf("Received file \"%s\" from server (size %d)\n",filename,*recv_filesize);

  return ll_serial;
}

// initialize server connection with time and framenum info based on a linklist serial
unsigned int initialize_connection(struct TCPCONN * tc, uint32_t serial)
{
  // initialize connection if not done so already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint32_t * recv_frame_num = 0;
  uint16_t * recv_i = 0, * recv_n = 0;
  uint32_t * recv_ser;
  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // initialization with serial
  writeTCPHeader(request_msg,serial,0,0,0);
  if (send(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,0) <= 0) {
    printf("Failed to send initialization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }

  if (recv(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
    printf("Failed to receive initalization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }
  readTCPHeader(request_msg,&recv_ser,&recv_frame_num,&recv_i,&recv_n);
  
  tc->theday = *recv_i;
  tc->themonth = ((*recv_n-1)%12)+1;
  tc->theyear = (*recv_n-1)/12;
  
	tc->serial = *recv_ser;

  return *recv_frame_num;
}


// manually sets the linklist name used by the server
void set_server_linklist_name(struct TCPCONN * tc, char *linklistname)
{
  // initiate server connection if not done already
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeTCPHeader(request_msg,SERVER_SET_LL_NAME_REQ,strlen(linklistname)+1,0,0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
    printf("Failed to send link name select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }
  if (send(tc->fd, linklistname, strlen(linklistname)+1, 0) <= 0)
  {
    printf("Failed to send link name\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

}

// fills the name array with archived linklist names from the server
int request_server_archive_list(struct TCPCONN * tc, char name[][64])
{
  // initiate server connection if not done alread
  while (tc->fd <= 0) {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeTCPHeader(request_msg,SERVER_ARCHIVE_LIST_REQ,0,0,0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
    printf("Failed to send archive select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  uint16_t *recv_i, *recv_n;
  uint32_t *recv_ser, *recv_fn;

  while (1) {
    // receive name header
    while (recv(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
      printf("Failed to receive archive name header\n");
      close_connection(tc);
      return 0;
    }
    readTCPHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

    // receive name
    if (recv(tc->fd,name[*recv_i],*recv_fn,MSG_WAITALL) <= 0) {
      printf("Failed to receive archive name\n");
      close_connection(tc);
      return 0;
    }

    if (*recv_n  == ((*recv_i)+1)) break;

  }
  return *recv_n;

}

// fills the name array with linklist names from the server
int request_server_list(struct TCPCONN * tc, char name[][64]) {
  // initiate server connection if not done already
  while (tc->fd <= 0)
  {
    tc->fd = connect_tcp(tc);
    sleep(1);
  }

  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeTCPHeader(request_msg,SERVER_LL_LIST_REQ,0,0,0);
  if (send(tc->fd, request_msg, TCP_PACKET_HEADER_SIZE, 0) <= 0) {
    printf("Failed to send link select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  uint16_t *recv_i, *recv_n;
  uint32_t *recv_ser, *recv_fn;


  while (1)
  {
    // receive name header
    if (recv(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
      printf("Failed to receive linklist name header\n");
      close_connection(tc);
      return 0;
    }
    readTCPHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

    // receive name
    if (recv(tc->fd,name[*recv_i],*recv_fn,0) <= 0) {
      printf("Failed to receive linklist name\n");
      close_connection(tc);
      return 0;
    }  

    if (*recv_n  == ((*recv_i)+1)) break;

  }
  return *recv_n;
}

// retrieves data from the server and writes it to the specified buffer of size bufsize
// data is requested based on serial number and framenumber
// returns the number of retrieved bytes, including header info
int retrieve_data(struct TCPCONN * tc, uint64_t fn, unsigned int bufsize, uint8_t * buffer)
{
  uint8_t request_msg[TCP_PACKET_HEADER_SIZE] = {0};	
  int rsize = 0;

  // request the next frame
  writeTCPHeader(request_msg,tc->serial,fn,0,0);
  if (send(tc->fd,request_msg,TCP_PACKET_HEADER_SIZE,0) <= 0) {
    printf("Server connection lost on send.\n");
    return -1; // connection error
  }

  // receive the next frame
  if ((rsize = recv(tc->fd,buffer,bufsize,MSG_WAITALL)) <= 0) {
    printf("Server connection lost on recv.\n");
    return -1; // connection error
  }

	return rsize;
}

#ifdef __cplusplus
}
#endif
