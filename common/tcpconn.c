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

#include "bitserver.h"
#include "../bit_config/address.h"
#include "comms.h"
#include "tcpconn.h"

#define DATAETC "/data/etc"
#define MAX_CONNECT_TRIES 5

#ifndef MIN 
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif

#ifdef __cplusplus
extern "C" {
#endif

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
  uint8_t header[PACKET_HEADER_SIZE] = {0};

  writeHeader(header,0x1badfeed, 0, 0, 1);
  send(tc->fd, header, PACKET_HEADER_SIZE, 0);
}

// send a file to the client
int send_client_file(struct TCPCONN * tc, char * filename, uint32_t serial)
{
  uint8_t header[PACKET_HEADER_SIZE] = {0};
  uint8_t buffer[1024] = {0};  

  // open the requested file
  FILE * req_file = fopen(filename,"r");
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
  writeHeader(header, SERVER_LL_REQ, req_filesize, (serial & 0xffff), (serial & 0xffff0000)>>16);
  if (send(tc->fd, header, PACKET_HEADER_SIZE, 0) <= 0) {
    printf("::CLIENT %d:: unable to send file transfer initialization message\n", tc->fd);
    return -1;
  }
  memset(header, 0, PACKET_HEADER_SIZE);

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
  uint8_t msg[PACKET_HEADER_SIZE] = {0};

  uint32_t *recv_ser, *recv_filesize;
  uint16_t *recv_i, *recv_n;
  uint32_t ll_serial;

  char savedname[256] = {0};
  sprintf(savedname,"%s/%s",DATAETC,filename);

  //printf("Requesting file \"%s\" from server...\n",filename);

  // send request for file by name
  writeHeader(buffer,SERVER_LL_REQ,strlen(filename),flags,0);
  if (send(sock,buffer,PACKET_HEADER_SIZE,0) <= 0) {
    printf("request_server_file: failed to send file request message\n");
    return 0;
  }
  if (send(sock,filename,strlen(filename),0) <= 0) {
    printf("request_server_file: failed to send filename\n");
    return 0;
  }

  // get file info before receiving
  if (recv(sock, msg, PACKET_HEADER_SIZE, MSG_WAITALL) <= 0) {
    printf("request_server_file: failed to recv file stats\n");
    return 0;
  }
  readHeader(msg, &recv_ser, &recv_filesize, &recv_i, &recv_n);
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
  uint8_t request_msg[PACKET_HEADER_SIZE] = {0};

  // initialization with serial
  writeHeader(request_msg,serial,0,0,0);
  if (send(tc->fd,request_msg,PACKET_HEADER_SIZE,0) <= 0) {
    printf("Failed to send initialization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }

  if (recv(tc->fd,request_msg,PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
    printf("Failed to receive initalization message\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
    return -1;
  }
  readHeader(request_msg,&recv_ser,&recv_frame_num,&recv_i,&recv_n);
  
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

  uint8_t request_msg[PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeHeader(request_msg,SERVER_SET_LL_NAME_REQ,strlen(linklistname)+1,0,0);
  if (send(tc->fd, request_msg, PACKET_HEADER_SIZE, 0) <= 0) {
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

  uint8_t request_msg[PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeHeader(request_msg,SERVER_ARCHIVE_LIST_REQ,0,0,0);
  if (send(tc->fd, request_msg, PACKET_HEADER_SIZE, 0) <= 0) {
    printf("Failed to send archive select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  uint16_t *recv_i, *recv_n;
  uint32_t *recv_ser, *recv_fn;

  while (1)
  {
    // receive name header
    while (recv(tc->fd,request_msg,PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
      printf("Failed to receive archive name header\n");
      close_connection(tc);
      return 0;
    }
    readHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

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

  uint8_t request_msg[PACKET_HEADER_SIZE] = {0};

  // request file list from server
  writeHeader(request_msg,SERVER_LL_LIST_REQ,0,0,0);
  if (send(tc->fd, request_msg, PACKET_HEADER_SIZE, 0) <= 0) {
    printf("Failed to send link select request\n");
    close_connection(tc);
    tc->fd = connect_tcp(tc);
  }

  uint16_t *recv_i, *recv_n;
  uint32_t *recv_ser, *recv_fn;


  while (1)
  {
    // receive name header
    if (recv(tc->fd,request_msg,PACKET_HEADER_SIZE,MSG_WAITALL) <= 0) {
      printf("Failed to receive linklist name header\n");
      close_connection(tc);
      return 0;
    }
    readHeader(request_msg,&recv_ser,&recv_fn,&recv_i,&recv_n);

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
  uint8_t request_msg[PACKET_HEADER_SIZE] = {0};	
  int rsize = 0;

  // request the next frame
  writeHeader(request_msg,tc->serial,fn,0,0);
  if (send(tc->fd,request_msg,PACKET_HEADER_SIZE,0) <= 0) {
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
