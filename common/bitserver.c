/* ---------------------------------------------------------------------
 * ----------------------------- BITSERVER -----------------------------
 * ---------------------------------------------------------------------
 *
 * Copyright 2014 Javier Romualdez
 *
 * This program is distributed under the GNU General Public License (GPL)
 * Version 2 or higher.
 *
 * ------------------------- Description -------------------------------
 * The functions here enable UDP communication between various network
 * machines.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: May 1, 2014
 *
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h>
#include <sys/types.h>
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <unistd.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <netdb.h>
#include <pthread.h> // threads

#ifdef __cplusplus
extern "C" {
#endif

#include "FIFO.h"
#include "CRC_func.h"
#include "bitserver.h"
#include "blast.h"

/* packet header (12 bytes)
 * ----------------
 * [0-3] = unique sender recver serial
 * [4-7] = frame number
 * [8-9] = packet number
 * [10-11] = total number of packets
 */
int verbosity = 0;

uint16_t writeHeader(uint8_t * header, uint32_t serial, uint32_t frame_num, uint16_t i_pkt, uint16_t n_packets) {
  // allocate crc table if necessary
  if (crctable == NULL) {
    if ((crctable = mk_crctable((uint16_t) CRC_POLY, crchware)) == NULL) {
      blast_fatal("mk_crctable() memory allocation failed");
    }
  }

  int j;
  uint16_t checksum = 0;

  // build header
  *((uint32_t *) (header+0)) = serial;
  *((uint32_t *) (header+4)) = frame_num;
  *((uint16_t *) (header+8)) = i_pkt;
  *((uint16_t *) (header+10)) = n_packets;

  for (j = 0; j < PACKET_HEADER_SIZE; j++) crccheck(header[j], &checksum, crctable); // check the checksum

  return checksum;
}

uint16_t readHeader(uint8_t * header, uint32_t **ser, uint32_t **frame_num, uint16_t **i_pkt, uint16_t **n_pkt) {
  // allocate crc table if necessary
  if (crctable == NULL) {
    if ((crctable = mk_crctable((uint16_t) CRC_POLY, crchware)) == NULL) {
      blast_fatal("mk_crctable() memory allocation failed");
    }
  }

  int j;
  uint16_t checksum = 0;

  // extract header
  *ser = (uint32_t *) (header+0);
  *frame_num = (uint32_t *) (header+4);
  *i_pkt = (uint16_t *) (header+8);
  *n_pkt = (uint16_t *) (header+10);

  for (j = 0; j < PACKET_HEADER_SIZE; j++) crccheck(header[j], &checksum, crctable); // check the checksum

  return checksum;
}

/* -------------------------
 * ---- sendDataThread -----
 * -------------------------
 * 
 * A worker thread function that sends data to a socket from the BITSender
 * FIFO. This function will automatically divide each FIFO element into
 * packets of the desired maximum packet size (see initBITSender) and send
 * them sequentially over UDP with the appropriate headers for reassembly
 * on the receiving end.
 * 
 * Parameters:
 * --------------------------
 * arg :   A void pointer to the BITSender struct that this thread is 
 *       attached to.
 * 
*/
void * sendDataThread(void *arg) {
  struct BITSender *server = (struct BITSender *) arg;
  unsigned int start, size, n_packets, packet_size, packet_maxsize;
  unsigned int i;
  uint8_t *header;
  uint32_t serial, frame_num;
  uint8_t *buffer;
  uint8_t flags;

  header = (uint8_t *) calloc(PACKET_HEADER_SIZE+server->packet_maxsize, 1);

  while (1) {
    if (!fifoIsEmpty(server->send_fifo)) { // something in the buffer
      serial = server->serial;
      start = server->send_fifo->start;
      size = server->send_fifo->size[start];
      flags = server->send_fifo->flags[start];
      frame_num = server->send_fifo->frame_num[start];
      packet_maxsize = server->packet_maxsize;
      buffer = getFifoRead(server->send_fifo);
      n_packets = (size-1)/packet_maxsize+1;

      if (flags & NO_HEADER) { // send packet with header
        if (n_packets > 1) {
          blast_err("cannot send headerless multi-packet message.");
        } else {
          // printf("Sending headerless packet\n");
          if (sendto(server->sck, buffer, size,
            MSG_NOSIGNAL, (struct sockaddr *) &(server->send_addr),
            server->slen) < 0) {
              blast_err("sendTo failed (errno %d)", errno);
            }
        }
      } else {
        i = 0;
        n_packets = 1;
        packet_size = server->packet_maxsize;
        while (i < n_packets) {
          uint8_t * pkt_buffer = packetizeBuffer(buffer, size, (uint32_t *) &packet_size,
                                                 (uint16_t *) &i, (uint16_t *) &n_packets);
          writeHeader(header, serial, frame_num, i++, n_packets);
          // packet_size = MIN(packet_maxsize, size-(i*packet_maxsize));
#ifdef MSG_MORE // general Linux kernel

          // add header to packet (adds due to MSG_MORE flag)
          if (sendto(server->sck, header, PACKET_HEADER_SIZE,
            MSG_NOSIGNAL | MSG_MORE,
            (struct sockaddr *) &(server->send_addr),
            server->slen) < 0) {
              blast_err("sendTo failed (errno %d)", errno);
            }

          // add data to packet and send
          // if (sendto(server->sck, buffer+(i*packet_maxsize), packet_size,
          if (sendto(server->sck, pkt_buffer, packet_size,
            MSG_NOSIGNAL, (struct sockaddr *) &(server->send_addr),
            server->slen) < 0) {
              blast_err("sendTo failed (errno %d)", errno);
            }

#else // for QNX kernel

          memcpy(header+PACKET_HEADER_SIZE, buffer+(packet_maxsize*i), packet_size);
          // send packet
          if (sendto(server->sck, header, PACKET_HEADER_SIZE+packet_size,
            MSG_NOSIGNAL, (struct sockaddr *) &(server->send_addr),
            server->slen) < 0) blast_err("sendTo failed()");
          // memset(header+PACKET_HEADER_SIZE, 0, packet_size); // clear data

#endif
        // usleep(10000/n_packets); // don't overflow udp sendto buffer
        }
      }
      decrementFifo(server->send_fifo);
    } else { // sleep the FIFO
      usleep(1000); // FIFO is sleeping
    }
  }

return NULL;
}

/* -------------------------
 * ---- recvDataThread -----
 * -------------------------
 * 
 * A worker thread function that receives data from a socket to the BITRecver
 * FIFO. This function will only add packets to the FIFO without reading
 * headers and/or reassembling packetted data. See buildBITRecverData() 
 * for details on reassembling packetted data.
 * 
 * Parameters:
 * --------------------------
 * arg :   A void pointer to the BITRecver struct that this thread is 
 *       attached to.
 * 
*/
void * recvDataThread(void *arg) {
  struct BITRecver * server = (struct BITRecver *) arg;
  unsigned int num_bytes;
  /*
  struct sched_param param;
  int pol = 0;

  pthread_t this_thread = pthread_self();
  pthread_getschedparam(this_thread, &pol, &param);
  param.sched_priority = 0;
  pthread_setschedparam(this_thread, pol, &param);
  */
  while (1) {
    num_bytes = recvfrom(server->sck, getFifoWrite(server->recv_fifo),
      UDPMAXBUFLEN, 0, (struct sockaddr*)&server->recv_addr, &server->slen);

    if (num_bytes <= 0) blast_err("recvfrom() failed!");
    else if (num_bytes > server->packet_maxsize) {
      blast_err("Buffer overflow! Received %d bytes for %d byte buffer\n",
        num_bytes, server->packet_maxsize);
      // exit(2);
    }

    server->recv_fifo->size[server->recv_fifo->end] = num_bytes;
    incrementFifo(server->recv_fifo); // update the fifo
  }
  return NULL;
}


/* -------------------------
 * ----- initBITSender -----
 * -------------------------
 * 
 * This function initializes the BITSender struct, which sets up an
 * outbound address on the specified UDP port and associates a FIFO of a
 * particular size. Note that enough memory will be allocated to accomodate
 * packet headers.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITSender struct to be initialized.
 * 
 * send_addr:    The IP address or hostname for sending data over UDP.
 *           If address 255.255.255.255 is used, the sender broadcasts
 *           to all addresses on the subnet at the specified port.
 *           Note that in this case, my_addr is unset and the socket
 *           is not bound to an address.
 * 
 * port:      The UDP port over which data is sent.
 * 
 * fifo_length:    The number of full size elements in the FIFO []
 * 
 * fifo_maxsize:  The maximum size of a full element in the FIFO [bytes].
 *           Add data to the FIFO by requesting an address with 
 *           getBITSenderAddr().
 *           
 *           When fifo_maxsize=0, the pointers in the FIFO will be
 *           set to NULL, in which case the FIFO will be in an
 *           unallocated state. To add data to to this type of FIFO,
 *           use setBITSenderAddr() (or equivalently setFifoWrite()).
 * 
 * packet_maxsize:  The maximum size of the packet to be sent over UDP.
 *           If the fifo_maxsize > packet_maxsize, each FIFO element
 *           will be broken up into the minimum number of packets 
 *           to be sent over UDP to satisfy the packet_maxsize
 *           requirement.
 *           
*/
int initBITSender(struct BITSender *server, const char *send_addr,
  unsigned int port, unsigned int fifo_length,
  unsigned int fifo_maxsize, unsigned int packet_maxsize) {
  /* ----- BEGIN UDP SOCKET SETUP ------ */
  blast_info("Initializing BITSender:");

  if (packet_maxsize == 0) {
    blast_err("cannot have a zero packet size");
    return -1;
  }
  if (fifo_length < 2) {
    blast_err("a fifo_length<2 will always overflow.");
    return -1;
  }

  int udpbuffersize = UDPMAXBUFLEN;
  struct hostent* the_target;
  server->slen = sizeof(server->send_addr);

  // set up port
  if ((server->sck = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1)  {
    blast_err("socket() unsuccessful");
    return -1;
  }

  if (setsockopt(server->sck, SOL_SOCKET, SO_SNDBUF, &udpbuffersize,
    sizeof(udpbuffersize)) < 0)  {
    blast_err("unable to set socket options.");
    return -1;
  }
  int optval = 1;
  if (setsockopt(server->sck, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1) {
    blast_err("unable to set reusable port");
  }

  if (strcmp(send_addr, "255.255.255.255") == 0) { // broadcaster
    int broadcast = 1;
    if (setsockopt(server->sck, SOL_SOCKET, SO_BROADCAST, &broadcast,
      sizeof(broadcast)) == -1)  {
      perror("setsockopt (SO_BROADCAST)");
      return -1;
    }
    blast_info("-> Broadcast mode");
  } else { // non-broadcast, so bind to local address
    // set up socket address
    bzero(&(server->my_addr), sizeof(server->my_addr));
    server->my_addr.sin_family = AF_INET;
    server->my_addr.sin_port = htons(port);
    server->my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    if (bind(server->sck, (struct sockaddr *) &(server->my_addr),
         sizeof(server->my_addr)) < 0) {
      perror("bind");
      blast_err("Unable to bind to socket (errno=%d)", errno);
    }
  }

  // set up target address
  the_target = gethostbyname(send_addr);
  if (the_target == NULL) {
    blast_err("host lookup %s failed.", send_addr);
    return -1;
  }

  bzero(&(server->send_addr), sizeof(server->send_addr));
  server->send_addr.sin_family = AF_INET;
  server->send_addr.sin_port = htons(port);
  memcpy(&server->send_addr.sin_addr.s_addr, the_target->h_addr,
    the_target->h_length);

  // set up FIFO
  server->send_fifo = (struct Fifo *) calloc(1, sizeof(struct Fifo));

  if (fifo_maxsize != 0) {
    allocFifo(server->send_fifo, fifo_length, fifo_maxsize+PACKET_HEADER_SIZE);
  } else {
    allocFifo(server->send_fifo, fifo_length, 0);
  }
  if (packet_maxsize > fifo_maxsize) {
    blast_err("cannot set packet size larger than frame size");
    return -1;
  }
  server->packet_maxsize = packet_maxsize;
  server->serial = DEFAULT_SERIAL;
  server->frame_num = 0;

  blast_info("-> SendTo %s:%d", send_addr, port);
  pthread_create(&server->send_thread, NULL, sendDataThread, (void *) server);

  return 1;
}

/* -------------------------
 * ----- initBITRecver -----
 * -------------------------
 * 
 * This function initializes the BITRecver struct, which sets up an
 * outbound address on the specified UDP port and associates a FIFO of a
 * particular size. Note that enough memory will be allocated to accomodate
 * packet headers.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITRecievr struct to be initialized.
 * 
 * recv_addr:    The IP address or hostname for receiving data over UDP.
 * 
 * port:      The UDP port over which data is received.
 * 
 * fifo_length:    The number of full size elements in the FIFO []
 * 
 * fifo_maxsize:  The maximum size of a full element in the FIFO [bytes].
 *           Read data from the FIFO by requesting an address with 
 *           getBITRecverAddr().
 * 
 * packet_maxsize:  The maximum size of the packet to be received over UDP.
 *           If the fifo_maxsize > packet_maxsize, the number of 
 *           elements assigned to the FIFO will be the number of 
 *           packets required to send a full element times fifo_length.
 * 
 *           num_packets = (fifo_maxsize-1)/packet_maxsize+1
 *
 *           
*/
int initBITRecver(struct BITRecver *server, const char *recv_addr,
  unsigned int port, unsigned int fifo_length,
  unsigned int fifo_maxsize, unsigned int packet_maxsize) {
  /* ----- BEGIN UDP SOCKET SETUP ------ */
  blast_info("Initializing BITRecver:");

  if (fifo_maxsize == 0) {
    blast_err("cannot initialize an unallocated receiver");
    return -1;
  }
  if (packet_maxsize == 0) {
    blast_err("cannot have a zero packet size");
    return -1;
  }
  if (fifo_length < 1) {
    blast_err("cannot have a FIFO with 0 elements");
    return -1;
  }

  int udpbuffersize = UDPMAXBUFLEN;
  struct hostent* the_target;
  server->slen = sizeof(server->recv_addr);

  // set up port
  if ((server->sck = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP)) == -1) {
    blast_err("socket() unsuccessful");
    return -1;
  }

  if (setsockopt(server->sck, SOL_SOCKET, SO_SNDBUF, &udpbuffersize,
    sizeof(udpbuffersize)) < 0) {
    blast_err("unable to set socket options.");
    return -1;
  }
  int optval = 1;
    if (setsockopt(server->sck, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval)) == -1) {
    blast_err("unable to set reusable port");
  }

  // set up socket address
  bzero(&(server->my_addr), sizeof(server->my_addr));
  server->my_addr.sin_family = AF_INET;
  server->my_addr.sin_port = htons(port);
  server->my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

  if (bind(server->sck, (struct sockaddr *) &(server->my_addr),
    sizeof(server->my_addr)) == -1) {
    blast_err("Bind address already in use");
    return -1;
  }

  // set up target address
  the_target = gethostbyname(recv_addr);
  if (the_target == NULL) {
    blast_err("Host lookup %s failed.", recv_addr);
    return -1;
  }

  bzero(&(server->recv_addr), sizeof(server->recv_addr));
  server->recv_addr.sin_family = AF_INET;
  server->recv_addr.sin_port = htons(port);
  memcpy(&server->recv_addr.sin_addr.s_addr, the_target->h_addr,
    the_target->h_length);

  // set up FIFO
  // need enough space for all packets that constitute a full element
  server->recv_fifo = (struct Fifo *) calloc(1, sizeof(struct Fifo));

  unsigned int num_packets = (fifo_maxsize-1)/packet_maxsize+1;
  allocFifo(server->recv_fifo, fifo_length*num_packets,
    packet_maxsize+PACKET_HEADER_SIZE);
  server->packet_maxsize = packet_maxsize+PACKET_HEADER_SIZE;
  server->serial = DEFAULT_SERIAL;
  server->frame_num = 0;

  blast_info("-> RecvFrom %s:%d", recv_addr, port);
  pthread_create(&server->recv_thread, NULL, recvDataThread, (void *) server);


  return 1;
}

/* -------------------------
 * ---- closeBITSender -----
 * -------------------------
 * 
 * This function closes the BITSender struct and deallocates all memory
 * in the FIFO.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITSender struct to be closed.
 * 
 *           
*/
int closeBITSender(struct BITSender *server) {
  pthread_cancel(server->send_thread);
  usleep(100000);
  close(server->sck);
  freeFifo(server->send_fifo);
  blast_info("Closed BITSender");
  return 1;
}

/* -------------------------
 * ---- closeBITRecver -----
 * -------------------------
 * 
 * This function closes the BITRecver struct and deallocates all memory
 * in the FIFO.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITRecver struct to be closed.
 * 
 *           
*/
int closeBITRecver(struct BITRecver *server) {
  pthread_cancel(server->recv_thread);
  usleep(100000);
  close(server->sck);
  freeFifo(server->recv_fifo);
  blast_info("Closed BITRecver");
  return 1;
}

/* -------------------------
 * --- getBITSenderAddr ----
 * -------------------------
 * 
 * This function returns a pointer to the newest element in the BITSender
 * FIFO to write to. Note that this function only returns a valid pointer for
 * a FIFO that has memory allocated to it (maxsize != 0).
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITSender struct to be written to.
 * 
 *           
*/
uint8_t *getBITSenderAddr(struct BITSender *server) {
  return getFifoWrite(server->send_fifo);
}

/* -------------------------
 * -- setBITSenderSerial ---
 * -------------------------
 *
 * This function sets the unique serial associated with BITSender. The receiving
 * end can validate message from this BITSender based on this serial.
 *
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITSender.
 * serial:     The 32-bit serial number assigned BITSender
 *
*/
int setBITSenderSerial(struct BITSender *server, uint32_t serial) {
  if (verbosity) blast_info("-> BITSender serial: %.08x", serial);
  server->serial = serial;
  return 1;
}

/* -------------------------
 * -- setBITRecverSerial ---
 * -------------------------
 *
 * This function sets the unique serial associated with BITRecver. Messages
 * are validated from the sender based on this serial.
 *
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITRecver.
 * serial:     The 32-bit serial number assigned BITRecver
 *
*/
int setBITRecverSerial(struct BITRecver *server, uint32_t serial) {
  if (verbosity) blast_info("-> BITRecver serial: %.08x", serial);
  server->serial = serial;
  return 1;
}

int setBITSenderFramenum(struct BITSender * server, uint32_t frame_num)
{
  server->frame_num = frame_num;
  server->send_fifo->frame_num[server->send_fifo->end] = frame_num;
  return 1;
}

int setBITRecverFramenum(struct BITRecver * server, uint32_t frame_num)
{
  server->frame_num = frame_num;
  return 1;
}

/* -------------------------
 * --- setBITSenderAddr ----
 * -------------------------
 * 
 * This function sets the pointer for the newest element in the BITSender 
 * FIFO to be written to. Note that this funcion is only successful for 
 * a FIFO that is unallocated (maxsize == 0).
 * 
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITSender.
 * buffer:     A pointer to the external buffer containing data to be 
 *         added to the FIFO.
 * 
*/
int setBITSenderAddr(struct BITSender *server, uint8_t *buffer) {
  return setFifoWrite(server->send_fifo, buffer);
}


/* -------------------------
 * --- getBITRecverAddr ----
 * -------------------------
 * 
 * This function returns a pointer to the oldest element in the BITRecver
 * FIFO to read from and also reports the size of the data in
 * the BITRecver FIFO. This function blocks until there is something in
 * the FIFO to read.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITRecver struct to be read from.
 * size:      Returns the size of data in the FIFO.
 * 
 *           
*/
uint8_t *getBITRecverAddr(struct BITRecver *server, unsigned int *size) {
  while (fifoIsEmpty(server->recv_fifo)) usleep(500);
  if (size != NULL) *size = server->recv_fifo->size[server->recv_fifo->start];
  return getFifoRead(server->recv_fifo);
}

/* -------------------------
 * --- appendBITSenderAddr --
 * -------------------------
 * 
 * This function clears all the metadata at the current element in the BITRecver
 * FIFO and decrements the FIFO so that the next element can be read.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITSender struct to clear.
 * 
 *           
*/
int appendBITSenderAddr(struct BITSender *server, unsigned int size) {
  server->send_fifo->frame_num[server->send_fifo->end]= server->frame_num;
  server->send_fifo->size[server->send_fifo->end] = size;
  server->frame_num = server->frame_num+1; // increment the frame number
  return incrementFifo(server->send_fifo);
}


/* -------------------------
 * --- removeBITRecverAddr --
 * -------------------------
 * 
 * This function clears all the data at the current element in the BITRecver
 * FIFO and decrements the FIFO so that the next element can be read.
 * 
 * Parameters:
 * --------------------------
 * server:      A pointer to the BITSender struct to clear.
 * 
 *           
*/
int removeBITRecverAddr(struct BITRecver *server) {
  if (fifoIsEmpty(server->recv_fifo)) {
    blast_err("Nothing in the FIFO to remove!");
    return -1;
  }
  return decrementFifo(server->recv_fifo);
}


/* -------------------------
 * ---- sendToBITSender ----
 * -------------------------
 * 
 * This function takes a BITSender and increments the end of the FIFO by 1.
 * As a result, the buffer is incremented in a way that newest data is added to
 * the tail of the FIFO. A data size and serial will be associated with the
 * data in the FIFO as well. Returns the number of bytes sent to the FIFO.
 * 
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITSender that will be incremented.
 * buffer:    A pointer to the buffer to add to BITSender.
 * size:    Size of the data written to the BITSender FIFO [bytes].
 *         
 *         size <= fifo_maxsize when fifo_maxsize != 0
 * 
 * flags:    Auxilliary flags:
 *
 *         NO_HEADER
 *         Send packets without header information. This is only
 *         valid when fifo_maxsize <= packet_maxsize so that the maximum FIFO
 *         element can fit in a single packet
 * 
*/
int sendToBITSender(struct BITSender *server, uint8_t *buffer,
  unsigned int size, uint8_t flags) {
  if ((size > server->send_fifo->maxsize) && (server->send_fifo->maxsize != 0)) {
    blast_err("specified size %d is larger than maximum %d",
      size, server->send_fifo->maxsize);
    return -1;
  }
  // assign pointer if unallocated and copy if allocated
  if (server->send_fifo->maxsize == 0) {
     setBITSenderAddr(server, buffer);
  } else {
     memcpy(getBITSenderAddr(server), buffer, size);
  }
  server->send_fifo->flags[server->send_fifo->end] |= flags; // set flags
  appendBITSenderAddr(server, size);
  return size;
}

/* -------------------------
 * ----- peekBITRecver -----
 * -------------------------
 * 
 * This function returns 1 if there is data in the recv_fifo and 0 if there
 * isn't data in the recv_fifo. This can be used to check if there is data
 * before calling recvFromBITRecver.
 * 
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITRecver.
 *  
*/

int peekBITRecver(struct BITRecver *server) {
  return !(fifoIsEmpty(server->recv_fifo));
}

/* -------------------------
 * --- recvFromBITRecver ---
 * -------------------------
 * 
 * This function reads packets from the BITRecver FIFO and stores them in 
 * a single reassembled buffer provided by the user. The function returns
 * the total size of the reassembled message as well as the serial number
 * associated with the message. Note that there must be enough space 
 * allocated by the user in the user-defined buffer to received the entire
 * message. Also note that this function is blocking (see peekBITRecver to
 * check for data before receiving).
 * 
 * If the first packet received has a non-packet header, it is assumed that
 * a single message packet has been received, so the returned message consists
 * of just the one packet (no serial number).
 *
 * If packets are missed from a multipacket message, the entire message is dropped.
 * 
 * On general error, the function returns -1. In the case of a serial 
 * mismatch, the function returns -2.
 * 
 * Parameters:
 * --------------------------
 * server:    A pointer to the BITRecver that will be incremented.
 * buffer:     A pointer to the buffer in which the reassembled message
 *         will be stored. The buffer must be sufficiently large to
 *         hold the entire reassembled message or only a partial 
 *         message will be received.
 * size:    The size of the user-defined buffer.
 * flags:    Auxilliary flags:
 *
 *         NO_HEADER
 *         Receive a headerless packet. This is only valid when
 *         fifo_maxsize<=packet_maxsize so that the entire message
 *         fits in the FIFO as a single packet.
 * 
*/
int recvFromBITRecver(struct BITRecver *server, uint8_t *buffer,
  unsigned int size, uint8_t flags) {
  uint8_t *readbuffer;
  unsigned int totalbytes = 0;
  int tempnum = -1;
  unsigned int tempsize = 0;
  uint32_t *ser;
  uint32_t *frame_num;
  uint16_t *i_pkt;
  uint16_t *n_pkt;
  uint16_t cur = 0;
  uint8_t complete = 0;
  unsigned int offset = server->packet_maxsize-PACKET_HEADER_SIZE;

  while (!complete) {
    if (flags & RECV_TIMEOUT) {
      int timeout_count = 0;
      while (!peekBITRecver(server)) {
        if (timeout_count >= RECV_TIMEOUT_NUM) {
          blast_err("recvFrom timeout");
	        return -1;
        }
        timeout_count++;
        usleep(100000);
      }
    }
    readbuffer = getBITRecverAddr(server, &tempsize);

    if (flags & NO_HEADER) {
      // printf("Receiving headerless packet\n");
      memcpy(buffer, readbuffer, MIN(tempsize, size));
      totalbytes = MIN(tempsize, size);
      removeBITRecverAddr(server);
      break;
    } else {
      // read and verify the header
      readHeader(readbuffer, &ser, &frame_num, &i_pkt, &n_pkt);
      tempsize -= PACKET_HEADER_SIZE; // data size

      if (*ser != server->serial) { // matched serial
        blast_info("Serial mismatch: 0x%.08x != 0x%.08x", *ser, server->serial);
        removeBITRecverAddr(server);
        return -2;
      }
      // set the frame number
      server->frame_num = *frame_num;
      server->recv_fifo->frame_num[server->recv_fifo->start] = *frame_num;

      // printf("%ld %d %d : %d\n",*frame_num,*i_pkt,*n_pkt,tempsize);

      // broken buffer breaks
      if ((offset != tempsize) && ((*i_pkt+1) != *n_pkt)) {
        blast_info("Packet mismatch!");
        removeBITRecverAddr(server);
        return -1;
      }
      if ((totalbytes+tempsize) > size) {
        blast_info("Buffer overflow!");
        removeBITRecverAddr(server);
        return -1;
      }

      // missed packet breaks
      if (tempnum == -1) { // first packet in message
        if (*i_pkt != 0) { // missed first packet, so bail
          blast_info("Missed first packet! Packet %d/%d, serial 0x%.08x,"
                       "framenumber %d", *i_pkt, *n_pkt, *ser, *frame_num);
          removeBITRecverAddr(server);
          return -1;
        } else {
          tempnum = *frame_num; // set frame number for message
        }
      } else if (cur != *i_pkt) { // missed middle packet, so bail
        blast_info("Missed packet %d != %d (framenum=%d)", cur, *i_pkt, *frame_num);
        // removeBITRecverAddr(server);
        // return -1;
      }

      // normal breaks
      if (*frame_num != ((unsigned int) tempnum))  {
        complete = 1; // next message, so exit
        break;
      } else if ((*i_pkt+1) == *n_pkt) {
        complete = 1; // done message
      }

      memcpy(buffer+((*i_pkt)*offset),
        readbuffer+PACKET_HEADER_SIZE, tempsize);
      cur++;
    }
    totalbytes += tempsize;
    if (flags & DONT_REMOVE) {
      break;
    } else {
      removeBITRecverAddr(server);
    }
  }
  return totalbytes;
}

#ifdef __cplusplus
}
#endif
