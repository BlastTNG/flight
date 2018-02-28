/* ---------------------------------------------------------------------
 * ----------------------------- BITSERVER -----------------------------
 * ---------------------------------------------------------------------
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
 * Copyright 2014 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */
#ifndef BITSERVER_H_
#define BITSERVER_H_

#define UDPMAXBUFLEN 65536   // maximum UDP buffer size [bytes]
#define PACKET_HEADER_SIZE 12 // header size for packets [bytes]
#define DEFAULT_SERIAL 0xabadfeed // default serial for packets
#define NO_HEADER 0x02 // flag to send message without a header
#define DONT_REMOVE 0x04 // bit receiver without removing from bit receiver
#define RECV_TIMEOUT 0x08 // recvFromBITRecver times out

#define RECV_TIMEOUT_NUM 10 // number of tenths of seconds until recvFrom timeout

/* OSX doesn't support MSG_NOSIGNAL (it never signals) */
#ifdef __APPLE__

#define COMMON_DIGEST_FOR_OPENSSL
#include <CommonCrypto/CommonDigest.h>
#define SHA1 CC_SHA1

#include <sys/socket.h>
#include "FIFO.h"

#ifndef MSG_NOSIGNAL
#define MSG_NOSIGNAL 0
#endif
#endif

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifdef __cplusplus
extern "C" {
#endif

struct BITSender
{
  int sck;
  struct sockaddr_in my_addr;
  struct sockaddr_in send_addr;
  socklen_t slen;
  uint32_t serial;
  uint32_t frame_num;
  
  pthread_t send_thread;
  unsigned int packet_maxsize;
  struct Fifo * send_fifo;
};

struct BITRecver
{
  int sck;
  struct sockaddr_in my_addr;
  struct sockaddr_in recv_addr;
  socklen_t slen;
  uint32_t serial;
  uint32_t frame_num;
  
  pthread_t recv_thread;
  unsigned int packet_maxsize;
  struct Fifo * recv_fifo;
};

extern int verbosity;

// some function prototypes
int initBITSender(struct BITSender *, const  char *, 
  unsigned int , unsigned int ,
  unsigned int , unsigned int );
int initBITRecver(struct BITRecver *, const char *, 
  unsigned int , unsigned int ,
  unsigned int , unsigned int );
  
int closeBITSender(struct BITSender *);
int closeBITRecver(struct BITRecver *);

int sendToBITSender(struct BITSender *, uint8_t *, unsigned int, uint8_t );
int recvFromBITRecver(struct BITRecver *, uint8_t *, unsigned int, uint8_t );
int peekBITRecver(struct BITRecver *);

uint16_t writeHeader(uint8_t *, uint32_t, uint32_t, uint16_t, uint16_t);
uint16_t readHeader(uint8_t *, uint32_t**, uint32_t**, uint16_t**, uint16_t**);

int setBITSenderAddr(struct BITSender *, uint8_t *);
uint8_t *getBITSenderAddr(struct BITSender *);
uint8_t *getBITRecverAddr(struct BITRecver *, unsigned int *);

int setBITSenderSerial(struct BITSender *, uint32_t );
int setBITRecverSerial(struct BITRecver *, uint32_t );
int setBITSenderFramenum(struct BITSender *, uint32_t);
int setBITRecverFramenum(struct BITRecver *, uint32_t);

int appendBITSenderAddr(struct BITSender *, unsigned int);
int removeBITRecverAddr(struct BITRecver *);

#ifdef __cplusplus
}
#endif

#endif /* BITSERVER_H_ */
