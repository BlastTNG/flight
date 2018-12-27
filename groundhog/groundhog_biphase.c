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

#include "linklist.h" // This gives access to channel_list and frame_size
#include "linklist_compress.h"
#include "linklist_writer.h"

#include "bbc_pci.h"
#include "decom_pci.h"
#include "bi0.h"
#include "crc.h"
#include "blast.h"
#include "blast_time.h"
#include "groundhog.h"
#include "bitserver.h"
#include "FIFO.h"

#define BIPHASE_FRAME_SIZE_BYTES (BI0_FRAME_SIZE*2)
#define BIPHASE_PACKET_SIZE (BIPHASE_FRAME_SIZE_BYTES-2-PACKET_HEADER_SIZE)
#define BIPHASE_PACKET_WORD_START (7)

#define DEV "/dev/decom_pci"

void print_packet(uint8_t * packet, size_t length) {
    printf("Packet Received is: \n");
    for (int i = 0; i < length; i++) {
        printf("%04x ", *(packet + i));
    }
}

void get_stripped_packet(bool *normal_polarity, uint8_t *receive_buffer_stripped, const uint16_t *receive_buffer, 
                         const uint16_t *anti_receive_buffer, linklist_t **ll, uint32_t **frame_number, 
                         uint16_t **i_pkt, uint16_t **n_pkt) {

  uint32_t *serial;

  if (*normal_polarity) {
      // Reading the header
      readHeader((uint8_t *) (receive_buffer+1), &serial, frame_number, i_pkt, n_pkt);
      // Checking for polarity
      if ((*ll = linklist_lookup_by_serial(*serial))) {
          // blast_dbg("\n=== I found a linklist with serial %x, counter: %x, i_pkt: %d, n_pkt: %d==\n", *serial, *frame_number, **i_pkt, **n_pkt);
          *normal_polarity = true;
          memcpy(receive_buffer_stripped, receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      } else if ((*ll = linklist_lookup_by_serial((uint16_t) ~(*serial)))) {
          *normal_polarity = false;
      }
  }
  if (!(*normal_polarity)) {
      // Reading the header
      readHeader((uint8_t *) (anti_receive_buffer+1), &serial, frame_number, i_pkt, n_pkt);
      // Checking for polarity
      if ((*ll = linklist_lookup_by_serial(*serial))) {
          // blast_dbg("Inverted 1: I found a linklist with serial %x, counter: %d, i_pkt: %d, n_pkt: %d\n", *serial, *frame_number, **i_pkt, **n_pkt);
          *normal_polarity = false;
          memcpy(receive_buffer_stripped, anti_receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      } else if ((*ll = linklist_lookup_by_serial((uint16_t) ~(*serial)))) {
          *normal_polarity = true;
          readHeader((uint8_t *) (receive_buffer+1), &serial, frame_number, i_pkt, n_pkt);
          // blast_dbg("Inverted 2: I found a linklist with serial %x, counter: %d, i_pkt: %d, n_pkt: %d\n", *serial, *frame_number, **i_pkt, **n_pkt);
          memcpy(receive_buffer_stripped, receive_buffer+BIPHASE_PACKET_WORD_START, BIPHASE_PACKET_SIZE);
          // print_packet(receive_buffer_stripped, BIPHASE_PACKET_SIZE);
      }
  }
}

void biphase_receive(void *args)
{

  int decom_fp;
  int i_word = 0;
  int16_t du; // num_unlocked: info only
  const uint16_t sync_word = 0xeb90;
  uint16_t raw_word_in;
  uint32_t transmit_size;

  uint16_t receive_buffer[BI0_FRAME_SIZE];
  uint16_t anti_receive_buffer[BI0_FRAME_SIZE];
  uint8_t receive_buffer_stripped[BIPHASE_PACKET_SIZE];
  uint8_t *compbuffer = calloc(1, BI0_MAX_BUFFER_SIZE);
  uint32_t compbuffer_size = 0;

  linklist_t *ll = NULL;
  uint16_t *i_pkt;
  uint16_t *n_pkt;
  uint32_t *frame_number;
  int retval;

  uint8_t *local_superframe = calloc(1, superframe->size);
  uint8_t *local_allframe = calloc(1, superframe->allframe_size);

  // open a file to save all the raw linklist data
  linklist_rawfile_t * ll_rawfile = NULL;
  uint32_t prev_serial = 0;

  bool normal_polarity = true;

  // buos_use_stdio();

  /* Open Decom */
  if ((decom_fp = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
      berror(fatal, "fatal error opening " DEV);
  }

  /* Initialise Decom */
  ioctl(decom_fp, DECOM_IOC_RESET);
  //ioctl(decom_fp, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE);
  ioctl(decom_fp, DECOM_IOC_FRAMELEN, 2*BI0_FRAME_SIZE-1);

  /* set up our outputs */
  openlog("decomd", LOG_PID, LOG_DAEMON);
  // buos_use_syslog();
  uint8_t * dummy_buffer = calloc(1, superframe->size);
  uint64_t framenum = 0;
  int af = 0;

  while(true) {
      while ((read(decom_fp, &raw_word_in, sizeof(uint16_t))) > 0) {
          // Fill receive and anti receive
          receive_buffer[i_word] = raw_word_in;
          anti_receive_buffer[i_word] = ~raw_word_in;
          if (i_word == 0) {
              // Beginning of packet?
              du = ioctl(decom_fp, DECOM_IOC_NUM_UNLOCKED);
              if ((raw_word_in != sync_word) && (raw_word_in != (uint16_t) ~sync_word)) {
                  // This was not the beginning actually
                  i_word = 0;
                  continue;
              }
           //   printf("\n=== Frame Start ==\n");
          } else if ((i_word) == (BI0_FRAME_SIZE-1)) {
              get_stripped_packet(&normal_polarity, receive_buffer_stripped, receive_buffer, anti_receive_buffer, 
                                 &ll, &frame_number, &i_pkt, &n_pkt);
              retval = depacketizeBuffer(compbuffer, &compbuffer_size, 
                                       BIPHASE_PACKET_SIZE-BI0_ZERO_PADDING, 
                                       i_pkt, n_pkt, receive_buffer_stripped);
              memset(receive_buffer_stripped, 0, BIPHASE_PACKET_SIZE);

       
              if ((retval == 0) && (ll != NULL)) {
                  // hijack the frame number for transmit size
                  transmit_size = *frame_number;

                  // blast_info("Transmit size=%d, blk_size=%d", transmit_size, ll->blk_size);
                  // The compressed linklist has been fully reconstructed
                  // blast_info("[Biphase] Received linklist with serial_number 0x%x\n", *(uint32_t *) ll->serial);

                  // this is a file that has been downlinked, so unpack and extract to disk
                  if (!strcmp(ll->name, FILE_LINKLIST)) {
                      unsigned int bytes_unpacked = 0;
                      while ((bytes_unpacked+ll->blk_size) <= transmit_size) {
                          decompress_linklist(dummy_buffer, ll, compbuffer+bytes_unpacked);
                          bytes_unpacked += ll->blk_size;
                          usleep(1000);
                      }
                      framenum = ll->blocks[0].i*100/ll->blocks[0].n;

                  } else { // write the linklist data to disk
                      af = read_allframe(local_superframe, superframe, compbuffer);
                      if (af > 0) { // an allframe was received
                          if (verbose) blast_info("[Biphase] Received an allframe :)\n");
                          memcpy(local_allframe, compbuffer, superframe->allframe_size);
                      } else if (af == 0) { // just a regular rame (< 0 indicates problem reading allframe)
                          // check the serials
                          if (*(uint32_t *) ll->serial != prev_serial) {
                            ll_rawfile = groundhog_open_new_rawfile(ll_rawfile, ll, "BI0");
                          }
                          prev_serial = *(uint32_t *) ll->serial;
                          if (verbose) blast_info("[Biphase] Received linklist \"%s\"", ll->name);

                          if (transmit_size > ll->blk_size) {
                              blast_err("Transmit size larger than assigned linklist");
                              transmit_size = ll->blk_size;
                          }  
                          if (ll_rawfile) {
                            // copy the allframe, write to disk, and flush
                            memcpy(compbuffer+ll->blk_size, local_allframe, superframe->allframe_size);
                            write_linklist_rawfile(ll_rawfile, compbuffer);
                            flush_linklist_rawfile(ll_rawfile);
                            framenum = tell_linklist_rawfile(ll_rawfile); 
                          }
                      }
                  }

                  // fill out the telemetry report
                  bi0_report.ll = ll;
                  bi0_report.framenum = framenum; 
                  bi0_report.allframe = af; 

                  memset(compbuffer, 0, BI0_MAX_BUFFER_SIZE);
                  compbuffer_size = 0;
              }
          }
          i_word++;
          i_word = (i_word % BI0_FRAME_SIZE);
          //printf("%04x ", raw_word_in);
      }
      usleep(100);
  }
}

