#include <math.h>
#include <fcntl.h>
#include <stdbool.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // threads
#include <openssl/md5.h>
#include <float.h>
#include <time.h>
#include <sys/time.h>

#include "groundhog.h"

struct TlmReport highrate_report = {0};
struct TlmReport sbd_report = {0};

enum HeaderType{NONE, TD_HK, TD_OMNI_HGA, IRID_HK, IRID_DIAL, PAYLOAD};

struct CSBFHeader {
  uint8_t route;
  uint8_t origin;
  uint8_t comm;
  uint8_t zero;
  uint16_t size;

  uint8_t checksum;
  enum HeaderType mode;
  uint8_t sync;

  char namestr[80];
};


char * comm_label[2] = {"COMM1", "COMM2"};

enum HeaderType read_csbf_header(struct CSBFHeader * header, uint8_t byte) {
  header->mode = NONE;

  if ((header->sync == 0) && (byte == HIGHRATE_SYNC1)) {
      header->sync++;
  } else if (header->sync == 1) {
      switch (byte) {
          case LOWRATE_COMM1_SYNC2 :
          case LOWRATE_COMM2_SYNC2 :
          case HIGHRATE_TDRSS_SYNC2 :
          case HIGHRATE_IRIDIUM_SYNC2 :
              header->route = byte;
              header->sync++;
              break;
          default :
              header->sync = 0;
      }
  } else if (header->sync == 2) {
      header->origin = byte & 0x07; // bits 0-2 == 0 is hk, == 1 is low rate, == 2 is high rate
      header->comm = ((byte & 0x08) >> 3); // bit 3 == 0 is comm1,  == 1 is comm2
      header->checksum = byte;             

      header->sync++;
  } else if (header->sync == 3) {
      header->zero = byte;
      header->checksum += byte;

      header->sync++; 
  } else if (header->sync == 4) { // size MSB
      header->size = byte << 8;
      header->checksum += byte;

      header->sync++;
  } else if (header->sync == 5) { // size LSB
      header->size += byte;
      header->checksum += byte;       

      if (header->zero) {
          header->mode = PAYLOAD;
      } else if (header->route == HIGHRATE_IRIDIUM_SYNC2) {
          switch (header->origin) {
              case 0x01:
              case 0x00:
                  header->mode = IRID_HK;
                  sprintf(header->namestr, "Iridium HK %s", comm_label[header->comm]);
                  break;
              case 0x02:
                  header->mode = IRID_DIAL;
                  sprintf(header->namestr, "Iridium Dialup %s", comm_label[header->comm]);
                  break;
              default:
                  blast_info("Unrecognized Iridium origin byte 0x%x\n", header->origin+(header->comm << 3));
              
          }
      } else if (header->route == HIGHRATE_TDRSS_SYNC2) {
          switch (header->origin) {
              case 0x01:
              case 0x00:
                  header->mode = TD_HK;
                  sprintf(header->namestr, "TDRSS HK %s", comm_label[header->comm]);
                  break;
              case 0x02:
                  header->mode = TD_OMNI_HGA;
                  sprintf(header->namestr, "TDRSS Omni/HGA %s", comm_label[header->comm]);
                  break;
              default:
                  blast_info("Unrecognized TDRSS origin byte 0x%x\n", header->origin+(header->comm << 3));
              
          }
      }
      header->sync = 0;
  }
  return header->mode;
}

// grabs a packet from the gse stripped of it gse header
void read_gse_sync_frame(int fd, uint8_t * buffer, struct CSBFHeader * header) {
  uint8_t byte = 0;
  unsigned int bytes_read = 0;
  memset(header, 0, sizeof(struct CSBFHeader));

  while (true) {
      if (read(fd, &byte, 1) == 1) {
          if (header->mode == NONE) {
              read_csbf_header(header, byte);
          } else if (header->mode != PAYLOAD) { // end of gse/payload header
              if (bytes_read < header->size) { // keep reading to the buffer 
                  buffer[bytes_read++] = byte;
                  header->checksum += byte;
              } else {
                  if (header->checksum == byte) { // done reading 
                      // printf("Received gse packet size %d (0x%x == 0x%x)\n", header->size, byte, header->checksum); 
                      return;
                  } else {
                      blast_info("[%s] Bad checksum 0x%.2x != 0x%.2x", header->namestr, header->checksum, byte);
                  }
                  bytes_read = 0;
                  memset(header, 0, sizeof(struct CSBFHeader));
              }
          } else { // payload packet, so ignore
              bytes_read = 0;
              memset(header, 0, sizeof(struct CSBFHeader));
          }
      } else { // nothing read
          usleep(1000);
      }
  }
}

// grabs a packet from the gse stripped of it gse header
void read_gse_sync_frame_direct(int fd, uint8_t * buffer, unsigned int length) {
  uint8_t byte = 0;
  unsigned int bytes_read = 0;

  while (true) {
      if (read(fd, &byte, 1) == 1) {
          buffer[bytes_read++] = byte;

          if (bytes_read >= length) { // done reading from the buffer 
              return;
          }
      } else { // nothing read
          usleep(1000);
      }
  }
}

void highrate_receive(void *arg) {
  // Open serial port
  unsigned int serial_mode = (unsigned long long) arg;
  comms_serial_t *serial = comms_serial_new(NULL);
  char linkname[80];
  if (serial_mode) { // direct
      comms_serial_connect(serial, DIRECT_PORT);
      sprintf(linkname, "TDRSSDirect");
  } else { // highrate
      comms_serial_connect(serial, HIGHRATE_PORT);
      sprintf(linkname, "SerHighRate");
  }
  comms_serial_setspeed(serial, B115200);
  int fd = serial->sock->fd; 

  linklist_t * ll = NULL;
  linklist_t * sbd_ll = NULL;

  // open a file to save all the raw linklist data
  linklist_rawfile_t * ll_rawfile = NULL;
  linklist_rawfile_t * sbd_ll_rawfile = NULL;
  uint32_t ser = 0, prev_ser = 0, sbd_ser = 0, sbd_prev_ser = 0;

  // packet sizes
  unsigned int payload_packet_size = (HIGHRATE_DATA_PACKET_SIZE+CSBF_HEADER_SIZE+1)*2;
  uint16_t datasize = HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE;
  uint32_t buffer_size = ((HIGHRATE_MAX_SIZE-1)/datasize+1)*datasize;

  // buffer allocations
  uint8_t *payload_packet = calloc(1, payload_packet_size);
  uint8_t *csbf_header = payload_packet+0;
  uint8_t *header_buffer = csbf_header+CSBF_HEADER_SIZE;
  uint8_t *data_buffer = header_buffer+PACKET_HEADER_SIZE;
  // uint8_t *csbf_checksum = header_buffer+HIGHRATE_DATA_PACKET_SIZE;

  // packetization variables
  uint32_t *serial_number = 0;
  uint16_t *i_pkt, *n_pkt;
  uint32_t *frame_number;
  uint32_t transmit_size;
  uint8_t *compbuffer = calloc(1, buffer_size);

  uint8_t *local_allframe = calloc(1, superframe->allframe_size);

  struct CSBFHeader gse_packet_header = {0};
  uint8_t * gse_packet = calloc(1, 4096);
  unsigned int gse_read = 0;

  struct CSBFHeader payload_packet_header = {0};
  unsigned int payload_read = 0;
  unsigned int payload_copy = 0;
  uint8_t payload_packet_lock = 0;
  uint16_t payload_size = 0;

  int retval = 0;
  uint32_t recv_size = 0;
  int64_t framenum = 0;
  int af = 0;

  char * source_str = NULL;

  while (true) {
      // printf("-------------START (lock = %d)---------\n", payload_packet_lock);

      // get the sync frame from the gse
      if (serial_mode) { // direct
          // mimic the return header values for highrate
          gse_packet_header.size = 2048;
          gse_packet_header.origin = 0xe;
          sprintf(gse_packet_header.namestr, "TDRSS Direct");

          read_gse_sync_frame_direct(fd, gse_packet, gse_packet_header.size);
      } else { // highrate
          read_gse_sync_frame(fd, gse_packet, &gse_packet_header);
      }

      gse_read = 0;
      source_str = gse_packet_header.namestr;

      /* 
      for (int i = 0; i < gse_packet_header.size; i++) {
          if (i%32 == 0) printf("\n%.3d : ", i/32);
          printf("0x%.2x ", gse_packet[i]);
      }
      printf("\n");
      */

      // printf("Got a GSE packet size %d origin 0x%x\n", gse_packet_header.size, gse_packet_header.origin);
 
      while (gse_read < gse_packet_header.size) { // read all the gse data

          if ((gse_packet_header.mode != TD_HK) && (gse_packet_header.mode != IRID_HK)) { // packet not from the hk stack (origin != 0)
              if (payload_packet_lock) { // locked onto payload header     
                  payload_copy = MIN(payload_size-payload_read, gse_packet_header.size-gse_read);
                  
                  if (payload_read+payload_copy > payload_packet_size) {
                      blast_err("Received more data (%d) than expected (%d). Losing lock\n", payload_read+payload_copy, payload_packet_size);
                      payload_packet_lock = 0;
                      continue;
                  }

                  memcpy(payload_packet+payload_read, gse_packet+gse_read, payload_copy);

                  gse_read += payload_copy; // update bytes read from gse
                  // printf("Copied %d bytes to location %d. %d bytes left to read\n", payload_copy, payload_read, gse_packet_header.size-gse_read);
                  payload_read += payload_copy; // update bytes read to to payload packet

                  if (payload_read == payload_size) { // read all of the payload packet
                      // printf("Got the entire payload message\n");
                      // reset bookkeeping variables
                      payload_read = 0;
                      payload_packet_lock = 0;
                      memset(&payload_packet_header, 0, sizeof(struct CSBFHeader));

                      // process the packet
                      readHeader(header_buffer, &serial_number, &frame_number, &i_pkt, &n_pkt);

                      // hijack frame number for transmit size
                      transmit_size = *frame_number;

                      if (!(ll = linklist_lookup_by_serial(*serial_number))) {
                          blast_err("[%s] Could not find linklist with serial 0x%.4x", source_str, *serial_number);
                          continue; 
                      }
                      ser = *serial_number;

                      // blast_info("Transmit size=%d, blk_size=%d, payload_size=%d, datasize=%d, i=%d, n=%d", transmit_size, ll->blk_size, payload_size, datasize, *i_pkt, *n_pkt);

                      retval = depacketizeBuffer(compbuffer, &recv_size, 
                                           HIGHRATE_DATA_PACKET_SIZE-PACKET_HEADER_SIZE,
                                           i_pkt, n_pkt, data_buffer);

                      memset(payload_packet, 0, payload_packet_size);

                      // the packet is complete, so decompress
                      if ((retval == 0) && (ll != NULL))
                      {
                          if (groundhog_check_for_fileblocks(ll, FILE_LINKLIST)) {
                              // unpack and extract to disk
                              framenum = groundhog_unpack_fileblocks(ll, transmit_size, compbuffer, NULL,
                                                                     NULL, NULL, NULL, GROUNDHOG_EXTRACT_TO_DISK);
                          } else { // write the linklist data to disk
                              // set flags for data extraction
                              unsigned int flags = 0;
                              if (ser != prev_ser) flags |= GROUNDHOG_OPEN_NEW_RAWFILE;
                              prev_ser = ser;

                              // process the linklist and write the data to disk
                              framenum = groundhog_process_and_write(ll, transmit_size, compbuffer,
                                                                    local_allframe, linkname, 
                                                                    source_str, &ll_rawfile, flags);
                          }
                          // fill out the telemetry report
                          highrate_report.ll = ll;
                          highrate_report.framenum = abs(framenum);
                          highrate_report.allframe = af; 

                          memset(compbuffer, 0, buffer_size);
                          recv_size = 0;
                      }
                  }
              } else if (!payload_packet_lock) { // search for payload header
                  read_csbf_header(&payload_packet_header, gse_packet[gse_read++]);
                  if (payload_packet_header.mode == PAYLOAD) {
                      payload_size = payload_packet_header.size+CSBF_HEADER_SIZE+1; // payload includes the header and checksum
                      payload_packet_lock = 1;
                      payload_read = CSBF_HEADER_SIZE;
                      // printf("Found a payload header @ %d size %d bytes\n", gse_read, payload_size);
                  }
              }    
     
          } else if ((gse_packet_header.origin == 0) || (gse_packet_header.origin == 1)) { // housekeeping packet
              gse_read += gse_packet_header.size;

              // short burst data (sbd) packets are simple:
              // 4 bytes for the linklist serial followed by the data
              if (!(*(uint32_t *) gse_packet)) {
                  blast_info("[%s] Empty HK packet", source_str);
                  continue;
              }
              
              if (!(sbd_ll = linklist_lookup_by_serial(*(uint32_t *) gse_packet))) {
                  blast_err("[%s] Could not find linklist with serial 0x%.4x", source_str, *(uint32_t *) gse_packet);
                  continue; 
              }
              sbd_ser = *(uint32_t *) gse_packet;

              // set flags for data extraction
              unsigned int flags = 0;
              if (sbd_ser != sbd_prev_ser) flags |= GROUNDHOG_OPEN_NEW_RAWFILE;
              sbd_prev_ser = sbd_ser;

              // TODO(javier): check this with sipsim or real sip
              blast_info("[%s] Received packet \"%s\" from HK stack (size=%d)\n", source_str, sbd_ll->name, gse_packet_header.size);

              // process the linklist and write the data to disk
              framenum = groundhog_process_and_write(sbd_ll, MIN(gse_packet_header.size, sbd_ll->blk_size), 
                                                  gse_packet+sizeof(uint32_t), local_allframe,
                                                  "ShortBurst", source_str, &sbd_ll_rawfile, flags);
              // fill out the telemetry report
              sbd_report.ll = sbd_ll;
              sbd_report.framenum = abs(framenum);
              sbd_report.allframe = 0; 
              /*
              if (*(uint32_t *) gse_packet == SLOWDLSYNCWORD) {
              
              } else {
                  blast_info("[%s] Bad syncword 0x%.08x\n", source_str, *(uint32_t *) gse_packet);
              }
              */
          }
      }
  }
}

