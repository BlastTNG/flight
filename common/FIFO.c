/* "Copyright 2017 Javier Romualdez"
 *
 * -----------------------------------------------------------------------
 * ----------------------------- BIT ADCS FIFO ---------------------------
 * -----------------------------------------------------------------------
 * This program is distributed under the GNU General Public License (GPL)
 * Version 2 or higher.
 *
 * The following was developed in the QNX Momentics IDE and compiled on
 * using QNX RTOS version 6.5.
 *
 * ------------------------- Description -------------------------------
 * This code includes all necessary routines and data structures for the
 * circular FIFO buffer used on BIT for ground telemetry links.
 *
 * ---------------------------- Author ---------------------------------
 * L. Javier Romualdez (B.Eng Aerospace)
 * Institute for Aerospace Studies (UTIAS)
 * University of Toronto
 *
 * Created: November 21, 2013
 *
 * Copyright 2013 Javier Romualdez
 *
 * -------------------------- Revisions --------------------------------
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <unistd.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include "FIFO.h"
#ifdef __cplusplus
extern "C" {
#endif



/* -------------------------
 * ------- allocFifo -------
 * -------------------------
 *
 * This function allocates memory for the specified FIFO, which includes
 * sizes, flags, and buffers.
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to allocate memory for.
 * length:		The number of elements in the FIFO
 * maxsize:		The maximum size of each element in the FIFO. If the maxsize
 * 				is set to 0, each buffer pointer will be NULL. This is
 * 				desirable if the FIFO only points to buffer elements without
 * 				copying data.
 *
*/
int allocFifo(struct Fifo *fifo, unsigned int length, unsigned int maxsize) {
	unsigned int i;

	fifo->length = length;
	fifo->maxsize = maxsize;
	fifo->start = 0;
	fifo->end = 0;
	fifo->size = (uint32_t *) calloc(length, sizeof(uint32_t));
	fifo->frame_num = (uint32_t *) calloc(length, sizeof(uint32_t));
	fifo->flags = (uint8_t *) calloc(length, sizeof(uint8_t));
	fifo->buffer = (uint8_t **) calloc(length, sizeof(uint8_t*));

	if (maxsize != 0) {
		for (i = 0; i < length; i++) {
			fifo->buffer[i] = (uint8_t *) calloc(maxsize, sizeof(uint8_t));
			fifo->flags[i] = MEMALLOC;
		}
	}

	return 1;
}

/* -------------------------
 * ------- freeFifo --------
 * -------------------------
 *
 * This function frees memory for the specified FIFO, including all memory
 * allocated to FIFO elements.
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to free memory.
 *
*/
int freeFifo(struct Fifo *fifo) {
	unsigned int i;
	for (i = 0; i < fifo->length; i++) {
		if (fifo->flags[i] & MEMALLOC) free(fifo->buffer[i]);
	}
	free(fifo->flags);
	free(fifo->size);
	free(fifo->frame_num);
	free(fifo->buffer);
	return 1;
}


/* -------------------------
 * ------ fifoIsEmpty ------
 * -------------------------
 *
 * This function returns 1 the FIFO is empty and 0 if it isn't...
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO in question.
 *
*/
int fifoIsEmpty(struct Fifo * fifo) {
	if ((fifo->start == fifo->end) && (fifo->length > 1)) return 1;
	return 0;
}



/* -------------------------
 * ----- incrementFifo -----
 * -------------------------
 *
 * This function takes a FIFO and increments the end of the buffer by 1.
 * As a result, the buffer is incremented in a way that newest data is added to
 * the tail of the buffer.
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to increment.
 *
*/
int incrementFifo(struct Fifo * fifo) {
	if ((fifo->end+1)%(fifo->length) == fifo->start) {
		// printf("Dump FIFO!\n");
		return -1; // can't increment because will overflow
		// decrementFifo(fifo); // OR dump the oldest measurement
	}
	fifo->end = (fifo->end+1)%(fifo->length); // update FIFO end
	if (fifo->flags[fifo->end] & MEMALLOC) {
        memset(fifo->buffer[fifo->end], 0, fifo->maxsize);
        fifo->flags[fifo->end] = MEMALLOC; // clear flags
    } else {
        fifo->flags[fifo->end] = 0; // clear flags
    }
    fifo->size[fifo->end] = 0; // clear size
    fifo->frame_num[fifo->end] = 0; // clear the frame number
	// if (fifoIsEmpty(fifo)) printf("SOMETHING IS VERY WRONG!!!\n");
	return 1;
}


/* -------------------------
 * ----- decrementFifo -----
 * -------------------------
 *
 * This function takes a FIFO and increments the start of the buffer by 1.
 * As a result, the buffer is incremented in a way that oldest data is removed
 * from the head of the FIFO.
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to decrement.
 *
*/
int decrementFifo(struct Fifo * fifo) {
	if (fifoIsEmpty(fifo)) {
		printf("Cannot decrement empty FIFO!\n");
		return -1;
	} else {
		if (fifo->flags[fifo->start] & MEMALLOC) {
			// memset(fifo->buffer[fifo->start], 0, fifo->maxsize);
			// fifo->flags[fifo->start] = MEMALLOC; // clear flags
		} else {
			// fifo->flags[fifo->start] = 0; // clear flags
		}
		// fifo->size[fifo->start] = 0; // clear size
		// fifo->frame_num[fifo->start] = 0; // clear the frame number
		fifo->start = (fifo->start+1)%(fifo->length); // update FIFO
	}
	return 1;
}

/* ------------------------
 * ------ clearFifo -------
 * ------------------------
 *
 * This function clears all pending data in the FIFO
 *
 * Parameters:
 * -------------------------
 * fifo:    A pointer to the FIFO to be cleared
 *
*/

void clearFifo(struct Fifo *fifo) {
  fifo->start = fifo->end;
}

int getFifoSize(struct Fifo *fifo) {
  int end = fifo->end;
  int start = fifo->start;
  int diff = end-start;
  return (diff < 0) ? diff + fifo->length : diff;
}

/* -------------------------
 * ------ getFifoRead ------
 * -------------------------
 *
 * This function returns a pointer to the oldest element in the FIFO to
 * be read.
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to read.
 *
*/
uint8_t * getFifoRead(struct Fifo * fifo) {
	return fifo->buffer[fifo->start];
}

/* -----------------------------
 * ------ getFifoLastRead ------
 * -----------------------------
 *
 * This function returns a pointer to the second oldest element in the
 * FIFO to be read. Returns NULL if there is only one sample in the FIFO
 * (i.e no last sample).
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to read.
 *
*/
uint8_t * getFifoLastRead(struct Fifo * fifo) {
	unsigned int last = (fifo->start+fifo->length-1)%fifo->length; // get last element circularly
	if ((last == fifo->end) || fifoIsEmpty(fifo)) {
        return NULL; // not enough data in the FIFO
	} else {
        return fifo->buffer[last];
    }
}

/* -------------------------
 * ------ getFifoWrite -----
 * -------------------------
 *
 * This function returns a pointer to the newest element in the FIFO to
 * be written to. Note that this function only returns a valid pointer for
 * a FIFO that has memory allocated to it (maxsize != 0).
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO to be written to.
 *
*/
uint8_t * getFifoWrite(struct Fifo * fifo) {
	if (fifo->maxsize == 0) {
		printf("Cannot get write address for unallocated FIFO\n");
		printf("Point to data with setFifoWrite()\n");
		return NULL;
	}
	return fifo->buffer[fifo->end];
}

/* -------------------------
 * ------ setFifoWrite -----
 * -------------------------
 *
 * This function sets the pointer for the newest element in the FIFO to be
 * written to. Note that this funcion is only successful for a FIFO that
 * is unallocated (maxsize == 0).
 *
 * Parameters:
 * --------------------------
 * fifo:		A pointer to the FIFO.
 * buffer: 		A pointer to the external buffer containing data to be
 * 				added to the FIFO.
 *
*/
int setFifoWrite(struct Fifo * fifo, uint8_t *buffer) {
	if (fifo->maxsize != 0) {
		printf("Cannot set write address for allocated FIFO\n");
		printf("Get write address with getFifoWrite()\n");
		return -1;
	}
	fifo->buffer[fifo->end] = buffer;

	return 1;
}


/* -------------------------
 * ---- packetizeBuffer ----
 * -------------------------
 *
 * This function takes a buffer and splits it into n packets of a given size.
 * Returns the location in the buffer for the ith packet.
 *
 * Parameters:
 * --------------------------
 *      buffer:    A pointer to the buffer to be packetized.
 * buffer_size:    The total size of the buffer to be packetized.
 * packet_size:    A pointer to the size of packets from the buffer.
                   This value will be updated to reflect the size of the packet to
                   be read from the buffer.
 *       i_pkt:    A pointer to the packet number to be written.
 *       n_pkt:    A pointer to the total number of packets required. If i_pkt=0, the number
 *                 of packets will be written based on the total buffer size and packet size.
 *
 *
*/
uint8_t * packetizeBuffer(uint8_t * buffer, uint32_t buffer_size, uint32_t * packet_size,
                          uint16_t * i_pkt, uint16_t * n_pkt) {
  // compute the total number of packets if necessary
  if (*i_pkt == 0) *n_pkt = (buffer_size-1)/(*packet_size)+1;

  // reached the end of the buffer
  if (*i_pkt >= *n_pkt) return NULL;

  // get the packet_size
  uint32_t read_size = MIN(*packet_size, buffer_size-((*i_pkt)*(*packet_size)));
  uint8_t * retval = buffer+(*packet_size)*(*i_pkt);

  // update packet size and location
  *packet_size = read_size;

  return retval;
}

/* -------------------------
 * --- depacketizeBuffer ---
 * -------------------------
 *
 * This function takes sequential n packets and assembles them into a main buffer.
 * Data for the ith packet (i<n) is written to an output buffer. Returns the
 * location in the buffer for the ith packet.
 *
 * Parameters:
 * --------------------------
 *      buffer:    A pointer to the buffer containing depacketized data.
 * buffer_size:    A pointer to the total size of the buffer to be packetized.
                   This value is updated as packets are added.
 * packet_size:    The size of packet to add to the buffer.
 *       i_pkt:    A pointer to the packet number to be written. This value will be
 *                 incremented on successful execution of this function.
 *       n_pkt:    A pointer to the total number of packets required.
 *
 *
*/
int depacketizeBuffer(uint8_t * buffer, uint32_t * buffer_size, uint32_t packet_size,
                          uint16_t * i_pkt, uint16_t * n_pkt, uint8_t * packet) {
  // check for invalid pointers
  if (!buffer || !packet || !buffer_size || !i_pkt || !n_pkt) return -1;

  // nonsense values for i_pkt
  if ((*i_pkt)*packet_size > *buffer_size) return -1;

  // get the packet_size
  uint8_t * retval = buffer+packet_size*(*i_pkt);

  // copy the data to the buffer
  memcpy(retval, packet, packet_size);

  // update packet size and location
  *buffer_size += packet_size;
  *i_pkt += 1;

  // reached the end of the buffer
  if (*i_pkt >= *n_pkt) return 0;


  return 1;
}

/* telemetry packet header (12 bytes)
 * ----------------
 * [0-1] = unique sender recver serial
 * [2-6] = auxilliary data
 * [7-8] = packet number
 * [9-10] = total number of packets
 * [11] = checksum (XOR of all preceding bytes)
 */
uint8_t writePacketHeader(uint8_t * header, uint16_t serial, uint64_t aux_data, uint16_t i_pkt, uint16_t n_pkt)
{
  int j;
  uint8_t checksum = 0;
  aux_data &= 0x000000ffffffffff; // will only copy 1st 5 bytes of aux data

  // build header
  memcpy(header+0, &serial,   2);
  memcpy(header+2, &aux_data, 5);
  memcpy(header+7, &i_pkt,    2);
  memcpy(header+9, &n_pkt,    2);

  for (j = 0; j < (PACKET_HEADER_SIZE-1); j++) checksum ^= header[j]; // check the checksum
  header[j] = checksum;

  // return the checksum
  return checksum;
}

uint8_t readPacketHeader(uint8_t * header, uint16_t *serial, uint64_t *aux_data, uint16_t *i_pkt, uint16_t *n_pkt)
{
  int j;
  uint8_t checksum = 0;
  *aux_data = 0; // zero out aux data before copying

  // extract header
  memcpy(serial,   header+0, 2);
  memcpy(aux_data, header+2, 5);
  memcpy(i_pkt,    header+7, 2);
  memcpy(n_pkt,    header+9, 2);

  for (j = 0; j < PACKET_HEADER_SIZE-1; j++) checksum ^= header[j]; // check the checksum
  checksum ^= header[j];

  // non-zero implies data error
  return checksum;
}

#ifdef __cplusplus
}

#endif


