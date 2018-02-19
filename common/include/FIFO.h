/* -----------------------------------------------------------------------
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

#ifndef FIFO_H_
#define FIFO_H_

#define MEMALLOC 0x01

#define MAX_TELEM_FIFO_SIZE 20000	// maximum number of telemetry entries in the FIFO

#ifndef MIN
#define MIN(a,b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
#define MAX(a,b) ((a) > (b) ? (a) : (b))
#endif

#ifdef __cplusplus

extern "C" {

#endif


struct Fifo
{
	unsigned int start, end;
	unsigned int length, maxsize;
	
	uint32_t *frame_num;
	uint8_t *flags;
	uint32_t *size;
	uint8_t **buffer;
};

// some function prototypes
int allocFifo(struct Fifo *, unsigned int, unsigned int);
int freeFifo(struct Fifo *);
int fifoIsEmpty(struct Fifo *);
int incrementFifo(struct Fifo *);
int decrementFifo(struct Fifo *);
void clearFifo(struct Fifo *);
uint8_t *getFifoRead(struct Fifo *);
uint8_t *getFifoLastRead(struct Fifo *);
uint8_t *getFifoWrite(struct Fifo *);
int setFifoWrite(struct Fifo *, uint8_t *);
uint8_t * packetizeBuffer(uint8_t *, uint32_t, uint32_t *, uint16_t *, uint16_t *);
uint8_t * depacketizeBuffer(uint8_t *, uint32_t *, uint32_t, uint16_t *, uint16_t *, uint8_t *);

#ifdef __cplusplus

}

#endif



#endif /* FIFO_H_ */
