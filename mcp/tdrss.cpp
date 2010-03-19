/* mcp: the BLAST master control program
 *
 * This software is copyright (C) 2002-2006 University of Toronto
 *
 * This file is part of mcp.
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*\
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
 *()                                                                        ()*|
 *() TDRSS.CPP                                                              ()*|
 *()                                                                        ()*|
 *() Compression for high rate TDRSS downlink.  Requires dataholder.o and   ()*|
 *() crc.o.                                                                 ()*|
 *()                                                                        ()*|
 *() See readme file for description of compression algorithm.              ()*|
 *()                                                                        ()*|
 *() Adam Hincks, Summer 2002, Toronto waterfront                           ()*|
 *()   `-> revised Summer 2004, Toronto                                     ()*|
 *()                                                                        ()*|
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*/

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <string.h>

#include "dataholder.h"
#include "tdrss.h"
#include "bbc_pci.h"
#include "alice.h"

extern "C" void rdft(int, int, double *); /* in fftsg_h.c */
extern "C" void nameThread(const char*);  /* in mcp.c */

extern "C" {
#include "crc.h"
#include "pointing_struct.h"
#include "channels.h"
#include "mcp.h"
#include "command_struct.h"
}

#define SMALL_LOG_FILE  "/data/etc/tdrss.log"

#define MULTIPLEX_WORD  3

#define INPUT_TTY "/dev/ttySI4"

static int tty_fd;
extern unsigned short* slow_data[FAST_PER_SLOW];

#ifdef USE_SMALL_LOG
static int smalllogspaces;
static FILE *smalllog;
#endif

/******************************************************************************\
 *                                                                            *|
 * OpenSerial: open serial port.                                              *|
 *                                                                            *|
 ******************************************************************************/

static int OpenSerial(void)
{
  int fd;
  struct termios term;

  SMALL_TRACE("");

  if ((fd = open(INPUT_TTY, O_RDWR)) < 0)
    berror(tfatal, "Unable to open serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "Unable to get serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200))
    berror(tfatal, "Error setting serial output speed");

  if (cfsetispeed(&term, B19200))
    berror(tfatal, "Error setting serial input speed");

  if (tcsetattr(fd, TCSANOW, &term))
    berror(tfatal, "Unable to set serial attributes");

  SMALL_RTN("%i", fd);

  return fd;
}


/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                              CLASS Buffer                              |**|
 **|                                                                        |**|
 **|     Constructs a buffer of data as you feed them to it.  (Not the      |**|
 **|     same class as Buffer in big.cpp.)                                  |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/


/******************************************************************************\
 *                                                                            *|
 * Buffer: constructor.                                                       *|
 *                                                                            *|
 ******************************************************************************/

Buffer::Buffer(void)
{
  SMALL_TRACE("");

  buf = (unsigned char *)balloc(fatal, 1);
  startbyte = 0;
  size = 1;

  SMALL_RTN("");
}

Buffer::~Buffer(void)
{
  SMALL_TRACE("");

  bfree(fatal, buf);

  SMALL_RTN("");
}

/******************************************************************************\
 *                                                                            *|
 * SetSize (public): set the size of the buffer and alloc.                    *|
 *                                                                            *|
 ******************************************************************************/


void Buffer::SetSize(int s)
{
  SMALL_TRACE("%i", s);

  if (s > size) {
    size = s;
    // Important to allocate more than requested, in case a frame overflows and
    // has to be truncated.  Weird memory errors can ensue . . .
    safeallocsize = size * BUFFER_SAFE_ALLOC;
    buf = (unsigned char *)reballoc(fatal, buf, safeallocsize);
  }

  SMALL_RTN("");
}


/******************************************************************************
 *                                                                            *
 * SectionSize (public): calculates size that a single field has taken in the *
 * buffer.                                                                    *
 *                                                                            *
 * Returns: amount buffer has grown since Introduce() was                     *
 * last called.                                                               *
 *                                                                            *
 ******************************************************************************/
int Buffer::SectionSize(void)
{
  int ret;

  SMALL_TRACE("");

  ret = (bitpos > 0) ?  bytepos - startbyte + 1 : bytepos - startbyte;

  SMALL_RTN("%i", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * CurrSize (public): get total size of the buffer.                           *|
 *                                                                            *|
 * Returns: size of buffer (bytes).                                           *|
 *                                                                            *|
 ******************************************************************************/

int Buffer::CurrSize(void)
{
  int ret;

  SMALL_TRACE("");

  ret = (bitpos > 0) ? bytepos + 1 : bytepos;

  SMALL_RTN("%i", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * MaxSize (public)                                                           *|
 *                                                                            *|
 * Returns: size of the buffer.                                               *|
 *                                                                            *|
 ******************************************************************************/


int Buffer::MaxSize(void)
{
  SMALL_TRACE("");

  SMALL_RTN("%i", size);

  return size;
}


void Buffer::CheckBytePosRange(int num)
{
  SMALL_TRACE("%i", num);

  if (bytepos >= safeallocsize) {
    bprintf(err,
        "serious error (%i)!  Class BUFFER was not properly allocated.  "
        "Size was set to %d; tried to write to %d.  Make sure the size is "
        "being set correctly with the Buffer::SetSize function.  Resetting "
        "bytepos to %d (compression will not work).", num, size, bytepos - 1,
        size - 1);
    bytepos = size - 1;
  }

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * Start (public): start a new buffer and write the first sync bytes.         *|
 *                                                                            *|
 * filenum  - The index of the AML file being used (files are named 0.aml,    *|
 *            1.aml . . . 15.aml).                                            *|
 *                                                                            *|
 ******************************************************************************/


void Buffer::Start(char filenum, unsigned int framenum)
{
  int i;

  SMALL_TRACE("%i %i", filenum, framenum);

  // Clear the buffer
  for (i = 0; i < size; i++)
    buf[i] = 0;

  // Start bytes
  //   11111111 11111111 11111111 11111111
  //   1111 | AML file num
  //   next 3 bytes = framenum
  //   next 2 bytes = length of frame
  //   next 2 bytes = CRC
  //
  //   Four FF bytes in a row will (almost) never happen naturally

  //for (i = BUF_POS_FRAME_SYNC; i < BUF_POS_FRAME_SYNC +
  //	BUF_LEN_FRAME_SYNC; i++)
  //buf[i] |= BUF_FRAME_SYNC;

  *(unsigned short *)buf = BUF_FRAME_SYNC;

  buf[BUF_POS_FILE_NUM] |= BUF_FILE_NUM_PADDER + filenum;

  for (i = BUF_POS_FRAME_NUM; i < BUF_POS_FRAME_NUM + BUF_LEN_FRAME_NUM; i++)
    buf[i] = (framenum >> (8 * (i - BUF_POS_FRAME_NUM))) & 0xff;

  // Frame length & CRC are written in by Stop().

  bytepos = BUF_POS_DATA_START;
  bitpos = 0;

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * Introduce (public): to be called before a new field is written to the      *|
 * buffer -- it writes the sync byte and reserves two bytes for writing in    *|
 * later the size of the section.                                             *|
 *                                                                            *|
 ******************************************************************************/


void Buffer::Introduce(void)
{
  SMALL_TRACE("");

  if (bitpos > 0)
    bytepos++;
  CheckBytePosRange(1);

  // Section intro:  10101010 + 16 bits = num bytes in section
  buf[bytepos++] = BUF_SECTION_SYNC;
  bytepos += 2; // Reserve 2 bytes to write in num bytes afterwards
  CheckBytePosRange(2);

  bitpos = 0;
  overnum = 0;
  startbyte = bytepos;

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * NoDataMarker (public): to be called if for some reason the data cannot be  *|
 * retrieved from MCP -- a marker that the field in this frame is blank.      *|
 *                                                                            *|
 ******************************************************************************/

void Buffer::NoDataMarker(void)
{
  SMALL_TRACE("");

  if (bitpos > 0)
    bytepos++;
  CheckBytePosRange(3);

  // No data marker: 10011001 = 0x99
  buf[bytepos++] = BUF_NO_DATA;
  CheckBytePosRange(4);

  bitpos = 0;
  overnum = 0;

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * RecordNumByes (public): to be called after a field has been written to the *|
 * buffer -- goes back and records the size the field's compression took up.  *|
 *                                                                            *|
 ******************************************************************************/

void Buffer::RecordNumBytes(void)
{
  SMALL_TRACE("");

  buf[startbyte - 2] = SectionSize() & 0xff;
  buf[startbyte - 1] = SectionSize() >> 8;

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * EraseLastSection (public): if the buffer has become too large, we need to  *|
 * remove the last section that was written.                                  *|
 *                                                                            *|
 ******************************************************************************/

void Buffer::EraseLastSection(void) {
  SMALL_TRACE("");

  bytepos = startbyte - 3;
  bitpos = 0;

  SMALL_RTN("");
}

/******************************************************************************\
 *                                                                            *|
 * Stop (public): writes final sync byte and sends buffer to serial port.     *|
 *                                                                            *|
 ******************************************************************************/

void Buffer::Stop(void)
{
  SMALL_TRACE("");

  if (bitpos > 0)
    bytepos++;
  CheckBytePosRange(5);

  buf[bytepos++] = BUF_END_SYNC;
  CheckBytePosRange(6);
  bitpos = 0;

  if (CurrSize() - BUF_POS_DATA_START >= 0) {
    *(unsigned short *)(buf + BUF_POS_FRAME_LEN) = CurrSize();
    *(unsigned short *)(buf + BUF_POS_CRC) =
      CalculateCRC(CRC_INIT, buf + BUF_POS_DATA_START,
          CurrSize() - BUF_POS_DATA_START);

    // Send packets
//    bprintf(info, "Emitting %i bytes of high-rate data\n", CurrSize());
    if (write(tty_fd, buf, CurrSize()) != CurrSize())
      bprintf(err, "Error sending through serial port.");
  } else
    bprintf(err, "CurSize is bogus in Buffer::Stop\n");

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * WriteChunk (private): writes a datum to the buffer.                        *|
 *                                                                            *|
 * numbits  - Length of datum in bits.                                        *|
 * datum    - The datum.                                                      *|
 *                                                                            *|
 ******************************************************************************/


void Buffer::WriteChunk(char numbits, long long datum)
{
  SMALL_TRACE("%i, %li", numbits, datum);

  // Do we have room for the whole datum in this byte?
  if (numbits - 1 > 7 - bitpos) {
    buf[bytepos++] |= (datum & (((long long)1 << (8 - bitpos)) - 1)) << bitpos;
    CheckBytePosRange(7);
    datum = datum >> (8 - bitpos);
    numbits -= 8 - bitpos;
    bitpos = 0;
  }

  // Is the remaining datum larger than 8 bits?
  while (datum > 0xff) {
    buf[bytepos++] |= datum & 0xff;
    CheckBytePosRange(8);
    datum = datum >> 8;
    numbits -= 8;
  }

  // Write the (rest of the) datum to the buffer
  buf[bytepos] |= datum << bitpos;
  bitpos += numbits;
  while(bitpos > 7) {
    bitpos -= 8;
    bytepos++;
    CheckBytePosRange(9);
  }
  //if (bitpos > 7) {
  //  bitpos = 0;
  //  bytepos++;
  //}

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * WriteTo (public): writes a datum to the buffer, adding an offset for the   *|
 * sign and dealing with any data that are larger than numbits                *|
 *                                                                            *|
 * datum    - The datum.                                                      *|
 * numbits  - The number of bits into which we ideally want to fit the datum. *|
 * oversize - The size (bits) of overflow chunks (for data that don't fit in  *|
 *            numbits).                                                       *|
 * hassign  - If the data are signed, we add an offset so all numbers are     *|
 *            positive                                                        *|
 *                                                                            *|
 ******************************************************************************/


void Buffer::WriteTo(long long datum, char numbits, char oversize,
    bool hassign)
{
  long long i;

  SMALL_TRACE("%li, %i, %i %i", datum, numbits, oversize, hassign);

  if (hassign)
    datum += ((long long)1 << (numbits - 1)) - 1;

  if (datum < ((long long)1 << numbits) - 1 && datum >= 0) { // Does the datum
    // fit in numbits?
    WriteChunk(numbits, datum);
  } else {
    // See the readme file for the compression algorithm -- it should make
    // this part somewhat clear

    overnum++;
    i = (((long long)1 << oversize) - 1);
    WriteChunk(numbits, ((long long)1 << numbits) - 1);
    if (datum < 0) {
      datum =  datum * -1 - 1;
      WriteChunk(1, 1);
    } else {
      datum = datum - ((long long)1 << numbits) + 1;
      WriteChunk(1, 0);
    }
    while (datum > i) {
      WriteChunk(oversize, datum & i);
      datum = datum >> oversize;
      WriteChunk(1, 1);
    }
    WriteChunk(oversize, datum);
    WriteChunk(1, 0);
  }

  SMALL_RTN("");
}



/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                                CLASS Alice                             |**|
 **|                                                                        |**|
 **|     `What a curious feeling!' said Alice; `I must be shutting up       |**|
 **|     like a telescope.'                                                 |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/


/******************************************************************************\
 *                                                                            *|
 * Alice: Constructor.                                                        *|
 *                                                                            *|
 ******************************************************************************/

Alice::Alice(void)
{
  SMALL_TRACE("");

  AMLsrc = -1;
  DataSource = new FrameBuffer(&tdrss_index, tdrss_data, slow_data, 1);
  sendbuf = new Buffer();    // 10 bits per byte
  DataInfo = new DataHolder();

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * GetCurrentAML (private): mcp writes the AML file number to the command     *|
 * struct whenever it receives commanding to do so.  This function            *|
 * checks this struct, and if it has changed, loads all the information from  *|
 * the file.                                                                  *|
 *                                                                            *|
 * Returns: true if the AML file changed and was loaded in.                   *|
 *                                                                            *|
 ******************************************************************************/


bool Alice::GetCurrentAML(void)
{
  char tmp[20];
  int newxml;
  int ret = false;

  SMALL_TRACE("");

  newxml = CommandData.alice_file;

  if (newxml != AMLsrc) {
    sprintf(tmp, "%s%d.aml", ALICEFILE_DIR, newxml);
    if (DataInfo->LoadFromAML(tmp)) {
      AMLsrc = newxml;
      sendbuf->SetSize(DataInfo->maxbitrate / 10 * DataInfo->looplength);
      bprintf(info, "Alice now using %s.", tmp);
      ret = true;
    }
  }

  SMALL_RTN("%i", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * Differentiate (private): differentiates data; i.e., finds the differences  *|
 * between adjacent data in a time stream, and divides all these values by    *|
 * some specified value.                                                      *|
 *                                                                            *|
 * *invals  - The data to differentiate.  The function overwrites these data  *|
 *            with their differentials.                                       *|
 * num      - The number of samples in invals.                                *|
 * divider  - Number by which to divide differential data.                    *|
 *                                                                            *|
 * Returns: Offset (integration constant).                                    *|
 *                                                                            *|
 ******************************************************************************/


double Alice::Differentiate(double *invals, int num, int divider)
{
  double offset;
  int i;

  SMALL_TRACE("%p, %i, %i", invals, num, divider);

  offset = invals[0];

  for (i = 0; i < num; i++)
    invals[i] = Round(invals[i] / divider);

  for (i = num - 1; i > 0; i--)
    invals[i] = invals[i] - invals[i - 1];
  invals[0] = 0;

  SMALL_TRACE("%i", offset);

  return offset;
}


/******************************************************************************\
 *                                                                            *|
 * Integrate (private): add adjacent data in the time stream.                 *|
 *                                                                            *|
 * *invals  - Data in, which is overwritten with the integrated data.         *|
 * num      - Number of samples in invals.                                    *|
 *                                                                            *|
 ******************************************************************************/

void Alice::Integrate(double *invals, int num)
{
  int i;

  SMALL_TRACE("%p, %i", invals, num);

  for (i = 1; i < num; i++)
    invals[i] += invals[i - 1];

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * Round (private): Smart rounding of a float.                                *|
 *                                                                            *|
 * num  - Float to be rounded.                                                *|
 *                                                                            *|
 * Returns: Integer double.                                                   *|
 *                                                                            *|
 ******************************************************************************/

double Alice::Round(double num)
{
  int ret;

  SMALL_TRACE("%e", num);

  ret = (num >= 0) ? (int)(num + 0.5) : (int)(num - 0.5);

  SMALL_RTN("%e", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * SendDiff (private): Differentiate data and send it to the buffer.          *|
 *                                                                            *|
 * *data        - Data to be compressed.                                      *|
 * num          - Number of samples in *data.                                 *|
 * *currInfo    - Pointer to structure containing info on the field           *|
 * maxover      - The maximum fraction of data we want to spill over numbits. *|
 * minover      - The minimum fraction etc.                                   *|
 *                                                                            *|
 ******************************************************************************/

void Alice::SendDiff(double *data, int num, struct DataStruct_glob *currInfo,
    float maxover, float minover)
{
  long long offset;
  int i;

  SMALL_TRACE("%p, %i, %p, %e, %e", data, num, currInfo, maxover, minover);

  // Write introductory bytes
  sendbuf->Introduce();

  offset = (long long)(Differentiate(data, num, currInfo->divider));

  sendbuf->WriteTo(offset, 24, 16, true);   // Send offset, divider information
  sendbuf->WriteTo(currInfo->divider, 4, 4, false);

  // Write the data
  for (i = 1; i < num; i++)
    sendbuf->WriteTo((long long)data[i], currInfo->numbits,
        currInfo->overflowsize, true);

  sendbuf->RecordNumBytes();

  // Adjust the divider if too much/little was going over numbits
  if (currInfo->forcediv == false) {
    if (100 * float(sendbuf->overnum) / float(num) > maxover)
      currInfo->divider ++;
    else if (100 * float(sendbuf->overnum) / float(num) < minover)
      currInfo->divider--;
    if (currInfo->divider <= 0)
      currInfo->divider = 1;
  }

#ifdef USE_SMALL_LOG
  if (smalllog != NULL) {
    fprintf(smalllog, "DIFF (%d samples): offset = %lld, divider = %d\n", num,
        offset, currInfo->divider);
    for (i = 0; i < smalllogspaces; i++)
      fprintf(smalllog, " ");
    fprintf(smalllog, "      compression = %0.2f%% (%d bytes), "
        "spillover = %0.2f%%\n",
        100 * float(sendbuf->SectionSize()) / float(num * sizeof(int)),
        sendbuf->SectionSize(), 100 * float(sendbuf->overnum) / float(num));
    fflush(smalllog);
  }
#endif

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * SendInt (private):  prepare integral preserving data and send it to the    *|
 * buffer.                                                                    *|
 *                                                                            *|
 * *data        - Data to be compressed.                                      *|
 * num          - Number of samples in *data.                                 *|
 * *currInfo    - Pointer to struct containing info on the current field.     *|
 * maxover      - The maximum fraction of data we want to spill over numbits. *|
 * minover      - The minimum fraction etc.                                   *|
 *                                                                            *|
 ******************************************************************************/

void Alice::SendInt(double *data, int num, struct DataStruct_glob *currInfo,
    float maxover, float minover)
{
  int i;

  SMALL_TRACE("%p, %i, %p, %e, %e", data, num, currInfo, maxover, minover);

  // Write introductory bytes
  sendbuf->Introduce();

  // We want the integral to be preserved, so the place where we do the division
  // is important.
  Integrate(data, num);
  data[0] = Round(Differentiate(data, num, currInfo->divider) /
      currInfo->divider);

  sendbuf->WriteTo(currInfo->divider, 4, 4, false);  // Send offset and divider
  sendbuf->WriteTo((long long)data[0], 24, 16, true);

  // Send data
  for (i = 1; i < num; i++)
    sendbuf->WriteTo((long long)(data[i] - data[0]), currInfo->numbits,
        currInfo->overflowsize, true);

  sendbuf->RecordNumBytes();

  // Adjust the divider if too much/little was going over numbits
  if (currInfo->forcediv == false) {
    if (100 * float(sendbuf->overnum) / float(num) > maxover)
      currInfo->divider ++;
    else if (100 * float(sendbuf->overnum) / float(num) < minover)
      currInfo->divider--;
    if (currInfo->divider <= 0)
      currInfo->divider = 1;
  }

#ifdef USE_SMALL_LOG
  if (smalllog != NULL) {
    fprintf(smalllog, "DIFF (%d samples): offset = %lld, divider = %d\n", num,
        (long long)data[0], currInfo->divider);
    for (i = 0; i < smalllogspaces; i++)
      fprintf(smalllog, " ");
    fprintf(smalllog, "      compression = %0.2f%% (%d bytes), "
        "spillover = %0.2f%%\n",
        100 * float(sendbuf->SectionSize()) / float(num * sizeof(int)),
        sendbuf->SectionSize(), 100 * float(sendbuf->overnum) / float(num));
    fflush(smalllog);
  }
#endif

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * SendSingle (private):  send down the first value in the time stream.       *|
 *                                                                            *|
 * *data        - The time stream.                                            *|
 * *currInfo    - Pointer to struct containing info on the current field.     *|
 *                                                                            *|
 ******************************************************************************/

void Alice::SendSingle(double *data, struct DataStruct_glob *currInfo)
{
  double divider, sendval;

  SMALL_TRACE("%p, %p", data, currInfo);

  divider = (double)(currInfo->maxval - currInfo->minval) /
    (double)(((long long)1 << currInfo->numbits) - 1);
  sendval = Round((data[0] - (double)currInfo->minval) / divider);

  // Write the first value from *data
  sendbuf->WriteTo((long long)sendval, currInfo->numbits,
      currInfo->overflowsize, false);
#ifdef USE_SMALL_LOG
  if (smalllog != NULL) {
    fprintf(smalllog, "SINGLE: value = %lld, sendval = %lld, divider = %g\n",
        (long long)data[0], (long long)sendval, divider);
    fflush(smalllog);
  }
#endif

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * SendAverage (private): obtain average of time stream and write to buffer.  *|
 *                                                                            *|
 * *data        - The data.                                                   *|
 * *currInfo    - Pointer to struct containing info on the current field.     *|
 *                                                                            *|
 ******************************************************************************/

void Alice::SendAverage(double *data, int num,
    struct DataStruct_glob *currInfo) {
  int i;
  double sum;
  double divider, sendval;

  SMALL_TRACE("%p, %i, %p", data, num, currInfo);

  // Find average
  sum = 0;
  for (i = 0; i < num; i++)
    sum += data[i];

  divider = (double)(currInfo->maxval - currInfo->minval) /
    (double)(((long long)1 << currInfo->numbits) - 1);
  sendval = Round((sum / (double)num - (double)currInfo->minval) / divider);

  // Write average value to buffer
  sendbuf->WriteTo((long long)sendval, currInfo->numbits,
      currInfo->overflowsize, false);
#ifdef USE_SMALL_LOG
  if (smalllog != NULL) {
    fprintf(smalllog, "AVG: value = %.12g, sendval = %lld, divider = %g\n",
        Round(sum / num), (long long)sendval, divider);
    fflush(smalllog);
  }
#endif

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * MaxPowTwo (private): checks all fields and sees which has the greatest     *|
 * FindPowTwo (see below).                                                    *|
 *                                                                            *|
 * val          - The number of frames to consider (since # samples =         *|
 *                # frames x samples per frame).                              *|
 * threshold    - See FindPowTwo below.                                       *|
 *                                                                            *|
 * Returns:  the maximum FindPowTwo.                                          *|
 *                                                                            *|
 ******************************************************************************/

int Alice::MaxPowTwo(int val, float threshold)
{
  struct DataStruct_glob *currInfo;
  int powtwo;

  SMALL_TRACE("%i %e", val, threshold);

  powtwo = 0;
  for (currInfo = DataInfo->FirstFast(); currInfo != NULL;
      currInfo = DataInfo->NextFast()) {
    if (int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1 > powtwo)
      powtwo = int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1;
  }

  SMALL_RTN("%i", powtwo);

  return powtwo;
}


/******************************************************************************\
 *                                                                            *|
 * MaxFrameFreq (private): checks all fields to see which has the greatest    *|
 * samples per frame.                                                         *|
 *                                                                            *|
 * Returns:  maximum samples per frame.                                       *|
 *                                                                            *|
 ******************************************************************************/

int Alice::MaxFrameFreq(void)
{
  struct DataStruct_glob *currInfo;
  int max = 0;

  SMALL_TRACE("");

  for (currInfo = DataInfo->FirstFast(); currInfo != NULL;
      currInfo = DataInfo->NextFast()) {
    if (currInfo->framefreq > max)
      max = currInfo->framefreq;
  }

  for (currInfo = DataInfo->FirstSlow(); currInfo != NULL;
      currInfo = DataInfo->NextSlow()) {
    if (currInfo->framefreq > max)
      max = currInfo->framefreq;
  }

  SMALL_RTN("%i", max);

  return max;
}


/******************************************************************************\
 *                                                                            *|
 * FindPowTwo (private): finds the smallest power of two greater than a given *|
 * value.                                                                     *|
 *                                                                            *|
 * val          - The value greater than which the power of two must be.      *|
 * threshold    - The fraction of the value by which the power of two must    *|
 *                exceed that value.                                          *|
 *                                                                            *|
 * Returns:  the power of two.                                                *|
 *                                                                            *|
 ******************************************************************************/

int Alice::FindPowTwo(int val, float threshold)
{
  int i, num;

  SMALL_TRACE("%i %e", val, threshold);

  num = val;
  for (i = 0; num > 0; i++)
    num = num >> 1;

  if ((1 << i) - val < int(val * threshold))
    i++;

  SMALL_RTN("%i", 1 << i);

  return 1 << i;
}


/******************************************************************************\
 *                                                                            *|
 * CompressionLoop (public): main control loop                                *|
 *                                                                            *|
 ******************************************************************************/

void Alice::CompressionLoop(void)
{
  double *filterdata, *rawdata;
  int i, j, k, l;
  struct DataStruct_glob *currInfo;
  int numframes = 0, numtoread, readleftpad, readrightpad = 0;
  int rawsize, powtwo, leftpad, rightpad;
  bool earlysend;
  int numread;
  int framepos = 0;
  int rawdatasize, filterdatasize, ts;
#ifdef USE_SMALL_LOG
  char tmpstr[80];
  time_t t;
  struct tm now;
#endif

  SMALL_TRACE("");

  rawdata = (double *)balloc(fatal, 1);
  rawdatasize = 1;
  filterdata = (double *)balloc(fatal, 1);
  filterdatasize = 1;

  for (;;) {
    // Check for new AML file command written by mcp.
    if (GetCurrentAML()) {

      // Make sure our buffers for reading in data from disk are big enough.
      ts = sizeof(double) * DataInfo->samplerate * MaxFrameFreq() *
        DataInfo->looplength;
      if (ts > rawdatasize) {
        rawdatasize = ts;
        rawdata = (double *)reballoc(fatal, rawdata, ts);
      }

      ts = sizeof(double) * (3 * DataInfo->samplerate * MaxFrameFreq() *
          DataInfo->looplength);
      if (ts > filterdatasize) {
        filterdatasize = ts;
        filterdata = (double *)reballoc(fatal, filterdata, ts);
      }

      // Figure out how much we need to read each time around.  For FFT purposes
      // it needs to be a power of two, with at least 5 % buffering on the ends.
      numframes = DataInfo->looplength * DataInfo->samplerate;
      numtoread = MaxPowTwo(numframes, 0.05);

      // We need real data on either side of the data we are interested in so
      // that funny things don't happen to the edges of our data during an FFT.
      // Calculate the maximum padding we will need for all our fields.
      readleftpad = int((numtoread - numframes) / 2);
      readrightpad = numtoread - numframes - readleftpad + 2;

      // Make sure our data source has a large enough buffer:  keep three
      // compression chunks in memory at a time.
      DataSource->Resize(numtoread * 3);


      // Wait until there exists enough data for this padding
      framepos = DataSource->NumFrames();
      while (DataSource->NumFrames() < framepos + readleftpad + readrightpad)
        usleep(1000);

      framepos += readleftpad + readrightpad;
    }

    // Wait until mcp has written numframes of data
    while (DataSource->NumFrames() < framepos + numframes)
      usleep(1000);

    // Start a new buffer for the downlink
#ifdef USE_SMALL_LOG
    if (smalllog != NULL) {
      t = mcp_systime(NULL);
      strftime(tmpstr, 80, "%F %T GMT >> ", gmtime_r(&t, &now));
      fprintf(smalllog, "\n%sStarting new TDRSS frame.  AML file = %d.\n",
          tmpstr, AMLsrc);
      fflush(smalllog);
    }
#endif
    sendbuf->Start(AMLsrc, (unsigned int)(framepos - readrightpad));
    i = 0;
    earlysend = false;

    // Go through each of the slow fields we want to compress.  Slow fields only
    // send down one value per frame.  They are treated like one big "fast"
    // field -- we Introduce() before all of them and then RecordNumBytes()
    // after them.  All the slow fields are written one after the other here
    // at the beginning of the frame.
    if ((currInfo = DataInfo->FirstSlow()) != NULL) {
      sendbuf->Introduce();

      for (currInfo = DataInfo->FirstSlow(); currInfo != NULL;
          currInfo = DataInfo->NextSlow()) {

#ifdef USE_SMALL_LOG
        if (smalllog != NULL) {
          fprintf(smalllog, "  %s -> ", currInfo->src);
          smalllogspaces = strlen(currInfo->src);
        }
#endif
        switch (currInfo->type) {
          case COMP_SINGLE:
            if ((numread = DataSource->ReadField(rawdata, currInfo->src,
                    framepos - readrightpad, 1)) != currInfo->framefreq) {
              bprintf(err, "Error accessing correct number of data "
                  "from frames (%s, %d, %d).", currInfo->src, numread,
                  currInfo->framefreq);
              rawdata[0] = 0;
              SendSingle(rawdata, currInfo);  // Send down a zero
            } else
                      SendSingle(rawdata, currInfo);
            break;

          case COMP_AVERAGE:
            rawsize = numframes * currInfo->framefreq;
            if ((numread = DataSource->ReadField(rawdata, currInfo->src,
                    framepos - readrightpad, numframes))
                != rawsize) {
              bprintf(err, "Error accessing correct number of data "
                  "from frames (%d, %d).", numread, currInfo->framefreq);
              rawdata[0] = 0;
              SendSingle(rawdata, currInfo);  // Send down a zero
            } else
                  SendAverage(rawdata, rawsize, currInfo);
            break;
        }
      }

      sendbuf->RecordNumBytes();
    }


    // Go through each of the fast fields we want to compress.  Fast fields
    // are those which send down more than one value per frame.
    for (currInfo = DataInfo->FirstFast(); currInfo != NULL && !earlysend;
        currInfo = DataInfo->NextFast()) {

      // Figure out how much to read and how much padding the current field
      // requires
      rawsize = numframes * currInfo->framefreq;
      powtwo = FindPowTwo(rawsize, 0.05);
      leftpad = int((powtwo - rawsize) / (2 * currInfo->framefreq));
      rightpad = int((powtwo - rawsize - leftpad * currInfo->framefreq) /
          currInfo->framefreq) + 1;

#ifdef USE_SMALL_LOG
      if (smalllog != NULL) {
        fprintf(smalllog, "  %s -> ", currInfo->src);
        smalllogspaces = strlen(currInfo->src);
      }
#endif

      // Read data from Frodo's disk
      if ((numread = DataSource->ReadField(filterdata, currInfo->src,
              framepos - readrightpad - leftpad,
              numframes + rightpad + leftpad))
          != rawsize + currInfo->framefreq * (rightpad + leftpad)) {

        bprintf(err, "Error accessing correct number of data from "
            "frames (%d, %d).", numread, rawsize + currInfo->framefreq *
            (rightpad + leftpad));
        sendbuf->NoDataMarker();
      } else {
        // If we aren't sending down every frame, we must do a FFT filter to get
        // rid of high frequency junk.
        if (currInfo->samplefreq > 1) {
          rdft(powtwo, 1, filterdata);
          for (j = int(powtwo / currInfo->samplefreq); j < powtwo; j++)
            filterdata[j] = 0;
          rdft(powtwo, -1, filterdata);
          k = int(rawsize / currInfo->samplefreq);
          l = currInfo->framefreq * leftpad;
          for (j = 0; j < k; j++)
            rawdata[j] = filterdata[j * currInfo->samplefreq + l] *
              2.0 / powtwo;
          rawsize = int(rawsize / currInfo->samplefreq);
        } else
          memcpy(rawdata, filterdata + leftpad * currInfo->framefreq,
              sizeof(double) * rawsize);

        switch (currInfo->type) {
          case COMP_DIFFERENTIAL:
            SendDiff(rawdata, rawsize, currInfo, DataInfo->maxover,
                DataInfo->minover);
            break;
          case COMP_INT_PRESERVING:
            SendInt(rawdata, rawsize, currInfo, DataInfo->maxover,
                DataInfo->minover);
            break;
        }

        // Check the overall size
        if (sendbuf->CurrSize() > sendbuf->MaxSize()) {
          sendbuf->EraseLastSection();
          bputs(warning, "Frame Truncated.");
#ifdef USE_SMALL_LOG
          if (smalllog != NULL) {
            fprintf(smalllog, "WARNING: frame truncated!\n");
            fflush(smalllog);
          }
#endif
          earlysend = true;
        }
      }
    }
    // Send down the compressed buffer
    sendbuf->Stop();
#ifdef USE_SMALL_LOG
    if (smalllog != NULL) {
      fprintf(smalllog, "End of TDRSS frame.  Total size = %d bytes.\n",
          sendbuf->CurrSize());
      fflush(smalllog);
    }
#endif
    framepos += numframes;
  }

  SMALL_RTN("");
}


/******************************************************************************\
 *                                                                            *|
 * Alice: Destructor                                                          *|
 *                                                                            *|
 ******************************************************************************/

Alice::~Alice()
{
  delete DataSource;
  delete sendbuf;
  delete DataInfo;

#ifdef USE_SMALL_LOG
  if (smalllog != NULL)
    fclose(smalllog);
#endif

  SMALL_RTN("");
}


/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                            CLASS FrameBuffer                           |**|
 **|                                                                        |**|
 **|     Intercepts mcp frames and buffers them locally for Alice.          |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/


/******************************************************************************\
 *                                                                            *|
 * FrameBuffer:  constructor.                                                 *|
 *                                                                            *|
 ******************************************************************************/

FrameBuffer::FrameBuffer(unsigned int *mcpindex_in,
    unsigned short **fastdata_in,
    unsigned short **slowdata_in, int numframes_in)
{
  SMALL_TRACE("%p, %p, %p, %i", mcpindex_in, fastdata_in, slowdata_in,
      numframes_in);

  mcpindex = mcpindex_in;
  lastmcpindex = 2;
  fastdata = fastdata_in;
  slowdata = slowdata_in;
  memallocated = false;
  numframes = -1;

  Resize(numframes_in);

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * Resize (public): change the size of the circular buffer grabbed from mcp   *|
 * frames.                                                                    *|
 *                                                                            *|
 * numframes_in     - The size of the buffer, in units of 5 Hz frames.        *|
 *                                                                            *|
 ******************************************************************************/

void FrameBuffer::Resize(int numframes_in)
{
  int i, j;

  SMALL_TRACE("%i", numframes_in);

  if (numframes_in == numframes) {
    // Do nothing.

    SMALL_RTN("");

    return;
  }

  if (memallocated) {
    exitupdatethread = true;
    while (exitupdatethread) {
      usleep(1000);
    }

    for (i = 0; i < numframes; i++) {
      for (j = 0; j < FAST_PER_SLOW; j++) {
        bfree(fatal, slowbuf[i][j]);
        bfree(fatal, fastbuf[i][j]);
      }
      bfree(fatal, slowbuf[i]);
      bfree(fatal, fastbuf[i]);
    }
    bfree(fatal, slowbuf);
    bfree(fatal, fastbuf);
  }

  numframes = numframes_in;

  fastbuf = (unsigned short ***)balloc(fatal, numframes *
      sizeof(unsigned short **));

  slowbuf = (unsigned short ***)balloc(fatal, numframes *
      sizeof(unsigned short **));

  for (i = 0; i < numframes; i++) {
    fastbuf[i] = (unsigned short **)balloc(fatal, FAST_PER_SLOW *
        sizeof(unsigned short *));

    slowbuf[i] = (unsigned short **)balloc(fatal, FAST_PER_SLOW *
        sizeof(unsigned short *));

    for (j = 0; j < FAST_PER_SLOW; j++) {
      fastbuf[i][j] = (unsigned short *)balloc(fatal, BiPhaseFrameSize);
      slowbuf[i][j] = (unsigned short *)balloc(fatal, slowsPerBi0Frame *
          sizeof(unsigned short));
    }
  }

  framenum = -1;
  memallocated = true;
  multiplexsynced = false;
  pseudoframe = -1;
  exitupdatethread = false;

  pthread_create(&update_id, NULL, FrameBuffer::UpdateThreadEntry, this);

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * UpdateThreadEntry (private): an annoying circumposition to start another   *|
 * thread in an object.  One creates a pointer to the calling object and then *|
 * calls the thread routine.                                                  *|
 *                                                                            *|
 * *pthis   - Pointer to the calling object.                                  *|
 *                                                                            *|
 ******************************************************************************/

void *FrameBuffer::UpdateThreadEntry(void *pthis)
{
  SMALL_TRACE("%p", pthis);

  bputs(startup, "TDRSS Update: Startup.\n");

  FrameBuffer *mine = (FrameBuffer *)pthis;
  mine->Update();

  bputs(info, "TDRSS Update: Done.\n");

  SMALL_RTN("%p", NULL);

  return NULL;
}


/******************************************************************************\
 *                                                                            *|
 * Update (private): copy new mcp frames into the circular buffer.            *|
 *                                                                            *|
 ******************************************************************************/

void FrameBuffer::Update(void)
{
  unsigned int i;
  int j;

  SMALL_TRACE("");

  while (1 == 1) {
    usleep(1000);
    if (exitupdatethread) {
      exitupdatethread = false;
      break;
    }

    if ((i = GETREADINDEX(*mcpindex)) != lastmcpindex) {
      lastmcpindex = i;
      multiplexindex = fastdata[i][MULTIPLEX_WORD];
      if (!multiplexsynced && multiplexindex) // We want to start out with
        continue;                             // multiplex index 0
      multiplexsynced = true;

      if (multiplexindex < 0 || multiplexindex >= FAST_PER_SLOW) {
        bprintf(err, "Multiplex index out of range (= %d)",
            multiplexindex);
        continue;
      }
      if (i < 0 || i >= 3) {
        bprintf(err, "Biphase buffer index out of range (= %d)", i);
        continue;
      }

      if (!multiplexindex) {
        if (++framenum >= numframes)
          framenum = 0;
        pseudoframe++;

        for (j = 0; j < FAST_PER_SLOW; j++)
          memcpy(slowbuf[framenum][j], slowdata[j], slowsPerBi0Frame *
              sizeof(unsigned short));
      }
      memcpy(fastbuf[framenum][multiplexindex], fastdata[i], BiPhaseFrameSize);
    }
  }

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * NumFrames (public)                                                         *|
 *                                                                            *|
 * Returns: the linear frame counter.                                         *|
 *                                                                            *|
 ******************************************************************************/

int FrameBuffer::NumFrames(void)
{
  SMALL_TRACE("");

  SMALL_RTN("%i", pseudoframe);

  return pseudoframe;
}


/******************************************************************************\
 *                                                                            *|
 * ReadField (public): read a field of the 5 Hz frame.  User must ensure that *|
 *                                                                            *|
 * *returnbuf   - Buffer in which to place the field's data.  User must       *|
 *                ensure that enough memory is allocated to fit requested     *|
 *                data.                                                       *|
 * *fieldname   - The field source name.                                      *|
 * framenum_in  - The starting frame to read.                                 *|
 * numframes_in - The number of frames past framenum_in to read.              *|
 *                                                                            *|
 * Returns: the number of values placed in *returnbuf.                        *|
 *                                                                            *|
 ******************************************************************************/

int FrameBuffer::ReadField(double *returnbuf, const char *fieldname,
    int framenum_in, int numframes_in)
{
  int i, j, k, truenum, wide, mindex, chnum[2]={0,0};
  unsigned short mask;
  struct NiosStruct *address[2];
  unsigned short msb, lsb;
  char tmpstr[64];

  SMALL_TRACE("%p, %p, %i %i", returnbuf, fieldname, framenum_in, numframes_in);

  // Is the field a bolometer?  The first character is 'n', followed by a number
  // of 1 or 2 characters, followed by 'c'.
  if (fieldname[0] == 'n' &&
      fieldname[1] >= 48 && fieldname[1] <= 57 &&
      ((fieldname[2] >= 48 && fieldname[2] <= 57 &&
        fieldname[3] == 'c') ||
       (fieldname[2] == 'c'))) {

    // Get the channel number.
    for (i = 0; i < (signed int)strlen(fieldname) && fieldname[i] != 'c'; i++);
    // Let 'wide' represent whether the ch. num. is even or odd.
    wide = atoi(fieldname + i + 1) % 2;

    // Get the address.
    sprintf(tmpstr, "%slo", fieldname);
    if ((address[0] = GetNiosAddr(tmpstr)) == NULL) {
      SMALL_RTN("%i", 0);
      return 0;
    }
    strcpy(tmpstr, "");
    strncpy(tmpstr, fieldname, i);
    tmpstr[i + 1] = '\0';
    sprintf(tmpstr, "%s%dhi", tmpstr, atoi(fieldname + i + 1) - wide);
    if ((address[1] = GetNiosAddr(tmpstr)) == NULL) {
      SMALL_RTN("%i", 0);
      return 0;
    }

    mindex = NOT_MULTIPLEXED + 1;  // Marker for bolometers.
    if (wide)
      mask = 0x00ff;
    else
      mask = 0xff00;
    wide = 8 + wide * 8; // Now let wide be the amount to shift the 'hi' word.
    chnum[0] = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].channel;
    chnum[1] = BiPhaseLookup[BI0_MAGIC(address[1]->bbcAddr)].channel;
  } else {
    if ((address[0] = GetNiosAddr(fieldname)) == NULL) {
      SMALL_RTN("%i", 0);
      return 0;
    }
    wide = address[0]->wide;
    mindex = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].index;
    chnum[0] = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].channel;
    mask = 0;
  }

  if (pseudoframe - framenum_in > numframes || framenum_in > pseudoframe) {
    SMALL_RTN("%i", 0);
    return 0;
  }

  truenum = framenum_in - (int)((double)framenum_in / (double)numframes) *
    numframes;
  if (truenum < 0)
    truenum += numframes;
  for (i = framenum_in, j = 0; i < framenum_in + numframes_in; i++) {
    if (mindex == NOT_MULTIPLEXED) {
      for (k = 0; k < FAST_PER_SLOW; k++) {
        lsb = fastbuf[truenum][k][chnum[0]];
        if (wide)
          msb = fastbuf[truenum][k][chnum[0] + 1];
        else
          msb = 0;

        returnbuf[j++] = (double)((msb << 16) | lsb);
      }
    } else if (mindex == NOT_MULTIPLEXED + 1) {
      // Bolometers are all fast channels.
      for (k = 0; k < FAST_PER_SLOW; k++)
        returnbuf[j++] = ((fastbuf[truenum][k][chnum[1]] & mask) << wide) |
          fastbuf[truenum][k][chnum[0]];
    } else {
      lsb = slowbuf[truenum][mindex][chnum[0]];
      if (wide)
        msb = slowbuf[truenum][mindex][chnum[0] + 1];
      else
        msb = 0;

      returnbuf[j++] = (double)((msb << 16) | lsb);
    }

    if (++truenum >= numframes)
      truenum = 0;
  }

  SMALL_RTN("%i", j);

  return j;
}


/******************************************************************************\
 *                                                                            *|
 * ~FrameBuffer: destructor.                                                  *|
 *                                                                            *|
 ******************************************************************************/

FrameBuffer::~FrameBuffer(void)
{
  int i, j;

  SMALL_TRACE("");

  if (memallocated) {
    for (i = 0; i < numframes; i++) {
      for (j = 0; j < FAST_PER_SLOW; j++) {
        bfree(fatal, slowbuf[i][j]);
        bfree(fatal, fastbuf[i][j]);
      }
      bfree(fatal, slowbuf[i]);
      bfree(fatal, fastbuf[i]);
    }
    bfree(fatal, slowbuf);
    bfree(fatal, fastbuf);
  }

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                                TDRSWriter                              |**|
 **|                                                                        |**|
 **|                          Entry point from mcp.                         |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/

extern "C" void TDRSSWriter(void) {
  Alice *drinkme;

  nameThread("TDRSS");

#ifdef USE_SMALL_LOG
  if ((smalllog = fopen(SMALL_LOG_FILE, "w")) == NULL)
    bprintf(warning, "Could not open tdrss log file (%s).",
        SMALL_LOG_FILE);
#endif

  SMALL_TRACE("");

  bputs(startup, "Startup.\n");

  tty_fd = OpenSerial();

  drinkme = new Alice();
  drinkme->CompressionLoop();

  delete drinkme;

  close(tty_fd);

  SMALL_RTN("");

  return;
}
