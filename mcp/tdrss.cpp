/*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*\
|*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
|*()                                                                        ()*|
|*() SMALL.CPP                                                              ()*|
|*()                                                                        ()*|
|*() Compression for high rate TDRSS downlink.  Requires dataholder.o and   ()*|
|*() crc.o.                                                                 ()*|
|*()                                                                        ()*|
|*() See readme file for description of compression algorithm.              ()*|
|*()                                                                        ()*|
|*() Adam Hincks, Summer 2002, Toronto waterfront                           ()*|
|*()   `-> revised Summer 2004, Toronto                                     ()*|
|*()                                                                        ()*|
|*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
\*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*/

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
#include "small.h"
#include "fftsg_h.c"
#include "bbc_pci.h"
#include "alice.h"

extern "C" {
#include "crc.h"
#include "pointing_struct.h"
#include "tx_struct.h"
#include "mcp.h"
}

#define ALICEFILE_DIR   "./"
#define MULTIPLEX_WORD  3

#define INPUT_TTY "/dev/ttyS5"

int tty_fd;
extern unsigned short* slow_data[FAST_PER_SLOW];

/******************************************************************************\
|*                                                                            *|
|* OpenSerial: open serial port.                                              *|
|*                                                                            *|
\******************************************************************************/

int OpenSerial() {
  int fd;
  struct termios term;

  if ((fd = open(INPUT_TTY, O_RDWR)) < 0)
    merror(MCP_TFATAL, "Unable to open serial port");

  if (tcgetattr(fd, &term))
    merror(MCP_TFATAL, "Unable to get serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200))
    merror(MCP_TFATAL, "Error setting serial output speed");

  if (cfsetispeed(&term, B19200))
    merror(MCP_TFATAL, "Error setting serial input speed");

  if (tcsetattr(fd, TCSANOW, &term))
    merror(MCP_TFATAL, "Unable to set serial attributes");

  return fd;
}


/******************************************************************************\
|*                                                                            *|
|* printbin: prints binary number at cursor position.                         *|
|*                                                                            *|
\******************************************************************************/


void printbin(long num) {
  int i;

  for (i = 24; i >= 0; i--) {
    if (((1 << i) & num) != 0)
      printf("1");
    else
      printf("0");
  }
}


/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                              CLASS Buffer                              |**|
|**|                                                                        |**|
|**|     Constructs a buffer of data as you feed them to it.  (Not the      |**|
|**|     same class as Buffer in big.cpp.)                                  |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/


/******************************************************************************\
|*                                                                            *|
|* Buffer: constructor.                                                       *|
|*                                                                            *|
\******************************************************************************/

Buffer::Buffer() {
  buf = (unsigned char *)malloc(1);
  startbyte = 0;
  size = 1;
}

Buffer::~Buffer() {
  free(buf);
}

/******************************************************************************\
|*                                                                            *|
|* SetSize (public): set the size of the buffer and malloc.                   *|
|*                                                                            *|
\******************************************************************************/


void Buffer::SetSize(int s) {
  if (s > size) {
    size = s;
    // Important to allocate more than requested, in case a frame overflows and
    // has to be truncated.  Weird memory errors can ensue . . .
    safeallocsize = size * BUFFER_SAFE_ALLOC;
    buf = (unsigned char *)realloc(buf, safeallocsize);
  }
}


/******************************************************************************\
|*                                                                            *|
|* SectionSize (public): calculates size that a single field has taken in the *|
|* buffer.                                                                    *|
|*                                                                            *|
|* Returns: amount buffer has grown since Introduce() was                     *|
|* last called.                                                               *|
|*                                                                            *|
\******************************************************************************/


  int Buffer::SectionSize() {
    if (bitpos > 0)
      return bytepos - startbyte + 1;
    else
      return bytepos - startbyte;
  }


/******************************************************************************\
|*                                                                            *|
|* CurrSize (public): get total size of the buffer.                           *|
|*                                                                            *|
|* Returns: size of buffer (bytes).                                           *|
|*                                                                            *|
\******************************************************************************/


  int Buffer::CurrSize() {
    if (bitpos > 0)
      return bytepos + 1;
    else
      return bytepos;
  }


/******************************************************************************\
|*                                                                            *|
|* MaxSize (public)                                                           *|
|*                                                                            *|
|* Returns: size of the buffer.                                               *|
|*                                                                            *|
\******************************************************************************/


int Buffer::MaxSize() {
  return size;
}


void Buffer::CheckBytePosRange() {
  if (bytepos >= safeallocsize) {
    mprintf(MCP_ERROR,
        "Alice: serious error!!  Class BUFFER was not properly allocated.  "
        "Size was set to %d; tried to write to %d.  Make sure the size is "
        "being set correctly with the Buffer::SetSize function.  Resetting "
        "bytepos to %d (compression will not work).", size, bytepos - 1,
        size - 1);
    bytepos = size - 1;
  }

  return;
}


/******************************************************************************\
|*                                                                            *|
|* Start (public): start a new buffer and write the first sync bytes.         *|
|*                                                                            *|
|* filenum  - The index of the AML file being used (files are named 0.aml,    *|
|*            1.aml . . . 15.aml).                                            *|
|*                                                                            *|
\******************************************************************************/


void Buffer::Start(char filenum, unsigned int framenum) {
  int i;

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

  for (i = BUF_POS_FRAME_SYNC; i < BUF_POS_FRAME_SYNC + BUF_LEN_FRAME_SYNC; i++)
    buf[i] |= BUF_FRAME_SYNC;

  buf[BUF_POS_FILE_NUM] |= BUF_FILE_NUM_PADDER + filenum;

  for (i = BUF_POS_FRAME_NUM; i < BUF_POS_FRAME_NUM + BUF_LEN_FRAME_NUM; i++)
    buf[i] = (framenum >> (8 * (i - BUF_POS_FRAME_NUM))) & 0xff;

  // Frame length & CRC are written in by Stop().

  bytepos = BUF_POS_DATA_START;
  bitpos = 0;
}


/******************************************************************************\
|*                                                                            *|
|* Introduce (public): to be called before a new field is written to the      *|
|* buffer -- it writes the sync byte and reserves two bytes for writing in    *|
|* later the size of the section.                                             *|
|*                                                                            *|
\******************************************************************************/


  void Buffer::Introduce() {
    if (bitpos > 0)
      bytepos++;
    CheckBytePosRange();

    // Section intro:  10101010 + 16 bits = num bytes in section
    buf[bytepos++] = BUF_SECTION_SYNC;
    bytepos += 2; // Reserve 2 bytes to write in num bytes afterwards
    CheckBytePosRange();

    bitpos = 0;
    overnum = 0;
    startbyte = bytepos;
  }


/******************************************************************************\
|*                                                                            *|
|* NoDataMarker (public): to be called if for some reason the data cannot be  *|
|* retrieved from MCP -- a marker that the field in this frame is blank.      *|
|*                                                                            *|
\******************************************************************************/

  void Buffer::NoDataMarker() {
    if (bitpos > 0)
      bytepos++;
    CheckBytePosRange();

    // No data marker: 10011001 = 0x99
    buf[bytepos++] = BUF_NO_DATA;
    CheckBytePosRange();

    bitpos = 0;
    overnum = 0;
  }


/******************************************************************************\
|*                                                                            *|
|* RecordNumByes (public): to be called after a field has been written to the *|
|* buffer -- goes back and records the size the field's compression took up.  *|
|*                                                                            *|
\******************************************************************************/

void Buffer::RecordNumBytes() {
  buf[startbyte - 2] = SectionSize() & 0xff;
  buf[startbyte - 1] = SectionSize() >> 8;
}


/******************************************************************************\
|*                                                                            *|
|* EraseLastSection (public): if the buffer has become too large, we need to  *|
|* remove the last section that was written.                                  *|
|*                                                                            *|
\******************************************************************************/

void Buffer::EraseLastSection() {
  bytepos = startbyte - 3;
  bitpos = 0;
}

/******************************************************************************\
|*                                                                            *|
|* Stop (public): writes final sync byte and sends buffer to serial port.     *|
|*                                                                            *|
\******************************************************************************/

  void Buffer::Stop() {
    if (bitpos > 0)
      bytepos++;
    CheckBytePosRange();

    buf[bytepos++] = BUF_END_SYNC;
    CheckBytePosRange();
    bitpos = 0;

    *(unsigned short *)(buf + BUF_POS_FRAME_LEN) = CurrSize();
    *(unsigned short *)(buf + BUF_POS_CRC) = 
      CalculateCRC(CRC_INIT, buf + BUF_POS_DATA_START, 
          CurrSize() - BUF_POS_DATA_START);

    // Send packets
    if (write(tty_fd, buf, CurrSize()) != CurrSize())
      mprintf(MCP_ERROR, "Error sending through serial port.");
  }


/******************************************************************************\
|*                                                                            *|
|* WriteChunk (private): writes a datum to the buffer.                        *|
|*                                                                            *|
|* numbits  - Length of datum in bits.                                        *|
|* datum    - The datum.                                                      *|
|*                                                                            *|
\******************************************************************************/


void Buffer::WriteChunk(char numbits, long long datum) {
  // Do we have room for the whole datum in this byte?
  if (numbits - 1 > 7 - bitpos) {
    buf[bytepos++] |= (datum & (((long long)1 << (8 - bitpos)) - 1)) << bitpos;
    CheckBytePosRange();
    datum = datum >> (8 - bitpos);
    numbits -= 8 - bitpos;
    bitpos = 0;
  }

  // Is the remaining datum larger than 8 bits?
  while (datum > 0xff) {
    buf[bytepos++] |= datum & 0xff;
    CheckBytePosRange();
    datum = datum >> 8;
    numbits -= 8;
  }

  // Write the (rest of the) datum to the buffer
  buf[bytepos] |= datum << bitpos;
  bitpos += numbits;
  while(bitpos > 7) {
    bitpos -= 8;
    bytepos++;
    CheckBytePosRange();
  }
  //if (bitpos > 7) {
  //  bitpos = 0;
  //  bytepos++;
  //}
}


/******************************************************************************\
|*                                                                            *|
|* WriteTo (public): writes a datum to the buffer, adding an offset for the   *|
|* sign and dealing with any data that are larger than numbits                *|
|*                                                                            *|
|* datum    - The datum.                                                      *|
|* numbits  - The number of bits into which we ideally want to fit the datum. *|
|* oversize - The size (bits) of overflow chunks (for data that don't fit in  *|
|*            numbits).                                                       *|
|* hassign  - If the data are signed, we add an offset so all numbers are     *|
|*            positive                                                        *|
|*                                                                            *|
\******************************************************************************/


void Buffer::WriteTo(long long datum, char numbits, char oversize,
    bool hassign) {
  long long i;

  if (hassign)
    datum += ((long long)1 << (numbits - 1)) - 1;

  if (datum < ((long long)1 << numbits) - 1 && datum >= 0) { // Does the datum 
    // fit in numbits?
    WriteChunk(numbits, datum);
  }
  else {
    // See the readme file for the compression algorithm -- it should make
    // this part somewhat clear

    overnum++;
    i = (((long long)1 << oversize) - 1);
    WriteChunk(numbits, ((long long)1 << numbits) - 1);
    if (datum < 0) {
      datum =  datum * -1 - 1;
      WriteChunk(1, 1);
    }
    else {
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
}



/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                                CLASS Alice                             |**|
|**|                                                                        |**|
|**|     `What a curious feeling!' said Alice; `I must be shutting up       |**|
|**|     like a telescope.'                                                 |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/


/******************************************************************************\
|*                                                                            *|
|* Alice: Constructor.                                                        *|
|*                                                                            *|
\******************************************************************************/

Alice::Alice() {
  AMLsrc = -1;
  DataSource = new FrameBuffer(&tdrss_index, tdrss_data, slow_data, 1);
  sendbuf = new Buffer();    // 10 bits per byte
  DataInfo = new DataHolder();
}


/******************************************************************************\
|*                                                                            *|
|* GetCurrentAML (private): mcp writes the AML file number to                 *|
|* /tmp/alice_index whenever it receives commanding to do so.  This function  *|
|* checks this file, and if it has changed, loads all the information from    *|
|* the file.                                                                  *|
|*                                                                            *|
|* Returns: true if the AML file changed and was loaded in.                   *|
|*                                                                            *|
\******************************************************************************/


bool Alice::GetCurrentAML() {
  char tmp[20];
  int newxml;
  FILE *fp;

  if ((fp = fopen("/tmp/alice_index", "r")) == NULL)
    newxml = 0;
  else {
    fscanf(fp, "%d", &newxml);
    fclose(fp);
  }

  if (newxml != AMLsrc) {
    sprintf(tmp, "%s%d.aml", ALICEFILE_DIR, newxml);
    if (DataInfo->LoadFromAML(tmp)) {
      AMLsrc = newxml;
      sendbuf->SetSize(DataInfo->maxbitrate / 10 * DataInfo->looplength);
      mprintf(MCP_INFO, "SMALL (Alice):  now using .aml file number %d.", 
          AMLsrc);
      return true;
    }
  }

  return false;
}


/******************************************************************************\
|*                                                                            *|
|* Differentiate (private): differentiates data; i.e., finds the differences  *|
|* between adjacent data in a time stream, and divides all these values by    *|
|* some specified value.                                                      *|
|*                                                                            *|
|* *invals  - The data to differentiate.  The function overwrites these data  *|
|*            with their differentials.                                       *|
|* num      - The number of samples in invals.                                *|
|* divider  - Number by which to divide differential data.                    *|
|*                                                                            *|
|* Returns: Offset (integration constant).                                    *|
|*                                                                            *|
\******************************************************************************/


double Alice::Differentiate(double *invals, int num, int divider) {
  double offset;
  int i;

  offset = invals[0];

  for (i = 0; i < num; i++)
    invals[i] = Round(invals[i] / divider);

  for (i = num - 1; i > 0; i--)
    invals[i] = invals[i] - invals[i - 1];
  invals[0] = 0;

  return offset;
}


/******************************************************************************\
|*                                                                            *|
|* Integrate (private): add adjacent data in the time stream.                 *|
|*                                                                            *|
|* *invals  - Data in, which is overwritten with the integrated data.         *|
|* num      - Number of samples in invals.                                    *|
|*                                                                            *|
\******************************************************************************/

void Alice::Integrate(double *invals, int num) {
  int i;

  for (i = 1; i < num; i++)
    invals[i] += invals[i - 1];

  return;
}


/******************************************************************************\
|*                                                                            *|
|* Round (private): Smart rounding of a float.                                *|
|*                                                                            *|
|* num  - Float to be rounded.                                                *|
|*                                                                            *|
|* Returns: Integer double.                                                   *|
|*                                                                            *|
\******************************************************************************/

  double Alice::Round(double num) {
    if (num >= 0)
      return (int)(num + 0.5);
    else
      return (int)(num - 0.5);
  }


/******************************************************************************\
|*                                                                            *|
|* SendDiff (private): Differentiate data and send it to the buffer.          *|
|*                                                                            *|
|* *data        - Data to be compressed.                                      *|
|* num          - Number of samples in *data.                                 *|
|* *currInfo    - Pointer to structure containing info on the field           *|
|* maxover      - The maximum fraction of data we want to spill over numbits. *|
|* minover      - The minimum fraction etc.                                   *|
|*                                                                            *|
\******************************************************************************/

void Alice::SendDiff(double *data, int num, struct DataStruct_glob *currInfo,
    float maxover, float minover) {
  long long offset;
  int i;

  // Write introductory bytes
  sendbuf->Introduce();

  offset = (long long)(Differentiate(data, num, currInfo->divider));

  //printf("Differentiated Compression (%d samples):  offset = %lld, "
  //       "divider = %d\n", num, offset, currInfo->divider);

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

  //printf("Compression: %0.2f %% (%d bytes)\n",
  //       100 * float(sendbuf->SectionSize()) / float(num * sizeof(int)),
  //       sendbuf->SectionSize());
  //printf("%0.2f %% of the samples spilled over the %d bit requested size\n"
  //       "   into %d bit overflow chunks.\n\n",
  //       100 * float(sendbuf->overnum) / float(num), currInfo->numbits,
  //       currInfo->overflowsize);
}


/******************************************************************************\
|*                                                                            *|
|* SendInt (private):  prepare integral preserving data and send it to the    *|
|* buffer.                                                                    *|
|*                                                                            *|
|* *data        - Data to be compressed.                                      *|
|* num          - Number of samples in *data.                                 *|
|* *currInfo    - Pointer to struct containing info on the current field.     *|
|* maxover      - The maximum fraction of data we want to spill over numbits. *|
|* minover      - The minimum fraction etc.                                   *|
|*                                                                            *|
\******************************************************************************/

void Alice::SendInt(double *data, int num, struct DataStruct_glob *currInfo,
    float maxover, float minover) {
  int i;

  // Write introductory bytes
  sendbuf->Introduce();

  // We want the integral to be preserved, so the place where we do the division
  // is important.
  Integrate(data, num);
  data[0] = Round(Differentiate(data, num, currInfo->divider) /
      currInfo->divider);

  //printf("Integral Preserving Compression (%d samples):  offset = %lld, "
  //    "divider = %d\n", num, (long long)data[0], currInfo->divider);

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

  //printf("Compression: %0.2f %% (%d bytes)\n",
  //    100 * float(sendbuf->SectionSize()) / float(num * sizeof(int)),
  //    sendbuf->SectionSize());
  //printf("%0.2f %% of the samples spilled over the %d bit requested size\n"
  //    "   into %d bit overflow chunks.\n\n",
  //    100 * float(sendbuf->overnum) / float(num), currInfo->numbits,
  //    currInfo->overflowsize);
}


/******************************************************************************\
|*                                                                            *|
|* SendSingle (private):  send down the first value in the time stream.       *|
|*                                                                            *|
|* *data        - The time stream.                                            *|
|* *currInfo    - Pointer to struct containing info on the current field.     *|
|*                                                                            *|
\******************************************************************************/

void Alice::SendSingle(double *data, struct DataStruct_glob *currInfo) {
  double divider, sendval;

  divider = (double)(currInfo->maxval - currInfo->minval + 1) /
    (double)((long long)1 << currInfo->numbits);
  sendval = Round((data[0] - (double)currInfo->minval) / divider);

  //printf("Single Value Compression: value = %lld (%lld) - %g\n\n",
  //    (long long)data[0], (long long)sendval, divider);

  // Write the first value from *data
  sendbuf->WriteTo((long long)sendval, currInfo->numbits,
      currInfo->overflowsize, false);
}


/******************************************************************************\
|*                                                                            *|
|* SendAverage (private): obtain average of time stream and write to buffer.  *|
|*                                                                            *|
|* *data        - The data.                                                   *|
|* *currInfo    - Pointer to struct containing info on the current field.     *|
|*                                                                            *|
\******************************************************************************/

void Alice::SendAverage(double *data, int num,
    struct DataStruct_glob *currInfo) {
  int i;
  double sum;
  double divider, sendval;

  // Find average
  sum = 0;
  for (i = 0; i < num; i++)
    sum += data[i];

  divider = (double)(currInfo->maxval - currInfo->minval + 1) /
    (double)((long long)1 << currInfo->numbits);
  sendval = Round((sum / (double)num - (double)currInfo->minval) / divider);

  //printf("Average Value Compression: average = %.12g (%lld, %g)\n\n",
  //    Round(sum / num), (long long)sendval, divider);

  // Write average value to buffer
  sendbuf->WriteTo((long long)sendval, currInfo->numbits,
      currInfo->overflowsize, false);
}


/******************************************************************************\
|*                                                                            *|
|* MaxPowTwo (private): checks all fields and sees which has the greatest     *|
|* FindPowTwo (see below).                                                    *|
|*                                                                            *|
|* val          - The number of frames to consider (since # samples =         *|
|*                # frames x samples per frame).                              *|
|* threshold    - See FindPowTwo below.                                       *|
|*                                                                            *|
|* Returns:  the maximum FindPowTwo.                                          *|
|*                                                                            *|
\******************************************************************************/

int Alice::MaxPowTwo(int val, float threshold) {
  struct DataStruct_glob *currInfo;
  int powtwo;

  powtwo = 0;
  for (currInfo = DataInfo->FirstFast(); currInfo != NULL;
      currInfo = DataInfo->NextFast()) {
    if (int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1 > powtwo)
      powtwo = int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1;
  }

  return powtwo;
}


/******************************************************************************\
|*                                                                            *|
|* MaxFrameFreq (private): checks all fields to see which has the greatest    *|
|* samples per frame.                                                         *|
|*                                                                            *|
|* Returns:  maximum samples per frame.                                       *|
|*                                                                            *|
\******************************************************************************/

int Alice::MaxFrameFreq() {
  struct DataStruct_glob *currInfo;
  int max = 0;

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



  return max;
}


/******************************************************************************\
|*                                                                            *|
|* FindPowTwo (private): finds the smallest power of two greater than a given *|
|* value.                                                                     *|
|*                                                                            *|
|* val          - The value greater than which the power of two must be.      *|
|* threshold    - The fraction of the value by which the power of two must    *|
|*                exceed that value.                                          *|
|*                                                                            *|
|* Returns:  the power of two.                                                *|
|*                                                                            *|
\******************************************************************************/

int Alice::FindPowTwo(int val, float threshold) {
  int i, num;

  num = val;
  for (i = 0; num > 0; i++)
    num = num >> 1;

  if ((1 << i) - val < int(val * threshold))
    i++;

  return 1 << i;
}


/******************************************************************************\
|*                                                                            *|
|* CompressionLoop (public): main control loop                                *|
|*                                                                            *|
\******************************************************************************/

void Alice::CompressionLoop() {
  double *filterdata, *rawdata;
  int i, j, k, l;
  struct DataStruct_glob *currInfo;
  int numframes = 0, numtoread, readleftpad, readrightpad = 0;
  int rawsize, powtwo, leftpad, rightpad;
  bool earlysend;
  int numread;
  int framepos = 0;
  int rawdatasize, filterdatasize, ts;

  rawdata = (double *)malloc(1);
  rawdatasize = 1;
  filterdata = (double *)malloc(1);
  filterdatasize = 1;

  for (;;) {
    // Check for new AML file command written by mcp.
    if (GetCurrentAML()) {

      // Make sure our buffers for reading in data from disk are big enough.
      ts = sizeof(double) * DataInfo->samplerate * MaxFrameFreq() *
        DataInfo->looplength;
      if (ts > rawdatasize) {
        rawdatasize = ts;
        rawdata = (double *)realloc(rawdata, ts);
      }

      ts = sizeof(double) * (3 * DataInfo->samplerate * MaxFrameFreq() *
          DataInfo->looplength);
      if (ts > filterdatasize) {
        filterdatasize = ts;
        filterdata = (double *)realloc(filterdata, ts);
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

        //printf("Reading from %s . . .\n", currInfo->src);

        switch (currInfo->type) {
          case COMP_SINGLE:
            if ((numread = DataSource->ReadField(rawdata, currInfo->src,
                    framepos - readrightpad, 1)) != currInfo->framefreq) {
              mprintf(MCP_ERROR,
                  "Error accessing correct number of data from frames "
                  "(%d, %d).", numread, currInfo->framefreq);
              rawdata[0] = 0;
              SendSingle(rawdata, currInfo);  // Send down a zero
            }
            else              
              SendSingle(rawdata, currInfo);
            break;

          case COMP_AVERAGE:
            rawsize = numframes * currInfo->framefreq;
            if ((numread = DataSource->ReadField(rawdata, currInfo->src,
                    framepos - readrightpad, numframes))
                != rawsize) {
              mprintf(MCP_ERROR,
                  "Error accessing correct number of data from frames "
                  "(%d, %d).", numread, currInfo->framefreq);
              rawdata[0] = 0;
              SendSingle(rawdata, currInfo);  // Send down a zero
            }
            else
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

      //printf("Reading from %s . . .\n", currInfo->src);

      // Read data from Frodo's disk
      if ((numread = DataSource->ReadField(filterdata, currInfo->src,
              framepos - readrightpad - leftpad,
              numframes + rightpad + leftpad))
          != rawsize + currInfo->framefreq * (rightpad + leftpad)) {

        mprintf(MCP_ERROR, "Error accessing correct number of data from frames "
            "(%d, %d).", numread, rawsize + currInfo->framefreq *
            (rightpad + leftpad));
        sendbuf->NoDataMarker();
      }
      else {
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
        }
        else
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
          mputs(MCP_WARNING, "TDRSS frame truncated.");
          earlysend = true;
        }
      }
    }
    // Send down the compressed buffer
    sendbuf->Stop();
    framepos += numframes;
  }
}


/******************************************************************************\
|*                                                                            *|
|* Alice: Destructor                                                          *|
|*                                                                            *|
\******************************************************************************/

Alice::~Alice()
{
  delete DataSource;
  delete sendbuf;
  delete DataInfo;
}


/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                            CLASS FrameBuffer                           |**|
|**|                                                                        |**|
|**|     Intercepts mcp frames and buffers them locally for Alice.          |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/


/******************************************************************************\
|*                                                                            *|
|* FrameBuffer:  constructor.                                                 *|
|*                                                                            *|
\******************************************************************************/

FrameBuffer::FrameBuffer(unsigned int *mcpindex_in, 
    unsigned short **fastdata_in,
    unsigned short **slowdata_in, int numframes_in) {
  mcpindex = mcpindex_in;
  lastmcpindex = 2;
  fastdata = fastdata_in;
  slowdata = slowdata_in;
  memallocated = false;
  numframes = -1;

  Resize(numframes_in);

  return;
}


/******************************************************************************\
|*                                                                            *|
|* Resize (public): change the size of the circular buffer grabbed from mcp   *|
|* frames.                                                                    *|
|*                                                                            *|
|* numframes_in     - The size of the buffer, in units of 5 Hz frames.        *|
|*                                                                            *|
\******************************************************************************/

void FrameBuffer::Resize(int numframes_in) {
  int i, j;
  bool err = false; 

  if (numframes_in == numframes) {
    // Do nothing.
    return;
  }

  if (memallocated) {
    exitupdatethread = true;
    while (exitupdatethread) {
      usleep(1000);
    }

    for (i = 0; i < numframes; i++) {
      for (j = 0; j < FAST_PER_SLOW; j++) {
        free(slowbuf[i][j]);
        free(fastbuf[i][j]);
      }
      free(slowbuf[i]);
      free(fastbuf[i]);
    }
    free(slowbuf);
    free(fastbuf);
  }

  numframes = numframes_in; 

  if ((fastbuf = (unsigned short ***)malloc(numframes * 
          sizeof(unsigned short **))) == NULL)
    err = true;
  if ((slowbuf = (unsigned short ***)malloc(numframes * 
          sizeof(unsigned short **))) == NULL)
    err = true;
  for (i = 0; i < numframes; i++) {
    if ((fastbuf[i] = (unsigned short **)malloc(FAST_PER_SLOW *
            sizeof(unsigned short *))) == NULL)
      err = true;
    if ((slowbuf[i] = (unsigned short **)malloc(FAST_PER_SLOW * 
            sizeof(unsigned short *))) == NULL)
      err = true;
    for (j = 0; j < FAST_PER_SLOW; j++) {
      if ((fastbuf[i][j] = (unsigned short *)malloc(BiPhaseFrameSize)) == NULL)
        err = true;
      if ((slowbuf[i][j] = (unsigned short *)malloc(slowsPerBi0Frame *
              sizeof(unsigned short))) == NULL)
        err = true;
    }
  }

  if (err)
    merror(MCP_TFATAL, "SMALL (FrameBuffer): unable to malloc either fastbuf "
        "or slowbuf.");

  framenum = -1;
  memallocated = true;
  multiplexsynced = false;
  pseudoframe = -1;
  exitupdatethread = false;

  pthread_create(&update_id, NULL, FrameBuffer::UpdateThreadEntry, this);

  return;
}


/******************************************************************************\
|*                                                                            *|
|* UpdateThreadEntry (private): an annoying circumposition to start another   *|
|* thread in an object.  One creates a pointer to the calling object and then *|
|* calls the thread routine.                                                  *|
|*                                                                            *|
|* *pthis   - Pointer to the calling object.                                  *|
|*                                                                            *|
\******************************************************************************/

void *FrameBuffer::UpdateThreadEntry(void *pthis) {
  pthread_setspecific(identity, "updt");

  FrameBuffer *mine = (FrameBuffer *)pthis;
  mine->Update();

  return NULL;
}


/******************************************************************************\
|*                                                                            *|
|* Update (private): copy new mcp frames into the circular buffer.            *|
|*                                                                            *|
\******************************************************************************/

void FrameBuffer::Update() {
  unsigned int i;
  int j;

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
        mprintf(MCP_ERROR, "Multiplex index out of range (= %d)",
            multiplexindex);
        continue;
      }
      if (i < 0 || i >= 3) {
        mprintf(MCP_ERROR, "Biphase buffer index out of range (= %d)", i);
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

  return;
}


/******************************************************************************\
|*                                                                            *|
|* NumFrames (public)                                                         *|
|*                                                                            *|
|* Returns: the linear frame counter.                                         *|
|*                                                                            *|
\******************************************************************************/

int FrameBuffer::NumFrames() {
  return pseudoframe;
}


/******************************************************************************\
|*                                                                            *|
|* ReadField (public): read a field of the 5 Hz frame.  User must ensure that *|
|*                                                                            *|
|* *returnbuf   - Buffer in which to place the field's data.  User must       *|
|*                ensure that enough memory is allocated to fit requested     *|
|*                data.                                                       *|
|* *fieldname   - The field source name.                                      *|
|* framenum_in  - The starting frame to read.                                 *|
|* numframes_in - The number of frames past framenum_in to read.              *|
|*                                                                            *|
|* Returns: the number of values placed in *returnbuf.                        *|
|*                                                                            *|
\******************************************************************************/

int FrameBuffer::ReadField(double *returnbuf, const char *fieldname, 
    int framenum_in, int numframes_in) {
  int i, j, k, truenum, wide, mindex, chnum[2];
  unsigned short mask;
  struct NiosStruct *address[2];
  unsigned short msb, lsb;
  char tmpstr[64];

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
    if ((address[0] = GetNiosAddr(tmpstr)) == NULL)
      return 0;
    strcpy(tmpstr, "");
    strncpy(tmpstr, fieldname, i);
    tmpstr[i + 1] = '\0';
    sprintf(tmpstr, "%s%dhi", tmpstr, atoi(fieldname + i + 1) - wide);
    if ((address[1] = GetNiosAddr(tmpstr)) == NULL)
      return 0;
        
    mindex = NOT_MULTIPLEXED + 1;  // Marker for bolometers.
    if (wide)
      mask = 0x00ff;
    else
      mask = 0xff00;
    wide = 8 + wide * 8; // Now let wide be the amount to shift the 'hi' word.
    chnum[0] = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].channel;
    chnum[1] = BiPhaseLookup[BI0_MAGIC(address[1]->bbcAddr)].channel;
  }
  else {
    if ((address[0] = GetNiosAddr(fieldname)) == NULL)
      return 0;
    wide = address[0]->wide;
    mindex = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].index;
    chnum[0] = BiPhaseLookup[BI0_MAGIC(address[0]->bbcAddr)].channel;
    mask = 0;
  }

  if (pseudoframe - framenum_in > numframes || framenum_in > pseudoframe)
    return 0;

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
    }
    else if (mindex == NOT_MULTIPLEXED + 1) {
      // Bolometers are all fast channels.
      for (k = 0; k < FAST_PER_SLOW; k++)
        returnbuf[j++] = ((fastbuf[truenum][k][chnum[1]] & mask) << wide) | 
                         fastbuf[truenum][k][chnum[0]];
    }
    else {
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

  return j;
}


/******************************************************************************\
|*                                                                            *|
|* ~FrameBuffer: destructor.                                                  *|
|*                                                                            *|
\******************************************************************************/

FrameBuffer::~FrameBuffer() {
  int i, j;

  if (memallocated) {
    for (i = 0; i < numframes; i++) {
      for (j = 0; j < FAST_PER_SLOW; j++) {
        free(slowbuf[i][j]);
        free(fastbuf[i][j]);
      }
      free(slowbuf[i]);
      free(fastbuf[i]);
    }
    free(slowbuf);
    free(fastbuf);
  }

  return;
}


/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                                TDRSWriter                              |**|
|**|                                                                        |**|
|**|                          Entry point from mcp.                         |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/

extern "C" void TDRSSWriter(void) {
  Alice *drinkme;

  pthread_setspecific(identity, "tdrs");
  mputs(MCP_STARTUP, "Alice start-up.\n");

  tty_fd = OpenSerial();

  drinkme = new Alice();
  drinkme->CompressionLoop();

  delete drinkme;

  close(tty_fd);
  return;
}
