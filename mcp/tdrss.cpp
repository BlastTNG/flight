// **********************************************************
// *                                                        *
// *  Programmed by Adam Hincks                             *
// *                                                        *
// *  N.B.  For information on compression algorithms for   *
// *  high rate TDRSS downlink, see readme file             *
// *                                                        *
// *  See also dataholder.cpp which contains a class used   *
// *  in this file                                          *
// *                                                        *
// *  See the readme file for a description of the          *
// *  compression algorithm                                 *
// *                                                        *
// **********************************************************

#define ALICEFILE_DIR   "./"
#define MULTIPLEX_WORD  3

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

#define INPUT_TTY "/dev/ttyS1"

int tty_fd;
extern unsigned short* slow_data[FAST_PER_SLOW];

//-------------------------------------------------------------
//
// OpenSerial: open serial port
//
//-------------------------------------------------------------

int OpenSerial() {
  int fd;
  struct termios term;

  if ((fd = open(INPUT_TTY, O_RDWR)) < 0) {
    printf("Unable to open serial port.\n");
    exit(1);
  }
  if (tcgetattr(fd, &term)) {
    printf("Unable to get serial device attributes.");
    return -1;
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if (cfsetospeed(&term, B19200)) {
    printf("Error setting serial output speed.");
    return -1;
  }
  if (cfsetispeed(&term, B19200)) {
    printf("Error setting serial output speed.");
    return -1;
  }
  if(tcsetattr(fd, TCSANOW, &term)) {
    printf("Unable to set serial attributes.");
    return -1;
  }

  return fd;
}


//-------------------------------------------------------------
//
// printbin: prints binary number at cursor position
//
//-------------------------------------------------------------


void printbin(long num) {
   int i;

   for (i = 24; i >= 0; i--) {
     if (((1 << i) & num) != 0)
       printf("1");
     else
       printf("0");
   }
 }




//|||****_______________________________________________________________________
//|||***************************************************************************
//|||****
//|||****
//|||****     CLASS Buffer -- This class contructs a buffer of data as you
//|||****                     feed them to it.  (Not the same class as the
//|||****                     Buffer in big.cpp.)
//|||****
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************




//-------------------------------------------------------------
//
// Buffer: constructor
//
//-------------------------------------------------------------

Buffer::Buffer() {
  buf = (unsigned char *)malloc(1);
  startbyte = 0;
  size = 1;
}

Buffer::~Buffer() {
  free(buf);
}

//-------------------------------------------------------------i
//
// SetSize (public): set the size of the buffer and malloc
//
//-------------------------------------------------------------


void Buffer::SetSize(int s) {
  if (s > size) {
    size = s;
    // Important to allocate more than requested, in case a frame overflows and
    // has to be truncated.  Weird memory errors can ensue . . .
    safeallocsize = size * BUFFER_SAFE_ALLOC;
    buf = (unsigned char *)realloc(buf, safeallocsize);
  }
}


//-------------------------------------------------------------
//
// Tester (public): for testing purposes
//
//   pos: index to look at in buffer
//
//   Returns: value of buffer at index pos
//
//-------------------------------------------------------------


short int Buffer::Tester(int pos) {
  return buf[pos];
}


//-------------------------------------------------------------
//
// SectionSize (public): calculates size that a single field
//      has taken in the buffer
//
//   Returns: amount buffer has grown since Introduce() was
//      last called
//
//-------------------------------------------------------------


int Buffer::SectionSize() {
  if (bitpos > 0)
    return bytepos - startbyte + 1;
  else
    return bytepos - startbyte;
}


//-------------------------------------------------------------
//
// CurrSize (public): get total size of the buffer
//
//   Returns: size of buffer (bytes)
//
//-------------------------------------------------------------


int Buffer::CurrSize() {
  if (bitpos > 0)
    return bytepos + 1;
  else
    return bytepos;
}


//-------------------------------------------------------------
//
// MaxSize (public)
//
//   Returns: size
//
//-------------------------------------------------------------


int Buffer::MaxSize() {
  return size;
}


void Buffer::CheckBytePosRange() {
  if (bytepos >= safeallocsize) {
    printf("Alice: serious error!!  Class BUFFER was not properly allocated.  "
           "Size was set to %d; tried to write to %d.  Make sure the size is "
           "being set correctly with the Buffer::SetSize function.  Resetting "
           "bytepos to %d (compression will not work).\n", size, bytepos - 1,
           size - 1);
    bytepos = size - 1;
  }

  return;
}


//-------------------------------------------------------------
//
// Start (public): start a new buffer and write the first sync
//      bytes
//
//   filenum: the index of the XML file being used (files are
//      named 0.al, 1.al . . . 15.al)
//
//-------------------------------------------------------------


void Buffer::Start(char filenum, unsigned int framenum) {
  int i;

  // Clear the buffer
  for (i = 0; i < size; i++)
    buf[i] = 0;

  // Start bytes
  //   11111111 11111111 11111111 11111111
  //   1111 | XML file num
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


//-------------------------------------------------------------
//
// Introduce (public): to be called before a new field is
//      written to the buffer -- it writes the sync byte and
//      reserves two bytes for writing in later the size of
//      the section
//
//-------------------------------------------------------------


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


//-------------------------------------------------------------
//
// NoDataMarker (public): to be called if for some reason the
//      data cannot be retrieved from AliceFile -- a marker that
//      the field in this frame is blank
//
//-------------------------------------------------------------

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


//-------------------------------------------------------------
//
// RecordNumByes (public): to be called after a field has been
//      written to the buffer -- goes back and records the
//      size the field's compression took up
//
//-------------------------------------------------------------

void Buffer::RecordNumBytes() {
  buf[startbyte - 2] = SectionSize() & 0xff;
  buf[startbyte - 1] = SectionSize() >> 8;
}


//-------------------------------------------------------------
//
// EraseLastSection (public): if the buffer has become too
//      large, we need to remove the last section that was
//      written
//
//-------------------------------------------------------------

void Buffer::EraseLastSection() {
  bytepos = startbyte - 3;
  bitpos = 0;
}

//-------------------------------------------------------------
//
// Stop (public): writes final sync byte and sends buffer to
//      serial port
//
//-------------------------------------------------------------

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
    printf("\nError sending through serial port.\n\n");
}


//-------------------------------------------------------------
//
// WriteChunk (private): writes a datum to the buffer
//
//   numbits: length of datum in bits
//   datum: the datum
//
//-------------------------------------------------------------


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


//-------------------------------------------------------------
//
// WriteTo (public): writes a datum to the buffer, adding an
//      offset for the sign and dealing with any data that are
//      larger than numbits
//
//   datum: the datum
//   numbits: the number of bits we ideally want to fit each
//      datum into
//   oversize: the size (bits) of overflow chunks (for data
//      that don't fit in numbits)
//   hassign: if the data are signed, we add an offset so all
//      numbers are positive
//
//-------------------------------------------------------------


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
|**                                                                          **|
|**                                                                          **|
|**                                CLASS Alice                               **|
|**                                                                          **|
!**     `What a curious feeling!' said Alice; `I must be shutting up         **|
|**     like a telescope.'                                                   **|
|**                                                                          **|
|**                                                                          **|
|******************************************************************************|
\******************************************************************************/



/*----------------------------------------------------------------------------*\
|*                                                                            *|
|* Alice: Constructor                                                         *|
|*                                                                            *|
\*----------------------------------------------------------------------------*/

Alice::Alice() {
  XMLsrc = -1;
  DataSource = new FrameBuffer(&small_index, smalldata, slow_data, 1);
  sendbuf = new Buffer();    // 10 bits per byte
  DataInfo = new DataHolder();
}


//-------------------------------------------------------------
//
// GetCurrentXML (private): mcp writes the XML file number to
//      /tmp/alice_index whenever it receives commanding to do so.
//      This function checks this file, and if it has changed,
//      loads all the information from the file.
//
//   Returns: true if the XML file changed and was loaded in,
//      false otherwise
//
//-------------------------------------------------------------


bool Alice::GetCurrentXML() {
  char tmp[20];
  int newxml;
  FILE *fp;

  if ((fp = fopen("/tmp/alice_index", "r")) == NULL)
    newxml = 0;
  else {
    fscanf(fp, "%d", &newxml);
    fclose(fp);
  }

  if (newxml != XMLsrc) {
    sprintf(tmp, "%s%d.al", ALICEFILE_DIR, newxml);
    if (DataInfo->LoadFromXML(tmp)) {
      XMLsrc = newxml;
      sendbuf->SetSize(DataInfo->MaxBitRate / 10 * DataInfo->LoopLength);
      mprintf(MCP_INFO, "SMALL (Alice):  now using .aml file number %d.\n", 
              XMLsrc);
      return true;
    }
  }

  return false;
}


//-------------------------------------------------------------
//
// Differentiate (private): differentiates data; i.e., finds
//      the differences between adjacent data in a time stream,
//      and divides all these values by some specified value
//
//   *invals: the data to differentiate.  The function
//      overwrites these data with their differentials
//   num: number of samples in invals
//   divider: number by which to divide differential data
//
//   Returns: offset (integration constant)
//
//-------------------------------------------------------------


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


//-------------------------------------------------------------
//
// Integrate (private): add adjacent data in the time stream
//
//   *invals: data in, which is overwritten with the integrated
//      data
//   num: number of samples in invals
//
//-------------------------------------------------------------


void Alice::Integrate(double *invals, int num) {
  int i;

  for (i = 1; i < num; i++)
    invals[i] += invals[i - 1];

  return;
}


//-------------------------------------------------------------
//
// Round (private): smart rounding of a float
//
//   num: float to be rounded
//
//   Returns: integer double
//
//-------------------------------------------------------------


double Alice::Round(double num) {
  if (num >= 0)
    return (int)(num + 0.5);
  else
    return (int)(num - 0.5);
}


//-------------------------------------------------------------
//
// SendDiff (private): differentiate data and send it to the
//      buffer
//
//   *data: data to be compressed
//   num: number of samples in *data
//   *currInfo: pointer to structure containing info on the
//      field
//   maxover: the maximum fraction of data we want to spill
//      over numbits
//   minover: the minimum fraction etc.
//
//-------------------------------------------------------------

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


//-------------------------------------------------------------
//
// SendInt (private): prepare integral preserving data and send
//      it to the buffer
//
//   *data: data to be compressed
//   num: number of samples in *data
//   *currInfo: pointer to struct containing info on the
//      current field
//   maxover: the maximum fraction of data we want to spill
//      over numbits
//   minover: the minimum fraction etc.
//
//-------------------------------------------------------------

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


//-------------------------------------------------------------
//
// SendSingle (private): send down the first value in the time
//      stream
//
//   *data: the time stream
//   *currInfo: pointer to struct containing info on the
//      current field
//
//-------------------------------------------------------------

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


//-------------------------------------------------------------
//
// SendAverage (private): obtain average of time stream and
//      write to buffer
//
//   *data: the data
//   *currInfo: pointer to struct containing info on the
//      current field
//
//-------------------------------------------------------------

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


//-------------------------------------------------------------
//
// MaxPowTwo (private): checks all fields and sees which has
//      the greatest FindPowTwo (see below)
//
//   val: the number of frames to consider (since # samples =
//      # frames * samples per frame)
//   threshold: see FindPowTwo below
//
//   Returns: the maximum FindPowTwo
//
//-------------------------------------------------------------

int Alice::MaxPowTwo(int val, float threshold) {
  struct DataStruct_glob *currInfo;
  int powtwo;

  powtwo = 0;
  for (currInfo = DataInfo->firstFast(); currInfo != NULL;
      currInfo = DataInfo->nextFast()) {
    if (int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1 > powtwo)
      powtwo = int(FindPowTwo(val * currInfo->framefreq, threshold) /
          currInfo->framefreq) + 1;
  }

  return powtwo;
}


//-------------------------------------------------------------
//
// MaxFrameFreq (private): checks all fields to see which has
//      the greatest samples per frame
//
//   Returns: maximum samples per frame
//
//-------------------------------------------------------------

int Alice::MaxFrameFreq() {
  struct DataStruct_glob *currInfo;
  int max = 0;

  for (currInfo = DataInfo->firstFast(); currInfo != NULL;
      currInfo = DataInfo->nextFast()) {
    if (currInfo->framefreq > max)
      max = currInfo->framefreq;
  }

  for (currInfo = DataInfo->firstSlow(); currInfo != NULL;
      currInfo = DataInfo->nextSlow()) {
    if (currInfo->framefreq > max)
      max = currInfo->framefreq;
  }



  return max;
}


//-------------------------------------------------------------
//
// FindPowTwo (private): finds the smallest power of two
//      greater than a given value
//
//   val: the value the power of two must be greater than
//   threshold: the fraction of the value by which the power
//      of two must exceed the value
//
//   Returns: the power of two
//
//-------------------------------------------------------------

int Alice::FindPowTwo(int val, float threshold) {
  int i, num;

  num = val;
  for (i = 0; num > 0; i++)
    num = num >> 1;

  if ((1 << i) - val < int(val * threshold))
    i++;

  return 1 << i;
}


/*----------------------------------------------------------------------------*\
|*                                                                            *|
|* CompressionLoop (public): main control loop                                *|
|*                                                                            *|
\*----------------------------------------------------------------------------*/

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
    // Check for new XML file command written by mcp.
    if (GetCurrentXML()) {

      // Make sure our buffers for reading in data from disk are big enough.
      ts = sizeof(double) * DataInfo->SampleRate * MaxFrameFreq() *
           DataInfo->LoopLength;
      if (ts > rawdatasize) {
        rawdatasize = ts;
        rawdata = (double *)realloc(rawdata, ts);
      }

      ts = sizeof(double) * (3 * DataInfo->SampleRate * MaxFrameFreq() *
                             DataInfo->LoopLength);
      if (ts > filterdatasize) {
        filterdatasize = ts;
        filterdata = (double *)realloc(filterdata, ts);
      }

      // Figure out how much we need to read each time around.  For FFT purposes
      // it needs to be a power of two, with at least 5 % buffering on the ends.
      numframes = DataInfo->LoopLength * DataInfo->SampleRate;
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
      while (DataSource->NumFrames() < framepos + readleftpad + readrightpad) {
        usleep(1000);
//        DataSource->Update();
      }
      framepos += readleftpad + readrightpad;
    }
    
    // Wait until mcp has written numframes of data
    while (DataSource->NumFrames() < framepos + numframes) {
      usleep(1000);
//      DataSource->Update();
    }
    
    // Start a new buffer for the downlink
    sendbuf->Start(XMLsrc, (unsigned int)(framepos - readrightpad));
    i = 0;
    earlysend = false;

    // Go through each of the slow fields we want to compress.  Slow fields only
    // send down one value per frame.  They are treated like one big "fast"
    // field -- we Introduce() before all of them and then RecordNumBytes()
    // after them.  All the slow fields are written one after the other here
    // at the beginning of the frame.
    if ((currInfo = DataInfo->firstSlow()) != NULL) {
      sendbuf->Introduce();

      for (currInfo = DataInfo->firstSlow(); currInfo != NULL;
          currInfo = DataInfo->nextSlow()) {

        //printf("Reading from %s . . .\n", currInfo->src);

        switch (currInfo->type) {
          case COMP_SINGLE:
            if ((numread = DataSource->ReadField(rawdata, currInfo->src,
                framepos - readrightpad, 1)) != currInfo->framefreq) {
              printf("Error accessing correct number of data from frames "
                  "(%d, %d).\n\n", numread, currInfo->framefreq);
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
              printf("Error accessing correct number of data from frames "
                  "(%d, %d).\n\n", numread, currInfo->framefreq);
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
    for (currInfo = DataInfo->firstFast(); currInfo != NULL && !earlysend;
        currInfo = DataInfo->nextFast()) {

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

        printf("Error accessing correct number of data from frames "
            "(%d, %d).\n\n", numread, rawsize + currInfo->framefreq *
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
          mputs(MCP_WARNING, "TDRSS frame truncated.\n");
          earlysend = true;
        }
      }
    }
    // Send down the compressed buffer
    sendbuf->Stop();
    framepos += numframes;
  }
}


/*----------------------------------------------------------------------------*\
|*                                                                            *|
|* Alice: Destructor                                                          *|
|*                                                                            *|
\*----------------------------------------------------------------------------*/

Alice::~Alice()
{
  delete DataSource;
  delete sendbuf;
  delete DataInfo;

  // No need to delete child widgets:  Qt does it all for us.
}





/******************************************************************************\
|******************************************************************************|
|**                                                                          **|
|**                                                                          **|
|**                             CLASS FrameBuffer                            **|
|**                                                                          **|
|**                                                                          **|
|******************************************************************************|
\******************************************************************************/


/*----------------------------------------------------------------------------*\
|*                                                                            *|
|* FrameBuffer: Constructor                                                   *|
|*                                                                            *|
\*----------------------------------------------------------------------------*/

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

void *FrameBuffer::UpdateThreadEntry(void *pthis) {
  FrameBuffer *mine = (FrameBuffer *)pthis;
  mine->Update();

  return NULL;
}

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
        mprintf(MCP_ERROR, "Multiplex index out of range (= %d)!\n",
                multiplexindex);
        continue;
      }
      if (i < 0 || i >= 3) {
        mprintf(MCP_ERROR, "Biphase buffer index out of range (= %d)!\n", i);
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

int FrameBuffer::NumFrames() {
  return pseudoframe;
}

int FrameBuffer::ReadField(double *returnbuf, const char *fieldname, 
    int framenum_in, int numframes_in) {
  int i, j, k, truenum, wide, mindex, chnum;
  struct NiosStruct* address; 
  unsigned short msb, lsb;

  if ((address = GetNiosAddr(fieldname)) == NULL)
    return 0;

  wide = address->wide;
  mindex = BiPhaseLookup[BI0_MAGIC(address->bbcAddr)].index;
  chnum = BiPhaseLookup[BI0_MAGIC(address->bbcAddr)].channel;

  if (pseudoframe - framenum_in > numframes || framenum_in > pseudoframe)
    return 0;

  truenum = framenum_in - (int)((double)framenum_in / (double)numframes) * 
    numframes;
  if (truenum < 0)
    truenum += numframes;
  for (i = framenum_in, j = 0; i < framenum_in + numframes_in; i++) {
    if (mindex == NOT_MULTIPLEXED)
      for (k = 0; k < FAST_PER_SLOW; k++) {
        lsb = fastbuf[truenum][k][chnum];
        if (wide)
          msb = fastbuf[truenum][k][chnum + 1];
        else
          msb = 0;

        returnbuf[j++] = (double)((msb << 16) | lsb);
      }
    else {
      lsb = slowbuf[truenum][mindex][chnum];
      if (wide)
        msb = slowbuf[truenum][mindex][chnum + 1];
      else
        msb = 0;

      returnbuf[j++] = (double)((msb << 16) | lsb);
    }

    if (++truenum >= numframes)
      truenum = 0;
  }

  return j;
}

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


//|||****_______________________________________________________________________
//|||***************************************************************************
//|||****
//|||****
//|||****     Main()
//|||****
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************




int startsmall() {
  Alice *drinkme;

  if ((tty_fd = OpenSerial()) < 0)
   mprintf(MCP_TFATAL, "Couldn't open TDRSS serial port.");

  drinkme = new Alice();
  drinkme->CompressionLoop();

  delete drinkme;

  close(tty_fd);
  return 0;
}

extern "C" void smallinit(void) {
  mputs(MCP_STARTUP, "Alice start-up.\n");
  startsmall();
}
