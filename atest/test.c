/*****************************************************************
 *
 *    This program tests functions in compressionwriter.c.
 *    Although it can test multiple functions when flag[X] > 0,
 *    it's set to run from the command prompt via ./test X for
 *    any given function X.
 *
 *****************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include "test.h"

/*****************************************************************
 *
 *    Substitutions from mcp.c and tx.c
 *
 *****************************************************************/

// hiGain_buffer, needed by compressionwriter.c

#define BI0_FRAME_BUFLEN 400

struct frameBuffer { // declared in mcp.c, but that can't be included
  int i_in;
  int i_out;
  unsigned short *framelist[BI0_FRAME_BUFLEN];
  unsigned short** slow_data_list[BI0_FRAME_BUFLEN];
  int has_bi0_padding;
};

struct frameBuffer hiGain_buffer; // Compressionwriter needs hiGain_buffer

// This function desn't work within test.c because buffer->i_out
// gives a segmentation fault. It only works in ComPressionWriterFunction.
unsigned short *PopFrameBufferAndSlow(struct frameBuffer *buffer, unsigned short ***slow) {
  unsigned short *frame;
  int i_out = buffer->i_out;

  if (buffer->i_in == i_out) { // no data
    return (NULL);
  }
  frame = buffer->framelist[i_out];

  *slow = buffer->slow_data_list[i_out];

  i_out++;
  if (i_out>=BI0_FRAME_BUFLEN) {
    i_out = 0;
  }
  buffer->i_out = i_out;
  return (frame);
}


// ClearBuffer, needed by compressionwriter.c

void ClearBuffer(struct frameBuffer *buffer) {
  buffer->i_out = buffer->i_in;
}


// InCharge, needed by compressionwriter.c from tx.c

short int InCharge = 1;



// mcp_systime, used by commands.c from mcp.c

#define TEMPORAL_OFFSET 0;

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}



// WritePrevStatus, used by commands.c from sip.c
// Note: Commented into oblivion. To get this working, define macros
// given at commandstruct.h

/** Write the Previous Status: called whenever anything changes */
void WritePrevStatus()
{ /*
  int fp, n;

  // write the default file
  fp = open(PREV_STATUS_FILE, O_WRONLY|O_CREAT|O_TRUNC, 00666);
  if (fp < 0) {
    berror(err, "mcp.prev_status open()");
    return;
  }

  if ((n = write(fp, &CommandData, sizeof(struct CommandDataStruct))) < 0) {
    berror(err, "mcp.prev_status write()");
    return;
  }

  if ((n = close(fp)) < 0) {
    berror(err, "mcp.prev_status close()");
    return;
  }
  */
}




/*****************************************************************
 *
 *
 *
 *****************************************************************/









/*****************************************************************
 *
 *    Main Program
 *
 *****************************************************************/

int main(int argc, char *argv[]) {

	  /* MWG: Populate the Nios Addresses */
	  printf (" Test.c: Attempting MakeAddressLookups: ");
	  MakeAddressLookups('\0');
	  printf (" ...success\n");




	int flag[10] = {0}; // Initialize the program to call no functions
	char *funcname[] = {"CompressionWriter",
			"CompressionWriterFunction",
			"WriteSuperFrame",
			"WriteStreamFrame",
			"WriteData",
			"writeHiGainData",
			"writeOmni1Data",
			"writeOmni2Data",
			"OpenSerial",
			"BufferSteamData"};

	if (argc < 2) {
		printf (" To test a function, use './test X' where X ranges from 0-9.\n");
		return 0;
		}

	if (argc > 1) {
		int testfunction = argv[1][0] - 48; // convert text input to a digit
		if (testfunction < 0 || testfunction > 9 || argc > 2) {
			printf ("Error - test requires a single argument from 0 to 9.\n");
			return 1;
			}
		printf (" Test.c: Testing %s...\n", funcname[testfunction]);
		flag[testfunction] = 1;
		}

	if (flag[0]){
		// InitCommandData();
		CompressionWriter();
		printf (" Test.c: CompressionWriter Completed\n");
		// Output:  *BREAK*
		// Channels: Nios Lookup for channel acc_act failed.
		}

	if (flag[1]){
		// InitCommandData();
		CompressionWriterFunction();
		printf (" Test.c: CompressionWriterFunction Completed\n");
		// Output: *BREAK*
		// Channels: Nios Lookup for channel acc_act failed.
		}

	if (flag[2]){
		// Note! Writesuperframe should take *frame as its argument!
		WriteSuperFrame();
		printf (" Test.c: WriteSuperFrame Completed\n");
		// Output:
		// High gain: 0 stream fields use 20 out of 0 bytes per stream frame (-20 free)
		// Omni1: 0 stream fields use 20 out of 0 bytes per stream frame (-20 free)
		// Omni2: 0 stream fields use 20 out of 0 bytes per stream frame (-20 free)
		}

	if (flag[3]){
		WriteStreamFrame();
		printf (" Test.c: WriteStreamFrame Completed\n");
		// Output:  *none*
		}

	if (flag[4]){
		char *x = '\0';
		int size = 1;
		int i_field = 0;
		writeData(x, size, i_field);
		printf (" Test.c: WriteData completed\n");
		// Output:  *none*
		}

	if (flag[5]){
		char *x = '\0';
		int size = 1;
		writeHiGainData(x, size);
		printf (" Test.c: writeHiGainData completed\n");
		// Output:  *none*
		}

	if (flag[6]){
		char *x = '\0';
		int size = 1;
		writeOmni1Data(x, size);
		printf (" Test.c: writeOmni1Data completed\n");
		// Output:  *none*
		}

	if (flag[7]){
		char *x = '\0';
		int size = 1;
		writeOmni2Data(x, size);
		printf (" Test.c: writeOmni2Data completed\n");
		// Output:  *none*
		}

	if (flag[8]){
		int fd = OpenSerialTest();
		printf (" Test.c: OpenSerial returns %d.\n", fd);
		// Output:
		// Could not open downlink serial port /dev/ttySI2.  Retrying...
		// Returns -1; comes through fd = open(tty, O_RDWR | O_NOCTTY))
		}

	if (flag[9]){
		int i = 99;
		unsigned short *frame;
		printf (" Test.c: Attempting to initialize frame\n");
		frame = malloc (20);
		printf (" Test.c: frame == %d\n", *frame);
		BufferStreamData(i, frame);
		printf (" Test.c: BufferStreamData Completed\n");
		}

	printf (" Test.c: End Program\n");
	return 0;
	}



