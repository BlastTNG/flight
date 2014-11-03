#include <stdio.h>
#include <stdlib.h>


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
 *    Forward Declarations for functions to be tested
 *
 *****************************************************************/

// extern int OpenSerial(char *);





/*****************************************************************
 *
 *    Main Program
 *
 *****************************************************************/

int main(int argc, char *argv[]) {
	char *tty = '\0';

//	OpenSerial(tty);
//	printf ("OpenSerial\n");
	printf ("\nThis is an early version of the test function; it does nothing.\n\n");
	return 0;
	}



