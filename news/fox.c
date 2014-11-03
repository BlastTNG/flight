#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <math.h>
#include <string.h>
#include <ctype.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/ioctl.h>
#include <time.h>
#include <string.h>
#include <errno.h>

#include "compressstruct.h"
#include "compressconst.h"
#include "channels.h"
#include "derived.h"
#include "news.h"

#ifdef __SPIDER__
#include "tes.h"
#endif

#define RAWDIR "/data/rawdir"

#define FOX_LNKFILE "/data/etc/fox.lnk"
#define RNC_PORT 41114
#define FOX_EXT 'h'
#define MSNBC_LNKFILE "/data/etc/msnbc.lnk"
#define DNC_PORT 14441
#define MSNBC_EXT 't'
#define RUSH_LNKFILE "/data/etc/rush.lnk"
#define TEA_PORT 14141
#define RUSH_EXT 's'

#define MAX_STREAM_FIELDS 400
#define MAX_SAMPLES_PER_FRAME 100

char channelset_oth;

char LNKFILE[4096];
int PORT;
char EXT;

char *GetArrayFieldName(int i_field);

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];
extern struct ChannelStruct DecomChannels[];

extern char *frameList[];
extern struct fieldStreamStruct streamList[N_OTH_SETS][MAX_OTH_STREAM_FIELDS];

extern union DerivedUnion DerivedChannels[];

int n_framefields = 0;
struct ChannelStruct **framefields;
int *framefieldUnique;

unsigned short n_streamfields = 0;
unsigned short n_streamfieldlist = 0;
int n_array_in_sframe;

struct ChannelStruct **streamfields;
unsigned *stream_gains;
long long *stream_offsets;

int n_sync = 0;
int n_bytemon = 0;
int is_lost = 1;

int tty_fd;
char hostname[255];

#ifdef NUM_ARRAY_STAT
uint8_t array_statistics[NUM_ARRAY_STAT];
#endif

//*********************************************************
// usage
//*********************************************************
void Usage(char *name) {
  fprintf(stderr,"%s <hostname>\n"
    "File Output eXtractor: \n"
    "Connects to an rnc server and downloads and\n"
    "produce defiles from an over the horizon link.\n", name);
    exit(0);
}


//*********************************************************
// See if a field already exists - case sensitive
//*********************************************************
int FieldExists(char *field) {
  int i_field;

  for (i_field = 0; i_field<n_framefields; i_field++) {
    if (strcmp(field, framefields[i_field]->field)==0) {
      return(1);
    }
  }
  for (i_field = 0; i_field<n_streamfields; i_field++) {
    if (strcmp(field, streamfields[i_field]->field)==0) {
      return(1);
    }
  }
  return(0);
}

//*********************************************************
// verify that a field is not listed in the stream field list
//*********************************************************
int FrameFieldIsUnique(char *field) {
  int i_field;
  for (i_field = 0; i_field<n_streamfields; i_field++) {
    if (strcmp(field, streamfields[i_field]->field)==0) {
      return(0);
    }
  }
  return(1);  
}

//*********************************************************
// Check to see if a raw field is in one of the lists
// ignore case...
//*********************************************************
int FieldSupported(char *field) {
  int i_field;
  char lowerfield[512];

  convertToLower(field, lowerfield);

  for (i_field = 0; i_field<n_framefields; i_field++) {
    if (strcmp(lowerfield, framefields[i_field]->field)==0) {
      return(1);
    }
  }
  for (i_field = 0; i_field<n_streamfields; i_field++) {
    if (strcmp(lowerfield, streamfields[i_field]->field)==0) {
      return(1);
    }
  }
  return(0);
}


//*********************************************************
// Make the format file
//*********************************************************
void MakeFormatFile(char *filedirname) {
  char formatfilename[1024];
  FILE *formatfile;
  int i_field;
  int i_derived;
  char fieldU[1024];

  /*   Make format File   */
  sprintf(formatfilename, "%s/format", filedirname);

  formatfile = fopen(formatfilename, "w");
  if (formatfile == NULL) {
    fprintf(stderr,"Could not open format file %s", formatfilename);
    exit(0);
  }
  for (i_field = 0; i_field < n_framefields; i_field++) {
    // if the frame field already appears in the stream list
    // then we won't add it to the format file, and we will
    // set the file pointer to point to /dev/null in
    // OpenDirfilePointers.
    framefieldUnique[i_field] = FrameFieldIsUnique(frameList[i_field]);
    
    if (framefieldUnique[i_field]) {
      convertToUpper( framefields[i_field]->field, fieldU);
      fprintf(formatfile, "%-16s RAW    %c 1\n", framefields[i_field]->field, framefields[i_field]->type);
      fprintf(formatfile, "%-16s LINCOM 1 %16s %.12e %.12e 1\n", fieldU, framefields[i_field]->field,
              framefields[i_field]->m_c2e, framefields[i_field]->b_e2e);
      if (framefields[i_field]->quantity[0]!='\0') {
        fprintf(formatfile, "%s/quantity STRING %s\n",fieldU, framefields[i_field]->quantity);
      }
      if (framefields[i_field]->units[0]!='\0') {
        fprintf(formatfile, "%s/units STRING %s\n",fieldU, framefields[i_field]->units);
      }
    }
  }
#ifdef __SPIDER__
  for (i_field = 0; i_field < NUM_ARRAY_STAT; i_field++) {
    fprintf(formatfile, "%16s RAW c 1\n", GetArrayFieldName(i_field));
  }
#endif

  for (i_field = 0; i_field < n_streamfields; i_field++) {
    convertToUpper( streamfields[i_field]->field, fieldU);
    fprintf(formatfile, "%-16s RAW    %c %d\n", streamfields[i_field]->field, streamfields[i_field]->type,
            streamList[channelset_oth][i_field].samples_per_frame);
    fprintf(formatfile, "%-16s LINCOM 1 %16s %.12e %.12e 1\n", fieldU, streamfields[i_field]->field,
           streamfields[i_field]->m_c2e, streamfields[i_field]->b_e2e);
    if (streamfields[i_field]->quantity[0]!='\0') {
      fprintf(formatfile, "%s/quantity STRING %s\n",fieldU, streamfields[i_field]->quantity);
    }
    if (streamfields[i_field]->units[0]!='\0') {
      fprintf(formatfile, "%s/units STRING %s\n",fieldU, streamfields[i_field]->units);
    }
  }

  fprintf(formatfile, "/REFERENCE %s\n", streamfields[n_streamfields-1]->field);
  // the Time field
  //fprintf(formatfile, "Time RAW d 1\n/REFERENCE Time\n");

  // Derived channels
  for (i_derived = 0; DerivedChannels[i_derived].comment.type != DERIVED_EOC_MARKER; ++i_derived) {
      switch (DerivedChannels[i_derived].comment.type) {
      case 'b': /* bitfield */
        if (FieldSupported(DerivedChannels[i_derived].bitfield.source)) {
          int j;
          // write bitfield
          fprintf(formatfile, "\n# %s BITFIELD:\n", DerivedChannels[i_derived].bitfield.source);
          for (j = 0; j < 16; ++j) {
            if (DerivedChannels[i_derived].bitfield.field[j][0]!='\0') {
              fprintf(formatfile, "%-16s BIT %-16s %i\n", DerivedChannels[i_derived].bitfield.field[j],
                  DerivedChannels[i_derived].bitfield.source, j);
            }
          }
        }
        break;
      case '2': /* lincom2 */
        if (FieldExists(DerivedChannels[i_derived].lincom2.field)) {
          continue;
        }

        if (FieldSupported(DerivedChannels[i_derived].lincom2.source)) {
          if (FieldSupported(DerivedChannels[i_derived].lincom2.source2)) {
            // write lincom2
            fprintf(formatfile, 
                 "%-16s LINCOM 2 %-16s %.12e %.12e %-16s %.12e %.12e\n",
                 DerivedChannels[i_derived].lincom2.field, DerivedChannels[i_derived].lincom2.source,
                 DerivedChannels[i_derived].lincom2.m_c2e, DerivedChannels[i_derived].lincom2.b_e2e,
                 DerivedChannels[i_derived].lincom2.source2,
                 DerivedChannels[i_derived].lincom2.m2_c2e,
                 DerivedChannels[i_derived].lincom2.b2_e2e);
          }
        }
        break;
      case 'w': /* bitword  */
        if (FieldSupported(DerivedChannels[i_derived].bitword.source)) {
          // write bitword
          fprintf(formatfile, "%-16s BIT %-16s %i %i\n",
            DerivedChannels[i_derived].bitword.field, DerivedChannels[i_derived].bitword.source,
            DerivedChannels[i_derived].bitword.offset,
            DerivedChannels[i_derived].bitword.length);
        }
        break;
      case 't': /* linterp  */
        if (FieldSupported(DerivedChannels[i_derived].linterp.source)) {
          // write linterp
          fprintf(formatfile, "%-16s LINTERP %-16s %s\n",
            DerivedChannels[i_derived].linterp.field, DerivedChannels[i_derived].linterp.source,
            DerivedChannels[i_derived].linterp.lut);
        }
        break;
      case 'p': /* phase */
        if (FieldSupported(DerivedChannels[i_derived].phase.source)) {
          // write phase
          fprintf(formatfile, "%-16s PHASE %-16s %i\n",
            DerivedChannels[i_derived].phase.field, DerivedChannels[i_derived].phase.source,
            DerivedChannels[i_derived].phase.shift);
        }
        break;
      case 'c': /* lincom */
        if (FieldExists(DerivedChannels[i_derived].lincom.field)) {
          continue;
        }

        if (FieldSupported(DerivedChannels[i_derived].lincom.source)) {
          // write lincom
          fprintf(formatfile, "%-16s LINCOM 1 %-16s %.12e %.12e\n",
            DerivedChannels[i_derived].lincom.field, DerivedChannels[i_derived].lincom.source,
            DerivedChannels[i_derived].lincom.m_c2e, DerivedChannels[i_derived].lincom.b_e2e);
        }
        break;
      case '#': /* comment -- do nothing */
        break;
      case 'u': /* Units metadata */
        if (FieldSupported(DerivedChannels[i_derived].units.source)) {
          // write units
          fprintf(formatfile, "%s/units STRING %s\n%s/quantity STRING %s\n",
            DerivedChannels[i_derived].units.source, DerivedChannels[i_derived].units.units,
            DerivedChannels[i_derived].units.source, DerivedChannels[i_derived].units.quantity);
        }
        break;
      default:  // unrecognized -- do nothing
        break;
    }
  }

  /* Hack for a few important derived of derived fields: BLAST10 and probably BLAST12 */
/*
  fprintf(formatfile, "DR_INFO_IO_RW    LINCOM 2 DR_INFO_OPEN_RW  1.000000000000e+00 0.000000000000e+00 DR_INFO_INIT_1_RW 2.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "DR_INFO_IO_EL    LINCOM 2 DR_INFO_OPEN_EL  1.000000000000e+00 0.000000000000e+00 DR_INFO_INIT_1_EL 2.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "DR_INFO_IO_PIV   LINCOM 2 DR_INFO_OPEN_PIV 1.000000000000e+00 0.000000000000e+00 DR_INFO_INIT_1_PIV 2.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "DR_INFO_RIO_EL   LINCOM 2 DR_INFO_RESET_EL 4.000000000000e+00 0.000000000000e+00 DR_INFO_IO_EL    1.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "DR_INFO_RIO_PIV  LINCOM 2 DR_INFO_RESET_PIV 4.000000000000e+00 0.000000000000e+00 DR_INFO_IO_PIV   1.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "DR_INFO_RIO_RW   LINCOM 2 DR_INFO_RESET_RW 4.000000000000e+00 0.000000000000e+00 DR_INFO_IO_RW    1.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "POT_STATE        LINCOM 2 POT_IS_CLOSED    2.000000000000e+00 0.000000000000e+00 POT_IS_OPEN      1.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "LHE_STATE        LINCOM 2 LHE_IS_CLOSED    2.000000000000e+00 0.000000000000e+00 LHE_IS_OPEN      1.000000000000e+00 0.000000000000e+00\n");
  fprintf(formatfile, "LN_STATE         LINCOM 2 LN_IS_CLOSED     2.000000000000e+00 0.000000000000e+00 LN_IS_OPEN       1.000000000000e+00 0.000000000000e+00\n");
*/

  fclose(formatfile);

}

//*********************************************************
// Make the list of frame channels
//*********************************************************
void MakeFrameList() {
  int i_field;

  // Initialize the channel lists
  // Count the frame channels
  for (n_framefields = 0; frameList[n_framefields][0] !='\0'; n_framefields++);
  
  framefieldUnique = (int *) malloc(n_framefields * sizeof(int));

  // find and set the frame fields
  framefields = (struct ChannelStruct **)malloc(n_framefields * sizeof (struct ChannelStruct *));

  for (i_field = 0; i_field<n_framefields; i_field++) {
    
    framefields[i_field] = GetChannelStruct(frameList[i_field]);
    
    if (!framefields[i_field]) {
      fprintf(stderr,"Error: could not find field in tx_struct! |%s|\n", frameList[i_field]);
      exit(0);
    }
  }

}

//*********************************************************
// Make the list of stream channels
//*********************************************************
void MakeStreamList() {
  int i_streamfield;
  char *name;

  // Initialize the channel lists */
  // Count the frame channels
  for (n_streamfieldlist = 0; streamList[channelset_oth][n_streamfieldlist].name[0] !='\0'; n_streamfieldlist++);
  
  // find and set the frame fields
  streamfields = (struct ChannelStruct **)malloc(n_streamfieldlist * sizeof (struct ChannelStruct *));
  stream_gains = (unsigned *)malloc(n_streamfieldlist*sizeof(unsigned));
  stream_offsets = (long long *)malloc(n_streamfieldlist*sizeof(long long));

  for (i_streamfield = 0; i_streamfield<n_streamfieldlist; i_streamfield++) {
    stream_gains[i_streamfield] = 1;
    stream_offsets[i_streamfield] = 0;

    name = streamList[channelset_oth][i_streamfield].name;

    streamfields[i_streamfield] = GetChannelStruct(name);
    
    if (!streamfields[i_streamfield]) {
      fprintf(stderr,"Error: could not find field in tx_struct! |%s|\n", streamList[channelset_oth][i_streamfield].name);
      exit(0);
    }
  }
}

//*********************************************************
//   open the file pointers for the dirfile
//*********************************************************
void OpenDirfilePointers(int **fieldfp, int **streamfp, int **arraystatfp, char *filedirname) {
  int i_field;
  char filename[1024];
  
  *fieldfp = (int *) malloc((n_framefields+1) * sizeof(int));
  for (i_field = 0; i_field < n_framefields; i_field++) {
    if (framefieldUnique[i_field]) {
      sprintf(filename, "%s/%s", filedirname, framefields[i_field]->field);
      if( ((*fieldfp)[i_field] = open(filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
        fprintf(stderr,"rnc: Could not create %s\n", filename);
        exit(0);
      }
    } else {
      (*fieldfp)[i_field] = open("/dev/null", O_WRONLY);
    }
  }
    
  *streamfp = (int *) malloc(n_streamfields * sizeof(int));
  for (i_field = 0; i_field < n_streamfields; i_field++) {
    sprintf(filename, "%s/%s", filedirname, streamfields[i_field]->field);
    if( ((*streamfp)[i_field] = open(filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
      fprintf(stderr,"fox: Could not create %s\n", filename);
      exit(0);
    }
  }
  
#ifdef __SPIDER__
  *arraystatfp = (int *)malloc(n_array_in_sframe*sizeof(int));
  for (i_field = 0; i_field < n_array_in_sframe; i_field++) {
    sprintf(filename, "%s/%s", filedirname, GetArrayFieldName(i_field));
    if( ((*arraystatfp)[i_field] = open(filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
      fprintf(stderr,"fox: Could not create %s\n", filename);
      exit(0);
    }
  }
#endif
}

//*********************************************************
// main
//*********************************************************
int main(int argc, char *argv[]) {
  int index = 0;
  int i_framefield;
  int i_streamfield;
  int i_arrayfield;
  char filedirname[1024];
  int fieldsize = 0;
  int *fieldfp;
  int *streamfp;
  int *arraystatfp;
  char fielddata[2000][8];
  int n_wrote;
  int i_samp;
  int i_frame;
  
  int i_array, j_array;
  
  
  unsigned short us_in;
  short s_in;
  unsigned char uc_in;
  signed char c_in;
  int i_in;
  unsigned u_in;
  long long ll_in[MAX_STREAM_FIELDS][MAX_SAMPLES_PER_FRAME];
  int first_time = 1;
  char name[128];

  int len;
  int i0;
  
  struct fifoStruct fs;

  /* find if we are fox, rush, or msnbc from the file name */
  /* first strip out the directory */
  len = strlen(argv[0]);
  for (i0=len-1; i0>0 && (argv[0][i0-1] != '/'); i0--);
  for (i_in = i0; i_in<len; i_in++) {
    name[i_in-i0] = argv[0][i_in];
  }
  name[i_in-i0] = '\0';
    
  if (strncmp(name, "fox", 3)==0) {
    strcpy(LNKFILE, FOX_LNKFILE);
    PORT = RNC_PORT;
    EXT = FOX_EXT;
  } else if (strncmp(name, "msn", 3)==0) {
    strcpy(LNKFILE, MSNBC_LNKFILE);
    PORT = DNC_PORT;
    EXT = MSNBC_EXT;
  } else if (strncmp(name, "rus", 3)==0) {
    strcpy(LNKFILE, RUSH_LNKFILE);
    PORT = TEA_PORT;
    EXT = RUSH_EXT;
  } else {
    fprintf(stderr, "unknown program: %s\n", name);
    exit(0);
  }

  fs.i_in = fs.i_out = 0;
  
  if (argc!=2) Usage(name);
  if (argv[1][0]=='-') Usage(name);

  sprintf(filedirname, "%s/%lu.%c", RAWDIR, time(NULL),EXT);
  if (mkdir(filedirname, 0777)<0) {
    fprintf(stderr, "could not create dirfile %s\n", filedirname);
    exit(0);
  }

  unlink(LNKFILE);
  if (symlink(filedirname, LNKFILE)<0) {
    fprintf(stderr, "could not create link from `%s' to `%s'",
            filedirname, LNKFILE);
            exit(0);
  }

  strncpy(hostname, argv[1], 250);
  
  tty_fd = party_connect(hostname, PORT);

  MakeFrameList();
  
  //**   read and parse the data  */
  while (1) { // for each superframe
    index = 0; // start of superframe

    // Look for sync word
    while (index == 0) {
      n_bytemon+=BlockingRead(sizeof(unsigned), &fs, tty_fd, hostname, PORT);

      peek(&fs, (char *)&u_in, sizeof(unsigned));
      if (u_in==SYNCWORD) {
        printf("\rRead %d bytes for frame %d ", n_bytemon, n_sync);
        advance(&fs, sizeof(unsigned));
        index = 1;
        is_lost = 0;
        n_bytemon = 0;
        n_sync++;
        fflush(stdout);
      } else {
        is_lost = 1;
        advance(&fs, 1);
        printf("\rlost for %3d bytes (frame %d) ", n_bytemon, n_sync);
        fflush(stdout);
      }
    }
    
    // read frameset char
    n_bytemon+=BlockingRead(sizeof(char), &fs, tty_fd, hostname, PORT);
    pop(&fs, &channelset_oth, 1);
    printf("channelset %d       \n", (int)channelset_oth);
    // Read once per frame fields
    for (i_framefield = 0; i_framefield < n_framefields; i_framefield++) {
      switch (framefields[i_framefield]->type) {
        case 'u':
        case 's':
          fieldsize = 2;
          break;
        case 'U':
        case 'S':
          fieldsize = 4;
          break;
        default:
          break;
      }
      n_bytemon+=BlockingRead(fieldsize, &fs, tty_fd, hostname, PORT);

      pop(&fs, fielddata[i_framefield], fieldsize);
      index++;
    }
#ifdef __SPIDER__
    // Read Array Statistics
    // Array Stats per Superframe
    n_bytemon+=BlockingRead(2, &fs, tty_fd, hostname, PORT);
    pop(&fs, (char *)(&us_in), 2);
    n_array_in_sframe = us_in;
    if (n_array_in_sframe > NUM_ARRAY_STAT) {
      fprintf(stderr, "error: Unreasonably large number of array stats (%d).  Something is wrong.\n", 
        n_array_in_sframe);
      continue;
    }
    
    // starting index
    n_bytemon+=BlockingRead(2, &fs, tty_fd, hostname, PORT);
    pop(&fs, (char *)(&us_in), 2);
    i_array = us_in;
    if (i_array > NUM_ARRAY_STAT) {
      fprintf(stderr, "error: invalid array stats index.  Something is wrong.\n");
      continue;
    }
    // read them...
    for (j_array = 0; j_array<n_array_in_sframe; j_array++) {
      pop(&fs, (char *)(&uc_in), 1);
      array_statistics[i_array] = uc_in;
      i_array++;
      if (i_array>= NUM_ARRAY_STAT) i_array=0;
    }
    
#endif
    
    n_bytemon+=BlockingRead(2, &fs, tty_fd, hostname, PORT);
    pop(&fs, (char *)(&us_in), 2);

   
    if (first_time) {
      MakeStreamList();
      // reset first_time later
    }

    if (n_streamfields == 0) {
      n_streamfields = us_in;
      if (n_streamfields>n_streamfieldlist) {
        fprintf(stderr, "error file asks for more streamfields than are listed (%u > %u)\n", n_streamfields, n_streamfieldlist); 
        n_streamfields = 0;
      } else {
        printf("\nstream contains %u out of %u stream fields\n", n_streamfields, n_streamfieldlist);
      }
    }
    // check for bad n_streamfields 
    if ((n_streamfields != us_in) || (n_streamfields==0)) {
      printf("\nChange in number of stream fields (%d vs %u), or bad data...\n"
      "\nIf this persists, you might want to restart\n", n_streamfields, us_in);
      continue;
    }
    
    if (first_time) {
      MakeStreamList();
      MakeFormatFile(filedirname);
      OpenDirfilePointers(&fieldfp, &streamfp, &arraystatfp, filedirname);
      first_time = 0;
    }
    
    // Read stream gains and offsets
    for (i_streamfield = 0; i_streamfield < n_streamfields; i_streamfield++) {
      n_bytemon+=BlockingRead(sizeof(unsigned short), &fs, tty_fd, hostname, PORT);
      pop(&fs, (char *)&us_in, sizeof(unsigned short));
      stream_gains[i_streamfield] = us_in;
      if (stream_gains[i_streamfield] == 0) {
        printf("\nzero gain for field %s.  Setting to 1\n", streamfields[i_streamfield]->field);
        stream_gains[i_streamfield]=1;
      }
      switch (streamfields[i_streamfield]->type) {
        case 'u':
          n_bytemon+=BlockingRead(sizeof(short), &fs, tty_fd, hostname, PORT);
          pop(&fs, (char *)&us_in, sizeof(short));
          stream_offsets[i_streamfield] = us_in;
          break;
        case 's': // 16 bit offsets
          n_bytemon+=BlockingRead(sizeof(short), &fs, tty_fd, hostname, PORT);
          pop(&fs, (char *)&s_in, sizeof(short));
          stream_offsets[i_streamfield] = s_in;
          break;
        case 'U':
          n_bytemon+=BlockingRead(sizeof(int), &fs, tty_fd, hostname, PORT);
          pop(&fs, (char *)&u_in, sizeof(int));
          stream_offsets[i_streamfield] = u_in;
          break;
        case 'S': // 32 bit offsets
          n_bytemon+=BlockingRead(sizeof(int), &fs, tty_fd, hostname, PORT);
          pop(&fs, (char *)&i_in, sizeof(int));
          stream_offsets[i_streamfield] = i_in;
          break;
      }
    }

    // Handle the 1 Hz frame stuff.
    for (i_frame = 0; i_frame < STREAMFRAME_PER_SUPERFRAME; i_frame++) {
      // Read the >= 1 Hz streamed data.
      for (i_streamfield = 0; i_streamfield < n_streamfields; i_streamfield++) {
        n_bytemon+=BlockingRead(streamList[channelset_oth][i_streamfield].samples_per_frame*streamList[channelset_oth][i_streamfield].bits/8, &fs, tty_fd, hostname, PORT);
        for (i_samp = 0; i_samp<streamList[channelset_oth][i_streamfield].samples_per_frame; i_samp++) {
          // read streamfield;
          if (streamList[channelset_oth][i_streamfield].bits == 4) {
            // FIXME: deal with 4 bit fields.  There should always be a pair of them
          } else if (streamList[channelset_oth][i_streamfield].bits == 8) {
            pop(&fs, (char *)&c_in, 1);
            ll_in[i_streamfield][i_samp]  = (int)c_in * stream_gains[i_streamfield]+stream_offsets[i_streamfield];
            if (streamList[channelset_oth][i_streamfield].doDifferentiate) { // undiferentiate...
              ll_in[i_streamfield][i_samp] = stream_offsets[i_streamfield];
              stream_offsets[i_streamfield]+=(int)c_in * stream_gains[i_streamfield];
            }
          } else if (streamList[channelset_oth][i_streamfield].bits == 16) {
            pop(&fs, (char *)&s_in, 2);
            ll_in[i_streamfield][i_samp] = (int)s_in * stream_gains[i_streamfield]+stream_offsets[i_streamfield];
            if (streamList[channelset_oth][i_streamfield].doDifferentiate) { // undiferentiate...
              ll_in[i_streamfield][i_samp] = stream_offsets[i_streamfield];
              stream_offsets[i_streamfield]+=(int)s_in * stream_gains[i_streamfield];
            }
          } else {
            fprintf(stderr,"Unsupported stream resolution... (a definite bug!)\n");
          }
        }
      }
            // read frame sync byte
      n_bytemon+=BlockingRead(sizeof(char), &fs, tty_fd, hostname, PORT);
      pop(&fs, (char *)(&uc_in), sizeof(char));
      if (uc_in != 0xa5) {
        printf("bad sync byte: must be lost %x\n", (int)uc_in);
        i_frame = STREAMFRAME_PER_SUPERFRAME;
        break;
      } else {
        // write the slow data at 1 hz, even though it is only updated slower than that
        // this is so we can get near-realtime updated data.
        for (i_framefield=0; i_framefield<n_framefields; i_framefield++) {
          switch (framefields[i_framefield]->type) {
            case 'u':
            case 's':
              fieldsize = 2;
              break;
            case 'U':
            case 'S':
              fieldsize = 4;
              break;
            default:
              break;
          }
          
          n_wrote = write(fieldfp[i_framefield], fielddata[i_framefield], fieldsize);
          if (n_wrote != fieldsize) {
            fprintf(stderr, "\nWriting field data unsuccesful. Out of disk space?\n");
          }
        }
#ifdef __SPIDER__
        for (i_arrayfield = 0; i_arrayfield < n_array_in_sframe; i_arrayfield++) {
          n_wrote = write(arraystatfp[i_arrayfield], array_statistics+i_arrayfield, 1);
          if (n_wrote != 1) {
            fprintf(stderr, "\nWriting field data unsuccesful. %s %d %d\n", strerror(errno), arraystatfp[i_arrayfield],
                    i_arrayfield);
          }
        }

#endif
        for (i_streamfield = 0; i_streamfield < n_streamfields; i_streamfield++) {
          for (i_samp = 0; i_samp<streamList[channelset_oth][i_streamfield].samples_per_frame; i_samp++) {
            switch (streamfields[i_streamfield]->type) {
              case 'u':
                us_in = ll_in[i_streamfield][i_samp];
                n_wrote = write(streamfp[i_streamfield], (char *)(&us_in), 2);
                if (n_wrote != 2) {
                  fprintf(stderr, "Writing field data unsuccesful. Out of disk space?\n");
                }
                break;
              case 's': 
                s_in = ll_in[i_streamfield][i_samp];
                n_wrote = write(streamfp[i_streamfield], (char *)(&s_in), 2);
                if (n_wrote != 2) {
                  fprintf(stderr, "Writing field data unsuccesful. Out of disk space?\n");
                }
                break;
              case 'U':
                u_in = ll_in[i_streamfield][i_samp];
                n_wrote = write(streamfp[i_streamfield], (char *)(&u_in), 4);
                if (n_wrote != 4) {
                  fprintf(stderr, "Writing field data unsuccesful. Out of disk space?\n");
                }
                break;
              case 'S':
                i_in = ll_in[i_streamfield][i_samp];
                n_wrote = write(streamfp[i_streamfield], (char *)(&i_in), 4);
                if (n_wrote != 4) {
                  fprintf(stderr, "Writing field data unsuccesful. Out of disk space?\n");
                }
                break;
            }
          } // next samp
        } // next streamfield
      } // endif
    } // next frame
  } // end while 1

  return (0); // can't ever get here
}
