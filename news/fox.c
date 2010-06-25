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

#include "compressstruct.h"
#include "channels.h"
#include "derived.h"

#define FIFODEPTH 2048
#define RAWDIR "/data/rawdir"
#define LNKFILE "/data/etc/fox.lnk"

extern struct ChannelStruct WideSlowChannels[];
extern struct ChannelStruct SlowChannels[];
extern struct ChannelStruct WideFastChannels[];
extern struct ChannelStruct FastChannels[];
extern struct ChannelStruct DecomChannels[];

extern char *frameList[];

extern union DerivedUnion DerivedChannels[];

struct fifoStruct {
  char d[FIFODEPTH];
  int i_in;  // points at next place to write
  int i_out; // points at next place to read
};

int n_framefields = 0;
struct ChannelStruct **framefields;

// out needs to be allocated before we come here.
void convertToUpper(char *in, char *out) {
  int i;
  for (i=0; in[i] != '\0'; i++) {
    out[i] = toupper(in[i]);
  }
  out[i] = '\0';
}
 
// out needs to be allocated before we come here.
void convertToLower(char *in, char *out) {
  int i;
  for (i=0; in[i] != '\0'; i++) {
    out[i] = tolower(in[i]);
  }
  out[i] = '\0';
}

void push(struct fifoStruct *fs, char x[], int size) {
  int i;
  for (i=0; i<size; i++) {
    fs->d[fs->i_in] = x[i];
    fs->i_in++;
    if (fs->i_in>=FIFODEPTH) {
      fs->i_in = 0;
    }
  }
}

void peek(struct fifoStruct *fs, char x[], int size) {
  // warning: no error checking.  Use nFifo first to make
  // sure you don't wrap the fifo.
  int i;
  int i_out = fs->i_out;
  
  for (i=0; i< size; i++) {
    x[i] = fs->d[i_out];
    i_out++;
    if (i_out >= FIFODEPTH) {
      i_out = 0;
    }
  }
}

void advance(struct fifoStruct *fs, int size) {
  fs->i_out += size;
  if (fs->i_out >= FIFODEPTH) {
    fs->i_out -= FIFODEPTH;
  }
}

int nFifo(struct fifoStruct *fs) {
  int n;

  n = fs->i_in - fs->i_out;
  if (n < 0) n+= FIFODEPTH;

  return n;
}


int party_connect(const char *hostname) {
  int s;
  struct sockaddr_in sn;
  struct hostent *hostinfo;
  struct servent *sp;
  struct servent sp_real;
  int on =1 ;
  /* Get service */
  if ( (sp = getservbyname("blastd", NULL)) == NULL ) {
    fprintf(stderr,
        "Service blastd not found; using 44144 (default) instead\n");
    sp = &sp_real;
    sp->s_port = htons(44144);
  }

  sn.sin_family = AF_INET;
  sn.sin_port = sp->s_port;

  if(!inet_aton(hostname, &sn.sin_addr)) {
    hostinfo = gethostbyname(hostname);
    if (hostinfo == NULL) {
      herror(hostname);
      exit(1);
    }
    sn.sin_addr = *(struct in_addr *) hostinfo->h_addr;
  }

  /* Create the socket. */
  if ( (s = socket (AF_INET, SOCK_STREAM, 0)) < 0 ) {
    perror("socket");
    exit(1);
  }
  /* set socket options */
  (void) setsockopt(s, SOL_SOCKET, SO_REUSEADDR, &on, sizeof(on));

  if (connect(s, (struct sockaddr *)&sn, sizeof (sn)) < 0) {
    perror("socket");
  }

  return s;
}

void Usage() {
  fprintf(stderr,"fox <hostname>\n"
    "File Output eXtractor: \n"
    "Connects to an rnc server and downloads and\n"
    "produce defiles from the high gain tdrss link.\n");
    exit(0);
}

int fieldSupported(char *field) {
  int i_field;
  char lowerfield[512];

  convertToLower(field, lowerfield);

  for (i_field = 0; i_field<n_framefields; i_field++) {
    if (strcmp(lowerfield, framefields[i_field]->field)==0) {
      return(1);
    }
  }
  return(0);
}

 /*  \REFERENCE <rawfieldname> */
 
void MakeFormatFile(char *filedirname) {
  char formatfilename[1024];
  FILE *formatfile;
  int i_framefield;
  int i_derived;
  char fieldU[1024];

  /******************************************/
  /*   Make format File   */
  sprintf(formatfilename, "%s/format", filedirname);

  formatfile = fopen(formatfilename, "w");
  if (formatfile == NULL) {
    fprintf(stderr,"Could not open format file %s", formatfilename);
    exit(0);
  }
  for (i_framefield = 0; i_framefield < n_framefields; i_framefield++) {
    convertToUpper( framefields[i_framefield]->field, fieldU);
    fprintf(formatfile, "%-16s RAW    %c 1\n", framefields[i_framefield]->field, framefields[i_framefield]->type);
    fprintf(formatfile, "%-16s LINCOM 1 %16s %.12e %.12e 1\n", fieldU, framefields[i_framefield]->field,
           framefields[i_framefield]->m_c2e, framefields[i_framefield]->b_e2e);
    if (framefields[i_framefield]->quantity[0]!='\0') {
      fprintf(formatfile, "%s/quantity STRING %s\n",fieldU, framefields[i_framefield]->quantity);
    }
    if (framefields[i_framefield]->units[0]!='\0') {
      fprintf(formatfile, "%s/units STRING %s\n",fieldU, framefields[i_framefield]->units);
    }
  }

  // Derived channels
  for (i_derived = 0; DerivedChannels[i_derived].comment.type != DERIVED_EOC_MARKER; ++i_derived) {
      switch (DerivedChannels[i_derived].comment.type) {
      case 'b': /* bitfield */
        if (fieldSupported(DerivedChannels[i_derived].bitfield.source)) {
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
        if (fieldSupported(DerivedChannels[i_derived].lincom2.source)) {
          if (fieldSupported(DerivedChannels[i_derived].lincom2.source2)) {
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
        if (fieldSupported(DerivedChannels[i_derived].bitword.source)) {
          // write bitword
          fprintf(formatfile, "%-16s BIT %-16s %i %i\n",
            DerivedChannels[i_derived].bitword.field, DerivedChannels[i_derived].bitword.source,
            DerivedChannels[i_derived].bitword.offset,
            DerivedChannels[i_derived].bitword.length);
        }
        break;
      case 't': /* linterp  */
        if (fieldSupported(DerivedChannels[i_derived].linterp.source)) {
          // write linterp
          fprintf(formatfile, "%-16s LINTERP %-16s %s\n",
            DerivedChannels[i_derived].linterp.field, DerivedChannels[i_derived].linterp.source,
            DerivedChannels[i_derived].linterp.lut);
        }
        break;
      case 'p': /* phase */
        if (fieldSupported(DerivedChannels[i_derived].phase.source)) {
          // write phase
          fprintf(formatfile, "%-16s PHASE %-16s %i\n",
            DerivedChannels[i_derived].phase.field, DerivedChannels[i_derived].phase.source,
            DerivedChannels[i_derived].phase.shift);
        }
        break;
      case 'c': /* lincom */
        if (fieldSupported(DerivedChannels[i_derived].lincom.source)) {
          // write lincom
          fprintf(formatfile, "%-16s LINCOM 1 %-16s %.12e %.12e\n",
            DerivedChannels[i_derived].lincom.field, DerivedChannels[i_derived].lincom.source,
            DerivedChannels[i_derived].lincom.m_c2e, DerivedChannels[i_derived].lincom.b_e2e);
        }
        break;
      case '#': /* comment -- do nothing */
        break;
      case 'u': /* Units metadata */
        if (fieldSupported(DerivedChannels[i_derived].units.source)) {
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
 

  fclose(formatfile);

}

void MakeFieldList() {
  int i_framefield;
  int i_ch;
  int found_ch;

  /******************************************
  /** Initialize the channel lists */
  // Count the frame channels
  for (n_framefields = 0; frameList[n_framefields][0] !='\0'; n_framefields++);

  // find and set the frame fields
  framefields = (struct ChannelStruct **)malloc(n_framefields * sizeof (struct ChannelStruct *));

  for (i_framefield = 0; i_framefield<n_framefields; i_framefield++) {
    found_ch = 0;
    // search the slow channels
    for (i_ch = 0; SlowChannels[i_ch].field[0]!='\0'; i_ch++) {
      if (strcmp(frameList[i_framefield], SlowChannels[i_ch].field)==0) {
          framefields[i_framefield]= SlowChannels+i_ch; // point framefields[] to the right channels
          found_ch = 1;
          break;
      }
    }
    if (found_ch) continue;

    // search the slow wide channels
    for (i_ch = 0; WideSlowChannels[i_ch].field[0]!='\0'; i_ch++) {
      if (strcmp(frameList[i_framefield], WideSlowChannels[i_ch].field)==0) {
          framefields[i_framefield]= WideSlowChannels+i_ch; // point framefields[] to the right channels
          found_ch = 1;
          break;
      }
    }
    if (found_ch) continue;

// search the fast channels
    for (i_ch = 0; FastChannels[i_ch].field[0]!='\0'; i_ch++) {
      if (strcmp(frameList[i_framefield], FastChannels[i_ch].field)==0) {
          framefields[i_framefield]= FastChannels+i_ch; // point framefields[] to the right channels
          found_ch = 1;
          break;
      }
    }
    if (found_ch) continue;

    // search the wide fast channels
    for (i_ch = 0; WideFastChannels[i_ch].field[0]!='\0'; i_ch++) {
      if (strcmp(frameList[i_framefield], WideFastChannels[i_ch].field)==0) {
          framefields[i_framefield]= WideFastChannels+i_ch; // point framefields[] to the right channels
          found_ch = 1;
          break;
      }
    }
    fprintf(stderr,"Error: could not find field in tx_struct! |%s|\n", frameList[i_framefield]);
  }

}

void main(int argc, char *argv[]) {
  int tty_fd;
  int i_lost = 0;
  int index = 0;
  int numin, numread;
  char inbuf[FIFODEPTH];
  unsigned u_in;
  int n_sync = 0;
  int i_framefield, i_ch;
  char filedirname[1024];
  time_t filetime;
  char filename[1024];
  int fieldsize = 0;
  int *fieldfp;
  char fielddata[8];

  struct fifoStruct fs;
  
  fs.i_in = fs.i_out = 0;
  
  if (argc!=2) Usage();
  if (argv[1][0]=='-') Usage();


  sprintf(filedirname, "%s/%lu.h", RAWDIR, time(NULL));
  if (mkdir(filedirname, 0777)<0) {
    fprintf(stderr, "rnc: could not create dirfile %s\n", filedirname);
    exit(0);
  }

  unlink(LNKFILE);
  if (symlink(filedirname, LNKFILE)<0) {
    fprintf(stderr, "rnc: could not create link from `%s' to `%s'",
            filedirname, LNKFILE);
            exit(0);
  }

  tty_fd = party_connect(argv[1]);

  MakeFieldList();
  
 MakeFormatFile(filedirname);

  /******************************************/
  /*   open the file pointers for the dirfile  */
  fieldfp = (int *) malloc(n_framefields * sizeof(int));
  for (i_framefield = 0; i_framefield < n_framefields; i_framefield++) {
    sprintf(filename, "%s/%s", filedirname, framefields[i_framefield]->field);
    if( (fieldfp[i_framefield] = open(filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
      fprintf(stderr,"rnc: Could not create %s\n", filename);
      exit(0);
    }
  }
  
  
  /******************************************/
  /*   read and parse the data  */
  while (1) {
    ioctl(tty_fd, FIONREAD, &numin);

    // Read data from the port into the FIFO
    // don't over-fill the fifo.
    if (numin>=FIFODEPTH - nFifo(&fs) - 2) {
      numin = FIFODEPTH - nFifo(&fs) - 2;
    }
    if (numin) {
      numread = read(tty_fd, inbuf, numin);
      push(&fs, inbuf, numread);
    } else {
      usleep(10000);
    }

    
    // lost or at begining of frame, and enough bytes to read
    while ((index == 0) && (nFifo(&fs) >= sizeof(unsigned))) {
       peek(&fs, (char *)&u_in, sizeof(unsigned));
       if (u_in==SYNCWORD) {
         advance(&fs, sizeof(unsigned));
         index = 1;
         i_lost = 0;
         printf("Found Sync word %d                                     \r", n_sync++);
         fflush(stdout);
       } else {
         printf("Looking for sync word for %d bytes\r", ++i_lost);
         fflush(stdout);
         advance(&fs, 1);
       }
    }
    if (index==0) { // lost.  back to the top of while(1) to wait for data
      continue;
    }
    
    i_framefield = index-1;    

    if (i_framefield < n_framefields) {
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
      if (nFifo(&fs) < fieldsize) {
        continue; // no more data - go wait some more.
      }
      peek(&fs, fielddata, fieldsize);
      advance(&fs, fieldsize);
      write(fieldfp[i_framefield], fielddata, fieldsize);
      index++;
    } else {
      index = 0;
    }
  }
  /*
sleeptime.tv_sec = 0;
  sleeptime.tv_nsec = SLEEPTIME;

      */
}
