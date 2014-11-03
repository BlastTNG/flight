#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define RAWDIR "/data/rawdir"

#define FIFODEPTH 16384
#define NDATA 8192
unsigned char data[2][NDATA];

// port 0 is tdrss highgain.  Port 1 is tdrss omni, iridium dialup, and slow packets
#define FIRSTPORT 0
#define LASTPORT 1

char *ttydev[2] = {"/dev/ttyHIGAIN", "/dev/ttyOMNI"};
char *linkfile[5] = {"/data/etc/highgain.lnk","/data/etc/omni.lnk", "/data/etc/iromni.lnk", "/data/etc/tdomni.lnk", "/data/etc/irslow.lnk"};
char *ext[5]={"highgain", "omni", "iromni", "tdomni", "irslow"};

int n_tdomni = 0;
int n_iromni = 0;
int n_slow = 0;
int n_junk = 0;

enum ModeType{LOST, JUNK, IROMNI, TDOMNI, IRSLOW};

struct fifoStruct {
  char d[FIFODEPTH];
  int i_in;  // points at next place to write
  int i_out; // points at next place to read
};

struct fifoStruct parseFifo;

//*********************************************************
// insert data into the fifo
//*********************************************************
void push(struct fifoStruct *fs, unsigned char x[], int size) {
  int i;
  for (i=0; i<size; i++) {
    fs->d[fs->i_in] = x[i];
    fs->i_in++;
    if (fs->i_in>=FIFODEPTH) {
      fs->i_in = 0;
    }
  }
}

//*********************************************************
// return data w/out removing it
//*********************************************************
void peek(struct fifoStruct *fs, unsigned char x[], int size) {
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

//*********************************************************
// advance the fifo pointer (removes data)
//*********************************************************
void advance(struct fifoStruct *fs, int size) {
  fs->i_out += size;
  if (fs->i_out >= FIFODEPTH) {
    fs->i_out -= FIFODEPTH;
  }
}

//*********************************************************
// remove data from the fifo
//*********************************************************
void pop(struct fifoStruct *fs, unsigned char x[], int size) {
  peek(fs, x, size);
  advance(fs,size);
}

//*********************************************************
// how many bytes are availible in the fifo
//*********************************************************
int nFifo(struct fifoStruct *fs) {
  int n;

  n = fs->i_in - fs->i_out;
  if (n < 0) n+= FIFODEPTH;

  return n;
}

//*********************************************************
// OpenSerial: open serial port
//*********************************************************
int OpenSerial(char* device) {
  int fd;
  struct termios term;

  if ((fd = open(device, O_RDWR | O_NONBLOCK)) < 0) {
    syslog(LOG_ERR | LOG_DAEMON, "Unable to open serial port.\n");
    fprintf(stderr, "Unable to open serial port %s.\n", device);
    exit(1);
  }
  if (tcgetattr(fd, &term)) {
    syslog(LOG_ERR | LOG_DAEMON, "Unable to get serial device attributes.");
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

  cfmakeraw(&term);

  if (cfsetospeed(&term, B115200)) {       // Baud
    syslog(LOG_ERR | LOG_DAEMON, "Error setting serial output speed.");
    return -1;
  }
  if (cfsetispeed(&term, B115200)) {       // Baud
    syslog(LOG_ERR | LOG_DAEMON, "Error setting serial output speed.");
    return -1;
  }
  if(tcsetattr(fd, TCSANOW, &term)) {
    syslog(LOG_ERR | LOG_DAEMON, "Unable to set serial attributes.");
    return -1;
  }

  syslog(LOG_INFO | LOG_DAEMON, "%s open\n", device);
  return fd;
}

//*********************************************************
//  Initialize stream files...
//*********************************************************
int InitialiseStreamFile(int i) {
  int fd;
  
  char filename[255];
  static time_t filetime = 0;
  
  if (filetime==0) {
    filetime = time(NULL);
  }
  
  sprintf(filename, "%s/%lu.%s", RAWDIR, filetime, ext[i]);
  
  if( (fd = open(filename, O_WRONLY | O_CREAT, 00644)) < 0 ) {
    syslog(LOG_ERR | LOG_DAEMON, "Error opening data file\n");
    fprintf(stderr, "Error opening data file %s\n", filename);
    exit(0);
  }
  
  unlink(linkfile[i]);
  if (symlink(filename, linkfile[i])<0) {
    fprintf(stderr, "tdrsslogger: could not create link from `%s' to `%s'",
            filename, linkfile[i]);
            exit(0);
  }
  
  return fd;
}

//*********************************************************
// Return header type
//*********************************************************
enum ModeType isHeader(unsigned char *D) {
  int size;

  if (D[0]!=0xfa) return LOST;
  if ((D[1]&0xf8)!=0xf8) return LOST;
  if (D[1]==0xf8) return LOST;
  if (D[1]==0xf9) return LOST;
  if (D[1]==0xfe) return LOST; // FIXME: do these ever actually happen?
  if (D[3] != 0) return LOST;
  //if ((D[2] & 0xf0) !=0) return LOST;

  size = (int)D[5] + (int)D[4]*256;

  if (size > 4096) return LOST;

  if (D[1]==0xfd) { // iridium
    if ((D[2]&0x03) == 0x01) return IRSLOW;
    if ((D[2]&0x03) == 0x02) return IROMNI;
  }
  if (D[1]==0xff) { // tdrss
    if ((D[2]&0x03) == 0x01) return IRSLOW; // FIXME: change for real use
    if ((D[2]&0x03) == 0x02) return TDOMNI;
  }
  
  return (JUNK);
}

  
//*********************************************************
// Sparate omni data into separate streams
//*********************************************************
void processOmniData() {

  static int first_time = 1;
  static int fd[3];
  static enum ModeType mode = LOST;
  int i;
  unsigned char data_in[16384];
  static int size = 0;
  
  if (first_time) {
    first_time = 0;

    for (i=0; i<3; i++) {
      fd[i] = InitialiseStreamFile(i+2);
    }
  } // end first time

  while ((mode == LOST) && (nFifo(&parseFifo)>=6)) {
    peek(&parseFifo, data_in, 6);
    mode = isHeader(data_in);
    if (mode != LOST) {
      size = data_in[4]*256 + data_in[5];
      advance(&parseFifo, 6);
    } else {
      advance(&parseFifo, 1);
    }
  } // end while lost
  if ((mode != LOST) && (nFifo(&parseFifo)>=size+1)) {
    pop(&parseFifo, data_in, size+1);
    if (mode == TDOMNI) {
      write(fd[1], data_in, size);
      n_tdomni++;
    } else if (mode == IROMNI) {
      write(fd[0], data_in, size);
      n_iromni++;
    } else if (mode == IRSLOW) {
      write(fd[2], data_in, size);
      n_slow++;
    } else {
      n_junk++;
    }
    mode = LOST;
  }
}

//*********************************************************
// Main loop
//*********************************************************
int main(void) {
  int tty[2];
  int fd[2];
  int b_in[2];
  int bytes_read[2] = {0,0};
  time_t t0[2], t[2], dt[2];
  double rate[2] = {0,0};
  int bytes0[2] = {0,0};
  time_t last_t=0;

  int i;

  for (i=FIRSTPORT; i<=LASTPORT; i++) {
    if( (tty[i] = OpenSerial(ttydev[i])) < 0) return 1;
    if( (fd[i] = InitialiseStreamFile(i)) < 0 ) return 1;

    bytes0[i] = 0;
    rate[i] = 0;
    t0[i] = time(NULL);
  }
  
  while (1) {
    for (i=FIRSTPORT; i<=LASTPORT; i++) {
      b_in[i] = read(tty[i], data[i], NDATA);

      if (b_in[i]>0) {
        bytes_read[i]+=b_in[i];
        write(fd[i], data[i], b_in[i]);
        if (i==1) {
          push(&parseFifo, data[i], b_in[i]); // send data to the parse thread
        }
        t[i] = time(NULL);
        dt[i] = t[i] - t0[i];
        if (dt[i]>=20) {
          rate[i] = (double)(bytes_read[i] - bytes0[i])/(double)dt[i];
          t0[i] = t[i];
          bytes0[i] = bytes_read[i];
        }
        if (t[i]-last_t > 10) {
          last_t = t[i];
          printf("High Gain: %7d bytes at %5.0fbps  Omni: %6d bytes Packets: omni: %d dialup: %d slow: %d junk: %d at %s",
                 bytes_read[0], rate[0]*8.0, bytes_read[1], n_tdomni, n_iromni, n_slow, n_junk, ctime(t+i));
                 fflush(stdout);
        }
      }
    } // next port
    
    processOmniData();
    
    usleep(10000);
  } // continue while 1
  return 0; // can't get here
}
