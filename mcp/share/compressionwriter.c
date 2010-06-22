#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <syslog.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include "mcp.h"
#include "compressstruct.h"

#define FASTFRAME_PER_FRAME 1000
#define N_PORTS 1
#define DIALUP_BYTES_PER_FRAME (2000/9 * FASTFRAME_PER_FRAME/SR)
#define OMNI_BYTES_PER_FRAME (6000/9 * FASTFRAME_PER_FRAME/SR)
#define HIGAIN_BYTES_PER_FRAME (93000/8 * FASTFRAME_PER_FRAME/SR)

#define OMNI_TTY "/dev/ttySI2"


void nameThread(const char*);               /* mcp.c */

extern char *frameList[];
extern struct fieldStreamStruct streamList[];
extern short int InCharge;

static int OpenOmniSerial(void)
{
  int fd;
  struct termios term;

  if ((fd = open(OMNI_TTY, O_RDWR | O_NOCTTY)) < 0)
    berror(tfatal, "Unable to open omni tdrss serial port");

  if (tcgetattr(fd, &term))
    berror(tfatal, "Unable to get omni tdrss serial device attributes");

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  //term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  cfmakeraw(&term);
   
  if (cfsetospeed(&term, B19200))
    berror(tfatal, "Error setting omni tdrss serial output speed");

  if (cfsetispeed(&term, B19200))
    berror(tfatal, "Error setting omni tdrss serial input speed");

  if (tcsetattr(fd, TCSANOW, &term))
    berror(tfatal, "Unable to set omni tdrss serial attributes");

  return fd;
}

void writeOmniData(char *x, int size) {
  static int first_time = 1;
  static int fp;
  
  if (first_time) {
    first_time = 0;
    fp = OpenOmniSerial();
    //fp = open("/tmp/tmp.dat", O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO );
  }

  if (write(fp, x, size)!=size) {
    close(fp);
    first_time = 1;
    bprintf(err, "incomplete write...");
  }
}

void CompressionWriter() {
  int readindex, lastreadindex = 2;
  int n_framelist;
  int n_streamlist;
  int i_fastframe = 0;
  int i_field;
  unsigned int x;
  int isWide;
  int frame_bytes_written;
  int size;
    
  struct NiosStruct **frameNiosList;
  struct BiPhaseStruct **frameBi0List;
  
  //int bytesPerFrame[N_PORTS] = {OMNI_BYTES_PER_FRAME};
  
  nameThread("COMP");

  bputs(startup, "Startup.\n");

  // determine frameList length
  for (n_framelist=0; frameList[n_framelist][0]!='\0'; n_framelist++);
  
  frameNiosList = (struct NiosStruct **)malloc(n_framelist * sizeof(struct NiosStruct *));
  frameBi0List = (struct BiPhaseStruct **)malloc(n_framelist * sizeof(struct BiPhaseStruct *));
  
  for (i_field =0; i_field < n_framelist; i_field++) {
    frameNiosList[i_field] = GetNiosAddr(frameList[i_field]);
    frameBi0List[i_field] = GetBiPhaseAddr(frameList[i_field]);
  }
  
  for (n_streamlist = 0; streamList[n_streamlist].name[0] != '\0'; n_streamlist++);
  
  bprintf(startup, "frame list length: %d  stream list length: %d", n_framelist, n_streamlist);

  while (!InCharge) {  // wait to be the boss to open the port!
    usleep(10000);
  }
  
  while (1) {
    readindex = GETREADINDEX(tdrss_index);
    if (readindex != lastreadindex) {
      lastreadindex = readindex;
      if ((++i_fastframe) % FASTFRAME_PER_FRAME ==0) {
        i_fastframe = 0;
        frame_bytes_written = 0;
       
        x=SYNCWORD;
       writeOmniData((char *)(&x), 4);
       
        // write static frame
        for (i_field = 0; i_field<n_framelist; i_field++) {
            if (frameNiosList[i_field]->fast) {
              if (frameNiosList[i_field]->wide) {
                isWide = 1;
                x = (unsigned int)tdrss_data[readindex][frameBi0List[i_field]->channel] +
                  ((unsigned int)tdrss_data[readindex][frameBi0List[i_field]->channel+1] << 16);
              } else {
                isWide = 0;
                x = tdrss_data[readindex][frameBi0List[i_field]->channel];
              }
            } else { // slow
              if (frameNiosList[i_field]->wide) {
                isWide = 1;
                x = (unsigned int)slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel] + 
                  ((unsigned int)slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel+1] <<16);
              } else {
                isWide = 0;
                x = slow_data[frameBi0List[i_field]->index][frameBi0List[i_field]->channel];
              }
            }
            size = (1 + isWide)*sizeof(unsigned short);
            writeOmniData((char*)&x, size);
            
            frame_bytes_written += size;
        }
      }
    } else {
      usleep(10000);
    }
  }
  
}
