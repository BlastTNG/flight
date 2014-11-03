#include <ctype.h>
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
#include <stdint.h>

#include <getdata.h>

#include "slowdl.h"
#include "news.h"

#define RAWDIR "/data/rawdir"

#define LNKFILE "/data/etc/rumor.lnk"

#define PORT 11111

extern struct SlowDlStruct slowDLList[];

int tty_fd;
char hostname[255];

void Usage() {
  printf("rumor <hostname>\n  Connects to an lnc server at hostname to\n"
  "download and append to a dirfile.\n");
  exit(0);
}

int main(int argc, char *argv[]) {
  char dirfilename[1024];
  int i_ch;
  char fieldlist[255][255];
  DIRFILE *df_out;
  int fp_lnc;
  int n_read = 0;
  struct fifoStruct fs;
  
  int i_frame = 0;
  
  uint8_t c_in;
  int16_t s_in;
  uint16_t u_in;
  int32_t S_in;
  uint32_t U_in;
  double x_in;
  
  time_t t;

  fs.i_in = fs.i_out = 0;

  if (argc!=2) Usage();
  if (argv[1][0]=='-') Usage();
      
  strncpy(hostname, argv[1], 250);
  
  sprintf(dirfilename, "%s/%lu.l", RAWDIR, time(NULL));
  df_out = gd_open(dirfilename, GD_RDWR | GD_UNENCODED | GD_CREAT | GD_TRUNC);
  /* add fields to dirfile */
  for (i_ch =0; slowDLList[i_ch].name[0] != '\0'; i_ch++) {
    convertToUpper(slowDLList[i_ch].name, fieldlist[i_ch]);
    if (slowDLList[i_ch].encode == SDL_SCALE) {
      gd_add_raw(df_out, fieldlist[i_ch], GD_FLOAT64, 1, 0);
    } else {
      switch (slowDLList[i_ch].type) {
        case 'c':
          gd_add_raw(df_out, fieldlist[i_ch], GD_UINT8, 1, 0);
          break;
        case 's':
          gd_add_raw(df_out, fieldlist[i_ch], GD_INT16, 1, 0);
          break;
        case 'u':
          gd_add_raw(df_out, fieldlist[i_ch], GD_UINT16, 1, 0);
          break;
        case 'S':
          gd_add_raw(df_out, fieldlist[i_ch], GD_INT32, 1, 0);
          break;
        case 'U':
          gd_add_raw(df_out, fieldlist[i_ch], GD_UINT32, 1, 0);
          break;
        default:
          break; // shouldn't be possible
      }
    }      
  }
  gd_flush(df_out, NULL);
  
  unlink(LNKFILE);
  if (symlink(dirfilename, LNKFILE)<0) {
    fprintf(stderr, "could not create link from `%s' to `%s'",
            dirfilename, LNKFILE);
            exit(0);
  }

  strncpy(hostname, argv[1], 250);
  
  fp_lnc = party_connect(hostname, PORT);
  
  while (1) {
    do {
      if (nFifo(&fs)<4) {
        n_read += BlockingRead(4, &fs, fp_lnc, hostname, PORT);
      }
      peek(&fs, (char *)&U_in, 4);
      advance(&fs, 1);
    } while (U_in != SLOWDLSYNCWORD);
    
    advance(&fs, 4-1);
    
    for (i_ch =0; slowDLList[i_ch].name[0] != '\0'; i_ch++) {
      // read the word
      switch (slowDLList[i_ch].type) {
        case 'c':
          if (nFifo(&fs)<1) {
            n_read += BlockingRead(1, &fs, fp_lnc, hostname, PORT);
          }
          pop(&fs, (char *)&c_in, 1);
          break;
        case 's':
          if (nFifo(&fs)<2) {
            n_read += BlockingRead(2, &fs, fp_lnc, hostname, PORT);
          }
          pop(&fs, (char *)&s_in, 2);
          break;
        case 'u':
          if (nFifo(&fs)<2) {
            n_read += BlockingRead(2, &fs, fp_lnc, hostname, PORT);
          }
          pop(&fs, (char *)&u_in, 2);
          break;
        case 'S':
          if (nFifo(&fs)<4) {
            n_read += BlockingRead(4, &fs, fp_lnc, hostname, PORT);
          }
          pop(&fs, (char *)&S_in, 4);
          break;
        case 'U':
          if (nFifo(&fs)<4) {
            n_read += BlockingRead(4, &fs, fp_lnc, hostname, PORT);
          }
          pop(&fs, (char *)&U_in, 4);
          break;
        default:
          break;
      }
      // write the word
      if (slowDLList[i_ch].encode == SDL_SCALE) {
        switch (slowDLList[i_ch].type) {
          case 'c':
            x_in = (double)c_in / (double)0xff;
            break;
          case 'u':
            x_in = (double)u_in / (double)0xffff;
            break;
          case 'U':
            x_in = (double)U_in / (double)0xffff;
            break;
          default: // not allowed
            break;
        }
        x_in = slowDLList[i_ch].min + x_in * (slowDLList[i_ch].max - slowDLList[i_ch].min);
        gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_FLOAT64, &x_in);
      } else {
        switch (slowDLList[i_ch].type) {
          case 'c':
            gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_UINT8, &c_in);
            break;
          case 's':
            gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_INT16, &s_in);
            break;
          case 'u':
            gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_UINT16, &u_in);
            break;
          case 'S':
            gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_INT32, &S_in);
            break;
          case 'U':
            gd_putdata(df_out, fieldlist[i_ch], i_frame, 0, 1, 0, GD_UINT32, &U_in);
            break;
          default: // shouldn't happen
            break;
        }
      }
    } // next i_ch;
    t = time(NULL);
    printf("%s: frame %4d - %s", argv[0], i_frame, ctime(&t)); 
    i_frame++;
  }
  return 0;
}
