// GPL V2+
// (C) Javier Romuldez, 2016 - 2018
// (C) C. Barth Netterfield 2018

#define KATIE_VERSION "2.0"

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <getopt.h>
#include <signal.h>
#include <string.h>

#define NAMELEN 100

#define IDLE_SYNC 150

#define NOR "\x1B[0m"
#define RED "\x1B[31;1m"
#define GRN "\x1B[32;1m"
#define YLW "\x1B[33;1m"
#define BLU "\x1B[34;1m"
#define MAG "\x1B[35;1m"
#define CYN "\x1B[36;1m"
#define NOC "\x1B[?25l"
#define CUR "\x1B[?25h"

char filename[NAMELEN];
char dataname[NAMELEN];

char bit_filename[NAMELEN] = "/data/etc/mole.lnk";
char bit_dataname[NAMELEN] = "ifc_log";
char bit_colortag[NAMELEN] = "\n[] ";

char spider_filename[NAMELEN] = "/data/etc/defile.lnk";
char spider_tdrssname[NAMELEN] = "/data/etc/tdrss.lnk";
char spider_dataname[NAMELEN] = "logstream";
char spider_colortag[NAMELEN] = " \n";

char blast_filename[NAMELEN] = "/data/etc/mole.lnk";
char blast_dataname[NAMELEN] = "chatter";
char blast_colortag[NAMELEN] = " \n";


void usage(char *exe)
{
  fprintf(stderr, "%s [-s|-b] [-b|-k] [-d <DIRFILE>] [-c <FIELD>]\n", exe);
  fprintf(stderr, "Translate FIELD from DIRFILE into ASCII on stdout.\n");
  fprintf(stderr, "-d <PATH_TO_DIRFILE>\n");
  fprintf(stderr, "-c <FIELD>\n");
  fprintf(stderr, "-B                    BLAST:   same as -d %s -c %s\n", blast_filename, blast_dataname);
  fprintf(stderr, "-s                    Spider:   same as -d %s -c %s\n", spider_filename, spider_dataname);
  fprintf(stderr, "-t                    Spider TDRSS:   same as -d %s -c %s\n", spider_tdrssname, spider_dataname);
  fprintf(stderr, "-b                    superBIT: same as -d %s -c %s\n", bit_filename, bit_dataname);
  fprintf(stderr, "-k                    color (default)\n");
  fprintf(stderr, "-m                    monochrome\n");
  
  fprintf(stderr, "This is katie version %s\n", KATIE_VERSION);
  fprintf(stderr, "This is FREE software (GPL) with NO WARRANTY\n");
  exit(-1);
}

int main(int argc, char **argv)
{
  char c;
  char bname[NAMELEN];
  int is_color = 1;
  int sync = 0;
  char * color_tag = spider_colortag;
    

  // FIXME: sticky defaults
  snprintf(filename, NAMELEN, "%s", spider_filename);
  snprintf(dataname, NAMELEN, "%s", spider_dataname);
  
  while ((c = getopt(argc, argv, "d:c:sbtmkB")) != -1) {
    switch (c) {
      case 'd':
        snprintf(filename, NAMELEN, "%s", optarg);
        break;
      case 'c':
        snprintf(dataname, NAMELEN, "%s", optarg);
        break;
      case 's':
        snprintf(filename, NAMELEN, "%s", spider_filename);
        snprintf(dataname, NAMELEN, "%s", spider_dataname);
        color_tag = spider_colortag;
        break;
      case 't':
        snprintf(filename, NAMELEN, "%s", spider_tdrssname);
        snprintf(dataname, NAMELEN, "%s", spider_dataname);
        break;
      case 'b':
        snprintf(filename, NAMELEN, "%s", bit_filename);
        snprintf(dataname, NAMELEN, "%s", bit_dataname);
        color_tag = bit_colortag;
        break;
      case 'B':
        snprintf(filename, NAMELEN, "%s", blast_filename);
        snprintf(dataname, NAMELEN, "%s", blast_dataname);
        color_tag = blast_colortag;
        break;
      case 'k':
        is_color = 1;
        break;
      case 'm':
        is_color = 0;
        break;
      default:
        usage(argv[0]);
        break;
    }
  }
  
  sprintf(bname,"%s/%s",filename,dataname);
  
  if (access(filename,F_OK) != 0) {
    printf("Cannot find dirfile %s\n",filename);
    exit(-1);
  } else if (access(bname,F_OK) != 0) {
    printf("Cannot find \"%s\" in dirfile %s\n",dataname,filename);
    exit(-1);
  }
  
  FILE * df = fopen(bname,"rb");
  printf("Reading \"%s\" from %s\n",dataname,filename);

  int i = 0, j = 0;
  int new_state = 0;
  char state_c = color_tag[0];
  int color_tag_len = strlen(color_tag);
  
  fseek(df,0,SEEK_END);
  while (1) {
    while ((c = fgetc(df)) != EOF) {
      i = 0;
//      if ((c >= 0x20) || (c == '\n')) printf("%c",c);

      new_state = 0;
      for (j = 0; j < color_tag_len; j++) {
          if (c == color_tag[(j+1)%color_tag_len] && state_c == color_tag[j]) {
              new_state = 1;
              break;
          }
      }
      if (new_state) {
          state_c = c;
      }

      if (is_color) {
        if (state_c == color_tag[color_tag_len-1]) {
          if (c == '*' || c == '!' || c == '$') {
            printf(RED);
          }
          if (c == '+' || c == '#') {
            printf(CYN);
          }
          if (c == ':') {
            printf(YLW);
          }
          if (c == '=') {
            printf(MAG);
          }
          if (c == '&') {
            printf(BLU);
          }
        }
        if (c == '\n') {
          printf(NOR);
        }
      }
      if (c >= 0x20 || c == '\n') {
        if (is_color) {
          if (sync > IDLE_SYNC) {
            printf(CUR);
          }
          if (state_c == color_tag[color_tag_len-1] && c != state_c) {
            state_c = color_tag[0];
          }
        }
        putchar(c);
        sync = 0;
      } else {
        sync++;
        if (is_color) {
          if (sync % IDLE_SYNC == 0) {
            printf("%s", (sync / IDLE_SYNC) & 0x1 ? NOC : CUR);
            fflush(stdout);
          }
        }
      }
      fflush(stdout);
    }
    if (i >= 10) {
      fclose(df);
      fopen(bname,"rb");
      fseek(df,0,SEEK_END);
      i = 0;
    }
    i++;
    usleep(100000); // zzzz.....
  }
  
  fclose(df);
  
  return 0;
}
