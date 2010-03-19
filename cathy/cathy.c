#define _XOPEN_SOURCE 1000
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <getdata.h>
#include <getopt.h>

#define DIRFILE_DEFAULT "/data/etc/defile.lnk"
#define CHATTER_DEFAULT "chatter"
#define OLD_DATA_LIMIT 50
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


#define BUF_LEN 1024

void usage(char *exe)
{
  fprintf(stderr, "%s [-d DIRFILE] [-c CHAR_FIELD] [-r|-s] [-v|-q] [-b|-k]\n", exe);
  fprintf(stderr, "Translate CHAR_FIELD from DIRFILE into ASCII on stdout.\n");
  fprintf(stderr, "-d PATH_TO_DIRFILE [%s]\n", DIRFILE_DEFAULT);
  fprintf(stderr, "-c CHAR_FIELD      [%s]\n", CHATTER_DEFAULT);
  fprintf(stderr, "-r (reconnect) | -s (single shot) [-r]\n");
  fprintf(stderr, "-v (verbose) | -q (quiet) [-v]\n");
  fprintf(stderr, "-b (b&w) | -k (color) [auto-detect]\n");
  exit(-1);
}

int main (int argc, char **argv)
{
  char char_buffer[BUF_LEN];
  char dirfile_name[BUF_LEN];
  char chatter_name[BUF_LEN];
  DIRFILE *dirfile;
  off_t nf;
  off_t nf_old = 0;
  off_t ff = -1;
  size_t n_read;
  uint16_t *data;
  off_t i;
  char a, b, prev_char;
  int c;
  int sync;

  unsigned int old_data = 0;
  int reload = 1;
  int verbose = 1;
  int color = isatty(fileno(stdout));

  snprintf(dirfile_name, BUF_LEN, "%s", DIRFILE_DEFAULT);
  snprintf(chatter_name, BUF_LEN, "%s", CHATTER_DEFAULT);

  while ((c = getopt(argc, argv, "d:c:rqvsh?f:bk")) != -1)
  {
    switch (c)
    {
      case 'd':
        snprintf(dirfile_name, BUF_LEN, "%s", optarg);
        break;
      case 'c':
        snprintf(chatter_name, BUF_LEN, "%s", optarg);
        break;
      case 'r':
        reload = 1;
        break;
      case 's':
        reload = 0;
        break;
      case 'v':
        verbose = 1;
        break;
      case 'q':
        verbose = 0;
        break;
      case 'f':
        ff = (off_t)strtoll(optarg, NULL, 0);
        break;
      case 'b':
        color = 0;
        break;
      case 'k':
        color = 1;
        break;
      case 'h':
      case '?':
      default:
        usage(argv[0]);
        break;
    }
  }

  while (1) /* Main Loop */
  {
    if (verbose)
    {
      fprintf(stderr, "Reading %s from %s\n", chatter_name, dirfile_name);
    }
 
    dirfile = dirfile_open(dirfile_name, GD_RDONLY);
    if (get_error(dirfile))
    {
      fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
      dirfile_close(dirfile);
      exit(-1);
    }
 
    unsigned int spf = get_spf(dirfile, chatter_name);
    if (get_error(dirfile))
    {
      fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
      dirfile_close(dirfile);
      exit(-2);
    }
 
    data = malloc(BUF_LEN * spf * sizeof(uint16_t));
    if (data == NULL)
    {
      fprintf(stderr, "malloc error!\n");
      exit(-3);
    }

    if (ff < 0)
    {
      nf_old = get_nframes(dirfile);
      if (get_error(dirfile))
      {
        fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
        dirfile_close(dirfile);
        exit(-4);
      }
    } else {
      nf_old = ff;
    }

    prev_char = '\n';
    sync = 0;
  
    while (1) /* Data reading loop */
    {
      nf = get_nframes(dirfile);
      if (get_error(dirfile))
      {
        fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
        dirfile_close(dirfile);
        if (reload)
          break;
        else
          exit(-5);
      }
  
      if (nf > nf_old)
      {
        old_data = 0;
        n_read = getdata(dirfile, chatter_name, nf_old, 0, BUF_LEN, 0, GD_UINT16, data);
        nf_old += (n_read / spf);
        for (i = 0; i < n_read; i++)
        {
          a = data[i] & 0xFF;
          b = data[i] >> 8;

          if (color)
          {
            if (prev_char == '\n' && (a == '*' || a == '!' || a == '$'))
              printf(RED);
            if (a == '\n')
              printf(NOR);
          }
          if (a != 0x16 && a != 0x00)
          {
            if (color && sync > IDLE_SYNC)
              printf(CUR);
            putchar(a);
            prev_char = a;
            sync = 0;
          } else {
            sync++;
          }

          if (color)
          {
            if (prev_char == '\n' && (b == '*' || b == '!' || b == '$'))
              printf(RED);
            if (b == '\n')
              printf(NOR);
          }
          if (b != 0x16 && b != 0x00)
          {
            if (color && sync > IDLE_SYNC)
              printf(CUR);
            putchar(b);
            prev_char = b;
            sync = 0;
          } else {
            sync++;
            if (color)
              if (sync % IDLE_SYNC == 0)
              {
                printf("%s", (sync / IDLE_SYNC) & 0x1 ? NOC : CUR);
                fflush(stdout);
              }
          }
        }
        fflush(stdout);
      } else {
        usleep(100000);
        old_data++;

        if (reload)
        {
          if (old_data > OLD_DATA_LIMIT)
          {
            old_data = 0;
            nf_old = 0;
            dirfile_close(dirfile);
            break;
          }
        }
      }
    }
  } 
  return 0;
}
