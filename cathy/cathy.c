#define _XOPEN_SOURCE 1000
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <getdata.h>

#define DIRFILE_DEFAULT "/data/etc/defile.lnk"
#define CHATTER "chatter"

#define BUF_LEN 1024

int main (int argc, char **argv)
{
  char char_buffer[BUF_LEN];
  DIRFILE *dirfile;
  off_t nf;
  off_t nf_old = 0;
  size_t n_read;
  uint16_t *data;
  off_t i;
  char a, b;

  snprintf(char_buffer, BUF_LEN, "%s", DIRFILE_DEFAULT);

  if (argc == 2)
  {
    snprintf(char_buffer, BUF_LEN, "%s", argv[1]);
  }

  dirfile = dirfile_open(char_buffer, GD_RDONLY);
  if (get_error(dirfile))
  {
    fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
    dirfile_close(dirfile);
    exit(-1);
  }

  unsigned int spf = get_spf(dirfile, CHATTER);
  if (get_error(dirfile))
  {
    fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
    dirfile_close(dirfile);
    exit(-1);
  }

  data = malloc(BUF_LEN * spf * sizeof(uint16_t));
  if (data == NULL)
  {
    fprintf(stderr, "malloc error!\n");
    exit(-2);
  }

  while (1)
  {
    nf = get_nframes(dirfile);
    if (get_error(dirfile))
    {
      fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, char_buffer, BUF_LEN));
      dirfile_close(dirfile);
      exit(-1);
    }

    if (nf > nf_old)
    {
      n_read = getdata(dirfile, CHATTER, nf_old, 0, BUF_LEN, 0, GD_UINT16, data);
      nf_old += n_read / spf;
      for (i = 0; i < n_read; i++)
      {
        a = data[i] & 0xFF;
        b = data[i] >> 8;
        if (a != 0x16 && a != 0x00)
          putchar(a);
        if (b != 0x16 && b != 0x00)
          putchar(b);
      }
      fflush(stdout);
    } else {
      usleep(100000);
    }
  }
  
  return 0;
}
