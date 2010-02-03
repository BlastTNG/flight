#include <stdio.h>
#include <stdlib.h>
#include <getdata.h>

#define DIRFILE_DEFAULT "/data/etc/defile.lnk"
#define CHATTER "chatter"

int main (int argc, char **argv)
{
  char error_buffer[1024];
  DIRFILE *dirfile;

  if (argc >= 2)
    dirfile_open(argv[1], GD_RDONLY);
  else
    dirfile_open(DIRFILE_DEFAULT, GD_RDONLY);

  if (get_error(dirfile))
  {
    fprintf(stderr, "GetData error: %s\n", get_error_string(dirfile, error_buffer, 1024));
    dirfile_close(dirfile);
    exit(-1);
  }

  
  return 0;
}
