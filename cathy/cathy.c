/* cathy.c: Converts a dirfile field to a text stream on stdout
 *
 * This software is copyright (C) 2010 Matthew Truch
 *
 * This file is part of cathy licensed under the GNU
 * General Public License.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#define CATHY_VERSION "1.1.0"

#define _XOPEN_SOURCE 1000
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdint.h>
#include <getdata.h>
#include <getopt.h>
#include <signal.h>

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
  fprintf(stderr, "%s [-d DIRFILE] [-c CHAR_FIELD] [-r|-s] [-v|-q] [-b|-k] [-7|-8]\n", exe);
  fprintf(stderr, "Translate CHAR_FIELD from DIRFILE into ASCII on stdout.\n");
  fprintf(stderr, "-d PATH_TO_DIRFILE [%s]\n", DIRFILE_DEFAULT);
  fprintf(stderr, "-c CHAR_FIELD      [%s]\n", CHATTER_DEFAULT);
  fprintf(stderr, "-r (reconnect) | -s (single shot) [-r]\n");
  fprintf(stderr, "-v (verbose) | -q (quiet) [-v]\n");
  fprintf(stderr, "-b (b&w) | -k (color) [auto-detect]\n");
  fprintf(stderr, "-7 (7-bit ascii) | -8 (UTF-8) [-7]\n");
  fprintf(stderr, "This is cathy version %s\n", CATHY_VERSION);
  fprintf(stderr, "This is FREE software (GPL) with NO WARRANTY\n");
  exit(-1);
}

void clear_colors(int sig)
{
  printf("%s%s\n", NOR, CUR);
  signal(SIGINT, SIG_DFL);
  exit(1);
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
  int utf8 = 0;
  int index;
  int old_index = -1;

  snprintf(dirfile_name, BUF_LEN, "%s", DIRFILE_DEFAULT);
  snprintf(chatter_name, BUF_LEN, "%s", CHATTER_DEFAULT);

  while ((c = getopt(argc, argv, "d:c:rqvsh?f:bk78")) != -1)
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
      case '7':
        utf8 = 0;
        break;
      case '8':
        utf8 = 1;
        break;
      case 'h':
      case '?':
      default:
        usage(argv[0]);
        break;
    }
  }

  if (color)
    signal(SIGINT, clear_colors);

  while (1) /* Main Loop */
  {
    if (verbose)
    {
      fprintf(stderr, "Reading \"%s\" from %s\n", chatter_name, dirfile_name);
    }
 
    dirfile = gd_open(dirfile_name, GD_RDONLY);
    if (gd_error(dirfile))
    {
      fprintf(stderr, "GetData error: %s\n", gd_error_string(dirfile, char_buffer, BUF_LEN));
      gd_close(dirfile);
      exit(-1);
    }
 
    unsigned int spf = gd_spf(dirfile, chatter_name);
    if (gd_error(dirfile))
    {
      fprintf(stderr, "GetData error: %s\n", gd_error_string(dirfile, char_buffer, BUF_LEN));
      gd_close(dirfile);
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
      nf_old = gd_nframes(dirfile);
      if (gd_error(dirfile))
      {
        fprintf(stderr, "GetData error: %s\n", gd_error_string(dirfile, char_buffer, BUF_LEN));
        gd_close(dirfile);
        exit(-4);
      }
    } else {
      nf_old = ff;
    }

    prev_char = '\n';
    sync = 0;
  
    while (1) /* Data reading loop */
    {
      nf = gd_nframes(dirfile);
      if (gd_error(dirfile))
      {
        fprintf(stderr, "GetData error: %s\n", gd_error_string(dirfile, char_buffer, BUF_LEN));
        gd_close(dirfile);
        if (reload)
          break;
        else
          exit(-5);
      }
  
      if (nf > nf_old)
      {
        old_data = 0;
        n_read = gd_getdata(dirfile, chatter_name, nf_old, 0, BUF_LEN, 0, GD_UINT16, data);
        nf_old += (n_read / spf);
        for (i = 0; i < n_read; i++)
        {
          if (utf8)
          {
            a = data[i] & 0xFF;
            b = (data[i] >> 8) & 0xFF;
          } else {
            a = data[i] & 0x7F;
            b = (data[i] >> 8) & 0x7F;
            index = ((data[i] & 0x80) >> 6) + ((data[i] & 0x8000) >> 15);
            if (old_index == -1 && index > 0)
              old_index = index - 1;
            if (old_index > -1) //Check that the index is sequential.
            {
              if (index == old_index) {
                old_index = index;
                continue;
              } else if (index == (old_index + 2) % 0x4) {
                if (a != 0x16 && a != 0x00 && b != 0x16 && b != 0x00)
                  printf("__");
              } else if (index == (old_index + 3) % 0x4) {
                if (a != 0x16 && a != 0x00 && b != 0x16 && b != 0x00)
                  printf("____");
              }
              old_index = index;
            }
          }

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
            gd_close(dirfile);
            break;
          }
        }
      }
    }
  } 
  return 0;
}
