/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004 D. V. Wiebe
 * 
 * This file is part of defile.
 * 
 * defile is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defile; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <stdlib.h>
#include <limits.h>
#include <libgen.h>
#include <stdio.h>
#include <error.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "defile.h"
#include "tx_struct.h"

#define VERSION_MAJOR    "1"
#define VERSION_MINOR    "0"
#define VERSION_REVISION "0"
#define VERSION VERSION_MAJOR "." VERSION_MINOR "." VERSION_REVISION 

#define DEFAULT_CURFILE "/data/etc/defile.cur"
#define DEFAULT_DIR "/data/rawdir"
#define REMOUNT_PATH "../rawdir"

struct ri_struct ri;
struct rc_struct rc = {
  0, /* framefile */
  0, /* persist */
  0, /* remount */
  0, /* resume */
  0, /* write_curfile */
  NAME_MAX, /* sufflen */
  0, /* source_is_curfile */
  NULL, /* curfile_val */
  NULL, /* remount_dir */
  NULL, /* output_curfile */
  NULL, /* output_dirfile */
  NULL, /* source */
  NULL /* dest_dir */
};

void PathSplit(const char* path, const char** dname, const char** bname)
{
  static char static_base[NAME_MAX];
  static char static_path[PATH_MAX];
  char* base = NULL, *ptr;
  char* buffer;

  if ((buffer = strdup(path)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  for (ptr = buffer; *ptr != '\0'; ++ptr)
    if (*ptr == '/')
      base = ptr + 1;

  if (base == NULL) { /* this is "foo" */
    strncpy(static_base, buffer, NAME_MAX);
    strcpy(static_path, ".");
  } else if (base == buffer + 1) {
    if (base[0] == '\0') { /* this is "/" */
      strncpy(static_path, buffer, NAME_MAX);
      strcpy(static_base, ".");
    } else { /* this is "/foo" */
      strcpy(static_path, "/");
      strncpy(static_base, base, NAME_MAX);
    }
  } else { /* this is "foo/bar" */
    *(base - 1) = '\0';
    strncpy(static_base, base, NAME_MAX);
    strncpy(static_path, buffer, PATH_MAX);
  }

  if (dname != NULL)
    *dname = static_path;

  if (bname != NULL)
    *bname = static_base;

  free(buffer);
}

char* ResolveOutputDirfile(char* dirfile, const char* parent)
{
  const char* parent_part, *dirfile_part;
  char path[PATH_MAX];
  char gpb[GPB_LEN];

  /* is dirfile a relative path if so, we don't have to do anything */
  if (dirfile[0] != '/') {
    /* check string sizes */
    if (strlen(parent) + 1 + strlen(dirfile) >= PATH_MAX) {
      fprintf(stderr, "defile: output dirfile path is too long\n");
      exit(1);
    }

    strcpy(path, parent);
    if (path[strlen(path) - 1] != '/')
      strcat(path, "/");
    strcat(path, dirfile);
  } else
    strcpy(path, dirfile);

  /* realpath() will fail with a non-existant path, so strip off dirfile name */
  PathSplit(path, &parent_part, &dirfile_part);

  /* canonicalise the path */
  if (realpath(parent_part, dirfile) == NULL) {
    snprintf(gpb, GPB_LEN, "defile: cannot resolve output dirfile path `%s'",
        parent_part);
    perror(gpb);
    exit(1);
  }

  /* add back dirfile name */
  if (dirfile[strlen(dirfile) - 1] != '/')
    strcat(dirfile, "/");
  strcat(dirfile, dirfile_part);

  return dirfile;
}

void Remount(const char* source, char* buffer)
{
  const char* element;
  char real_path[PATH_MAX];
  char path[PATH_MAX];
  char gpb[GPB_LEN];

  /* is the remount_dir an absolute path? */
  if (rc.remount_dir[0] == '/') {
    strcpy(path, rc.remount_dir);
  } else {
    /* get the curfile's directory) */
    PathSplit(source, &element, NULL);

    /* check string sizes */
    if (strlen(element) + 1 + strlen(rc.remount_dir) >= PATH_MAX) {
      fprintf(stderr, "defile: remounted path is too long\n");
      exit(1);
    }

    strcpy(path, element);
    if (path[strlen(path) - 1] != '/')
      strcat(path, "/");
    strcat(path, rc.remount_dir);
  }

  /* canonicalise the path */
  if (realpath(path, real_path) == NULL) {
    snprintf(gpb, GPB_LEN, "defile: cannot resolve remounted path `%s'", path);
    perror(gpb);
    exit(1);
  }

  /* now get the indirect file's filename */
  PathSplit(buffer, NULL, &element);

  strcpy(buffer, real_path);
  if (buffer[strlen(buffer) - 1] != '/')
    strcat(buffer, "/");
  strcat(buffer, element);
}

/* check SOURCE input parameter and dereference curfile if applicable */
char* GetFileName(const char* source)
{
  FILE* stream;
  char* buffer;
  int index = 0;
  int is_binary = 0;
  struct stat stat_buf;
  char gpb[GPB_LEN];

  /* allocate our buffer */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  /* first attempt to stat SOURCE to see if it is indeed a regular file */
  if (stat(source, &stat_buf)) {
    snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", source);
    perror(gpb);
    exit(1);
  }

  /* the stat worked.  Now is this a regular file? */
  if (!S_ISREG(stat_buf.st_mode)) {
    fprintf(stderr, "defile: `%s' is not a regular file\n", source);
    exit(1);
  }

  /* attempt to open the file */
  if ((stream = fopen(source, "r")) == NULL) {
    snprintf(gpb, GPB_LEN, "defile: cannot open `%s'", source);
    perror(gpb);
    exit(1);
  }

  if (rc.framefile)
    /* user has indicated SOURCE is a framefile, nothing more to do */
    strcpy(buffer, source);
  else {
    /* attempt to determine if this is a curfile or not */
    do {
      if ((buffer[index] = fgetc(stream)) == EOF) {
        if (feof(stream)) {
          buffer[index] = '\0';
          break;
        } else {
          snprintf(gpb, GPB_LEN, "defile: error reading `%s'", source);
          perror(gpb);
          exit(1);
        }
      }

      if (buffer[index] == '\n') {
        buffer[index] = '\0';
        break;
      }
      if (buffer[index] < 32 || buffer[index] > 126) {
        is_binary = 1;
        break;
      }
    } while (index++ < FILENAME_LEN);

    if (is_binary || index >= FILENAME_LEN)
      /* found a non-ASCII character or went too long without finding a newline
       * or EOF, so guess that SOURCE is a real frame file */
      strcpy(buffer, source);
    else {
      /* no non-text characters found, assume we have a curfile */
      rc.source_is_curfile = 1;

      /* if we're in persistent mode, we need to remember the contents of the
       * curfile so we can check for changed */
      if (rc.persist)
        if ((rc.curfile_val = strdup(buffer)) == NULL) {
          perror("defile: cannot allocate heap");
          exit(1);
        }

      if (rc.remount)
        /* user indicated curfile filesystem has been remounted, so fix up the
         * source path */
        Remount(source, buffer);

      if (stat(buffer, &stat_buf)) {
        /* stat failed, complain and die */
        snprintf(gpb, GPB_LEN, "defile: cannot stat `%s' pointed to by curfile",
            buffer);
        perror(gpb);
        exit(1);
      } else
        /* the stat worked.  Now is this a regular file? */
        if (!S_ISREG(stat_buf.st_mode)) {
          /* buffer doesn't point to a regular file, assume SOURCE is a frame
           * file */
          fprintf(stderr, "defile: `%s' is not a regular file\n", buffer);
          exit(1);
        }
    }
  }

  fclose(stream);

  return buffer;
}

/* given a source filename, fills in the part of it which is static from chunk
 * to chunk, the value of the counter, returns the length of the non-static
 * suffix */
int StaticSourcePart(char* output, const char* source, long* value)
{
  char* buffer;
  char* ptr;
  int counter = 0;
  long number = 0;

  if ((buffer = strdup(source)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  /* walk backwards through source looking for first non-hex digit */
  for(ptr = buffer + strlen(buffer) - 1; counter < rc.sufflen && ptr != buffer;
      --ptr)
    if (*ptr >= '0' && *ptr <= '9') {
      number += (*ptr - '0') << 4 * counter++;
      *ptr = '\0';
    } else if (*ptr >= 'a' && *ptr <= 'f') {
      number += (*ptr - 'a') << 4 * counter++;
      *ptr = '\0';
    } else if (*ptr >= 'A' && *ptr <= 'F') {
      number += (*ptr - 'A') << 4 * counter++;
      *ptr = '\0';
    } else
      break;

    if (value != NULL)
      *value = number;
    strcpy(output, buffer);

    free(buffer);

    return counter;
}

/* given a destination path and a source filename, makes a dirfile name and
 * returns it in output */
char* MakeDirFile(char* output, const char* source, const char* directory)
{
  const char* bname;
  char* buffer;

  /* allocate our buffer */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  PathSplit(source, NULL, &bname);
  StaticSourcePart(buffer, bname, NULL);

  strcpy(output, directory);
  if (output[strlen(output) - 1] != '/')
    strcat(output, "/");
  strcat(output, buffer);

  free(buffer);

  return output;
}

/* generates a dirfile name given the source and destination passed on the
 * command line */
char* GetDirFile(const char* source, char* parent)
{
  char* buffer;
  struct stat stat_buf;
  char gpb[GPB_LEN];

  /* allocate our buffer */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  /* Step 1: stat parent to make sure it exists */
  if (stat(parent, &stat_buf)) {
    /* stat failed -- parent doesn't exist; complain and exit */
    snprintf(gpb, GPB_LEN, "defile: cannot stat `%s'", parent);
    perror(gpb);
    exit(1);
  }

  /* parent exists.  Is it a directory? */
  if (!S_ISDIR(stat_buf.st_mode)) {
    /* not a directory, complain and exit */
    fprintf(stderr, "defile: `%s' is not a directory", parent);
    exit(1);
  }

  /* parent is indeed a directory; make the dirfile name */
  return MakeDirFile(buffer, source, parent);
}

void PrintVersion(void)
{
  printf("Defile " VERSION "  (C) 2004 D. V. Wiebe\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n"
      "This program comes with NO WARRANTY, not even for MERCHANTABILITY or "
      "FITNESS\n"
      "FOR A PARTICULAR PURPOSE. You may redistribute it under the terms of "
      "the GNU\n"
      "General Public License; see the file named COPYING for details.\n"
      "Written by D.V. Wiebe.\n"
      );
  exit(0);
}

void PrintUsage(void)
{
  printf("Usage: defile [OPTION]... SOURCE [DIRECTORY]\n"
      "  or:  defile [OPTION]... SOURCE DESTINATION\n"
      "Convert the BLAST-type framefile SOURCE into dirfile DESTINATION or a "
      "dirfile\n"
      "under DIRECTORY.\n"
      "\nSOURCE may be either a .cur file or a framefile to start with.\n"
      "\nDefault DIRECTORY to use if no DIRECTORY or DESTINATION is given:\n"
      "\t" DEFAULT_DIR "\n\n"
      "Arguments to long options are required for short arguments as well.\n"
      "  -c --curfile          write a curfile called `" DEFAULT_CURFILE "'\n"
      "                          pointing to the destination dirfile.\n"
      "  -C --curfile-name=NAME same as `--curfile' but use NAME as the name of the\n"
      "                          file instead of `" DEFAULT_CURFILE "'.\n"
      "  -f --force-framefile  always assume SOURCE is a framefile, even when "
      "it\n"
      "                          appears to be a curfile\n"
      "  -p --persistent       do not exit upon reaching the end of the "
      "framefile.\n"
      "                          Instead, defile will monitor the file and "
      "wait until\n"
      "                          new data is added to the file.  If SOURCE is "
      "a\n"
    "                          curfile, defile will also look for changes "
    "in this.\n"
    "  -r --remounted-source when SOURCE is a curfile, assume that it is "
    "located\n"
    "                          on a filesystem that has been mounted in a "
    "different\n"
    "                          location from the one referred to by the "
    "curfile.\n"
    "                          The filename extracted from the curfile is "
    "assumed to\n"
    "                          be in the directory `" REMOUNT_PATH "' relative "
    "to\n"
    "                          the curfile's location.  This is especially "
    "useful\n"
    "                          for NSF mounted filesystems.  This option has "
    "no\n"
    "                          effect if SOURCE is not a curfile.\n"
    "     --remounted-using=DIR same as `--remounted-source' except use DIR as the\n"
    "                          relative path instead of the default `"
    REMOUNT_PATH "'.\n"
    //    "  -R --resume           resume an interrupted defiling.\n"
    "  -s --suffix-size=SIZE assume the framefile suffix (the portion of "
    "the\n"
    "                          framefile which is incremented as a "
    "hexadecimal\n"
    "                          number) is no more than SIZE characters "
    "large.\n"
    "                          SIZE should be an integer between 0 and %i.\n"
    "  --help                display this help and exit\n"
    "  --version              display this version information and exit\n"
    "  --                    last option; all following parameters are "
    "arguments.\n\n"
    "Summary of defaults:"
    "\n  Default curfile name :  " DEFAULT_CURFILE
    "\n  Default remount path :  " REMOUNT_PATH
    "\n  Default output path  :  " DEFAULT_DIR
    "\n", NAME_MAX
    );
  exit(0);
}

void ParseCommandLine(int argc, char** argv, struct rc_struct* rc)
{
  int opts_ok = 1;
  int i, j, nargs = 0;
  int nshortargs = 0; /* number of arguments needed for short options */
  char** argument;
  char* shortarg;

  if ((argument = (char**)malloc(argc * sizeof(char*))) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  if ((shortarg = (char*)malloc(argc * sizeof(char))) == NULL) {
    perror("defile: cannot allocate heap");
    exit(1);
  }

  for(i = 1; i < argc; ++i) {
    if (opts_ok && argv[i][0] == '-') { /* an option */
      if (argv[i][1] == '-') { /* a long option */
        if (argv[i][2] == '\0') /* -- last option */
          opts_ok = 0;
        else if (!strcmp(argv[i], "--help"))
          PrintUsage();
        else if (!strcmp(argv[i], "--version"))
          PrintVersion();
        else if (!strcmp(argv[i], "--curfile")) {
          if (!rc->write_curfile) {
            rc->write_curfile = 1;
            if ((rc->output_curfile = strdup(DEFAULT_CURFILE)) == NULL) {
              perror("defile: cannot allocate heap");
              exit(1);
            }
          }
        } else if (!strncmp(argv[i], "--curfile-name=", 15)) {
          rc->write_curfile = 1;
          free(rc->output_curfile);
          if ((rc->output_curfile = strdup(&argv[i][15])) == NULL) {
            perror("defile: cannot allocate heap");
            exit(1);
          }
        } else if (!strcmp(argv[i], "--force-framefile"))
          rc->framefile = 1;
        else if (!strncmp(argv[i], "--output-dirfile=", 17)) {
          free(rc->remount_dir);
          if ((rc->output_dirfile = strdup(&argv[i][17])) == NULL) {
            perror("defile: cannot allocate heap");
            exit(1);
          }
        } else if (!strcmp(argv[i], "--persistent"))
          rc->persist = 1;
        else if (!strcmp(argv[i], "--remounted-source")) {
          if (!rc->remount) {
            rc->remount = 1;
            if ((rc->remount_dir = strdup(REMOUNT_PATH)) == NULL) {
              perror("defile: cannot allocate heap");
              exit(1);
            }
          }
        } else if (!strncmp(argv[i], "--remounted-using=", 18)) {
          rc->remount = 1;
          free(rc->remount_dir);
          if ((rc->remount_dir = strdup(&argv[i][18])) == NULL) {
            perror("defile: cannot allocate heap");
            exit(1);
          }
        } else if (!strcmp(argv[i], "--resume"))
          rc->resume = 1;
        else if (!strncmp(argv[i], "--suffix-size=", 14)) {
          if (argv[i][14] >= '0' && argv[i][14] <= '9') {
            rc->sufflen = atoi(&argv[i][14]);
          } else {
            fprintf(stderr, "defile: suffix size `%s' is not a valid value\n"
                "Try `defile --help' for more information.\n", &argv[i][14]);
            exit(1);
          }
        } else {
          fprintf(stderr, "defile: unrecognised option `%s'\n"
              "Try `defile --help' for more information.\n", argv[i]);
          exit(1);
        }
      } else { /* a short option */
        for (j = 1; argv[i][j] != '\0'; ++j) {
          switch (argv[i][j]) {
            case 'C':
              rc->write_curfile = 1;
              if (nshortargs < argc)
                shortarg[nshortargs++] = 'C';
              break;
            case 'R':
              rc->resume = 1;
              break;
            case 'c':
              if (!rc->write_curfile) {
                rc->write_curfile = 1;
                if ((rc->output_curfile = strdup(DEFAULT_CURFILE)) == NULL) {
                  perror("defile: cannot allocate heap");
                  exit(1);
                }
              }
              break;
            case 'f':
              rc->framefile = 1;
              break;
            case 'o':
              if (nshortargs < argc)
                shortarg[nshortargs++] = 'o';
              break;
            case 'p':
              rc->persist = 1;
              break;
            case 'r':
              if (!rc->remount) {
                rc->remount = 1;
                if ((rc->remount_dir = strdup(REMOUNT_PATH)) == NULL) {
                  perror("defile: cannot allocate heap");
                  exit(1);
                }
              }
              break;
            case 's':
              if (nshortargs < argc)
                shortarg[nshortargs++] = 's';
              break;
            default:
              fprintf(stderr, "defile: invalid option -- %c\n"
                  "Try `defile --help' for more information.\n", argv[i][j]);
              exit(1);
          }
        }
      }
    } else { /* an argument */
      argument[nargs++] = argv[i];
    }
  }

  /* resolve short option arguments */
  for (i = 0; i < nshortargs; ++i) {
    if (i >= nargs) {
      fprintf(stderr, "defile: option requires an argument -- %c\n"
          "Try `defile --help' for more information.\n", shortarg[i]);
      exit(1);
    }

    switch(shortarg[i]) {
      case 'C':
        if (argument[i][0] != '\0') {
          free(rc->output_curfile);
          if ((rc->output_curfile = strdup(argument[i])) == NULL) {
            perror("defile: cannot allocate heap");
            exit(1);
          }
        } else {
          fprintf(stderr, "defile: curfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[i]);
          exit(1);
        }
        break;
      case 'o':
        if (argument[i][0] != '\0') {
          free(rc->output_dirfile);
          if ((rc->output_dirfile = strdup(argument[i])) == NULL) {
            perror("defile: cannot allocate heap");
            exit(1);
          }
        } else {
          fprintf(stderr,
              "defile: output dirfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[i]);
          exit(1);
        }
        break;
      case 's':
        if (argument[i][0] >= '0' && argument[i][0] <= '9') {
          rc->sufflen = atoi(argument[i]);
        } else {
          fprintf(stderr, "defile: suffix size `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[i]);
          exit(1);
        }
        break;
      default:
        fprintf(stderr, "defile: Unexpected trap in ParseCommandLine().  "
            "Bailing.\n");
        exit(1);
    }
  }

  if (nargs <= nshortargs) {
    fprintf(stderr, "defile: too few arguments\n"
        "Try `defile --help' for more information.\n");
    exit(1);
  }

  /* SOURCE */
  rc->source = argument[nshortargs];

  /* DESTINATION (if any) */
  if (nargs > nshortargs + 1) {
    rc->dest_dir = argument[nshortargs + 1];
    if (strlen(rc->dest_dir) > PATH_MAX) {
      fprintf(stderr, "defile: Destination path too long\n");
      exit(1);
    }
  } else
    rc->dest_dir = strdup(DEFAULT_DIR);

  /* Fix up output_dirfile, if present */
  if (rc->output_dirfile != NULL) {
    rc->output_dirfile = ResolveOutputDirfile(rc->output_dirfile, rc->dest_dir);
  }

  free(argument);
  free(shortarg);
}

int main (int argc, char** argv)
{
  struct timeval now;
  long int delta;

  pthread_t read_thread;
  pthread_t write_thread;

  /* fill rc struct from command line */
  ParseCommandLine(argc, argv, &rc);

  printf("Defile " VERSION " (C) 2004 D. V. Wiebe\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n");

  /* Get the name of the frame file. This function handles following the
   * curfile. Also fill the stat structure for the file, since we have
   * to stat the file anyways (This allocated rc.chunk for us.) */
  rc.chunk = GetFileName(rc.source);

  /* if rc.output_dirfile exists, we use that as the dirfile name, otherwise
   * we have to make one based on the input name */
  if (rc.output_dirfile != NULL)
    rc.dirfile = rc.output_dirfile;
  else
    /* this takes care of allocated rc.dirfile */
    rc.dirfile  = GetDirFile(rc.chunk, rc.dest_dir);

  /* Make the Channel Struct */
  MakeTxFrame();

  printf("Frame size: %i bytes\n", RX_FRAME_SIZE);

  /* Start */
  printf("Defiling `%s'\n    into `%s' ...\n\n", rc.chunk, rc.dirfile);

  /* Initialise things */
  ri.read = ri.wrote = ri.old_total = 0;
  gettimeofday(&rc.start, &rc.tz);

  /* Spawn reader and writer */
  pthread_create(&read_thread, NULL, (void*)&FrameFileReader, NULL);
  pthread_create(&write_thread, NULL, (void*)&DirFileWriter, NULL);

  /* Main status loop */
  do {
    gettimeofday(&now, &rc.tz);
    delta = (now.tv_sec - rc.start.tv_sec) * 1000000 - rc.start.tv_usec
      + now.tv_usec;
    printf("Read [%i of %i] Wrote [%i] Frame Rate %.3f kHz (%.1f sec)       \r",
        ri.read, ri.old_total + ri.chunk_total, ri.wrote, 1000. * ri.wrote /
        delta, delta / 1000000.);
    fflush(stdout);
    usleep(100000);
  } while (!ri.writer_done);
  printf("Read [%i of %i] Wrote [%i] Frame Rate %.3f kHz (%.3f sec)        \n",
      ri.read, ri.old_total + ri.chunk_total, ri.wrote, 1000. * ri.wrote /
      delta, delta / 1000000.);

  return 0;
}
