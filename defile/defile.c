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
#include <syslog.h>
#include <stdarg.h>
#include <error.h>
#include <signal.h>
#include <string.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "defile.h"
#include "blast.h"
#include "channels.h"

#define VERSION_MAJOR    "2"
#define VERSION_MINOR    "4"
#define VERSION_REVISION "0"
#define VERSION VERSION_MAJOR "." VERSION_MINOR "." VERSION_REVISION 

#define DEFAULT_CURFILE "/data/etc/defile.cur"
#define DEFAULT_DIR "/data/rawdir"
#define REMOUNT_PATH "../rawdir"

#define SUFF_MAX sizeof(chunkindex_t)
#define SUFF_DFLT 3

struct ri_struct ri;
struct rc_struct rc = {
  0, /* daemonise */
  0, /* framefile */
  0, /* gzip_output */
  0, /* persist */
  0, /* silent */
  0, /* remount */
  0, /* write_curfile */
  0, /* write_mode */
  SUFF_DFLT, /* sufflen */
  -1, /* resume_at */
  0, /* source_is_curfile */
  NULL, /* curfile_val */
  NULL, /* remount_dir */
  NULL, /* output_curfile */
  NULL, /* output_dirfile */
  NULL, /* source */
  NULL, /* dest_dir */
  NULL  /* spec_file */
};

sigset_t signals;

/* filters info messages out of syslog output */
void dputs(blog_t level, const char* string)
{
  if (level != info) 
    bputs_syslog(level, string);
}

char* ResolveOutputDirfile(char* dirfile, const char* parent)
{
  char parent_part[NAME_MAX];
  char dirfile_part[PATH_MAX];
  char path[PATH_MAX];

  /* is dirfile a relative path if so, we don't have to do anything */
  if (dirfile[0] != '/') {
    /* check string sizes */
    if (strlen(parent) + 1 + strlen(dirfile) >= PATH_MAX)
      bprintf(fatal, "output dirfile path is too long\n");

    strcpy(path, parent);
    if (path[strlen(path) - 1] != '/')
      strcat(path, "/");
    strcat(path, dirfile);
  } else
    strcpy(path, dirfile);

  /* realpath() will fail with a non-existant path, so strip off dirfile name */
  PathSplit_r(path, parent_part, dirfile_part);

  /* canonicalise the path */
  if (realpath(parent_part, dirfile) == NULL)
    berror(fatal, "cannot resolve output dirfile path `%s'", parent_part);

  /* add back dirfile name */
  if (dirfile[strlen(dirfile) - 1] != '/')
    strcat(dirfile, "/");
  strcat(dirfile, dirfile_part);

  return dirfile;
}

void Remount(const char* source, char* buffer)
{
  char element[PATH_MAX];
  char real_path[PATH_MAX];
  char path[PATH_MAX];

  /* is the remount_dir an absolute path? */
  if (rc.remount_dir[0] == '/') {
    strcpy(path, rc.remount_dir);
  } else {
    /* get the curfile's directory) */
    PathSplit_r(source, element, NULL);

    /* check string sizes */
    if (strlen(element) + 1 + strlen(rc.remount_dir) >= PATH_MAX)
      bprintf(fatal, "remounted path is too long\n");

    strcpy(path, element);
    if (path[strlen(path) - 1] != '/')
      strcat(path, "/");
    strcat(path, rc.remount_dir);
  }

  /* canonicalise the path */
  if (realpath(path, real_path) == NULL)
    berror(fatal, "cannot resolve remounted path `%s'", path);

  /* now get the indirect file's filename */
  PathSplit_r(buffer, NULL, element);

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

  /* allocate our buffer */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL)
    berror(fatal, "cannot allocate heap");

  /* first attempt to stat SOURCE to see if it is indeed a regular file */
  if (stat(source, &stat_buf))
    berror(fatal, "cannot stat `%s'", source);

  /* the stat worked.  Now is this a regular file? */
  if (!S_ISREG(stat_buf.st_mode))
    bprintf(fatal, "`%s' is not a regular file\n", source);

  /* attempt to open the file */
  if ((stream = fopen(source, "r")) == NULL)
    berror(fatal, "cannot open `%s'", source);

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
        } else
          berror(fatal, "error reading `%s'", source);
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
       * curfile so we can check for changes */
      if (rc.persist)
        if ((rc.curfile_val = strdup(buffer)) == NULL)
          berror(fatal, "cannot allocate heap");

      if (rc.remount)
        /* user indicated curfile filesystem has been remounted, so fix up the
         * source path */
        Remount(source, buffer);

      if (stat(buffer, &stat_buf))
        /* stat failed, complain and die */
        berror(fatal, "cannot stat `%s' pointed to by curfile", buffer);
      else
        /* the stat worked.  Now is this a regular file? */
        if (!S_ISREG(stat_buf.st_mode))
          /* buffer doesn't point to a regular file, assume SOURCE is a frame
           * file */
          bprintf(fatal, "`%s' is not a regular file\n", buffer);
    }
  }

  fclose(stream);

  return buffer;
}

/* given a destination path and a source filename, makes a dirfile name and
 * returns it in output */
char* MakeDirFile(char* output, const char* source, const char* directory)
{
  char bname[NAME_MAX];
  char* buffer;

  /* allocate our buffer */
  if ((buffer = (char*)malloc(FILENAME_LEN)) == NULL)
    berror(fatal, "cannot allocate heap");

  PathSplit_r(source, NULL, bname);
  StaticSourcePart(buffer, bname, NULL, rc.sufflen);

  strcpy(output, directory);
  if (output[strlen(output) - 1] != '/')
    strcat(output, "/");
  strcat(output, buffer);

  free(buffer);

  return output;
}

/* generates a dirfile name given the source and destination passed on the
 * command line */
void GetDirFile(char* buffer, const char* source, char* parent)
{
  struct stat stat_buf;

  /* Step 1: stat parent to make sure it exists */
  if (stat(parent, &stat_buf))
    /* stat failed -- parent doesn't exist; complain and exit */
    berror(fatal, "cannot stat `%s'", parent);

  /* parent exists.  Is it a directory? */
  if (!S_ISDIR(stat_buf.st_mode))
    /* not a directory, complain and exit */
    bprintf(fatal, "`%s' is not a directory", parent);

  /* parent is indeed a directory; make the dirfile name */
  MakeDirFile(buffer, source, parent);
}

/* Figures out the name of the channel specification file, and then tries to
 * open and read it. */
void ReconstructChannelLists(void)
{
  struct stat stat_buf;
  char buffer[200];
  char* ptr = buffer;
  FILE* stream;

  /* if rc.spec_file exists, the user has specified a spec file name, use it */
  if (rc.spec_file != NULL) {
    /* check for buffer overrun */
    if (strlen(rc.spec_file) >= 200)
      bprintf(fatal, "specification file path too long\n");
    strcpy(buffer, rc.spec_file);
  } else {
    /* if the chunk is 923488378.x000, the spec file will be 923488378.x.spec */
    strcpy(buffer, rc.chunk);
    while (*ptr != '.' && *ptr != '\0')
      ++ptr;
    if (*ptr != '\0') {
      ++ptr;
      if (*ptr != '\0')
        ++ptr;
    }

    /* check for buffer overrun */
    if (ptr - buffer > 190)
      bprintf(fatal, "specification file path too long\n");
    strcpy(ptr, ".spec");
  }

  /* first attempt to stat spec file to see if it is indeed a regular file */
  if (stat(buffer, &stat_buf))
    berror(fatal, "cannot stat spec file `%s'", buffer);

  /* the stat worked.  Now is this a regular file? */
  if (!S_ISREG(stat_buf.st_mode))
    bprintf(fatal, "spec file `%s' is not a regular file\n", buffer);

  /* attempt to open the file */
  if ((stream = fopen(buffer, "r")) == NULL)
    berror(fatal, "cannot open spec file `%s'", buffer);

  ReadSpecificationFile(stream);

  /* Make the Channel Struct */
  MakeAddressLookups();

  if (!rc.silent)
    printf("Frame size: %i bytes\n", DiskFrameSize);
}

void PrintVersion(void)
{
  printf("defile " VERSION "  (C) 2004 D. V. Wiebe\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n"
      "This program comes with NO WARRANTY, "
      "not even for MERCHANTABILITY or FITNESS\n"
      "FOR A PARTICULAR PURPOSE. You may "
      "redistribute it under the terms of the GNU\n"
      "General Public License; see the file named COPYING for details.\n"
      "Written by D.V. Wiebe.\n"
      );
  exit(0);
}

void PrintUsage(void)
{
  printf("Usage: defile [OPTION]... SOURCE [DIRECTORY]"
      "\nConvert the BLAST-type framefile SOURCE into dirfile under DIRECTORY"
      "\n"
      "\nSOURCE may be either a .cur file or a framefile to start with."
      "\nDefault DIRECTORY to use if no DIRECTORY is given:"
      "\n\t" DEFAULT_DIR
      "\n"
      "\nArguments to long options are required for short arguments as well."
      "\n  -C --curfile-name=NAME same as `-curfile' but use NAME as the name "
      "of the"
      "\n                          file instead of `" DEFAULT_CURFILE "'."
      "\n  -F --framefile        assume SOURCE is a framefile."
      "\n  -R --resume           resume an interrupted defiling.  Incompatible "
      "with"
      "\n                          `--gzip'"
      "\n  -S --spec-file=NAME   use NAME as the specification file."
      "\n  -c --curfile          write a curfile called `" DEFAULT_CURFILE "'."
      "\n  -d --daemonise        fork to background and daemonise on startup.  "
      "Implies"
      "\n                          `--persistent' and `--quiet'."
      "\n  -f --force            overwrite destination."
      "\n  -o --output-dirfile=NAME use name as the name of the dirfile. Name "
    "can either"
    "\n                          be an absolute path, or a path realtive to "
    "DIRECTORY."
    "\n  -p --persistent       do not exit, but monitor SOURCE for changes "
    "and keep"
    "\n                          writing to dirfile."
    "\n  -q --quiet            supress normal output, including the read and "
    "write"
    "\n                          counters."
    "\n  -r --remounted-source when SOURCE is a curfile, assume that the "
    "framefile is"
    "\n                          located in the directory `" REMOUNT_PATH"' "
    "relative"
    "\n                          to the curfile's location.  This option has "
    "no"
    "\n                          effect if SOURCE is not a curfile."
    "\n     --remounted-using=DIR same as `--remounted-source' except use "
    "DIR as the"
    "\n                          path instead of the default `" REMOUNT_PATH
    "'."
    "\n  -s --suffix-size=SIZE framefile suffix is no more than SIZE "
    "characters large."
    "\n                          SIZE should be an integer between 0 and %i."
    "\n                          Default: %i"
    "\n  -z --gzip             gzip compress the output dirfile.  "
    "Incompatible with"
    "\n                          `--resume'"
    "\n  --help                display this help and exit"
    "\n  --version             display version information and exit"
    "\n  --                    last option; all following parameters are "
    "arguments."
    "\n"
    "\nSummary of defaults:"
    "\n  Default curfile name :  " DEFAULT_CURFILE
    "\n  Default remount path :  " REMOUNT_PATH
    "\n  Default output path  :  " DEFAULT_DIR
    "\n", SUFF_MAX, SUFF_DFLT
    );
  exit(0);
}

void ParseCommandLine(int argc, char** argv, struct rc_struct* rc)
{
  int opts_ok = 1;
  int i, j, nargs = 0;
  int nshortargs = 0; /* number of arguments needed for short options */
  struct argument_s {
    char* value;
    int position;
    int used;
  } *argument;
  struct shortarg_s {
    char option;
    int position;
  } *shortarg;

  if ((argument = (struct argument_s*)malloc(argc * sizeof(struct argument_s)))
      == NULL)
    berror(fatal, "cannot allocate heap");

  memset(argument, 0, argc * sizeof(struct argument_s));

  if ((shortarg = (struct shortarg_s*)malloc(argc * sizeof(struct shortarg_s)))
      == NULL)
    berror(fatal, "cannot allocate heap");

  for (i = 1; i < argc; ++i) {
    if (opts_ok && argv[i][0] == '-') { /* an option */
      if (argv[i][1] == '-') { /* a long option */
        if (argv[i][2] == '\0') /* -- (last option flag) */
          opts_ok = 0;
        else if (!strcmp(argv[i], "--help"))
          PrintUsage();
        else if (!strcmp(argv[i], "--version"))
          PrintVersion();
        else if (!strcmp(argv[i], "--curfile")) {
          if (!rc->write_curfile) {
            rc->write_curfile = 1;
            if ((rc->output_curfile = strdup(DEFAULT_CURFILE)) == NULL)
              berror(fatal, "cannot allocate heap");
          }
        } else if (!strncmp(argv[i], "--curfile-name=", 15)) {
          rc->write_curfile = 1;
          free(rc->output_curfile);
          if ((rc->output_curfile = strdup(&argv[i][15])) == NULL)
            berror(fatal, "cannot allocate heap");
        } else if (!strcmp(argv[i], "--daemonise"))
          rc->daemonise = rc->persist = rc->silent = 1;
        else if (!strcmp(argv[i], "--force"))
          rc->write_mode = 1;
        else if (!strcmp(argv[i], "--framefile"))
          rc->framefile = 1;
        else if (!strcmp(argv[i], "--gzip"))
          rc->gzip_output = 1;
        else if (!strncmp(argv[i], "--output-dirfile=", 17)) {
          free(rc->remount_dir);
          if ((rc->output_dirfile = strdup(&argv[i][17])) == NULL)
            berror(fatal, "cannot allocate heap");
        } else if (!strcmp(argv[i], "--persistent"))
          rc->persist = 1;
        else if (!strcmp(argv[i], "--quiet"))
          rc->silent = 1;
        else if (!strcmp(argv[i], "--remounted-source")) {
          if (!rc->remount) {
            rc->remount = 1;
            if ((rc->remount_dir = strdup(REMOUNT_PATH)) == NULL)
              berror(fatal, "cannot allocate heap");
          }
        } else if (!strncmp(argv[i], "--remounted-using=", 18)) {
          rc->remount = 1;
          free(rc->remount_dir);
          if ((rc->remount_dir = strdup(&argv[i][18])) == NULL)
            berror(fatal, "cannot allocate heap");
        } else if (!strcmp(argv[i], "--resume")) 
          rc->write_mode = 2;
        else if (!strncmp(argv[i], "--spec-file=", 12)) {
          free(rc->spec_file);
          if ((rc->spec_file = strdup(&argv[i][12])) == NULL)
            berror(fatal, "cannot allocate heap");
        } else if (!strncmp(argv[i], "--suffix-size=", 14)) {
          if (argv[i][14] >= '0' && argv[i][14] <= '9') {
            if ((rc->sufflen = atoi(&argv[i][14])) > SUFF_MAX)
              bprintf(fatal, "suffix size `%s' is not a valid value\n"
                  "Try `defile --help' for more information.\n", &argv[i][14]);
          } else
            bprintf(fatal, "suffix size `%s' is not a valid value\n"
                "Try `defile --help' for more information.\n", &argv[i][14]);
        } else
          bprintf(fatal, "unrecognised option `%s'\n"
              "Try `defile --help' for more information.\n", argv[i]);
      } else /* a short option */
        for (j = 1; argv[i][j] != '\0'; ++j)
          switch (argv[i][j]) {
            case 'C':
              rc->write_curfile = 1;
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 'C';
              }
              break;
            case 'F':
              rc->framefile = 1;
              break;
            case 'R':
              rc->write_mode = 2;
              break;
            case 'S':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 'S';
              }
              break;
            case 'c':
              if (!rc->write_curfile) {
                rc->write_curfile = 1;
                if ((rc->output_curfile = strdup(DEFAULT_CURFILE)) == NULL)
                  berror(fatal, "cannot allocate heap");
              }
              break;
            case 'd':
              rc->daemonise = rc->persist = rc->silent = 1;
              break;
            case 'f':
              rc->write_mode = 1;
              break;
            case 'o':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 'o';
              }
              break;
            case 'p':
              rc->persist = 1;
              break;
            case 'q':
              rc->silent = 1;
              break;
            case 'r':
              if (!rc->remount) {
                rc->remount = 1;
                if ((rc->remount_dir = strdup(REMOUNT_PATH)) == NULL)
                  berror(fatal, "cannot allocate heap");
              }
              break;
            case 's':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 's';
              }
              break;
            case 'z':
              rc->gzip_output = 1;
              break;
            default:
              bprintf(fatal, "invalid option -- %c\n"
                  "Try `defile --help' for more information.\n", argv[i][j]);
          }
    } else { /* an argument */
      argument[nargs].value = argv[i];
      argument[nargs++].position = i;
    }
  } 

  if (rc->gzip_output == 1 && rc->write_mode == 2)
    bprintf(fatal, "cannot resume gzipped dirfiles\n"
        "Try `defile --help' for more information.\n");

  j = -1;
  /* resolve short option arguments */
  for (i = 0; i < nshortargs; ++i) {
    if (i >= nargs)
      bprintf(fatal, "option requires an argument -- %c\n"
          "Try `defile --help' for more information.\n", shortarg[i].option);

    /* find the next (unused) argument after this short option */
    for (j++; j < nargs && argument[j].position <= shortarg[i].position; ++j);

    /* we didn't find an argument, complain and die */
    if (j == nargs)
      bprintf(fatal, "option requires an argument -- %c\n"
          "Try `defile --help' for more information.\n", shortarg[i].option);

    argument[j].used = 1;
    switch(shortarg[i].option) {
      case 'C':
        if (argument[j].value[0] != '\0') {
          free(rc->output_curfile);
          if ((rc->output_curfile = strdup(argument[j].value)) == NULL)
            berror(fatal, "cannot allocate heap");
        } else
          bprintf(fatal, "curfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 'S':
        if (argument[j].value[0] != '\0') {
          free(rc->spec_file);
          if ((rc->spec_file = strdup(argument[j].value)) == NULL)
            berror(fatal, "cannot allocate heap");
        } else
          bprintf(fatal, "specification filename `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 'o':
        if (argument[j].value[0] != '\0') {
          free(rc->output_dirfile);
          if ((rc->output_dirfile = strdup(argument[j].value)) == NULL)
            berror(fatal, "cannot allocate heap");
        } else
          bprintf(fatal, "output dirfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 's':
        if (argument[j].value[0] >= '0' && argument[j].value[0] <= '9') {
          if ((rc->sufflen = atoi(argument[j].value)) > SUFF_MAX)
            bprintf(fatal, "suffix size `%s' is not a valid value\n"
                "Try `defile --help' for more information.\n", &argv[i][14]);
        } else
          bprintf(fatal, "suffix size `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      default:
        bprintf(fatal, "Unexpected trap in ParseCommandLine().  Bailing.\n");
    }
  }

  if (nargs <= nshortargs)
    bprintf(fatal, "too few arguments\n"
        "Try `defile --help' for more information.\n");

  /* first unused argument is SOURCE */
  for (j = 0; j < nargs && argument[j].used; ++j);

  /* SOURCE */
  rc->source = argument[j].value;

  /* next unused argument is DESTINATION */
  for (j++; j < nargs && argument[j].used; ++j);

  /* DIRECTORY (if any) */
  if (j < nargs) {
    rc->dest_dir = argument[j].value;
    if (strlen(rc->dest_dir) > PATH_MAX)
      bprintf(fatal, "Destination path too long\n");
  } else
    rc->dest_dir = strdup(DEFAULT_DIR);

  /* Fix up output_dirfile, if present */
  if (rc->output_dirfile != NULL)
    rc->output_dirfile = ResolveOutputDirfile(rc->output_dirfile, rc->dest_dir);

  free(argument);
  free(shortarg);
}

int main (int argc, char** argv)
{
  struct timeval now;
  long long int delta;

  pthread_t read_thread;
  pthread_t write_thread;

  /* set up our outputs */
  blog_use_stdio();

  /* fill rc struct from command line */
  ParseCommandLine(argc, argv, &rc);

  if (rc.daemonise) {
    int pid;
    FILE* stream;

    openlog("defile", LOG_PID, LOG_DAEMON);

    /* Fork to background */
    if ((pid = fork()) != 0) {
      if (pid == -1)
        berror(fatal, "unable to fork to background");

      if ((stream = fopen("/var/run/decomd.pid", "w")) == NULL) 
        berror(err, "unable to write PID to disk");
      else {
        fprintf(stream, "%i\n", pid);
        fflush(stream);
        fclose(stream);
      }
      closelog();
      printf("PID = %i\n", pid);
      exit(0);
    }

    /* switch output to syslog */
    blog_use_func(dputs);

    /* Daemonise */
    chdir("/");
    freopen("/dev/null", "r", stdin);
    freopen("/dev/null", "w", stdout);
    freopen("/dev/null", "w", stderr);
    setsid();
  }

  bprintf(info, "defile " VERSION " (C) 2004 D. V. Wiebe\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n");

  /* Get the name of the frame file. This function handles following the
   * curfile. Also fill the stat structure for the file, since we have
   * to stat the file anyways (This allocated rc.chunk for us.) */
  rc.chunk = GetFileName(rc.source);

  /* if rc.output_dirfile exists, we use that as the dirfile name, otherwise
   * we have to make one based on the input name */
  if ((rc.dirfile = malloc(FILENAME_LEN)) == NULL)
    berror(fatal, "cannot allocate heap");

  if (rc.output_dirfile != NULL)
    strncpy(rc.dirfile, rc.output_dirfile, FILENAME_LEN);
  else
    GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir);

  /* check the length of the output path */
  if (strlen(rc.dirfile) > PATH_MAX - FIELD_MAX - 1)
    bprintf(fatal, "destination dirfile `%s' too long\n", rc.dirfile);

  /* Attempt to Open the Specification file and read the channel lists */
  ReconstructChannelLists();

  /* Start */
  bprintf(info, "Defiling `%s'\n    into `%s' ...\n\n", rc.chunk, rc.dirfile);

  /* Initialise things */
  ri.read = ri.wrote = ri.old_total = 0;
  ri.tty = 0;
  delta = 1;
  gettimeofday(&rc.start, &rc.tz);
  PreInitialiseDirFile();
  InitialiseDirFile(1);

  /* set up signal masks */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  /* block signals */
  pthread_sigmask(SIG_BLOCK, &signals, NULL);

  /* Spawn reader and writer */
  pthread_create(&read_thread, NULL, (void*)&FrameFileReader, NULL);
  pthread_create(&write_thread, NULL, (void*)&DirFileWriter, NULL);

  /* Main status loop */
  if (!rc.silent)
    do {
      if (!ri.tty) {
        gettimeofday(&now, &rc.tz);
        delta = (now.tv_sec - rc.start.tv_sec) * 1000000LL - rc.start.tv_usec
          + now.tv_usec;
        printf("R:[%i of %i] W:[%i] %.3f kHz\r", ri.read, ri.old_total
            + ri.chunk_total, ri.wrote, 1000. * ri.wrote / delta);
        fflush(stdout);
      }
      usleep(100000);
    } while (!ri.writer_done);
  pthread_join(read_thread, NULL);
  pthread_join(write_thread, NULL);
  bprintf(info, "R:[%i of %i] W:[%i] %.3f kHz\n", ri.read, ri.old_total
      + ri.chunk_total, ri.wrote, 1000. * ri.wrote / delta);
  return 0;
}
