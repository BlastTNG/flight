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

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

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

#ifndef VERSION
#  define VERSION_MAJOR    "2"
#  define VERSION_MINOR    "6"
#  define VERSION VERSION_MAJOR "." VERSION_MINOR ".x"
#endif

#ifndef DEFAULT_CURFILE
#  define DEFAULT_CURFILE "/data/etc/defile.cur"
#endif

#ifndef DEFAULT_DIR
#  define DEFAULT_DIR "/data/rawdir"
#endif

#ifndef REMOUNT_PATH
#  define REMOUNT_PATH "../rawdir"
#endif

#define SUFF_MAX sizeof(chunkindex_t)
#define SUFF_DFLT 3

/* options */
struct {
  union {
    char* as_string;
    int   as_int;
  } value;
  char type;
  const char name[48];
} options[] = {
  {{NULL}, 'i', "CompressedOutput"},
  {{NULL}, 'i', "Daemonise"},
  {{NULL}, 's', "InputCurFileName"},
  {{NULL}, 's', "OutputCurFileName"},
  {{NULL}, 's', "OutputDirectory"},
  {{NULL}, 'i', "Persistent"},
  {{NULL}, 's', "PidFile"},
  {{NULL}, 's', "QuendiHost"},
  {{NULL}, 'i', "Quiet"},
  {{NULL}, 's', "RemountPath"},
  {{NULL}, 'i', "RemountedSource"},
  {{NULL}, 'i', "ResumeMode"},
  {{NULL}, 's', "SpecFile"},
  {{NULL}, 'i', "SuffixLength"},
  {{NULL}, 'i', "WriteCurfile"},
  {{NULL}, '\0', ""}
};

/* state structs */
struct ri_struct ri;
struct rc_struct rc = {
  0, /* daemonise */
  0, /* force_stdio */
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

/* filters info messages out of output when appropriate */
void dputs(buos_t level, const char* string)
{
  if (level == mem)
    return;
  
  if (rc.daemonise && !rc.force_stdio && level != info)
    bputs_syslog(level, string);
  else if (level != info || !rc.silent)
    bputs_stdio(level, string);
}

void ReadConfig(FILE* stream)
{
  char buffer[1024];
  char *option, *value;
  int i, found;

  while (fgets(buffer, sizeof(buffer), stream)) {
    /* remove comments */
    if ((option = strchr(buffer, '#')) != NULL)
      *option = '\0';

    /* strip newline */
    if ((option = strchr(buffer, '\n')) != NULL)
      *option = '\0';

    /* strip leading whitespace */
    option = buffer + strspn(buffer, " \t");
    if ((value = strchr(option, ' ')) != NULL) {
      *(value++) = '\0';
      value += strspn(value, " \t");
    }
    printf("o: %s\nv: %s\n", option, value);

    found = 0;
    for (i = 0; options[i].type != '\0'; ++i)
      if (strcasecmp(option, options[i].name) == 0) {
        found = 1;
        switch (options[i].type) {
          case 's':
            options[i].value.as_string = bstrdup(fatal, value);
            break;
          case 'i':
            options[i].value.as_int = atoi(value);
            break;
          default:
            printf("Unknown option type\n");
            exit(1);
        }
        break;
      }

    if (!found)
      bprintf(warning, "Unknown option `%s'", option);
  }
}

void LoadDefaultConfig(void)
{
  int i;

  for (i = 0; options[i].type; ++i)
    if (options[i].value.as_string == NULL)
      switch (i) {
        case CFG_PID_FILE:
          options[i].value.as_string = bstrdup(fatal, PID_FILE);
          break;
        case CFG_CUR_FILE:
          options[i].value.as_string
            = bstrdup(fatal, "/mnt/decom/etc/decom.cur");
          break;
        case CFG_SUFFIX_LENGTH:
          options[i].value.as_int = 3;
        default:
          bprintf(warning, "No default value for option `%s'",
              options[i].name);
      }
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
  buffer = (char*)balloc(fatal, FILENAME_LEN);

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
        rc.curfile_val = bstrdup(fatal, buffer);

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
  buffer = (char*)balloc(fatal, FILENAME_LEN);

  PathSplit_r(source, NULL, bname);
  StaticSourcePart(buffer, bname, NULL, rc.sufflen);

  strcpy(output, directory);
  if (output[strlen(output) - 1] != '/')
    strcat(output, "/");
  strcat(output, buffer);

  bfree(fatal, buffer);

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
      "\n  -R --resume           resume an interrupted defiling."
#ifdef HAVE_LIBZ
      "  Incompatible with"
      "\n                          `--gzip'"
#endif
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
#ifdef HAVE_LIBZ
    "\n  -z --gzip             gzip compress the output dirfile.  Incompatible "
    "with"
    "\n                          `--resume'"
#endif
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

  argument = (struct argument_s*)balloc(fatal, argc
      * sizeof(struct argument_s));

  memset(argument, 0, argc * sizeof(struct argument_s));

  shortarg = (struct shortarg_s*)balloc(fatal, argc
      * sizeof(struct shortarg_s));

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
            rc->output_curfile = bstrdup(fatal, DEFAULT_CURFILE);
          }
        } else if (!strncmp(argv[i], "--curfile-name=", 15)) {
          rc->write_curfile = 1;
          bfree(fatal, rc->output_curfile);
          rc->output_curfile = bstrdup(fatal, &argv[i][15]);
        } else if (!strcmp(argv[i], "--daemonise"))
          rc->daemonise = rc->persist = rc->silent = rc->force_stdio = 1;
        else if (!strcmp(argv[i], "--force"))
          rc->write_mode = 1;
        else if (!strcmp(argv[i], "--framefile"))
          rc->framefile = 1;
#ifdef HAVE_LIBZ
        else if (!strcmp(argv[i], "--gzip"))
          rc->gzip_output = 1;
#endif
        else if (!strncmp(argv[i], "--output-dirfile=", 17)) {
          bfree(fatal, rc->remount_dir);
          rc->output_dirfile = bstrdup(fatal, &argv[i][17]);
        } else if (!strcmp(argv[i], "--persistent"))
          rc->persist = 1;
        else if (!strcmp(argv[i], "--quiet"))
          rc->silent = 1;
        else if (!strcmp(argv[i], "--remounted-source")) {
          if (!rc->remount) {
            rc->remount = 1;
            rc->remount_dir = bstrdup(fatal, REMOUNT_PATH);
          }
        } else if (!strncmp(argv[i], "--remounted-using=", 18)) {
          rc->remount = 1;
          bfree(fatal, rc->remount_dir);
          rc->remount_dir = bstrdup(fatal, &argv[i][18]);
        } else if (!strcmp(argv[i], "--resume")) 
          rc->write_mode = 2;
        else if (!strncmp(argv[i], "--spec-file=", 12)) {
          bfree(fatal, rc->spec_file);
          rc->spec_file = bstrdup(fatal, &argv[i][12]);
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
                rc->output_curfile = bstrdup(fatal, DEFAULT_CURFILE);
              }
              break;
            case 'd':
              rc->daemonise = rc->persist = rc->silent = rc->force_stdio = 1;
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
                rc->remount_dir = bstrdup(fatal, REMOUNT_PATH);
              }
              break;
            case 's':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 's';
              }
              break;
#ifdef HAVE_LIBZ
            case 'z':
              rc->gzip_output = 1;
              break;
#endif
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
          bfree(fatal, rc->output_curfile);
          rc->output_curfile = bstrdup(fatal, argument[j].value);
        } else
          bprintf(fatal, "curfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 'S':
        if (argument[j].value[0] != '\0') {
          bfree(fatal, rc->spec_file);
          rc->spec_file = bstrdup(fatal, argument[j].value);
        } else
          bprintf(fatal, "specification filename `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 'o':
        if (argument[j].value[0] != '\0') {
          bfree(fatal, rc->output_dirfile);
          rc->output_dirfile = bstrdup(fatal, argument[j].value);
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
    rc->dest_dir = bstrdup(fatal, DEFAULT_DIR);

  /* Fix up output_dirfile, if present */
  if (rc->output_dirfile != NULL)
    rc->output_dirfile = ResolveOutputDirfile(rc->output_dirfile, rc->dest_dir);

  bfree(fatal, argument);
  bfree(fatal, shortarg);
}

int main (int argc, char** argv)
{
  struct timeval now;
  long long int delta;
  float freq = 0;

  pthread_t read_thread;
  pthread_t write_thread;

  /* set up our outputs */
  buos_use_func(dputs);

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
    rc.force_stdio = 0;

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
   * curfile. (This allocates rc.chunk for us.) */
  rc.chunk = GetFileName(rc.source);

  /* if rc.output_dirfile exists, we use that as the dirfile name, otherwise
   * we have to make one based on the input name */
  rc.dirfile = balloc(fatal, FILENAME_LEN);

  if (rc.output_dirfile != NULL)
    strncpy(rc.dirfile, rc.output_dirfile, FILENAME_LEN);
  else
    GetDirFile(rc.dirfile, rc.chunk, rc.dest_dir);

  /* check the length of the output path */
  if (strlen(rc.dirfile) > PATH_MAX - FIELD_MAX - 1)
    bprintf(fatal, "destination dirfile `%s' too long\n", rc.dirfile);

  /* Attempt to open the Specification file and read the channel lists */
  ReconstructChannelLists(rc.chunk, rc.spec_file);
  bprintf(info, "Frame size: %i bytes\n", DiskFrameSize);

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

  /* Main status loop -- if we're in silent mode we skip this entirely and
   * just wait for the read and write threads to exit */
  if (!rc.silent)
    do {
      if (!ri.tty) {
        gettimeofday(&now, &rc.tz);
        delta = (now.tv_sec - rc.start.tv_sec) * 1000000LL - rc.start.tv_usec
          + now.tv_usec;
        freq = 1000. * ri.wrote / delta;
        printf("R:[%i of %i] W:[%i] %.*f kHz\r", ri.read, ri.old_total
            + ri.chunk_total, ri.wrote, (freq > 100) ? 1 : (freq > 10) ? 2 : 3,
            freq);
        fflush(stdout);
      }
      usleep(100000);
    } while (!ri.writer_done);
  pthread_join(read_thread, NULL);
  pthread_join(write_thread, NULL);
  if (!rc.silent)
    bprintf(info, "R:[%i of %i] W:[%i] %.*f kHz", ri.read, ri.old_total +
        ri.chunk_total, ri.wrote, (freq > 100) ? 1 : (freq > 10) ? 2 : 3, freq);
  return 0;
}
