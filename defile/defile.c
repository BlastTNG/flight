/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This file is copyright (C) 2004-2005, 2013 D. V. Wiebe
 * Also (C) 2005-2010 Matthew Truch.
 * And I'm sure many others....
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

/* we have to load the config before we load framefile.h */
#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <stdlib.h>     /* ANSI C std library (atoi, exit, realpath) */
#include <pthread.h>    /* POSIX threads (pthread_create, pthread_join) */
#include <signal.h>     /* ANSI C signals (SIG(FOO), sigemptyset, sigaddset) */
#include <string.h>     /* ANSI C strings (strcat, memcpy, &c.)  */
#include <syslog.h>     /* BSD system logger (openlog, syslog, closelog) */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <unistd.h>     /* UNIX std library (fork, chdir, setsid, usleep &c.) */
#include <wordexp.h>    /* POSIX-shell-like word expansion (wordexp) */

#include "defile.h"
#include "blast.h"
#include "channels.h"
#include "quenya.h"

#ifndef VERSION
#  define VERSION "5.x"
#endif

#ifdef SVNREV
#  define FULL_VERSION VERSION "-svn" SVNREV
#else
#  define FULL_VERSION VERSION
#endif

#define SUFF_MAX ((int)(2 * sizeof(chunkindex_t)))

#define TC  (120.)  /* characteristic time (in seconds) */
#define FR  (5.)    /* assumed frame rate (in Hertz) for display purposes only*/

/* defaults */
#define CONFIG_FILE ETC_DIR "/defile.conf"
#define CUR_FILE "/data/etc/defile"
#define OUTPUT_DIR "/data/rawdir"
#define PID_FILE LOCALSTATEDIR "/run/defile.pid"
#define REMOUNT_PATH "../rawdir"

/* options */
enum {CFG_AutoReconnect, CFG_CompressedOutput, CFG_Daemonise, CFG_ExtraFormat,
  CFG_FlakeySource, CFG_InputSource, CFG_OutputCurFileName, CFG_OutputDirectory,
  CFG_OutputDirFile, CFG_Persistent, CFG_PidFile, CFG_Quiet,
  CFG_RemoteInputSource, CFG_RemountPath, CFG_RemountedSource, CFG_ResumeMode,
  CFG_SpecFile, CFG_SuffixLength, CFG_WriteCurFile};

struct {
  union {
    char* as_string;
    int   as_int;
  } value;
  char type;
  const char name[48];
} options[] = {
  {{NULL}, 'b', "AutoReconnect"},
  {{NULL}, 'b', "CompressedOutput"},
  {{NULL}, 'b', "Daemonise"},
  {{NULL}, 'b', "ExtraFormat"},
  {{NULL}, 'b', "FlakeySource"},
  {{NULL}, 's', "InputSource"},
  {{NULL}, 's', "OutputCurFileName"},
  {{NULL}, 's', "OutputDirectory"},
  {{NULL}, 's', "OutputDirFile"},
  {{NULL}, 'b', "Persistent"},
  {{NULL}, 's', "PidFile"},
  {{NULL}, 'b', "Quiet"},
  {{NULL}, 'b', "RemoteInputSource"},
  {{NULL}, 's', "RemountPath"},
  {{NULL}, 'b', "RemountedSource"},
  {{NULL}, 'b', "ResumeMode"},
  {{NULL}, 's', "SpecFile"},
  {{NULL}, 'i', "SuffixLength"},
  {{NULL}, 'b', "WriteCurFile"},
  {{NULL}, '\0', ""}
};

/* state structs */
struct ri_struct ri;
struct rc_struct rc;

sigset_t signals;

/* filters info messages out of output when appropriate */
void dputs(buos_t level, const char* string)
{
  if (rc.daemonise && !rc.force_stdio && level != info)
    bputs_syslog(level, string);
  else if (level != info || !rc.silent)
    bputs_stdio(level, string);

  if (level == fatal || level == tfatal) {
    /* raise sigterm and wait for the reader to stop */
    raise(SIGTERM);
    for (;;)
      sleep(1);
  }
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

    /* skip empty lines */
    if (!*buffer)
      continue;

    /* strip leading whitespace */
    option = buffer + strspn(buffer, " \t");
    if ((value = strchr(option, ' ')) != NULL) {
      *(value++) = '\0';
      value += strspn(value, " \t");
    }

    found = 0;
    for (i = 0; options[i].type != '\0'; ++i)
      if (strcasecmp(option, options[i].name) == 0) {
        found = 1;
        switch (options[i].type) {
          case 's':
            options[i].value.as_string = bstrdup(fatal, value);
            break;
          case 'b':
            options[i].value.as_int = 1;
            break;
          case 'i':
            options[i].value.as_int = atoi(value);
            break;
          default:
            printf("Unknown option type: %c for %s\n", options[i].type,
                options[i].name);
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
        case CFG_AutoReconnect:
        case CFG_CompressedOutput:
        case CFG_FlakeySource:
        case CFG_Daemonise:
        case CFG_Persistent:
        case CFG_Quiet:
        case CFG_RemoteInputSource:
        case CFG_RemountedSource:
        case CFG_ResumeMode:
        case CFG_WriteCurFile:
        case CFG_ExtraFormat:
          options[i].value.as_int = 0;
        case CFG_InputSource: /* these are null by default */
        case CFG_OutputDirFile:
        case CFG_SpecFile:
          break;
        case CFG_OutputCurFileName:
          options[i].value.as_string = bstrdup(fatal, CUR_FILE);
          break;
        case CFG_OutputDirectory:
          options[i].value.as_string = bstrdup(fatal, OUTPUT_DIR);
          break;
        case CFG_PidFile:
          options[i].value.as_string = bstrdup(fatal, PID_FILE);
          break;
        case CFG_RemountPath:
          options[i].value.as_string = bstrdup(fatal, REMOUNT_PATH);
          break;
        case CFG_SuffixLength:
          options[i].value.as_int = SUFF_MAX;
          break;
        default:
          bprintf(warning, "No default value for option `%s'",
              options[i].name);
      }
}

struct rc_struct InitRcStruct()
{ 
  struct rc_struct rc = {
    .auto_reconnect    = options[CFG_AutoReconnect].value.as_int,
    .curfile_val       = NULL,
    .daemonise         = options[CFG_Daemonise].value.as_int,
    .dest_dir          = options[CFG_OutputDirectory].value.as_string,
    .extra_format      = options[CFG_ExtraFormat].value.as_int,
    .flakey_source     = options[CFG_FlakeySource].value.as_int,
    .force_quenya      = 0,
    .force_stdio       = options[CFG_Daemonise].value.as_int,
    .framefile         = 0,
#ifdef HAVE_LIBZ
    .gzip_output       = options[CFG_CompressedOutput].value.as_int,
#else
    .gzip_output       = 0,
#endif
    .output_curfile    = options[CFG_OutputCurFileName].value.as_string,
    .output_dirfile    = options[CFG_OutputDirFile].value.as_string,
    .persist           = (options[CFG_Daemonise].value.as_int
        || options[CFG_Persistent].value.as_int),
    .quenya            = options[CFG_RemoteInputSource].value.as_int,
    .remount           = options[CFG_RemountedSource].value.as_int,
    .remount_dir       = options[CFG_RemountPath].value.as_string,
    .resume_at         = 0,
    .silent            = (options[CFG_Daemonise].value.as_int
        || options[CFG_Quiet].value.as_int),
    .source            = NULL,
    .source_is_curfile = 0,
    .spec_file         = options[CFG_SpecFile].value.as_string,
    .sufflen           = options[CFG_SuffixLength].value.as_int,
    .write_curfile     = options[CFG_WriteCurFile].value.as_int,
    .write_mode        = options[CFG_ResumeMode].value.as_int * 2,
  };

  return rc;
}

char* ResolveOutputDirfile(char* dirfile, const char* parent)
{
  char parent_part[NAME_MAX];
  char dirfile_part[FR_PATH_MAX];
  char path[FR_PATH_MAX];
  wordexp_t expansion;

  /* is dirfile a relative path? if so, we don't have to do anything */
  if (dirfile[0] != '/' && dirfile[0] != '~') {
    /* check string sizes */
    if (strlen(parent) + 1 + strlen(dirfile) >= FR_PATH_MAX)
      bprintf(fatal, "output dirfile path is too long\n");

    strcpy(path, parent);
    strcat(path, dirfile);
  } else
    strcpy(path, dirfile);

  if (path[strlen(path) - 1] == '/')
    path[strlen(path) - 1] = 0;

  /* shell expand the path, if necessary */
  if (wordexp(path, &expansion, 0) != 0)
    berror(fatal, "unable to expand output dirfile path `%s'", path);

  if (expansion.we_wordc > 1)
    bprintf(fatal, "cannot handle multiple expansion of `%s'", path);

  /* repace path with the output of the shell expansion */
  strcpy(path, expansion.we_wordv[0]);

  wordfree(&expansion);

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
  char element[FR_PATH_MAX];
  char real_path[FR_PATH_MAX];
  char path[FR_PATH_MAX];

  /* is the remount_dir an absolute path? */
  if (rc.remount_dir[0] == '/') {
    strcpy(path, rc.remount_dir);
  } else {
    /* get the curfile's directory) */
    PathSplit_r(source, element, NULL);

    /* check string sizes */
    if (strlen(element) + 1 + strlen(rc.remount_dir) >= FR_PATH_MAX)
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
char* GetFileName(const char* source, const char* herr)
{
  FILE* stream;
  char* buffer;
  int index = 0;
  int is_binary = 0;
  struct stat stat_buf;

  /* allocate our buffer */
  buffer = (char*)balloc(fatal, FILENAME_LEN);

  /* first attempt to stat SOURCE to see if it is indeed a regular file */
  if (stat(source, &stat_buf)) {
    if (rc.force_quenya)
      berror(fatal, "cannot stat `%s'", source);
    else
      berror(fatal, "error resolving source `%s':\n  DNS lookup returned: %s\n"
          "  local stat returned", source, herr);
  }

  /* the stat worked.  Now is this a regular file? */
  if (!S_ISREG(stat_buf.st_mode))
    bprintf(fatal, "`%s' is not a regular file\n", source);
   
  /* If this is a null-lengthed file, there's no point in continuing */
  if (stat_buf.st_blocks == 0)
    bprintf(fatal, "`%s' has zero size\n", source);

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
char* MakeDirFile(char* output, const char* source, const char* directory,
    int start)
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

  if (start > 0) {
    sprintf(buffer, "_%07i", start / FAST_PER_SLOW);
    strcat(output, buffer); 
  }

  bfree(fatal, buffer);

  return output;
}

int DetermineSourceType(const struct rc_struct* rc)
{
  char* x;

  /* assume anything with a colon in it is a host:port pair */
  if (strchr(rc->source, ':'))
    return 1;

  /* if the source ends in .cur, assume it's a local file -- this will create
   * problems once we get the .cur gTLD... */
  if (rc->source - strstr(rc->source, ".cur") + strlen(rc->source) == 4)
    return 0;

  /* if the source contains a character that isn't be allowed in a hostname,
   * assume it's a local file; this takes care of anything with a / in it
   * -- this doesn't work for the new internationalised domain name system */
  for (x = rc->source; *x; ++x) 
    if (*x != '-' && *x != '.' && (*x < '0' || *x > '9')
        && (*x < 'a' || *x > 'z') && (*x < 'A' || *x > 'Z'))
      return 0;

  /* hmm... still can't tell.  Further study is required */
  return -1;
}
/* generates a dirfile name given the source and destination passed on the
 * command line */
void GetDirFile(char* buffer, const char* source, char* parent, int start)
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
  MakeDirFile(buffer, source, parent, start);
}

void PrintVersion(void)
{
  printf("defile " FULL_VERSION "  (C) 2004-2013 D. V. Wiebe and others\n"
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
      "\nConverts BLAST-type framefile from SOURCE into dirfile under "
      "DIRECTORY."
      "\n"
      "\nSOURCE may be a curfile, a framefile to start with, or a blastd host, "
      "optionally"
      "\nwith a port.  Default DIRECTORY to use if no DIRECTORY is given:"
      "\n\t" OUTPUT_DIR
      "\n"
      "\nArguments to long options are required for short arguments as well."
      "\n  -C --curfile-name=NAME same as `-curfile' but use NAME as the name "
      "of the"
      "\n                          curfile instead of the default."
      "\n  -F --framefile        assume SOURCE is a framefile."
      "\n  -R --resume           resume an interrupted defiling."
#ifdef HAVE_LIBZ
      "  Incompatible with"
      "\n                          `--gzip'."
#endif
      "\n  -S --spec-file=NAME   use NAME as the specification file."
      "\n  -a --autoreconnect    try to reconnect to a remote host when the "
      "connection"
      "\n                          is dropped."
      "\n  -c --curfile          write a curfile called `" CUR_FILE "'"
      "\n                          and a symbolic link called `" CUR_FILE".lnk '"
      "\n                          pointing to the output directory."
      "\n  -d --daemonise        fork to background and daemonise on startup.  "
      "Implies"
      "\n                          `--persistent' and `--quiet'."
      "\n     --flakey-source    when reading from a file, don't give up on "
      "read errors."
      "\n                          Useful for unreliable or overloaded NFS "
      "mounts."
      "\n  -f --force            overwrite destination dirfile."
      "\n  -l --local-source     assume the input source is a local file, "
      "even when it"
      "\n                          looks like a remote hostname."
      "\n  -n --network-source   assume the input source is a remote host, "
      "even when it"
      "\n                          looks like a local filename."
      "\n     --no-1utoreconnect don't try to reconnect to a remote host. "
      " (default)"
      "\n     --no-clobber       don't resume or overwrite existing dirfiles. "
      "(default)"
      "\n     --no-curfile       don't write a curfile. (default)"
      "\n     --no-compress      don't compress the output. (default)"
      "\n     --no-daemonise     don't daemonise. (default)"
      "\n     --no-extra-format  don't add extra derived fields (defaut)"
      "\n     --no-flakey-source don't assume the input file is on a flakey"
      "\n                          filesystem. (default)"
      "\n     --no-persist       exit on reaching the end of the input stream. "
      "(default)"
      "\n     --no-remount       assume the input curfile points to the right "
      "place."
      "\n                          (default)"
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
      "relative to the"
      "\n                          curfile's location.  This option has no "
      "effect if"
      "\n                          SOURCE is not a curfile."
      "\n     --remounted-using=DIR same as `--remounted-source' except use "
      "DIR as the"
      "\n                          path instead of the default `" REMOUNT_PATH
      "'."
      "\n  -s --suffix-size=SIZE framefile suffix is no more than SIZE "
      "characters long"
      "\n                          SIZE should be an integer between 0 "
      "and %i. (default=%i)"
      "\n     --verbose          output status information to the tty (default)"
      "\n  -x --extra-format     add extra derived fields to the dirfile"
#ifdef HAVE_LIBZ
      "\n  -z --gzip             gzip compress the output dirfile.  Incompatible "
      "with"
      "\n                          `--resume'"
#endif
      "\n     --help             display this help and exit"
      "\n     --version          display version information and exit"
      "\n     --                 last option; all following parameters are "
      "arguments."
      "\n", SUFF_MAX, SUFF_MAX
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
        else if (!strcmp(argv[i], "--autoreconnect"))
          rc->auto_reconnect = 1;
        else if (!strcmp(argv[i], "--curfile"))
          rc->write_curfile = 1;
        else if (!strncmp(argv[i], "--curfile-name=", 15)) {
          rc->write_curfile = 1;
          bfree(fatal, rc->output_curfile);
          rc->output_curfile = bstrdup(fatal, &argv[i][15]);
        } else if (!strcmp(argv[i], "--daemonise"))
          rc->daemonise = 1;
        else if (!strcmp(argv[i], "--extra-format"))
          rc->extra_format = 1;
        else if (!strcmp(argv[i], "--flakey-source"))
          rc->flakey_source = 1;
        else if (!strcmp(argv[i], "--force"))
          rc->write_mode = 1;
        else if (!strcmp(argv[i], "--framefile"))
          rc->framefile = 1;
#ifdef HAVE_LIBZ
        else if (!strcmp(argv[i], "--gzip"))
          rc->gzip_output = 1;
#endif
        else if (!strcmp(argv[i], "--local-source")) {
          rc->force_quenya = 1;
          rc->quenya = 0;
        } else if (!strcmp(argv[i], "--network-source"))
          rc->force_quenya = rc->quenya = 1;
        else if (!strcmp(argv[i], "--no-autoreconnect"))
          rc->auto_reconnect = 0;
        else if (!strcmp(argv[i], "--no-clobber"))
          rc->write_mode = 0;
        else if (!strcmp(argv[i], "--no-compress"))
          rc->gzip_output = 0;
        else if (!strcmp(argv[i], "--no-curfile"))
          rc->write_curfile = 0;
        else if (!strcmp(argv[i], "--no-daemonise"))
          rc->daemonise = 0;
        else if (!strcmp(argv[i], "--no-extra-format"))
          rc->extra_format = 0;
        else if (!strcmp(argv[i], "--no-flakey-source"))
          rc->flakey_source = 0;
        else if (!strcmp(argv[i], "--no-persist"))
          rc->persist = 0;
        else if (!strcmp(argv[i], "--no-remount"))
          rc->remount = 0;
        else if (!strncmp(argv[i], "--output-dirfile=", 17)) {
          if (rc->output_dirfile)
            bfree(fatal, rc->output_dirfile);
          rc->output_dirfile = balloc(fatal, FR_PATH_MAX);
          strcpy(rc->output_dirfile, &argv[i][17]);
        } else if (!strcmp(argv[i], "--persistent"))
          rc->persist = 1;
        else if (!strcmp(argv[i], "--quiet"))
          rc->silent = 1;
        else if (!strcmp(argv[i], "--remounted-source"))
          rc->remount = 1;
        else if (!strncmp(argv[i], "--remounted-using=", 18)) {
          rc->remount = 1;
          bfree(fatal, rc->remount_dir);
          rc->remount_dir = bstrdup(fatal, &argv[i][18]);
        } else if (!strcmp(argv[i], "--resume")) 
          rc->write_mode = 2;
        else if (!strncmp(argv[i], "--spec-file=", 12)) {
          bfree(fatal, rc->spec_file);
          rc->spec_file = bstrdup(fatal, &argv[i][12]);
        } else if (!strncmp(argv[i], "--suffix-size=", 14)) {
          if (argv[i][14] >= '0' && argv[i][14] <= '9') {
            if ((rc->sufflen = atoi(&argv[i][14])) > SUFF_MAX)
              bprintf(fatal, "suffix size `%s' is not a valid value\n"
                  "Try `defile --help' for more information.\n", &argv[i][14]);
          } else
            bprintf(fatal, "suffix size `%s' is not a valid value\n"
                "Try `defile --help' for more information.\n", &argv[i][14]);
        } else if (!strcmp(argv[i], "--verbose"))
          rc->silent = 0;
        else
          bprintf(fatal, "unrecognised option `%s'\n"
              "Try `defile --help' for more information.\n", argv[i]);
      } else /* a short option */
        for (j = 1; argv[i][j] != '\0'; ++j)
          switch (argv[i][j]) {
            case 'C':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 'C';
              }
            case 'c':
              rc->write_curfile = 1;
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
            case 'a':
              rc->auto_reconnect = 1;
              break;
            case 'd':
              rc->daemonise = 1;
              break;
            case 'f':
              rc->write_mode = 1;
              break;
            case 'l':
              rc->force_quenya = 1;
              rc->quenya = 0;
              break;
            case 'n':
              rc->force_quenya = rc->quenya = 1;
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
              rc->remount = 1;
              break;
            case 's':
              if (nshortargs < argc) {
                shortarg[nshortargs].position = i - 1;
                shortarg[nshortargs++].option = 's';
              }
              break;
            case 'x':
              rc->extra_format = 1;
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

  /* fix up daemon mode */
  if (rc->daemonise)
    rc->persist = rc->silent = rc->force_stdio = 1;

  /* compressed output sanity check */
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
          if (rc->output_dirfile)
            bfree(fatal, rc->output_dirfile);
          rc->output_dirfile = balloc(fatal, FR_PATH_MAX);
          strcpy(rc->output_dirfile, argument[j].value);
        } else
          bprintf(fatal, "output dirfile name `%s' is not a valid value\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      case 's':
        if (argument[j].value[0] >= '0' && argument[j].value[0] <= '9') {
          if ((rc->sufflen = atoi(argument[j].value)) > SUFF_MAX)
            bprintf(fatal, "suffix size `%s' is too big.\n"
                "Try `defile --help' for more information.\n",
                argument[j].value);
        } else
          bprintf(fatal, "suffix size `%s' is not a valid value.\n"
              "Try `defile --help' for more information.\n", argument[j].value);
        break;
      default:
        bprintf(fatal, "Unexpected trap in ParseCommandLine().  Bailing.\n");
    }
  }

  /* first unused argument is SOURCE */
  for (j = 0; j < nargs && argument[j].used; ++j);

  if (j >= nargs || nargs == 0) {
    if (options[CFG_InputSource].value.as_string != NULL) {
      rc->source = options[CFG_InputSource].value.as_string;
      rc->force_quenya = 1;
    } else
      bprintf(fatal, "too few arguments\n"
          "Try `defile --help' for more information.\n");
  } else {
    /* SOURCE */
    rc->source = argument[j].value;

    /* next unused argument is DESTINATION */
    for (j++; j < nargs && argument[j].used; ++j);

    /* DIRECTORY (if any) */
    if (j < nargs && nargs > 1) {
      rc->dest_dir = argument[j].value;
      if (strlen(rc->dest_dir) > FR_PATH_MAX)
        bprintf(fatal, "Destination path too long\n");
    } else
      rc->dest_dir = options[CFG_OutputDirectory].value.as_string;
  }

  if (rc->source == NULL) bputs(fatal, "no source!\n");

  /* If the user hasn't told us what type source is, determine whether source
   * is a filename or a hostname */
  if (!rc->force_quenya)
    rc->quenya = DetermineSourceType(rc);

  /* are further checks required? */
  rc->force_quenya = (rc->quenya == -1) ? 0 : 1;

  /* Fix up output_dirfile, if present */
  if (rc->output_dirfile != NULL)
    rc->output_dirfile = ResolveOutputDirfile(rc->output_dirfile, rc->dest_dir);

  bfree(fatal, argument);
  bfree(fatal, shortarg);
}

int main (int argc, char** argv)
{
  struct timeval now;
  double delta;
  double fr = FR;
  double nf = 0;
  FILE* stream;
  const char* herr = NULL;

  pthread_t read_thread;
  pthread_t write_thread;

  /* set up our outputs */
#ifdef DEBUG
  buos_allow_mem();
#endif
  buos_disable_exit();
  buos_use_func(dputs);

  /* read config file */
  if ((stream = fopen(CONFIG_FILE, "rt")) != NULL) {
    ReadConfig(stream);
    fclose(stream);
  }

  /* fill uninitialised options with default values */
  LoadDefaultConfig();

  /* initialise the rc struct with values from the config file */
  rc = InitRcStruct();

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

  bprintf(info, "defile " FULL_VERSION
      " (C) 2004-2013 D. V. Wiebe and others\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n");

  if (!rc.force_quenya || rc.quenya) {
    /* Resolve the hostname */
    if ((herr = ResolveHost(rc.source, &rc.addr, rc.force_quenya)) == NULL) {
      rc.force_quenya = rc.quenya = 1;
    }
  }

  if (!rc.force_quenya || !rc.quenya) {
    /* Get the name of the framefile. This function handles following the
     * curfile. (This allocates rc.chunk for us.) */
    rc.chunk = GetFileName(rc.source, herr);
    rc.quenya = 0;
  }

  /* if rc.output_dirfile exists, we use that as the dirfile name, otherwise
   * we have to make one based on the input name */
  rc.dirfile = balloc(fatal, FILENAME_LEN);

  /* Initialise the reader or client */
  if (rc.quenya) {
    if (InitClient(NULL))
      bprintf(fatal,
          "Cannot continue after server disconnect in initialisation");
  } else
    InitReader();

  /* check the length of the output path */
  if (strlen(rc.dirfile) > FR_PATH_MAX - FIELD_MAX - 1)
    bprintf(fatal, "destination dirfile `%s' too long\n", rc.dirfile);

  /* Start */
  bprintf(info, "Defiling `%s'\n    into `%s'\n", rc.chunk, rc.dirfile);
  if (rc.resume_at > 0)
    bprintf(info, "    starting at frame %li\n", rc.resume_at);
  bprintf(info, "\n");

  /* Initialise things */
  ri.read = ri.wrote = ri.old_total = ri.lw = ri.frame_rate_reset = 0;
  ri.tty = 0;
  delta = 1;
  gettimeofday(&ri.last, &rc.tz);
  PreInitialiseDirFile();
  InitialiseDirFile(1, rc.quenya ? rc.resume_at : 0);

  /* set up signal masks */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  /* block signals */
  pthread_sigmask(SIG_BLOCK, &signals, NULL);

  /* Spawn client/reader and writer */
  if (rc.quenya)
    pthread_create(&read_thread, NULL, (void*)&QuenyaClient, NULL);
  else
    pthread_create(&read_thread, NULL, (void*)&FrameFileReader, NULL);
  pthread_create(&write_thread, NULL, (void*)&DirFileWriter, NULL);

  /* Main status loop -- if we're in silent mode we skip this entirely and
   * just wait for the read and write threads to exit */
  if (!rc.silent)
    do {
      if (!ri.tty) {
        gettimeofday(&now, &rc.tz);
        delta = (now.tv_sec - ri.last.tv_sec) + (now.tv_usec - ri.last.tv_usec)
          / 1000000.;
        nf = (ri.wrote - ri.lw) / 20.;
        ri.last = now;
        ri.lw = ri.wrote;
        fr = nf / TC + fr * (1 - delta / TC);
        if (ri.frame_rate_reset) {
          ri.frame_rate_reset = 0;
          fr = FR;
        }
#ifndef DEBUG
        if (rc.quenya)
          printf("%s R:[%i] W:[%i] %.*f Hz\r", rc.dirname, ri.read / 20, ri.wrote / 20, (fr
                > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
        else 
          printf("%s R:[%i of %i] W:[%i] %.*f Hz\r", rc.dirname, ri.read / 20, (ri.old_total
                + ri.chunk_total) / 20, ri.wrote / 20, (fr > 100) ? 1 : (fr > 10)
              ? 2 : 3, fr);
        fflush(stdout);
#endif
      }
      usleep(500000);
    } while (!ri.writer_done);
  pthread_join(read_thread, NULL);
  pthread_join(write_thread, NULL);
  if (!rc.silent) {
    if (rc.quenya)
      bprintf(info, "%s R:[%i] W:[%i] %.*f Hz", rc.dirname, ri.read / 20, ri.wrote / 20,
          (fr > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
    else
      bprintf(info, "%s R:[%i of %i] W:[%i] %.*f Hz", rc.dirname, ri.read / 20, (ri.old_total
            + ri.chunk_total) / 20, ri.wrote / 20, (fr > 100) ? 1 : (fr > 10)
          ? 2 : 3, fr);
  }
  return 0;
}
