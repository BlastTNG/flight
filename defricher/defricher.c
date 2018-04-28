/* defricher: converts BLASTPol-TNG framefiles into dirfiles
 *
 * This file is copyright (C) 2015 Seth Hillbrand
 *
 * Based on previous work copyright (C) 2004-2005, 2013 D. V. Wiebe
 * Also (C) 2005-2010 Matthew Truch.
 * 
 * This file is part of defricher.
 * 
 * defricher is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defricher is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defricher; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifdef HAVE_CONFIG_H
#  include "defricher_config.h"
#endif

#include <libgen.h>
#include <stdlib.h>     /* ANSI C std library (atoi, exit, realpath) */
#include <signal.h>     /* ANSI C signals (SIG(FOO), sigemptyset, sigaddset) */
#include <string.h>     /* ANSI C strings (strcat, memcpy, &c.)  */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <unistd.h>     /* UNIX std library (fork, chdir, setsid, usleep &c.) */
#include <wordexp.h>    /* POSIX-shell-like word expansion (wordexp) */
#include <errno.h>
#include <stdio.h>
#include <execinfo.h>
#include <signal.h>

#include <glib.h>
#include <mosquitto.h>
#include <getdata.h>

#include <blast.h>
#include <blast_time.h>
#include <channels_tng.h>


#include "defricher.h"
#include "defricher_writer.h"
#include "defricher_netreader.h"

char **remaining_args = NULL;

static GOptionEntry cmdline_options[] =
{
        { "autoreconnect", 0, 0, G_OPTION_ARG_NONE, &rc.auto_reconnect, "Automatically reconnect when dropped", NULL},
        { "daemonize", 0, 0, G_OPTION_ARG_NONE, &rc.daemonise, "Fork to the background on startup", NULL},
        { "force", 'f', 0, G_OPTION_ARG_NONE, &rc.force_stdio, "Overwrite destination file if exists", NULL},
        { "linklist", 'l', 0, G_OPTION_ARG_STRING, &rc.linklist_file, "Use NAME in linklist mode. Will only receive data specified in linklist", NULL},
        { "output-dirfile", 'o', 0, G_OPTION_ARG_STRING, &rc.output_dirfile, "Use NAME as the output Dirfile name", NULL},
        { G_OPTION_REMAINING, 0, 0, G_OPTION_ARG_FILENAME_ARRAY, &remaining_args, "<SOURCE> <TELEMETRY (lab/pilot/highrate/biphase)> [OUTPUT DIRFILE]", NULL},
        {NULL}
};


/* state structs */
struct ri_struct ri;
struct rc_struct rc;

struct Fifo fifo_data[RATE_END] = {{0}};

static const double TC = 1.7;
static pthread_t writer_thread;
static pthread_t reader_thread;

void log_handler(const gchar* log_domain, GLogLevelFlags log_level,
                const gchar* message, gpointer user_data)
{
    printf("%s\n", message);
}

void shutdown_defricher(int sig) {
    fprintf(stderr, "Shutting down Defricher on signal %d", sig);
    ri.writer_done = true;
    ri.reader_done = true;
}

void segv_handler(int sig) {
    void *array[10];
    size_t size;
    
    // get void*'s for all entries on the stack
    size = backtrace(array, 10);
    
    // print out all the frames to stderr
    fprintf(stderr, "Error: signal %d:\n", sig);
    backtrace_symbols_fd(array, size, STDERR_FILENO);
    exit(1);
}

/// TODO: Test ability to write to output location on startup

static char* resolve_output_dirfile(char* m_dirfile, const char* parent)
{
    char *parent_part;
    char dirfile_part[FR_PATH_MAX];
    char path[FR_PATH_MAX];
    wordexp_t expansion;

    /* is dirfile a relative path? if so, we don't have to do anything */
    if (m_dirfile[0] != '/' && m_dirfile[0] != '~') {
        /* check string sizes */
        if (strlen(parent) + 1 + strlen(m_dirfile) >= FR_PATH_MAX)
            bprintf(fatal, "output m_dirfile path is too long\n");

        strcpy(path, parent);
        strcat(path, m_dirfile);
    }
    else
        strcpy(path, m_dirfile);

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
    parent_part = dirname(path);

    /* canonicalise the path */
    if (realpath(parent_part, m_dirfile) == NULL)
        berror(fatal, "cannot resolve output m_dirfile path `%s'", parent_part);

    /* add back m_dirfile name */
    if (m_dirfile[strlen(m_dirfile) - 1] != '/')
        strcat(m_dirfile, "/");
    strcat(m_dirfile, dirfile_part);

    return m_dirfile;
}

static void defricher_defaults(struct rc_struct* m_rc)
{
    asprintf(&(m_rc->dest_dir), "/data/defricher");
    asprintf(&(m_rc->source), "fc1");
    asprintf(&(m_rc->telemetry), "lab");
    asprintf(&(m_rc->symlink_name), "/data/etc/defricher.cur");
}


void parse_cmdline(int argc, char** argv, struct rc_struct* m_rc)
{

    GError *error = NULL;
    GOptionContext *context;

    context = g_option_context_new("");
    g_option_context_set_summary(context, "Converts BLASTPol-TNG framefiles from SOURCE into dirfiles\ndefricher <SOURCE> <TELEMETRY (lab/pilot/highrate/biphase)> [OUTPUT DIRFILE]");
    g_option_context_set_description(context, "Please report any errors or bugs to <seth.hillbrand@gmail.com>");
    g_option_context_add_main_entries(context, cmdline_options, NULL);

    if (!g_option_context_parse (context, &argc, &argv, &error))
      {
        g_error("option parsing failed: %s\n", error->message);
        exit (1);
      }

  /* fix up daemon mode */
  if (m_rc->daemonise)
    m_rc->persist = m_rc->silent = m_rc->force_stdio = 1;

  if (remaining_args && remaining_args[0]) {
      g_debug("Source is %s", remaining_args[0]);
      free(m_rc->source);
      asprintf(&(m_rc->source), "%s", remaining_args[0]);

      if (remaining_args[1]) {
          g_debug("Telemetry is %s", remaining_args[1]);
          char *possible_telemetries[4] = {"lab", "highrate", "biphase", "pilot"};
          bool valid_telemetry = false;
          for(int i = 0; i < 4; ++i)
          {
               if(!strcmp(possible_telemetries[i], remaining_args[1]))
               {
                    free(m_rc->telemetry);
                    asprintf(&(m_rc->telemetry), "%s", remaining_args[1]);
                    valid_telemetry = true;
                    break;
               }
          }
          if (!valid_telemetry) {
              g_error("Wrong argument for telemetry: %s. Please choose between lab, highrate, pilot, or biphase", remaining_args[1]);
          }
      }

      if (remaining_args[2]) {
          g_debug("Dest is %s", remaining_args[2]);
          free(m_rc->dest_dir);
          asprintf(&(m_rc->dest_dir), "%s", remaining_args[2]);
          if (m_rc->dest_dir[strlen(m_rc->dest_dir) - 1] == '/') m_rc->dest_dir[strlen(m_rc->dest_dir) - 1] = '\0';
      }
  }

  /* Fix up output_dirfile, if present */
  if (m_rc->output_dirfile) {
    m_rc->output_dirfile = resolve_output_dirfile(m_rc->output_dirfile, m_rc->dest_dir);
  }

  g_option_context_free(context);
}



int main(int argc, char** argv)
{
    struct timeval now;
    double delta;
    double ofr = 200;
    double ifr = 200;
    double nf = 0;

    /* Load the hard-coded defaults if not over-written by command line */
    defricher_defaults(&rc);

    /* fill rc struct from command line */
    parse_cmdline(argc, argv, &rc);

    if (rc.daemonise) {
        int pid;
        FILE* stream;

        /* Fork to background */
        if ((pid = fork()) != 0) {
            if (pid == -1)
                g_critical("unable to fork to background");

            if ((stream = fopen("/var/run/decomd.pid", "w")) == NULL)
                g_error("unable to write PID to disk");
            else {
                fprintf(stream, "%i\n", pid);
                fflush(stream);
                fclose(stream);
            }
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

    /* Start */

    /* Initialise things */
    ri.read = ri.wrote = ri.old_total = ri.lw = ri.frame_rate_reset = 0;
    ri.tty = 0;
    delta = 1;
    gettimeofday(&ri.last, &rc.tz);

    /* set up signal masks */
    signal(SIGBUS, segv_handler);
    signal(SIGILL, segv_handler);

    signal(SIGINT, shutdown_defricher);
    signal(SIGQUIT, shutdown_defricher);

    /* Spawn client/reader and writer */
    writer_thread = defricher_writer_init();
    reader_thread = netreader_init(rc.source, rc.telemetry);

    /* Main status loop -- if we're in silent mode we skip this entirely and
     * just wait for the read and write threads to exit */
    if (!rc.silent)
        do {
            if (!ri.tty) {
                gettimeofday(&now, &rc.tz);
                delta = (now.tv_sec - ri.last.tv_sec) + (now.tv_usec - ri.last.tv_usec) / 1000000.;
                nf = (ri.wrote - ri.lw);
                ri.last = now;
                ri.lw = ri.wrote;
                ofr = nf / TC + ofr * (1 - delta / TC);
                nf = (ri.read - ri.lr);
                ri.lr = ri.read;
                ifr = nf / TC + ifr * (1 - delta / TC);
                if (ri.frame_rate_reset) {
                    ri.frame_rate_reset = 0;
                    ofr = 200;
                    ifr = 200;
                }
#ifndef DEBUG
                 printf("%s R:[%i] %.*f W:[%i] %.*f Hz\r", rc.output_dirfile, ri.read,
                 		(ifr > 100) ? 1 : (ifr > 10) ? 2 : 3, ifr,
                 		ri.wrote,
                 		(ofr > 100) ? 1 : (ofr > 10) ? 2 : 3, ofr);
                 fflush(stdout);
                 usleep(1000);
#endif
            }
            usleep(500000);
        } while (!ri.writer_done);
    pthread_join(reader_thread, NULL);
    pthread_join(writer_thread, NULL);
    if (!rc.silent) {
            bprintf(info, "%s R:[%i] W:[%i] %.*f Hz", rc.output_dirfile, ri.read, ri.wrote, (ofr > 100) ? 1 : (ofr > 10) ? 2 : 3, ofr);
    }
    return 0;
}
