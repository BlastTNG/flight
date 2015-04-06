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

#include <stdlib.h>     /* ANSI C std library (atoi, exit, realpath) */
#include <signal.h>     /* ANSI C signals (SIG(FOO), sigemptyset, sigaddset) */
#include <string.h>     /* ANSI C strings (strcat, memcpy, &c.)  */
#include <sys/stat.h>   /* SYSV stat (stat, struct stat S_IS(FOO)) */
#include <unistd.h>     /* UNIX std library (fork, chdir, setsid, usleep &c.) */
#include <wordexp.h>    /* POSIX-shell-like word expansion (wordexp) */
#include <errno.h>
#include <stdio.h>

#include <glib.h>
#include <mosquitto.h>
#include <getdata.h>

#include <blast.h>
#include <blast_time.h>
#include <channels_tng.h>

#include "defricher.h"
#include "defricher_writer.h"
#include "defricher_netreader.h"

static int frame_stop;

char **remaining_args = NULL;
channel_t *channels = NULL;

static GOptionEntry cmdline_options[] =
{
        { "autoreconnect", 0, 0, G_OPTION_ARG_NONE, &rc.auto_reconnect, "Automatically reconnect when dropped", NULL},
        { "daemonize", 0, 0, G_OPTION_ARG_NONE, &rc.daemonise, "Fork to the background on startup", NULL},
        { "force", 'f', 0, G_OPTION_ARG_NONE, &rc.force_stdio, "Overwrite destination file if exists", NULL},
        { "output-dirfile", 'o', 0, G_OPTION_ARG_STRING, &rc.output_dirfile, "Use NAME as the output Dirfile name", NULL},
        { G_OPTION_REMAINING, 0, 0, G_OPTION_ARG_FILENAME_ARRAY, &remaining_args, "<SOURCE> [OUTPUT DIRFILE]", NULL},
        {NULL}
};


/* state structs */
struct ri_struct ri;
struct rc_struct rc;

static const double TC = 0.7;
sigset_t signals;

void log_handler(const gchar* log_domain, GLogLevelFlags log_level,
                const gchar* message, gpointer user_data)
{
    printf("%s\n", message);
}





char* resolve_output_dirfile(char* m_dirfile, const char* parent)
{
    char parent_part[NAME_MAX];
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
//    PathSplit_r(path, parent_part, dirfile_part);

    /* canonicalise the path */
    if (realpath(parent_part, m_dirfile) == NULL)
        berror(fatal, "cannot resolve output m_dirfile path `%s'", parent_part);

    /* add back m_dirfile name */
    if (m_dirfile[strlen(m_dirfile) - 1] != '/')
        strcat(m_dirfile, "/");
    strcat(m_dirfile, dirfile_part);

    return m_dirfile;
}



void parse_cmdline(int argc, char** argv, struct rc_struct* rc)
{

    GError *error = NULL;
    GOptionContext *context;

    context = g_option_context_new("");
    g_option_context_set_summary(context, "Converts BLASTPol-TNG framefiles from SOURCE into dirfiles");
    g_option_context_set_description(context, "Please report any errors or bugs to <seth.hillbrand@gmail.com>");
    g_option_context_add_main_entries(context, cmdline_options, NULL);

    if (!g_option_context_parse (context, &argc, &argv, &error))
      {
        g_error("option parsing failed: %s\n", error->message);
        exit (1);
      }

  /* fix up daemon mode */
  if (rc->daemonise)
    rc->persist = rc->silent = rc->force_stdio = 1;

  if (!remaining_args || !remaining_args[0]) {
      g_critical("Must specify a SOURCE from which to read data!\n");
      exit(1);
  }
  g_debug("Source is %s", remaining_args[0]);
  rc->source = remaining_args[0];

  if (remaining_args[1]) {
      g_debug("Dest is %s", remaining_args[1]);
      rc->dest_dir = remaining_args[1];
  }

  /* Fix up output_dirfile, if present */
  if (rc->output_dirfile != NULL)
    rc->output_dirfile = resolve_output_dirfile(rc->output_dirfile, rc->dest_dir);

}



int main(int argc, char** argv)
{
    struct timeval now;
    double delta;
    double fr = 200;
    double nf = 0;
    FILE* stream;
    const char* herr = NULL;

    pthread_t read_thread;
    pthread_t write_thread;

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

    rc.dirfile = strdupa("outputdirfile");
    /* Start */
    bprintf(info, "Defiling `%s'\n    into `%s'\n", rc.chunk, rc.dirfile);
    if (rc.resume_at > 0)
        bprintf(info, "    starting at frame %li\n", rc.resume_at);
    bprintf(info, "\n");
    g_log_set_default_handler(log_handler, NULL);

    /* Initialise things */
    ri.read = ri.wrote = ri.old_total = ri.lw = ri.frame_rate_reset = 0;
    ri.tty = 0;
    delta = 1;
    gettimeofday(&ri.last, &rc.tz);

    /* set up signal masks */
//    sigemptyset(&signals);
//    sigaddset(&signals, SIGHUP);
//    sigaddset(&signals, SIGINT);
//    sigaddset(&signals, SIGTERM);
//
//    /* block signals */
//    pthread_sigmask(SIG_BLOCK, &signals, NULL);

    /* Spawn client/reader and writer */
    defricher_writer_init();
    netreader_init();

//    pthread_create(&write_thread, NULL, (void*) &DirFileWriter, NULL);

    /* Main status loop -- if we're in silent mode we skip this entirely and
     * just wait for the read and write threads to exit */
    if (!rc.silent)
        do {
            if (!ri.tty) {
                gettimeofday(&now, &rc.tz);
                delta = (now.tv_sec - ri.last.tv_sec) + (now.tv_usec - ri.last.tv_usec) / 1000000.;
                nf = (ri.wrote - ri.lw) / 20.;
                ri.last = now;
                ri.lw = ri.wrote;
                fr = nf / TC + fr * (1 - delta / TC);
                if (ri.frame_rate_reset) {
                    ri.frame_rate_reset = 0;
                    fr = 200;
                }
#ifndef DEBUG
                if (rc.quenya)
                    printf("%s R:[%i] W:[%i] %.*f Hz\r", rc.dirname, ri.read / 20, ri.wrote / 20, (fr > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
                else
                    printf("%s R:[%i of %i] W:[%i] %.*f Hz\r", rc.dirname, ri.read / 20, (ri.old_total + ri.chunk_total) / 20, ri.wrote / 20,
                            (fr > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
                fflush(stdout);
#endif
            }
            usleep(500000);
        } while (!ri.writer_done);
    pthread_join(read_thread, NULL);
//    pthread_join(write_thread, NULL);
    if (!rc.silent) {
        if (rc.quenya)
            bprintf(info, "%s R:[%i] W:[%i] %.*f Hz", rc.dirname, ri.read / 20, ri.wrote / 20, (fr > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
        else
            bprintf(info, "%s R:[%i of %i] W:[%i] %.*f Hz", rc.dirname, ri.read / 20, (ri.old_total + ri.chunk_total) / 20, ri.wrote / 20,
                    (fr > 100) ? 1 : (fr > 10) ? 2 : 3, fr);
    }
    return 0;
}
