/* decomd: reads the decom stream, performs integrity checking, and writes it
 * to disk
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of decomd.
 * 
 * decomd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * decomd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.    See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with decomd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA    02111-1307  USA
 *
 */

//#define DEBUG

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <syslog.h>
#include <signal.h>
#include <string.h>
#include <libgen.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/statvfs.h>
#include <unistd.h>
#include <pthread.h>

#include "blast.h"
#include "decom_pci.h"
#include "decomd.h"
#include "bbc_pci.h"
#include "channels_tng.h"
#include "decomd_framing.h"
#include "crc.h"

#define VERSION "1.1.0"

#define DEV "/dev/decom_pci"
// #define FILE_SUFFIX 'y'

int decom_fp;
int system_idled = 0;
int needs_join = 0;

sigset_t signals;
pthread_t framefile_thread;
pthread_t decom_thread;
pthread_t publish_thread;


uint16_t out_frame[BI0_FRAME_SIZE+3];
uint16_t anti_out_frame[BI0_FRAME_SIZE+3];

frames_list_t frames;

int status = 0;
double fs_bad = 1;
double dq_bad = 0;
unsigned short polarity = 1;
int du = 0;
int wfifo_size = 0;
unsigned long frame_counter = 0;
uint16_t crc_ok = 0;


void ReadDecom(void);

void CleanUp(void) {
    unlink("/var/run/decomd.pid");
    closelog();
}

void SigAction(int signo) {
    if (system_idled) {
        bprintf(info, "caught signal %i while system idle", signo);
    }
    else {
        bprintf(info, "caught signal %i; going down to idle", signo);
        // ShutdownFrameFile();
        system_idled = 1;
        needs_join = 1;
    }

    if (signo == SIGINT) { /* idle */
        ;
    } else if (signo == SIGHUP) { /* cycle */
        bprintf(info, "system idle, bringing system back up");
        // InitialiseFrameFile(FILE_SUFFIX);
        /* block signals */
        pthread_sigmask(SIG_BLOCK, &signals, NULL);
        // pthread_create(&framefile_thread, NULL, (void*)&FrameFileWriter, NULL);
        /* unblock signals */
        pthread_sigmask(SIG_UNBLOCK, &signals, NULL);
        system_idled = 0;
    } else { /* terminate */
        bprintf(info, "system idle, terminating");
        CleanUp();
        signal(signo, SIG_DFL);
        raise(signo);
    }
}

void daemonize()
{
    int pid;
    FILE* stream;
    struct sigaction action;

    if ((pid = fork()) != 0) {
    if (pid == -1) {
        berror(fatal, "unable to fork to background");
    }
    if ((stream = fopen("/var/run/decomd.pid", "w")) == NULL) {
        berror(err, "unable to write PID to disk");
    }
    else {
        fprintf(stream, "%i\n", pid);
        fflush(stream);
        fclose(stream);
    }
    closelog();
    printf("PID = %i\n", pid);
    exit(0);
    }
    atexit(CleanUp);

    /* Daemonise */
    chdir("/");
    freopen("/dev/null", "r", stdin);
    freopen("/dev/null", "w", stdout);
    freopen("/dev/null", "w", stderr);
    setsid();

    /* set up signal masks */
    sigemptyset(&signals);
    sigaddset(&signals, SIGHUP);
    sigaddset(&signals, SIGINT);
    sigaddset(&signals, SIGTERM);

    /* set up signal handlers */
    action.sa_handler = SigAction;
    action.sa_mask = signals;
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
    sigaction(SIGINT, &action, NULL);

    /* block signals */
    pthread_sigmask(SIG_BLOCK, &signals, NULL);
}


void PublishFrames(void)
{
    static char frame_name[RATE_END][32];
    int frame_offset = 0;
    uint16_t sub_previous_counter_1hz, sub_counter_1hz = 50;
    uint32_t previous_counter_1hz, counter_1hz, counter_100hz, counter_200hz = 50;
    size_t one_hundredth_1hz_frame = 0; 

    uint16_t    read_frame;
    uint16_t    write_frame;
    channel_t *subframe_1hz_Addr = NULL;
    channel_t *frame_1hz_counter_dl_Addr = NULL;
    channel_t *frame_100hz_counter_Addr = NULL;
    channel_t *frame_200hz_counter_Addr = NULL;
 
    channels_initialize(channel_list); 
    framing_init(channel_list, derived_list);
    one_hundredth_1hz_frame = (size_t) ceil(((float) frame_size[RATE_1HZ])/100.0);

    subframe_1hz_Addr = channels_find_by_name("subframe_counter_1hz");
    frame_1hz_counter_dl_Addr = channels_find_by_name("mcp_1hz_framecount_dl");
    frame_100hz_counter_Addr = channels_find_by_name("mcp_100hz_framecount");
    frame_200hz_counter_Addr = channels_find_by_name("mcp_200hz_framecount");


    for (int rate = 0; rate < RATE_END; rate++) {
        // decomd_channel_data[rate] = calloc(1, frame_size[rate]);
        snprintf(frame_name[rate], sizeof(frame_name[rate]), "frames/biphase/%s", RATE_LOOKUP_TABLE[rate].text);
        blast_info("there will be a topic with name %s", frame_name[rate]);
    }

    while (true) {
        write_frame = frames.i_out;
        read_frame = frames.i_in;
        if (read_frame == write_frame) {
            usleep(10000);
            continue;
        }

        while (read_frame != write_frame) {
            // 200 Hz
            memcpy(channel_data[RATE_200HZ], frames.framelist[write_frame], frame_size[RATE_200HZ]);
            framing_publish_200hz(channel_data[RATE_200HZ]);
            counter_200hz = GET_INT32(frame_200hz_counter_Addr);
            blast_info("Counter 200hz is %d", counter_200hz);
            frame_offset = frame_size[RATE_200HZ];
            memcpy(channel_data[RATE_200HZ], frames.framelist[write_frame]+frame_offset, frame_size[RATE_200HZ]);
            counter_200hz = GET_INT32(frame_200hz_counter_Addr);
            blast_info("Counter 200hz is %d", counter_200hz);
            framing_publish_200hz(channel_data[RATE_200HZ]);
            // 100 Hz
            frame_offset = 2*frame_size[RATE_200HZ];
            memcpy(channel_data[RATE_100HZ], frames.framelist[write_frame]+frame_offset, frame_size[RATE_100HZ]);
            framing_publish_100hz(channel_data[RATE_100HZ]);
            counter_100hz = GET_INT32(frame_100hz_counter_Addr);
            blast_info("Counter 100hz is %d", counter_100hz);
            // 1 Hz
            frame_offset = 2*frame_size[RATE_200HZ] + frame_size[RATE_100HZ];
            sub_counter_1hz = GET_UINT8(subframe_1hz_Addr);
            if (sub_counter_1hz < sub_previous_counter_1hz) {
                counter_1hz = GET_INT32(frame_1hz_counter_dl_Addr);
                blast_info("Counter 1hz is %d", counter_1hz);
                if (false) {
                    if (previous_counter_1hz == counter_1hz) {
                        SET_INT32(frame_1hz_counter_dl_Addr, (counter_100hz/100 - 1)); // Publishing the previous frame
                    }
                }
                framing_publish_1hz(channel_data[RATE_1HZ]);
                previous_counter_1hz = counter_1hz;
            }
            memcpy(channel_data[RATE_1HZ]+sub_counter_1hz*one_hundredth_1hz_frame, 
                   frames.framelist[write_frame]+frame_offset, one_hundredth_1hz_frame);
            sub_previous_counter_1hz = sub_counter_1hz;

            write_frame = (write_frame + 1) & BI0_FRAME_BUFMASK;
        }
        frames.i_out = write_frame;
        usleep(10000);
    }
}

int main(void) {
    struct statvfs vfsbuf;
    unsigned long long int disk_free = 0;
    struct timeval now;
    struct timeval then;
    struct timezone tz;
    double dt;


    buos_use_stdio();

    /* Open Decom */
    if ((decom_fp = open(DEV, O_RDONLY | O_NONBLOCK)) == -1) {
        berror(fatal, "fatal error opening " DEV);
    }

    // /* Initialise Channel Lists */
    // MakeAddressLookups("/data/etc/decomd/Nios.map");
    // initialize_biphase_buffer();

    /* Initialise Decom */
    ioctl(decom_fp, DECOM_IOC_RESET);
    ioctl(decom_fp, DECOM_IOC_FRAMELEN, BI0_FRAME_SIZE-1);

    /* set up our outputs */
    openlog("decomd", LOG_PID, LOG_DAEMON);
    // buos_use_syslog();

    /* Fork to background */
    if (false) {
    daemonize();
    }

    gettimeofday(&then, &tz);
    pthread_create(&publish_thread, NULL, (void*)&PublishFrames, NULL);
    pthread_create(&decom_thread, NULL, (void*)&ReadDecom, NULL);

    /* unblock signals */
    pthread_sigmask(SIG_UNBLOCK, &signals, NULL);


    /* main loop */
    while (true) {

        // if (needs_join) {
        //   pthread_join(framefile_thread, NULL);
        //   needs_join = 0;
        // }

        if (statvfs("/mnt/decom", &vfsbuf)) {
           berror(err, "statvfs");
        } else {
           disk_free = (unsigned long long int)vfsbuf.f_bavail * vfsbuf.f_bsize;
        }


        gettimeofday(&now, &tz);
        dt = (now.tv_sec + now.tv_usec / 1000000.) -
          (then.tv_sec + then.tv_usec / 1000000.);
        then = now;

        usleep(500000);
    }

    return 1;
}
