#include <math.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <stdint.h>
#include <signal.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h> // threads
#include <float.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "blast.h"


int system_idled = 0;
sigset_t signals;

void clean_up(void) {
    unlink("/var/run/groundhog.pid");
    closelog();
}

void signal_action(int signo) {

    if (system_idled) {
        bprintf(info, "caught signal %i while system idle", signo);
    }
    else {
        bprintf(info, "caught signal %i; going down to idle", signo);
        // ShutdownFrameFile();
        system_idled = 1;
        // needs_join = 1;
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
        clean_up();
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
    if ((stream = fopen("/var/run/groundhog.pid", "w")) == NULL) {
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
    atexit(clean_up);

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
    action.sa_handler = signal_action;
    action.sa_mask = signals;
    sigaction(SIGTERM, &action, NULL);
    sigaction(SIGHUP, &action, NULL);
    sigaction(SIGINT, &action, NULL);

    /* block signals */
    pthread_sigmask(SIG_BLOCK, &signals, NULL);
}

static inline int groundhog_get_rate(const E_RATE m_rate)
{
    E_RATE rate = -1;
    switch (m_rate) {
        case RATE_1HZ:
            rate = 1;
            break;
        case RATE_5HZ:
            rate = 5;
            break;
        case RATE_100HZ:
            rate = 100;
            break;
        case RATE_200HZ:
            rate = 200;
            break;
        case RATE_244HZ:
            rate = 244;
            break;
        case RATE_488HZ:
            rate = 488;
            break;
        default:
            defricher_err( "Unknown rate %d", m_rate);
    }
    return rate;
}

int main(int argc, char * argv[]) {

  channels_initialize(channel_list);
  framing_init(channel_list, derived_list);

  linklist_t * ll_list[2] = {parse_linklist("test.ll"), NULL};
  linklist_generate_lookup(ll_list);  

  if (false) {
    daemonize();
  }
 
  // Receiving data from telemetry
  pthread_t pilot_receive_worker;
  pthread_t biphase_receive_worker;

  pthread_create(&pilot_receive_worker, NULL, (void *) &pilot_receive, NULL);
  pthread_create(&biphase_receive_worker, NULL, (void *) &biphase_receive, NULL);

  // Publishing data to MSQT
  pthread_t pilot_publish_worker;
  pthread_t biphase_publish_worker;

  pthread_create(&pilot_publish_worker, NULL, (void *) &pilot_publish, NULL);
  pthread_create(&biphase_publish_worker, NULL, (void *) &biphase_publish, NULL);

  // Joining
  pthread_join(pilot_receive_worker, NULL);
  pthread_join(biphase_receive_worker, NULL);
  pthread_join(pilot_publish_worker, NULL);
  pthread_join(biphase_publish_worker, NULL);

  return 0;
}


