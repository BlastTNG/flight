#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <math.h>
#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <sys/types.h> // socket types
#include <sys/socket.h> // socket stuff
#include <sys/stat.h>
#include <dirent.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <pthread.h> // threads
#include <float.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "blast.h"
#include "groundhog_framing.h"
#include "channels_tng.h"
#include "groundhog.h"
#include "pilot.h"
#include "bi0.h"

char datestring[80] = {0};
int system_idled = 0;
sigset_t signals;

void clean_up(void) {
    unlink("/var/run/groundhog.pid");
    // closelog();
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
    // closelog();
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

int main(int argc, char * argv[]) {

  channels_initialize(channel_list);
  framing_init();

  linklist_t *ll_list[MAX_NUM_LINKLIST_FILES] = {NULL};
  load_all_linklists(DEFAULT_LINKLIST_DIR, ll_list);
  linklist_generate_lookup(ll_list);  
  linklist_to_file(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), DEFAULT_LINKLIST_DIR ALL_TELEMETRY_NAME ".auto");

  int pilot_on = 1;
  int bi0_on = 1;
  int highrate_on = 1;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-no_pilot") == 0) pilot_on = 0;
    else if (strcmp(argv[i], "-no_bi0") == 0) bi0_on = 0;
    else if (strcmp(argv[i], "-no_highrate") == 0) highrate_on = 0;
    else {
      blast_err("Unrecognized option \"%s\"", argv[i]);
      exit(1);
    }
  }

  if (false) {
    daemonize();
  }

  // initialize the framing mutex for MQQT
  framing_init_mutex();

  // get the date string for file saving
  time_t now = time(0);
  struct tm * tm_t = localtime(&now);
  strftime(datestring, sizeof(datestring)-1, "%Y-%m-%d-%H-%M", tm_t);
 
  // setup pilot receive udp struct
  struct UDPSetup pilot_setup = {"Pilot", PILOT_ADDR, PILOT_PORT, PILOT_MAX_SIZE, PILOT_MAX_PACKET_SIZE};
  struct UDPSetup udplos_setup = {"BI0-LOS", BI0LOS_GND_ADDR, BI0LOS_GND_PORT, BI0_MAX_BUFFER_SIZE, BI0LOS_MAX_PACKET_SIZE};

  // Receiving data from telemetry
  pthread_t pilot_receive_worker;
  pthread_t biphase_receive_worker;
  pthread_t tdrss_receive_worker;

  // Publishing data to MSQT
  pthread_t pilot_publish_worker;
  pthread_t biphase_publish_worker;
  pthread_t tdrss_publish_worker;

  if (pilot_on) {
    pthread_create(&pilot_receive_worker, NULL, (void *) &pilot_receive, (void *) &pilot_setup);
    pthread_create(&pilot_publish_worker, NULL, (void *) &pilot_publish, NULL);
  }

  if (bi0_on) {
    // pthread_create(&biphase_receive_worker, NULL, (void *) &biphase_receive, NULL);
    pthread_create(&biphase_receive_worker, NULL, (void *) &pilot_receive, (void *) &udplos_setup);
    pthread_create(&biphase_publish_worker, NULL, (void *) &biphase_publish, NULL);
  }

  if (highrate_on) {
    pthread_create(&tdrss_receive_worker, NULL, (void *) &tdrss_receive, NULL);
    pthread_create(&tdrss_publish_worker, NULL, (void *) &tdrss_publish, NULL);
  }


  // Joining
  if (pilot_on) {
    pthread_join(pilot_receive_worker, NULL);
    pthread_join(pilot_publish_worker, NULL);
  }

  if (bi0_on) {
    pthread_join(biphase_receive_worker, NULL);
    pthread_join(biphase_publish_worker, NULL);
  }

  if (highrate_on) {
    pthread_join(tdrss_receive_worker, NULL);
    pthread_join(tdrss_publish_worker, NULL);
  }

  return 0;
}


