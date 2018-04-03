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

#define GROUNDHOG_LOG "/data/etc/groundhog.log"

char datestring[80] = {0};
int system_idled = 0;
sigset_t signals;

void clean_up(void) {
    unlink("/var/run/groundhog.pid");
    // closelog();
}

void daemonize()
{
    int pid;
    FILE* stream;

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
    freopen(GROUNDHOG_LOG, "a", stdout);
    freopen("/dev/null", "w", stderr);
    setsid();
}


int main(int argc, char * argv[]) {
  channels_initialize(channel_list);
  define_superframe();

  linklist_t *ll_list[MAX_NUM_LINKLIST_FILES] = {NULL};
  load_all_linklists(DEFAULT_LINKLIST_DIR, ll_list);
  linklist_generate_lookup(ll_list);  
  linklist_to_file(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), DEFAULT_LINKLIST_DIR ALL_TELEMETRY_NAME ".auto");

  int pilot_on = 1;
  int bi0_on = 1;
  int highrate_on = 1;
  int daemon = 0;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-no_pilot") == 0) pilot_on = 0;
    else if (strcmp(argv[i], "-no_bi0") == 0) bi0_on = 0;
    else if (strcmp(argv[i], "-no_highrate") == 0) highrate_on = 0;
    else if (strcmp(argv[i], "-pilot_only") == 0) bi0_on = highrate_on = 0;
    else if (strcmp(argv[i], "-bi0_only") == 0) highrate_on = pilot_on = 0;
    else if (strcmp(argv[i], "-highrate_only") == 0) pilot_on = bi0_on = 0;
    else if (strcmp(argv[i], "-d") == 0) daemon = 1;
    else {
      blast_err("Unrecognized option \"%s\"", argv[i]);
      exit(1);
    }
  }

  if (daemon) {
    daemonize();
  }

  // initialize framing
  framing_init();

  // get the date string for file saving
  time_t now = time(0);
  struct tm * tm_t = localtime(&now);
  strftime(datestring, sizeof(datestring)-1, "%Y-%m-%d-%H-%M", tm_t);
 
  // setup pilot receive udp struct
  struct UDPSetup pilot_setup = {"Pilot", 
                                 PILOT_ADDR, 
                                 PILOT_PORT, 
                                 PILOT_MAX_SIZE, 
                                 PILOT_MAX_PACKET_SIZE,
                                 PILOT};

  struct UDPSetup udplos_setup = {"BI0-LOS", 
                                  BI0LOS_GND_ADDR, 
                                  BI0LOS_GND_PORT, 
                                  BI0_MAX_BUFFER_SIZE, 
                                  BI0LOS_MAX_PACKET_SIZE,
                                  BI0};

  // Publishing data to MSQT
  pthread_t groundhog_publish_worker;

  // Receiving data from telemetry
  pthread_t pilot_receive_worker;
  pthread_t biphase_receive_worker;
  pthread_t highrate_receive_worker;

  // publishing thread; handles all telemetry publishing to mosquitto
  pthread_create(&groundhog_publish_worker, NULL, (void *) &groundhog_publish, NULL);

  if (pilot_on) {
    pthread_create(&pilot_receive_worker, NULL, (void *) &udp_receive, (void *) &pilot_setup);
  }

  if (bi0_on) {
    // pthread_create(&biphase_receive_worker, NULL, (void *) &biphase_receive, NULL);
    pthread_create(&biphase_receive_worker, NULL, (void *) &udp_receive, (void *) &udplos_setup);
  }

  if (highrate_on) {
    pthread_create(&highrate_receive_worker, NULL, (void *) &highrate_receive, NULL);
  }

  // The Joining
  pthread_join(groundhog_publish_worker, NULL);

  if (pilot_on) {
    pthread_join(pilot_receive_worker, NULL);
  }

  if (bi0_on) {
    pthread_join(biphase_receive_worker, NULL);
  }

  if (highrate_on) {
    pthread_join(highrate_receive_worker, NULL);
  }

  return 0;
}


