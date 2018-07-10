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
#include <ctype.h>

#include "linklist.h"
#include "linklist_compress.h"
#include "linklist_writer.h"
#include "linklist_connect.h"
#include "blast.h"
#include "groundhog_framing.h"
#include "channels_tng.h"
#include "derived.h"
#include "groundhog.h"
#include "pilot.h"
#include "bi0.h"

#define GROUNDHOG_LOG "/data/etc/groundhog.log"

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

linklist_rawfile_t * groundhog_open_new_rawfile(linklist_rawfile_t * ll_rawfile, linklist_t * ll, char * symname) {
  if (ll_rawfile) {
    close_and_free_linklist_rawfile(ll_rawfile);
  } 
  char filename[128];
  make_linklist_rawfile_name(ll, filename);
  ll_rawfile = open_linklist_rawfile(filename, ll);

  char fname[128];
  sprintf(fname, "%s/%s_live", archive_dir, symname);
  create_rawfile_symlinks(ll_rawfile, fname);

  sprintf(fname, "%s" CALSPECS_FORMAT_EXT, filename);
  groundhog_write_calspecs(fname, derived_list);

  return ll_rawfile;
}

void groundhog_write_calspecs_item(FILE *calspecsfile, derived_tng_t *derived) {
    int j;

    switch (derived->type) {
      case 'w':
      case 'b':
        fprintf(calspecsfile, "%s BIT %s %u %u\n", derived->bitword.field, derived->bitword.source, derived->bitword.offset, derived->bitword.length);
        break;
      case 't':
        fprintf(calspecsfile, "%s LINTERP %s %s\n", derived->linterp.field, derived->linterp.source, derived->linterp.lut);
        break;
      case 'c':
        fprintf(calspecsfile, "%s LINCOM 1 %s %.16f %.16f\n", derived->lincom.field, derived->lincom.source, derived->lincom.m_c2e, derived->lincom.b_e2e);
        break;
      case '2':
        fprintf(calspecsfile, "%s LINCOM 2 %s %.16f %.16f %s %.16f %.16f\n", derived->lincom2.field, 
          derived->lincom2.source, derived->lincom2.m_c2e, derived->lincom2.b_e2e, 
          derived->lincom2.source2, derived->lincom2.m2_c2e, derived->lincom2.b2_e2e);
        break;
      case '#':
        break;
      case 'u':
        if (derived->units.quantity[0]) {
          fprintf(calspecsfile, "%s/quantity STRING \"", derived->units.source);
          for (j = 0; j < strlen(derived->units.quantity); j++) {
            if (derived->units.quantity[j] == 92) fprintf(calspecsfile, "\\"); // fix getdata escape
            fprintf(calspecsfile, "%c", derived->units.quantity[j]);
          }
          fprintf(calspecsfile,"\"\n");
        }
        if (derived->units.units[0]) {
          fprintf(calspecsfile, "%s/units STRING \"", derived->units.source);
          for (j = 0; j < strlen(derived->units.units); j++) {
            if (derived->units.units[j] == 92) fprintf(calspecsfile, "\\"); // fix getdata escape
            fprintf(calspecsfile, "%c", derived->units.units[j]);
          }
          fprintf(calspecsfile,"\"\n");
        }
        break;
      case 'p':
        fprintf(calspecsfile, "%s PHASE %s %d\n", derived->phase.field, derived->phase.source, derived->phase.shift);
        break;
      case 'r':
        fprintf(calspecsfile, "%s RECIP %s %.16f\n", derived->recip.field, derived->recip.source, derived->recip.dividend);
        break;
      case '*':
        fprintf(calspecsfile, "%s MULTIPLY %s %s\n", derived->math.field, derived->math.source, derived->math.source2);
        break;
      case '/':
        fprintf(calspecsfile, "%s DIVIDE %s %s\n", derived->math.field, derived->math.source, derived->math.source2);
        break;
      case 'x':
        fprintf(calspecsfile, "%s MPLEX %s %s %d %d\n", derived->mplex.field, derived->mplex.source, derived->mplex.index, derived->mplex.value, derived->mplex.max);
        break;
      default:
        blast_warn("Unknown type %c", derived->type);
        break;
    }
}

void groundhog_write_calspecs(char * fname, derived_tng_t *m_derived)
{
  FILE * calspecsfile = fopen(fname, "w");
  if (!calspecsfile) {
    blast_err("Could not open \"%s\" as calspecs file\n", fname);
    return;
  }

  for (derived_tng_t *derived = m_derived; derived && derived->type != DERIVED_EOC_MARKER; derived++) {
    groundhog_write_calspecs_item(calspecsfile, derived);
  }

  derived_tng_t derived = {0};
  char tmp_str[128] = {0};
  for (channel_t *channel = channel_list; channel->field[0]; channel++) {
    snprintf(tmp_str, 128, "%s", channel->field);
    double m = channel->m_c2e;
    double b = channel->b_e2e;

    // don't do roach channels; we have something special for that (see below)
    if ((strstr(tmp_str, "roach") != NULL) && (strstr(tmp_str, "kid") != NULL)) {
      continue;
    }

    /// By default we set the converted field to upper case
    for (int i = 0; tmp_str[i]; i++) tmp_str[i] = toupper(tmp_str[i]);
    /// If our scale/offset are unity/zero respectively, tell defile to use the easier zero-phase
    if (fabs(m - 1.0) <= DBL_EPSILON && fabs(b - 0.0) <= DBL_EPSILON) {
      derived.type = 'p';
      strcpy(derived.phase.field, tmp_str);
      strcpy(derived.phase.source, channel->field);
      derived.phase.shift = 0;
    } else {
      derived.type = 'c';
      strcpy(derived.lincom.field, tmp_str);
      strcpy(derived.lincom.source, channel->field);
      derived.lincom.m_c2e = m;
      derived.lincom.b_e2e = b;
    }
    groundhog_write_calspecs_item(calspecsfile, &derived);
  }

  // something special: generate an array of derived fields for each roach channel 
  // multiplex roach index fields will point to the corresponding name for display in kst
  fprintf(calspecsfile, "ROACH_NAMES SARRAY");

  int kid = 0, roach = 1, rtype = 0;
  for (rtype = 0; rtype < NUM_RTYPES; rtype++) {
    for (roach = 1; roach <= NUM_ROACHES; roach++) {
      for (kid = 0; kid < NUM_KIDS; kid++) {
        char tlm_name[64] = {0};
        unsigned int index = get_roach_index(roach, kid, rtype);
        make_name_from_roach_index(index, tlm_name);
        for (int i = 0; tlm_name[i]; i++) tlm_name[i] = toupper(tlm_name[i]);
        fprintf(calspecsfile, " '%s'", tlm_name);
/*
        derived.type = 'x';
        make_name_from_roach_index(index, derived.mplex.field);
        for (int i = 0; derived.mplex.field[i]; i++) derived.mplex.field[i] = toupper(derived.mplex.field[i]);
        strcpy(derived.mplex.source, ROACH_CHANNEL_BLOCK_NAME);
        strcpy(derived.mplex.index, ROACH_CHANNEL_BLOCK_INDEX_NAME);
        derived.mplex.value = index;
        derived.mplex.max = 0;
        groundhog_write_calspecs_item(calspecsfile, &derived);
*/
      }
    }
  }
  fprintf(calspecsfile, "\n");
  
  for (int i = 0; i < NUM_ROACH_TLM; i++) {
    char c = 65+i;
    fprintf(calspecsfile, "KID%c_ROACHN_NAME SINDIR kid%c_roachN_index ROACH_NAMES\n", c, c);
  }


  fflush(calspecsfile);
  fclose(calspecsfile);
}

int main(int argc, char * argv[]) {
  channels_initialize(channel_list);
  linklist_t *ll_list[MAX_NUM_LINKLIST_FILES] = {NULL};
  load_all_linklists(superframe, DEFAULT_LINKLIST_DIR, ll_list, LL_INCLUDE_ALLFRAME);
  linklist_generate_lookup(ll_list);  
  write_linklist_format(linklist_find_by_name(ALL_TELEMETRY_NAME, ll_list), DEFAULT_LINKLIST_DIR ALL_TELEMETRY_NAME ".auto");

  groundhog_write_calspecs("test.calspecs", derived_list); 

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

  // Serving up data received via telemetry
  pthread_t server_thread;

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

  // start the server thread for mole clients
  pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL);

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


