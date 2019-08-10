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

#include "groundhog.h"
#include "linklist_connect.h"

struct TlmReport ll_report[MAX_NUM_LINKLIST_FILES+1] = {{0}};

char * cmdfile = "/data/etc/bit_config/cmdlist.bit";
char * telemfile = "/data/etc/bit_config/telemlist.bit";
char * calspecfile = "/data/etc/bit_config/calspecs.bit";
char * linklistdir = "/data/etc/bit_config/linklists/";

void groundhog_write_calspecs(char * fname) {
  FILE * fin = fopen(calspecfile, "r");
  if (!fin) return;

  FILE * fout = fopen(fname, "w");
  if (!fout) {
    groundhog_warn("Unable to open calspecs file %s\n", fname);
    fclose(fin);
    return;
  }

  char *line = NULL;
  size_t len = 0;
  int began = 0;

  while (getline(&line, &len, fin) != -1) {
    if (!strncmp(line, "END", 3)) {
      break;
    }
    if (began) {
      fprintf(fout, "%s", line); 
    } else if (!strncmp(line, "BEGIN", 5)) {
      began = 1;
    }
  }
  fclose(fin);
  fclose(fout);
}

int main(int argc, char * argv[]) {
  // set the directory in which to save raw linklist files received from the payload
  sprintf(archive_dir, "/data/groundhog");

  // initialize the main telemetry superframe

  // load command list from text file
  if (parse_cmdlist(cmdfile) < 0) {
    groundhog_fatal("Command parser error\n");
  }
  // load telemetry list from text file
  if (parse_telemlist(telemfile, "superframe.txt") < 0) {
    groundhog_fatal("Telemetry parser error\n");
  }
  
  // initialize the linklists derived from the superframe
  linklist_t ** ll_list = calloc(MAX_NUM_LINKLIST_FILES, sizeof(linklist_t *));
  char *exts[] = {".bit",".ll",""};
	if (load_all_linklists_opt(superframe, linklistdir, ll_list, LL_AUTO_FLOAT_COMP | LL_INCLUDE_ALLFRAME, exts) < 0) {
    groundhog_fatal("Unable to load linklists\n");
  }
  linklist_generate_lookup(ll_list);  
  
  int pilot_on = 1;
  int bi0_on = 1;
  int highrate_on = 1;
  int daemon = 0;
  verbose = 0;

  for (int i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-no_pilot") == 0) pilot_on = 0;
    else if (strcmp(argv[i], "-no_bi0") == 0) bi0_on = 0;
    else if (strcmp(argv[i], "-no_highrate") == 0) highrate_on = 0;
    else if (strcmp(argv[i], "-pilot_only") == 0) bi0_on = highrate_on = 0;
    else if (strcmp(argv[i], "-bi0_only") == 0) highrate_on = pilot_on = 0;
    else if (strcmp(argv[i], "-highrate_only") == 0) pilot_on = bi0_on = 0;
    else if (strcmp(argv[i], "-d") == 0) daemon = 1;
    else if (strcmp(argv[i], "-quiet") == 0) verbose = 0;
    else if (strcmp(argv[i], "-verbose") == 0) verbose = 1;
    else {
      groundhog_warn("Unrecognized option \"%s\"", argv[i]);
      exit(1);
    }
  }

  if (daemon) {
    daemonize();
  }

  pthread_t server_thread;
  pthread_t pilot_receive_worker;
  pthread_t biphase_receive_worker;

  struct UDPSetup pilot_setup = {"Pilot", 
                                 GND_IP, 
                                 GND_TELEM_PORT, 
                                 superframe->size, 
                                 PILOT_PKT_SIZE,
                                 0};

  // start the server thread for mole clients
  pthread_create(&server_thread, NULL, (void *) &linklist_server, NULL);

  if (pilot_on) {
    pthread_create(&pilot_receive_worker, NULL, (void *) &udp_receive, (void *) &pilot_setup);
  }

  if (bi0_on) {
    // pthread_create(&biphase_receive_worker, NULL, (void *) &biphase_receive, NULL);
  }

  char fn_str[1024] = "";
  int len = 0, prev_len = 0;
  int update_freq = 20;
  int stale_ticks = 60;
  char * report_types[] = {
    BLU, // pilot
    GRN, // bi0
    YLW, // highrate
    RED  // sbd
  };

  // print out the reports
  while (true) {
    int i;
    fn_str[0] = '\0';
    for (i=0; ll_report[i].ll; i++) {
      if (ll_report[i].prev_framenum != ll_report[i].framenum) {
        ll_report[i].stale = 0;
      } else {
        ll_report[i].stale++;
      }
      ll_report[i].prev_framenum = ll_report[i].framenum;

      if (ll_report[i].stale > stale_ticks) continue;

			sprintf(fn_str, "%s %s   %s %s [%" PRIu64 "];" NOR, 
              fn_str, 
              report_types[ll_report[i].type],
							ll_report[i].ll->name, 
							(ll_report[i].allframe) ? "AF" : "  ",
							ll_report[i].framenum
			);
    }
    // print enough characters to overwrite the previous line
    len = strlen(fn_str);
    for (int i = len; i < prev_len; i++) fn_str[i] = ' ';
    fn_str[(len > prev_len) ? len : prev_len] = '\0'; // terminate
    prev_len = len;

    fprintf(stdout, "%s\r", fn_str);
    fflush(stdout); 

    usleep(1000000/update_freq);
  }
  return 0;
}
