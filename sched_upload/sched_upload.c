#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOS 0
#define IRIDIUM 2
#define DEFAULT 3

#define SIPCOM1 0x9
#define SIPCOM2 0xC

#define MAX_RTIME 65536.0
#define MAX_DAYS 21.0

struct SchedStruct {
  int index;
  int day;
  double hour;
  int reduced_time;
};

struct SchedStruct schedule[10000];

void usage(char *message) {
  fprintf(stderr, "%s\n", message);
  
  fprintf(stderr, "usage: sched_upload [-s <compressed sched>] [-f] [-l|-i] [-1|-2] <slot>\n"
		  "  -s <compressed sched>: compressed schedule file.  default: /data/etc/sched.S\n"
		  "  -f: fix - upload missing chunks only.\n"
		  "  -l: los upload\n"
		  "  -i: iridium upload - default\n"
		  "  -1: route to com1\n"
		  "  -2: route to com2 - default\n"
		  "  <slot>: index of slot to upload into.\n"
  );
  exit(0);
}

int main(int argc, char *argv[]) {
  char sched[1024];
  char instr[1024];
  int do_fix = 0;
  int sipcom = DEFAULT;
  int route = DEFAULT;
  int slot = -1;
  int i;
  FILE *fp;
  int n_sched;
  
  if (argc == 1) usage("Not enough entries");
  
  for (i=1; i<argc; i++) {
    if (argv[i][0] == '-') {
      switch (argv[i][1]) {
	case 'h':
	  usage("Help:");
	  break;
	case 's':
	  if (i+1<argc) {
	    strncpy(sched, argv[i+1], 1024);
	    ++i;
	  } else {
	    usage("no argument for -s");
	  }
	  break;
	case 'f':
	  do_fix = 1;
	  break;
	case 'l':
	  if (route == DEFAULT) {
	    route = LOS;
	  } else {
	    usage("los: route already set");
	  }
	  break;
	case 'i':
	  if (route == DEFAULT) {
	    route = IRIDIUM;
	  } else {
	    usage("los: route already set");
	  }
	  break;
	case '1':
	  if (sipcom == DEFAULT) {
	    sipcom = SIPCOM1;
	  } else {
	    usage("sipcom1: destination already set");
	  }
	  break;
	case '2':
	  if (sipcom == DEFAULT) {
	    sipcom = SIPCOM2;
	  } else {
	    usage("sipcom2: destination already set");
	  }
	  break;	  
	default:
	  usage("unknown option");
	  break;
      }
    } else {
      if (slot == -1) {	
        slot = atoi(argv[i]);
      } else {
	usage("slot already set");
      }
    }
  }
  
  if (route==DEFAULT) route = IRIDIUM;
  if (sipcom == DEFAULT) sipcom = SIPCOM2;
  if (slot==-1) usage("slot not set");

  /* read compressed file */
  fp = fopen(sched, "r");

  if (fp==NULL) {
    fprintf(stderr, "sched_upload: Could not open sched file %s.\n", sched);
    exit(0);
  }
  
  i=0;
  while (fgets(instr, 1024, fp)!=NULL) {
    if (sscanf(instr, "%d %d %lg", &schedule[i].index, &schedule[i].day, &schedule[i].hour) == 3) {
      schedule[i].reduced_time = ((double)schedule[i].day + schedule[i].hour/24.0)*MAX_RTIME/MAX_DAYS;
      i++;
    }
  }
  n_sched = i;
  
  for (i=0; i<n_sched; i++) {
    printf("%d %d %g %d\n", schedule[i].index, schedule[i].day, schedule[i].hour, schedule[i].reduced_time);
  }
  
  return 0;
}
