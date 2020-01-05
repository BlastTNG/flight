#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

#define LOS 0
#define TDRSS 1
#define IRIDIUM 2
#define DEFAULT 3

#define SIPCOM1 0x9
#define SIPCOM2 0xC

#define MAX_RTIME 65536.0
#define MAX_DAYS 26.0

#define MAX_SCHED_PER_CHUNK 59
//#define MAX_SCHED_PER_CHUNK 7
#define HEADER_LEN 4

#define COMMAND_TTY "/dev/ttyCMD"

struct SchedStruct {
  int index;
  int day;
  double hour;
  int reduced_time;
};

struct SchedStruct schedule[10000];

struct HeaderStruct {
  unsigned char s;
  unsigned char link;
  unsigned char route;
  unsigned char size;

  unsigned char command;
  unsigned char slot;
  unsigned char i_chunk;
  unsigned char n_chunk;
  unsigned char n_sched;
  unsigned char route_d;
} header;
  
void usage(char *message) {
  fprintf(stderr, "%s\n", message);
  
  fprintf(stderr, "usage: sched_upload [-s <compressed sched>] [-f] [-l|-i] [-1|-2] <slot>\n"
		  "  -s <compressed sched>: compressed schedule file.  default: /data/etc/sched.S\n"
		  "  -f <chunk>: fix - upload chunk <chunk> only.\n"
		  "  -l: los upload\n"
		  "  -i: iridium upload - default\n"
		  "  -t: iridium upload\n"
		  "  -1: route to com1\n"
		  "  -2: route to com2 - default\n"
		  "  <slot>: index of slot to upload into.\n"
  );
  exit(0);
}


int bc_setserial(void) {
  int fd;
  struct termios term; 

  if( (fd = open(COMMAND_TTY, O_RDWR)) < 0 ) {
    perror("Unable to open serial port");
    exit(2);
  }

  if( tcgetattr(fd, &term) ) {
    perror("Unable to get serial device attributes");
    exit(2);
  }

  term.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
  term.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
  term.c_iflag |= INPCK;
  term.c_cc[VMIN] = 0;
  term.c_cc[VTIME] = 0;

  term.c_cflag &= ~(CSTOPB | CSIZE);
  term.c_oflag &= ~(OPOST);
  term.c_cflag |= CS8;

  if(cfsetospeed(&term, B2400)) {          /*  <======= SET THE SPEED HERE */
    perror("error setting serial output speed");
    exit(2);
  }
  if(cfsetispeed(&term, B2400)) {          /*  <======= SET THE SPEED HERE */
    perror("error setting serial input speed");
    exit(2);
  }

  if( tcsetattr(fd, TCSANOW, &term) ) {
    perror("Unable to set serial attributes");
    exit(2);
  }

  return fd;
}

int main(int argc, char *argv[]) {
  char sched[1024];
  char instr[1024];
  int do_fix = 0;
  int do_sleep = 0;
  unsigned int pause = 0;
  int fix_chunk;
  int sipcom = DEFAULT;
  int link = DEFAULT;
  int slot = -1;
  int i,j,k;
  int tmp;
  FILE *fp;
  int n_sched;
  int n_chunk;
  int n_sched_in_chunk;
  int datasize;
  unsigned short idata[MAX_SCHED_PER_CHUNK*2];
  unsigned char footer = 0x03;
  int tty_fd;
  int bytes_written;
  int n;
  int counter;
  unsigned char buf[3];

  
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
          if (i+1<argc) {
            fix_chunk = atoi(argv[i+1]);
            ++i;
          } else {
            usage("no argument for -f");
          }
	  do_fix = 1;
          
	  break;
	case 'z':
          if (i+1<argc) {
            pause = atoi(argv[i+1]);
            ++i;
          } else {
            usage("set a delay time between sending chunks (s) -z");
          }
	  do_sleep = 1;
	  break;
	case 'l':
	  if (link == DEFAULT) {
	    link = LOS;
	  } else {
	    usage("los: link already set");
	  }
	  break;
	case 'i':
	  if (link == DEFAULT) {
	    link = IRIDIUM;
	  } else {
	    usage("iridium: link already set");
	  }
	  break;
	case 't':
	  if (link == DEFAULT) {
	    link = TDRSS;
	  } else {
	    usage("iridium: link already set");
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
  
  if (link==DEFAULT) link = IRIDIUM;
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
  
  n_chunk = (n_sched-1)/MAX_SCHED_PER_CHUNK + 1;
  
  if ((tty_fd = bc_setserial()) < 0) {
    perror("Unable to open serial port");
    exit(2);
  }
  
  while( read(tty_fd, buf, 3) >0 ) {
  }
  
  for (i=0; i<n_chunk; i++) {
    if (do_fix) {
      i = fix_chunk;
      if (i>=n_chunk) {
        fprintf(stderr, "invalid chunk: %d > %d\n", fix_chunk, n_chunk-1);
        exit(0);
      }
      if (i<0) {
        fprintf(stderr, "invalid chunk: %d <0\n", fix_chunk);
        exit(0);
      }
    }
    if (i<n_chunk-1) {
      n_sched_in_chunk = MAX_SCHED_PER_CHUNK;
    } else {
      n_sched_in_chunk = n_sched - i*MAX_SCHED_PER_CHUNK;
    }
    
    datasize = n_sched_in_chunk * 4 + 6;
    if (datasize<22) datasize = 22;
    
    /* csbf header */
    header.s = 0x10;
    header.link = link;
    header.route = sipcom;
    header.size =  datasize;
    /* our header - part of datasize */
    header.command = 0xfe;
    header.slot = ((slot & 0x0f)) << 4 | 0x0f; // the command is 0xffe, so the bottom nibble of slot must be set to 0xf
    header.i_chunk = i;
    header.n_chunk = n_chunk;
    header.n_sched = n_sched_in_chunk;
    header.route_d = sipcom;
    
    // build data packet
    for (j = 0; j < n_sched_in_chunk; j++) {
      idata[j*2] = schedule[i*MAX_SCHED_PER_CHUNK+j].reduced_time;
      idata[j*2+1] = schedule[i*MAX_SCHED_PER_CHUNK+j].index;
    }
    for (j=n_sched_in_chunk; j<MAX_SCHED_PER_CHUNK; j++) {
      idata[j*2] = 0;
      idata[j*2+1] = 0;
    }

    // write the data
    bytes_written = 0;
    bytes_written += write(tty_fd, (unsigned char *)(&header), sizeof(header)); // csbf header + our header
    bytes_written += write(tty_fd, (unsigned char *)idata, datasize-6); // the schedule file
    bytes_written += write(tty_fd, &footer, 1); // footer
  
    
    printf("written %d out of %d\n", bytes_written, datasize+5);
    /* Read acknowledgement */
    n = 0;
    counter = 0;
    while( ((n += read(tty_fd, buf + n, 3 - n)) != 3) &&(counter++<2000)) {
      usleep(10000);
    }
    if (n!=3) {
      fprintf(stderr, "aborting: acknowledgement not received\n");
      exit(0);
    }
    if ((buf[0]==0xfa) && (buf[1]==0xf3)) {
      if (buf[2]==0x00) {
      } else if (buf[2]==0x0a) {
	fprintf(stderr,"aborting: command not sent -  GSE operator disabled science from sending commands\n");
	exit(0);
      } else if (buf[2]==0x0b) {
	fprintf(stderr,"aborting: command not sent - Routing address does not match the selected link\n");
	exit(0);
      } else if (buf[2]==0x0c) {
	fprintf(stderr,"aborting: command not sent -  The link selected was not enabled\n");
	exit(0);
      }	else {
	fprintf(stderr,"aborting: command not sent -  error %x\n", buf[2]);
	exit(0);
      }
    } else {
      fprintf(stderr, "aborting: mangled response: %x %x %x\n", buf[0], buf[1], buf[2]);
      exit(0);
    }
    
    printf("packet %d sent: %d out of %d bytes\n", i, bytes_written, sizeof(header)+datasize-6+1);
    
    if (do_sleep) {
      sleep(pause);
    }
    if (do_fix) {
      break;
    }
    //sleep(11);
  }

  close(tty_fd);
  
  return 0;
}
