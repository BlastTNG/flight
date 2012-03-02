/* minicp: the Miniature master control program
 *
 * mcp.c: contains the main loop and creates all threads used by mcp
 *
 * This software is copyright (C) 2002-2011 University of Toronto
 *
 * mcp is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * mcp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with mcp; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <fcntl.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/ioctl.h>
#include <sys/io.h>
#include <sys/statvfs.h>
#include <stdarg.h>
#include <errno.h>
#include <signal.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>
#include <sys/syscall.h>
#include "bbc_pci.h"

#include "blast.h"
#include "command_struct.h"
#include "mcp.h"
#include "channels.h"
#include "tx.h"

#define BBC_EOF      (0xffff)
#define BBC_BAD_DATA (0xfffffff0)

#define STARTUP_VETO_LENGTH 250 /* "frames" */

/* Define global variables */
int bbc_fp = -1;
unsigned int BBFrameIndex;
unsigned short* slow_data[FAST_PER_SLOW];
unsigned short* RxFrame;
pthread_t watchdog_id;

int StartupVeto = STARTUP_VETO_LENGTH + 1;

static int Death = -STARTUP_VETO_LENGTH * 2;
static int RxFrameMultiplexIndex;

extern pthread_mutex_t mutex;       //commands.c
void WatchFIFO(void);               //commands.c
void FrameFileWriter(void);         //framefile.c
void InitialiseFrameFile(char);
void pushDiskFrame(unsigned short *RxFrame);
void ShutdownFrameFile();
void startAzEl();  // az-el.c
void endAzEl();    // az-el.c

static FILE* logfile = NULL;

#define MPRINT_BUFFER_SIZE 1024
#define MAX_MPRINT_STRING \
( \
  MPRINT_BUFFER_SIZE /* buffer length */ \
  - 3                /* start-of-line marker */ \
  - 24               /* date "YYYY-MM-DD HH:MM:SS.mmm " */ \
  - 2                /* Newline and NUL */ \
)

#if (TEMPORAL_OFFSET != 0)
#warning TEMPORAL_OFFSET NON-ZERO; FIX FOR FLIGHT
#endif

/* gives system time (in s) */
time_t mcp_systime(time_t *t) {
  time_t the_time = time(NULL) + TEMPORAL_OFFSET;
  if (t)
    *t = the_time;

  return the_time;
}

/* tid to name lookup list */
/* TODO setting/reading tid names not quite thread safe. is this a problem? */
#define TID_NAME_LEN  6	      //always change with TID_NAME_FMT
#define TID_NAME_FMT  "%6s"   //always change with TID_NAME_LEN
struct tid_name {
  int tid;
  char name[TID_NAME_LEN+1];
  struct tid_name* next;
};

struct tid_name* threadNames = NULL;

void nameThread(const char* name)
{
  struct tid_name* new_node = (struct tid_name*)malloc(sizeof(struct tid_name));
  new_node->tid = syscall(SYS_gettid);
  strncpy(new_node->name, name, TID_NAME_LEN);
  new_node->name[TID_NAME_LEN] = '\0';
  new_node->next = threadNames;
  threadNames = new_node;
  bprintf(startup, "New thread name for tid %d", new_node->tid);
}

char failed_lookup_buffer[TID_NAME_LEN+1];
char* threadNameLookup(int tid)
{
  struct tid_name* p;
  for(p=threadNames; p->next != NULL; p = p->next)
    if (p->tid == tid) return p->name;
  //not found, just print tid
  snprintf(failed_lookup_buffer, TID_NAME_LEN, "%u", (unsigned)tid);
  failed_lookup_buffer[TID_NAME_LEN] = '\0';
  return failed_lookup_buffer;
}
  

/* I/O function to be used by bputs, bprintf, etc.
 * it is assigned this purpose in main
 */
void mputs(buos_t flag, const char* message) {
  char buffer[MPRINT_BUFFER_SIZE];
  struct timeval t;
  struct timezone tz; /* We never use this, but gettimeofday won't let us
                         give it a NULL -- also it's obsolete under linux */
  struct tm now;
  char local[1024];
  char *bufstart, *bufptr, *lastchr, *firstchr;
  int len;
  char marker[4];

  /* time */
  gettimeofday(&t, &tz);
  t.tv_sec += TEMPORAL_OFFSET;

  switch(flag) {
    case err:
      strcpy(marker, "* ");
      break;
    case fatal:
      strcpy(marker, "! ");
      break;
    case info:
      strcpy(marker, "- ");
      break;
    case sched:
      strcpy(marker, "# ");
      break;
    case startup:
      strcpy(marker, "> ");
      break;
    case tfatal:
      strcpy(marker, "$ ");
      break;
    case warning:
      strcpy(marker, "= ");
      break;
    case mem:
      return;  /* don't record mem messages at all */
      strcpy(marker, "m ");
      break;
    default:
      strcpy(marker, "? ");
      break;
  }
  strcpy(buffer, marker);

  /* we need a writable copy of the string */
  strncpy(local, message, 1023);
  local[1023] = '\0';

  for(bufstart = buffer; *bufstart != '\0' && bufstart < buffer
      + 1024; ++bufstart);


  strftime(bufstart, 1023, "%F %T", gmtime_r(&t.tv_sec, &now));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, ".%03li ", t.tv_usec/1000);
  strcat(buffer, marker);

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

  sprintf(bufstart, TID_NAME_FMT ": ", threadNameLookup(syscall(SYS_gettid)));

  for(;*bufstart != '\0' && bufstart < buffer + 1024; ++bufstart);

    /* output the formatted string line by line */
    for (firstchr = bufptr = local; *bufptr != '\0' &&
        bufptr < local + 1023; ++bufptr) {

      /* writeout the string when we find a newline or the EOS */
      if (*bufptr == '\n' || *(bufptr + 1) == '\0') {
        lastchr = (*bufptr == '\n') ? bufptr : bufptr + 1;
        *lastchr = '\0';

        /* compute length of string to writeout */
        len = lastchr - firstchr + 1;
        if (len > MAX_MPRINT_STRING - 1)
          len = MAX_MPRINT_STRING - 1;

        /* append string part and a newline to preamble */
        strncpy(bufstart, firstchr, len);
        *(bufstart + len + 1) = '\0';
        strcat(bufstart, "\n");
        if (logfile != NULL) {
          fputs(buffer, logfile);
          fflush(logfile);
        }
        if (logfile == NULL || flag != mem) {
          fputs(buffer, stdout);
          fflush(stdout);
        }

        firstchr = bufptr + 1;
      }
    }

  if (flag == fatal) {
    if (logfile != NULL) {
      fputs("!! Last error is FATAL.  Cannot continue.\n", logfile);
      fflush(logfile);
    }
    fputs("!! Last error is FATAL.  Cannot continue.\n", stdout);
    fflush(stdout);

    exit(1);
  }

  if (flag == tfatal) {
    if (logfile != NULL) {
      fprintf(logfile, "$$ Last error is THREAD FATAL. Thread [" 
          TID_NAME_FMT " (%5u)] exits.\n",threadNameLookup(syscall(SYS_gettid)),
          (unsigned)syscall(SYS_gettid));
      fflush(logfile);
    }
    printf("$$ Last error is THREAD FATAL.  Thread [" 
          TID_NAME_FMT " (%5u)] exits.\n",threadNameLookup(syscall(SYS_gettid)),
          (unsigned)syscall(SYS_gettid));
    fflush(stdout);

    pthread_exit(NULL);
  }
}

/* fill_Rx_frame: places one 32 bit word into the RxFrame. 
 * Returns true on success */
static int fill_Rx_frame(unsigned int in_data)
{
  static int n_not_found = 0;
  struct BiPhaseStruct BiPhaseData;

  if (in_data == 0xffffffff)
    return 1;

  /* discard ADC sync words */
  if (in_data & BBC_ADC_SYNC)
    return 1;

  /* words with no write flag are ignored, don't process them */
  if (~in_data & BBC_WRITE)
    return 1;

  /* BBC reverse address lookup */
  BiPhaseData = BiPhaseLookup[BI0_MAGIC(in_data)];

  /* Return error if the lookup failed */
  if (BiPhaseData.index == -1) {
    if (++n_not_found > 2)
      return 0;
    else
      return 1;
  }

  n_not_found = 0;

  /* Discard words we're not saving */
  if (BiPhaseData.index == DISCARD_WORD)
    return 1;

  /* Write the data to the local buffers */
  if (BiPhaseData.index == NOT_MULTIPLEXED)
    RxFrame[BiPhaseData.channel] = BBC_DATA(in_data);
  else
    slow_data[BiPhaseData.index][BiPhaseData.channel] = BBC_DATA(in_data);

  return(1);
}

static void zero()
{
  int i;

  for (i = 0; i < SLOW_OFFSET + slowsPerBi0Frame; i++)
    RxFrame[i] = 0;
}

/******************************************************************/
/*                                                                */
/* IsNewFrame: returns true if d is a begining of frame marker,   */
/*    unless this is the first beginning of frame.                */
/*                                                                */
/******************************************************************/
static int IsNewFrame(unsigned int d)
{
  static int first_bof = 1;
  int is_bof;
  is_bof = (d == (BBC_FSYNC | BBC_WRITE | BBC_NODE(63) | BBC_CH(0) | 0xEB90));
  if (is_bof && first_bof) {
    is_bof = first_bof = 0;
  }

  return is_bof;
}

/* Signal handler called when we get a hup, int or term */
static void CloseBBC(int signo)
{
  bprintf(err, "System: Caught signal %i; stopping NIOS", signo);
  RawNiosWrite(0, BBC_ENDWORD, NIOS_FLUSH);
  RawNiosWrite(BBCPCI_MAX_FRAME_SIZE, BBC_ENDWORD, NIOS_FLUSH);
  bprintf(err, "System: Closing BBC and Bi0");
  if (bbc_fp >= 0)
    close(bbc_fp);

  ShutdownFrameFile();
  endAzEl();

  /* restore default handler and raise the signal again */
  signal(signo, SIG_DFL);
  raise(signo);
}

#if 0
static char segvregs[100];
static int segvcnt = 0;
static void SegV(int signo)
{
  fprintf(stderr, "SEGV caught: %s\n", segvregs);
  raise(SIGTERM);
}
#endif

int main(int argc, char *argv[])
{
  unsigned int in_data, i;
  int startup_test = 0;
  pthread_t CommandDatacomm1;
  pthread_t disk_id;
  struct stat fstats;

  if (argc == 1) {
    fprintf(stderr, "Must specify file type:\n"
        "p  pointing\n"
        "m  maps\n"
        "c  cryo\n"
        "n  noise\n"
        "x  software test\n"
        "f  flight\n");
    exit(0);
  }

  umask(0);  /* clear umask */

  if ((logfile = fopen("/data/etc/minicp/minicp.log", "a")) == NULL) {
    berror(err, "System: Can't open log file");
    fstats.st_size = -1;
  } else {
    if (fstat(fileno(logfile), &fstats) < 0)
      fstats.st_size = -1;
    fputs("!!!!!! LOG RESTART !!!!!!\n", logfile);
  }

  /* register the output function */
  nameThread("Dummy"); //insert dummy sentinel node first
  nameThread("Main");
  buos_use_func(mputs);

#if (TEMPORAL_OFFSET > 0)
  bprintf(warning, "System: TEMPORAL OFFSET = %i\n", TEMPORAL_OFFSET);
#endif

  bputs(startup, "System: Startup");


  if ((bbc_fp = open("/dev/bbcpci", O_RDWR)) < 0)
    berror(fatal, "System: Error opening BBC");

  //both modes want interrupts enabled (?)
  if (ioctl(bbc_fp, BBCPCI_IOC_ON_IRQ) < 0) startup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_SYNC) < 0) startup_test = -1;
  usleep(100000);  //TODO: needed after sync? shorter?

#ifdef USE_EXT_SERIAL
  bprintf(startup, "System: BBC using external synchronization/serial numbers");
  if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_ON) < 0) startup_test = -1;
  //external mode rates in units of DV pulse rate (200Hz on test box)
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE, 1) < 0) startup_test = -1;
  if (ioctl(bbc_fp, BBCPCI_IOC_FRAME_RATE, SERIAL_PER_FRAME) < 0)
    startup_test = -1;
#else
  bprintf(startup, "System: BBC generating internal serial numbers");
  if (ioctl(bbc_fp, BBCPCI_IOC_EXT_SER_OFF) < 0) startup_test = -1;
  //internal mode rates in units of bbc clock rate (32MHz or 4MHz) (?)
  //frame rate doesn't need to be specified int his mode
  //TODO don't think this irq rate is right; doesn't matter much in this mode
  if (ioctl(bbc_fp, BBCPCI_IOC_IRQ_RATE, 320000) < 0) startup_test = -1;
#endif
  if (startup_test < 0)
    bprintf(fatal, "System: BBC failed to set synchronization mode");

  MakeAddressLookups("/data/etc/minicp/Nios.map");

  InitCommandData();
  pthread_mutex_init(&mutex, NULL);

  bprintf(info, "Commands: MCP Command List Version: %s", command_list_serial);
  pthread_create(&CommandDatacomm1, NULL, (void*)&WatchFIFO, NULL);

  InitialiseFrameFile(argv[1][0]);
  pthread_create(&disk_id, NULL, (void*)&FrameFileWriter, NULL);
  startAzEl();

  signal(SIGHUP, CloseBBC);
  signal(SIGINT, CloseBBC);
  signal(SIGTERM, CloseBBC);

  /* Allocate the local data buffers */
  RxFrame = balloc(fatal, BiPhaseFrameSize);

  for (i = 0; i < FAST_PER_SLOW; ++i) {
    slow_data[i] = balloc(fatal, slowsPerBi0Frame * sizeof(unsigned short));
    //TODO fix "uninitialised value" valgrind errors. Ensure not more serious.
    memset(slow_data[i], 0, slowsPerBi0Frame * sizeof(unsigned short));
  }

  bputs(info, "System: Finished Initialisation, waiting for BBC to come up.\n");

  /* mcp used to wait here for a semaphore from the BBC, which makes the
   * presence of these messages somewhat "historical" */

  bputs(info, "System: BBC is up.\n");

  InitTxFrame();

  while (1) {  //main loop
    if (read(bbc_fp, (void *)(&in_data), 1 * sizeof(unsigned int)) <= 0)
      berror(err, "System: Error on BBC read");

    if (!fill_Rx_frame(in_data))
      bprintf(err, "System: Unrecognised word received from BBC (%08x)",
          in_data);

    if (IsNewFrame(in_data)) {
      if (StartupVeto > 1) {
        --StartupVeto;
      } else {
        /* Frame sequencing check */
        if (StartupVeto) {
          bputs(info, "System: Startup Veto Ends\n");
          StartupVeto = 0;
          Death = 0;
        } else if (RxFrame[3] != (RxFrameMultiplexIndex + 1) % FAST_PER_SLOW
            && RxFrameMultiplexIndex >= 0) {
          bprintf(err, "System: Frame sequencing error detected: wanted %i, "
              "got %i\n", RxFrameMultiplexIndex + 1, RxFrame[3]);
        }
        RxFrameMultiplexIndex = RxFrame[3];

        /* Save current fastsamp */
        /* NB: internal frame numbers from PCI don't work. TODO: fix! */
        BBFrameIndex = (RxFrame[1] + RxFrame[2] * 0x10000);

        UpdateBBCFrame();
        CommandData.bbcFifoSize = ioctl(bbc_fp, BBCPCI_IOC_BBC_FIONREAD);

        pushDiskFrame(RxFrame);

        zero();
      }
    }
  }
  return(0);
}
