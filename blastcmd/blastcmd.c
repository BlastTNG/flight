/* blastcmd: groundstation BLAST command software
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of blastcmd.
 * 
 * blastcmd is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * blastcmd is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with blastcmd; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <termios.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

double round(double x);

/* Include file containing the command definitions */
#include "command_list.h"

#ifndef LIB_DIR
#  define LIB_DIR "/data/etc/blastcmd"
#endif

#define INPUT_TTY "/dev/ttyS1"
#define LOGFILE LIB_DIR "/log.txt"

#define LINK_DEFAULT 0x01    /* Default link is TDRSS */
#define ROUTING_DEFAULT 0x09 /* Default routing is COM1 */

char *ack[16] = {
  "Commands Transmitted",
  "",
  "",
  "",
  "",
  "",
  "",
  "",
  "",
  "",
  "GSE operator disabled science from sending commands",
  "Routing address does not match the selected link",
  "The link selected was not enabled",
  "Bho!",
  "",
  "Received garbage"
};

int verbose = 0;

void USAGE(int flag) {
  printf("blastcmd [-v] [-f] [-s] [-los|-tdrss|-hf] [-com1|-com2] \\\n"
      "         command [param00 [param01 [param02 [ ... ]]]]\n"
      "blastcmd -c\n"
      "blastcmd --version\n\n"
      "Options:\n"
      "       -v   Verbose\n"
      "       -s   Silent\n"
      "       -c   Show the command list serial number and exit\n"
      "       -f   No confirm\n"
      "     -los   Set link to Line of Sight\n"
      "   -tdrss   Set link to TDRSS\n"
      "      -hf   Set link to HF\n"
      "    -com1   Set routing to comm1\n"
      "    -com2   Set routing to comm2\n"
      "--version   Show version and license information and exit\n\n"
      );

  if (!flag) {
    printf("Exit codes:\n"
        "     0  Command sent successfully.\n"
        "     1  No command specified or command cancelled by user.\n"
        "     2  Unable to open serial port.\n"
        "     3  Parameter out of range.\n"
        "     4  ACK == 0x0A\n"
        "     5  ACK == 0x0B\n"
        "     6  ACK == 0x0C\n"
        "     7  ACK == 0x0D\n"
        "     8  ACK == 0x0E\n"
        "     9  ACK == 0x0F\n"
        "    10  Unexpected error in command definitions\n"
        "    11  Syntax error on command line\n\n");

    printf("For a list of valid commands use `blastcmd -l'\n");
    exit(11);
  }
}

void CommandList(void)
{
  int i, j;

  USAGE(1);

  printf("Valid Multiword Commands:\n");

  for (i = 0; i < N_MCOMMANDS; i++) {
    printf("  %s - %s\n", mcommands[i].name, mcommands[i].about);
    for (j = 0; j < mcommands[i].numparams; j++) 
      printf("    param%02i: %s\n", j, mcommands[i].params[j].name);
    printf("\n");
  }

  printf("Valid Single Word Commands:\n");
  for (i = 0; i < N_SCOMMANDS; i++) {
    printf("  %s - %s\n", scommands[i].name, scommands[i].about);
  }

  exit(1);
}

void bc_close() {
}

int bc_setserial(void) {
  int fd;
  struct termios term; 

  if( (fd = open(INPUT_TTY, O_RDWR)) < 0 ) {
    perror("Unable to open serial port");
    return -1;
  }

  if( tcgetattr(fd, &term) ) {
    perror("Unable to get serial device attributes");
    return -1;
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
    return -1;
  }
  if(cfsetispeed(&term, B2400)) {          /*  <======= SET THE SPEED HERE */
    perror("error setting serial input speed");
    return -1;
  }

  if( tcsetattr(fd, TCSANOW, &term) ) {
    perror("Unable to set serial attributes");
    return -1;
  }

  return fd;
}

void WriteBuffer(unsigned char *buffer, int len, unsigned int *i_ack) {
  int i, n;
  int tty_fd;
  char buf[3];

  if (verbose) {
    for (i = 0; i < len; i++) {
      printf("%hx ", (unsigned short)buffer[i]);
    }
    printf("\n");
  }

  if ((tty_fd = bc_setserial()) < 0) {
    perror("Unable to open serial port");
    exit(2);
  }

  write(tty_fd, buffer, len);

  /* Read acknowledgement */
  n = 0;
  while( (n += read(tty_fd, buf+n, 3-n)) != 3) usleep(10000);
  if (*(unsigned short *)buf == 0xf3fa)
    *i_ack = (unsigned int)buf[2];
  else
    *i_ack = 0x0f;

  close(tty_fd);
}

void ConfirmSend() {
  char tmp[5];

  printf("Send this command? [Y, n] ");

  strcpy(tmp, fgets(tmp, 5, stdin));

  if (strlen(tmp) == 1 || tmp[0] == 'Y' || tmp[0] == 'y')
    printf("\n");
  else {
    printf("\nCommand aborted.\n\n");
    exit(1);
  }
}

  void ConfirmSingleSend(int i_cmd) {
    if (i_cmd > N_SCOMMANDS)
      printf("\nCustom Command-> c%02d\n", i_cmd);
    else
      printf("\nSingle Command-> %s\n", scommands[i_cmd].name);

    ConfirmSend();
  }

void ConfirmMultiSend(int i_cmd, char *params[], int np) {
  int i;

  printf("\nMulti Command-> %s (%s)\n", mcommands[i_cmd].name, mcommands[i_cmd].about);
  for (i = 0; i < np; i++)
    printf("                %s = %f\n", mcommands[i_cmd].params[i].name, atof(params[i]));

  ConfirmSend();
}

void McommandUSAGE(int mcmd) {
  int i;
  
  printf("blastcmd: Error in multiword command parameter.\n\n");

  printf("  %s - %s\n", mcommands[mcmd].name, mcommands[mcmd].about);
  for (i = 0; i < mcommands[mcmd].numparams; i++) 
    printf("    param%02i: %s\n", i, mcommands[mcmd].params[i].name);
  printf("\n");
  exit(11);
}

/* Packet encoding:
 * <start word> <data...> <end word> for multi word cmds
 * <cmd> for single word cmds
 * b15/b14: 100 -> begining of cmd
 *          110 -> end of cmd
 *          101 -> single word cmd
 *          0?? -> data word
 */

void SendScommand(int i_cmd, int t_link, int t_route, unsigned int *i_ack, char conf) {
  unsigned char buffer[7];

  if (!conf)
    ConfirmSingleSend(i_cmd);

  buffer[0] = 0x10;
  buffer[1] = t_link;
  buffer[2] = t_route;
  buffer[3] = 2;
  buffer[4] = scommands[i_cmd].command;
  buffer[5] = 0xa0;
  buffer[6] = 0x03;

  WriteBuffer(buffer, 7, i_ack);
}

void SendMcommand(int i_cmd, int t_link, int t_route, char *parms[], int np,
    unsigned int *i_ack, char conf) {
  unsigned short dataq[DATA_Q_SIZE];
  int dataqsize = 0;
  float flote, max, min;
  int ynt;
  char type;
  unsigned char *buffer;
  unsigned short *dataqbuffer;
  int i;
  time_t t;

  if (np != mcommands[i_cmd].numparams)
    McommandUSAGE(i_cmd);

  for (i = 0; i < np; i++) {
    min = mcommands[i_cmd].params[i].min;
    max = mcommands[i_cmd].params[i].max;
    type = mcommands[i_cmd].params[i].type;
    if (type == 'i') {
      /* 15 bit integer parameter */
      ynt = atoi(parms[i]);
      if (ynt < min) {
        printf("blastcmd: parameter %d out of range (%i < %g)\n", i+1, ynt,
            min);
        exit(3);
      } else if (ynt > MAX_15BIT) {
        printf("blastcmd: parameter %d out of range (%i > %g)\n", i+1, ynt,
            MAX_15BIT);
        exit(3);
      } else if (ynt > max) {
        printf("blastcmd: parameter %d out of range (%i > %g)\n", i+1, ynt,
            max);
        exit(3);
      }
      dataq[dataqsize++] = (unsigned short)(ynt - min);
    } else if (type == 'f') {
      /* 15 bit floating point parameter */
      flote = atof(parms[i]);
      if (flote < min) {
        printf("blastcmd: parameter %d out of range (%g < %g)\n", i+1, flote,
            min);
        exit(3);
      } else if (flote > max) {
        printf("blastcmd: parameter %d out of range (%g > %g)\n", i+1, flote,
            max);
        exit(3);
      }
      dataq[dataqsize++] = round((flote - min) * MAX_15BIT / (max - min)); 
    } else if (type == 'l') {
      /* 30 bit floating point parameter */
      flote = atof(parms[i]);
      if (flote < min) {
        printf("blastcmd: parameter %d out of range (%g < %g)\n", i+1, flote,
            min);
        exit(3);
      } else if (flote > max) {
        printf("blastcmd: parameter %d out of range (%g > %g)\n", i+1, flote,
            max);
        exit(3);
      }
      ynt = round((flote - min) * MAX_30BIT / (max - min)); 
      dataq[dataqsize++] = (ynt & 0x3fff8000) >> 15;  /* upper 15 bits */
      dataq[dataqsize++] = ynt & 0x00007fff;          /* lower 15 bits */
    } else {
      printf("\nError in command definitions:\n   invalid parameter type '%c' "
          "for parameter %i of mutlicommand: %s\n\n", type, i,
          mcommands[i_cmd].name);
      exit(10);
    }
  }

  if (!conf)
    ConfirmMultiSend(i_cmd, parms, np);

  time(&t);

  buffer = malloc(7);

  /* Initialize buffer */
  buffer[0] = 0x10;
  buffer[1] = t_link;
  buffer[2] = t_route;
  buffer[3] = 2;
  buffer[6] = 0x03;

  /* Send command */
  buffer[4] = mcommands[i_cmd].command;
  buffer[5] = 0x80 | (unsigned char)(t & 0x1F); /* t gives unique sync number */
  /* to this multi command */
  WriteBuffer(buffer, 7, i_ack);

  /* Send parameters */
  dataqbuffer = (unsigned short *) (buffer + 4);
  for (i = 0; i < dataqsize; i++) {
    dataq[i] &= 0x7fff; /* first bit must be a zero */
    *dataqbuffer = dataq[i];
    WriteBuffer(buffer, 7, i_ack);
  }

  /* Send command footer */
  buffer[4] = mcommands[i_cmd].command;
  buffer[5] = 0xc0 | (unsigned char)(t & 0x1F);
  WriteBuffer(buffer, 7, i_ack);

  free(buffer);
}

void WriteLogFile(int argc, char *argv[], unsigned int i_ack, char silent)
{
  FILE *f;
  time_t t;
  int n;

  t = time(NULL);
  f = fopen(LOGFILE, "a");

  if (f == NULL) {
    printf("blastcmd: could not open log file %s\n", LOGFILE);
    return;
  }

  fprintf(f, "%s", ctime(&t));

  for(n = 1; n < argc; n++)
    fprintf(f, "Sent: %s\n", argv[n]);

  fprintf(f, "Ack: %s\n\n", ack[i_ack]);

  fclose(f);

  if (!silent)
    printf("%s\n", ack[i_ack]);

  if (i_ack == 0x0a)
    exit(4);
  if (i_ack == 0x0b)
    exit(5);
  if (i_ack == 0x0c)
    exit(6);
  if (i_ack == 0x0d)
    exit(7);
  if (i_ack == 0x0e)
    exit(8);
  if (i_ack == 0x0f)
    exit(8);
  chmod(LOGFILE, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
}

void PrintCommandListSerial(void)
{
  printf("Command List Serial: %s\n", command_list_serial);
  exit(0);
}

void PrintVersion(void)
{
  printf("blastcmd " VERSION "  (C) 2002-2004 University of Toronto\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n"
      "This program comes with NO WARRANTY, not even for MERCHANTABILITY or "
      "FITNESS\n"
      "FOR A PARTICULAR PURPOSE. You may redistribute it under the terms of "
      "the GNU\n"
      "General Public License; see the file named COPYING for details.\n"
      );
  exit(0);
}

int main(int argc, char *argv[]) {
  int t_link, t_route;
  int i, i_cmd;
  unsigned int i_ack;
  char conf = 0;
  char silent = 0;

  t_link = LINK_DEFAULT;
  t_route = ROUTING_DEFAULT;

  if(argc <= 1)
    USAGE(0);

  /* Parse switches */
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-los") == 0)
      t_link = 0x00;
    else if (strcmp(argv[i], "-tdrss") == 0)
      t_link = 0x01;
    else if (strcmp(argv[i], "-hf") == 0)
      t_link = 0x02;
    else if (strcmp(argv[i], "-com1") == 0)
      t_route = 0x09;
    else if (strcmp(argv[i], "-com2") == 0)
      t_route = 0x0c;
    else if (strcmp(argv[i], "-v") == 0)
      verbose = 1;
    else if (strcmp(argv[i], "-f") == 0)
      conf = 1;
    else if (strcmp(argv[i], "-s") == 0)
      silent = 1;
    else if (strcmp(argv[i], "-l") == 0)
      CommandList();
    else if (strcmp(argv[i], "--version") == 0)
      PrintVersion();
    else if (strcmp(argv[i], "-c") == 0)
      PrintCommandListSerial();
  }

  i = 1;
  while ((i < argc) && (argv[i][0] == '-'))
    i++;

  if(i >= argc)
    USAGE(0);

  atexit(bc_close);

  /* Look for single packet commands */
  for (i_cmd = 0; i_cmd < N_SCOMMANDS; i_cmd++) {
    if (strncmp(argv[i], scommands[i_cmd].name, SIZE_NAME) == 0) {
      SendScommand(i_cmd, t_link, t_route, &i_ack, conf);
      WriteLogFile(argc, argv, i_ack, silent);
      exit(0);
    }
  }

  /* Look for multi packet commands */
  for (i_cmd = 0; i_cmd < N_MCOMMANDS; i_cmd++) {
    if (strncmp(argv[i], mcommands[i_cmd].name, SIZE_NAME) == 0) {
      SendMcommand(i_cmd, t_link, t_route, argv + i + 1 , argc - i - 1, &i_ack,
          conf);
      WriteLogFile(argc, argv, i_ack, silent);
      exit(0);
    }
  }

  printf("blastcmd: unknown command.  For a list of valid commands use `blastcmd -l'\n");

  return(1);
}
