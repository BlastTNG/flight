/* blastcmd: groundstation BLAST command software
 *
 * This software is copyright (C) 2002-2005 University of Toronto
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
#include <sys/socket.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <time.h>
#include <signal.h>

#include "daemon.h"
#include "share/netcmd.h"

double round(double x);

/* Include file containing the command definitions */
#include "command_list.h"

#ifndef DATA_ETC_DIR
#  define DATA_ETC_DIR "/data/etc/blastcmd"
#endif

#define ACK_COUNT 17

#define INPUT_TTY "/dev/ttyCMD"
#define LOGFILE DATA_ETC_DIR "/blastcmd.log"

int silent = 0;

char host[1024] = "localhost";

char *ack[ACK_COUNT] = {
  "Command transmitted.",
  "Unrecognised command.",
  "Unable to open output device.",
  "Parameter out of range.",
  "Science commanding disabled by GSE operator.",
  "Invalid SIP routing address.",
  "Selected SIP link not enabled.",
  "Unspecified error from GSE.",
  "GSE ACK == 0x0E.",
  "GSE ACK == 0x0F.",
  "Unexpected error in command definitions.",
  "Syntax error on command line.",
  "Command cancelled by user.",
  "Program timeout waiting for response from GSE.",
  "Unable to connect to daemon.",
  "Unable to start daemon.",
  "Connexion refused by daemon.",
};

void USAGE(int flag) {
  int i;
  
  printf("blastcmd [@host] [-v] [-f] [-s] [-los|-tdrss|-hf|-iridium] "
      "[-com1|-com2] \\\n"
      "         command [param00 [param01 [param02 [ ... ]]]]\n"
      "blastcmd [-l|-lg|-lc group_num]\n"
      "blastcmd -d [-nf] [-fifo|-null]\n"
      "blastcmd --version\n\n"
      "Options:\n"
      "    @host   Connect to the blastcmd daemon running on host "
      "(default localhost).\n"
      "       -f   Unused.  For backwards compatibility.\n"
      "       -s   Silent.\n"
      "       -v   Unused.  For backwards compatibility.\n"
      "     -los   Set link to Line of Sight.\n"
      "   -tdrss   Set link to TDRSS.\n"
      " -iridium   Set link to Iridium.\n"
      "      -hf   A synonym for -iridium.  For backwards compatibility.\n"
      "    -com1   Set routing to COMM1.\n"
      "    -com2   Set routing to COMM2.\n"
      "       -l   List valid commands and parameters and exit.\n"
      "      -lg   List all command groups in order.\n"
      "      -lc   List all commands in group number given.\n"
      "       -d   Start the blastcmd daemon.\n"
      "      -nf   Don't fork into the background when daemonizing.\n"
      "    -fifo   Route all commands through the local fifo.\n"
      "    -null   Route all commands through /dev/null.\n"
      "--version   Show version and license information and exit.\n\n"
      );

    printf("Exit codes:\n"
        "    -1  Unexpected internal error.\n");

    for (i = 0; i < ACK_COUNT; ++i)
      if (ack[i][0])
        printf("    %2i  %s\n", i, ack[i]);

    printf("\nFor a list of valid commands use `blastcmd -l'\n");
    exit(11);
}

void CommandList(void)
{
  int i, j;

  NetCmdGetCmdList();

  printf("\nCommand List Serial: %s\n\n", client_command_list_serial);
  
  printf("Valid Multiword Commands reported by server:\n");

  for (i = 0; i < client_n_mcommands; i++) {
    printf("  %s - %s\n", client_mcommands[i].name, client_mcommands[i].about);
    for (j = 0; j < client_mcommands[i].numparams; j++) 
      printf("    param%02i: %s\n", j, client_mcommands[i].params[j].name);
    printf("\n");
  }

  printf("Valid Single Word Commands reported by server:\n");
  for (i = 0; i < client_n_scommands; i++) {
    printf("  %s - %s\n", client_scommands[i].name, client_scommands[i].about);
  }

  exit(1);
}

void CommandGroupList(void)
{
  int i;

  NetCmdGetCmdList();

  printf("Valid (and ordered) group names:\n");

  for (i = 0; i < N_GROUPS; i++)
  {
    printf("%s\n", GroupNames[i]);
  }

  exit(1);
}

void CommandGroupListCommands(int groupnum)
{
  int i, j;

  if (groupnum >= N_GROUPS || groupnum < 0)
  {
    fprintf(stderr, "Invalid group number requested\n");
    exit(-1);
  }

  NetCmdGetCmdList();

  printf("Valid mcommands in group %d (%s):\n", groupnum, GroupNames[groupnum]);
  for (i = 0; i < client_n_mcommands; i++) {
    if (client_mcommands[i].group & (1 << groupnum))
    {
      printf("  %s - %s\n", client_mcommands[i].name, client_mcommands[i].about);
      for (j = 0; j < client_mcommands[i].numparams; j++)
        printf("    param%02i: %s\n", j, client_mcommands[i].params[j].name);
      printf("\n");
    }
  }

  printf("Valid scommands in group %d (%s):\n", groupnum, GroupNames[groupnum]);
  for (i = 0; i < client_n_scommands; i++) {
    if (client_scommands[i].group & (1 << groupnum))
    {
    printf("  %s - %s\n", client_scommands[i].name, client_scommands[i].about);
    }
  }
  exit(1);
}

int bc_setserial(void) {
  int fd;
  struct termios term; 

  if( (fd = open(INPUT_TTY, O_RDWR)) < 0 ) {
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

void WriteBuffer(int sock, int tty_fd, unsigned char *buffer, int len,
    unsigned int *i_ack)
{
  int i, n;
  int counter;
  char buf[3];
  char output[1024] = ":::sent:::";
  char hex[4];

  for (i = 0; i < len; i++) {
    sprintf(hex, "%02X.", buffer[i]);
    strcat(output, hex);
  }

  printf("%i<--%s\n", sock, output);
  strcat(output, "\r\n");

  send(sock, output, strlen(output), MSG_NOSIGNAL);

  /* Write the packet to the GSE */
  if (write(tty_fd, buffer, len) < 0) perror("WriteBuffer failed");

  /* Read acknowledgement */
  n = 0;
  counter = 0;
  while( (n += read(tty_fd, buf + n, 3 - n)) != 3) {
    if (counter++ == 2000)
      break;
    i = recv(sock, output, 1024, MSG_DONTWAIT);
    if (i > 0) {
      output[i] = '\0';
      printf("%i-->%s", sock, output);
      if (strncmp(output, "::kill::", 8) == 0) {
        *i_ack = 0x112;
        return;
      }
    }
    usleep(10000);
  }

  if (counter > 2000)
    *i_ack = 0x10;
  else if (*(unsigned short *)buf == 0xf3fa)
    *i_ack = (unsigned int)buf[2];
  else
    *i_ack = 0x0f;
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
  printf("\nSingle Command-> %s\n", scommands[i_cmd].name);

  ConfirmSend();
}

void ConfirmMultiSend(int i_cmd, char *params[], int np) {
  int i;

  printf("\nMulti Command-> %s (%s)\n", mcommands[i_cmd].name,
      mcommands[i_cmd].about);
  for (i = 0; i < np; i++)
    if (mcommands[i_cmd].params[i].type == 's')
      printf("                %s = %s\n", mcommands[i_cmd].params[i].name,
          params[i]);
    else
      printf("                %s = %f\n", mcommands[i_cmd].params[i].name,
          atof(params[i]));

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

void SendScommand(int sock, int i_cmd, int t_link, int t_route,
    unsigned int *i_ack)
{
  unsigned char buffer[7];
  int tty_fd;

  buffer[0] = 0x10;
  buffer[1] = t_link;
  buffer[2] = t_route;
  buffer[3] = 2;
  buffer[4] = scommands[i_cmd].command;
  buffer[5] = 0xa0;
  buffer[6] = 0x03;

  if ((tty_fd = bc_setserial()) < 0) {
    perror("Unable to open serial port");
    exit(2);
  }

  WriteBuffer(sock, tty_fd, buffer, 7, i_ack);

  close(tty_fd);
}

void SendMcommand(int sock, int i_cmd, int t_link, int t_route, char *parms[],
    int np, unsigned int *i_ack) {
  unsigned short dataq[DATA_Q_SIZE];
  int dataqsize = 0;
  float flote, max, min;
  int ynt;
  char type;
  int packet_length = 6;
  unsigned char buffer[25];
  unsigned short *dataqbuffer;
  int i;
  time_t t;
  char output[1024];
  int tty_fd;

  if (np != mcommands[i_cmd].numparams) {
    *i_ack = 0x111;
    return;
  }

  for (i = 0; i < np; i++) {
    min = mcommands[i_cmd].params[i].min;
    max = mcommands[i_cmd].params[i].max;
    type = mcommands[i_cmd].params[i].type;
    if (type == 'i') {
      /* 15 bit integer parameter */
      ynt = atoi(parms[i]);
      if (ynt < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%i < %g)\r\n",
            i + 1, ynt, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > MAX_15BIT) {
        sprintf(output, ":::limit:::parameter %d out of range (%i > %g)\r\n",
            i + 1, ynt, MAX_15BIT);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%i > %g)\r\n",
            i + 1, ynt, max);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      }
      dataq[dataqsize++] = (unsigned short)(ynt - min);
    } else if (type == 'l') {
      /* 30 bit integer parameter */
      ynt = atoi(parms[i]);
      if (ynt < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%i < %g)\r\n",
            i + 1, ynt, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > MAX_30BIT) {
        sprintf(output, ":::limit:::parameter %d out of range (%i > %g)\r\n",
            i + 1, ynt, MAX_30BIT);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%i > %g)\r\n",
            i + 1, ynt, max);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      }
      ynt -= min;
      dataq[dataqsize++] = ynt & 0x00007fff;         /* lower 15 bits */
      dataq[dataqsize++] = (ynt & 0x3fff8000) >> 15; /* upper 15 bits */
    } else if (type == 'f') {
      /* 15 bit floating point parameter */
      flote = atof(parms[i]);
      if (flote < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%g < %g)\r\n",
            i + 1, flote, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (flote > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%g > %g)\r\n",
            i + 1, flote, max);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      }
      dataq[dataqsize++] = round((flote - min) * MAX_15BIT / (max - min)); 
    } else if (type == 'd') {
      /* 30 bit floating point parameter */
      flote = atof(parms[i]);
      if (flote < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%g < %g)\r\n",
            i + 1, flote, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (flote > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%g > %g)\r\n",
            i + 1, flote, max);
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        *i_ack = 0x103;
        return;
      }
      ynt = round((flote - min) * MAX_30BIT / (max - min)); 
      dataq[dataqsize++] = (ynt & 0x3fff8000) >> 15;  /* upper 15 bits */
      dataq[dataqsize++] = ynt & 0x00007fff;          /* lower 15 bits */
    } else if (type == 's') {
      /* 7-bit character string */
      unsigned char c = 0xff;
      int j, len = strlen(parms[i]);
      ynt = rint(max);
      if (ynt > CMD_STRING_LEN)
        ynt = CMD_STRING_LEN;
      ynt += (ynt % 2);
      for (j = 0; j < ynt; ++j) {
        char q = (j > len) ? 0 : parms[i][j] & 0x7F;
        if (c == 0xff)
          c = q;
        else {
          dataq[dataqsize++] = (c << 8) | q;
          c = 0xff;
        }
      }
      if (c != 0xff)
        dataq[dataqsize++] = c << 8;
    } else {
      printf("\nError in command definitions:\n   invalid parameter type '%c' "
          "for parameter %i of mutlicommand: %s\n\n", type, i,
          mcommands[i_cmd].name);
      *i_ack = 0x110;
      return;
    }
  }

  time(&t);

  /* Initialize buffer */
  buffer[0] = 0x10;
  buffer[1] = t_link;
  buffer[2] = t_route;

  if ((tty_fd = bc_setserial()) < 0) {
    perror("Unable to open serial port");
    *i_ack = 0x102;
    return;
  }

  /* Send command */
  buffer[4] = mcommands[i_cmd].command;
  /* t gives unique sync number to this multi command */
  buffer[5] = 0x80 | (unsigned char)(t & 0x1F);

  /* Send parameters */
  for (i = 0; i < dataqsize; i++) {
    dataq[i] &= 0x7fff; /* first bit must be a zero */
    dataqbuffer = (unsigned short *)(buffer + packet_length);
    *dataqbuffer = dataq[i];
    packet_length += 2;

    /* If the packet is full write it out */
    if (packet_length == 24) {
      buffer[packet_length++] = 0x3;
      buffer[3] = packet_length - 5;
      WriteBuffer(sock, tty_fd, buffer, packet_length, i_ack);

      if (*i_ack >= 0x10)
        return;

      packet_length = 4;
    }
  }

  /* Send command footer */
  buffer[packet_length++] = mcommands[i_cmd].command;
  buffer[packet_length++] = 0xc0 | (unsigned char)(t & 0x1F);
  buffer[packet_length++] = 0x3;
  buffer[3] = packet_length - 5;

  WriteBuffer(sock, tty_fd, buffer, packet_length, i_ack);

  close(tty_fd);
}

void WriteLogFile(int count, char *token[], unsigned int i_ack)
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

  for(n = 0; n < count; n++)
    fprintf(f, "Sent: %s\n", token[n]);

  fprintf(f, "Ack: (0x%02x) %s\n\n", i_ack, ack[i_ack]);

  fclose(f);

  chmod(LOGFILE, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
}

void PrintVersion(void)
{
  printf("blastcmd " VERSION "  (C) 2002-2005 University of Toronto\n"
      "Compiled on " __DATE__ " at " __TIME__ ".\n\n"
      "Local command list serial: %s\n\n"
      "This program comes with NO WARRANTY, not even for MERCHANTABILITY or "
      "FITNESS\n"
      "FOR A PARTICULAR PURPOSE. You may redistribute it under the terms of "
      "the GNU\n"
      "General Public License; see the file named COPYING for details.\n",
      command_list_serial
      );
  exit(0);
}

int main(int argc, char *argv[]) {
  char t_link, t_route;
  int i;
  int daemon_route = 0, daemon_fork = 0;
  int daemonise = 0;
  char* command[200];
  int nc = 0;
  int group = -1;
  char request[1024];

  t_link = LINK_DEFAULT_CHAR;
  t_route = ROUTING_DEFAULT_CHAR;

  /* Parse switches */
  for (i = 1; i < argc; i++) {
    if (strcmp(argv[i], "-los") == 0)
      t_link = 'L';
    else if (strcmp(argv[i], "-tdrss") == 0)
      t_link = 'T';
    else if (strcmp(argv[i], "-hf") == 0)
      t_link = 'I';
    else if (strcmp(argv[i], "-iridium") == 0)
      t_link = 'I';
    else if (strcmp(argv[i], "-com1") == 0)
      t_route = '1';
    else if (strcmp(argv[i], "-com2") == 0)
      t_route = '2';
    else if (strcmp(argv[i], "-v") == 0)
      ; /* unused */
    else if (strcmp(argv[i], "-f") == 0)
      ; /* unused */
    else if (strcmp(argv[i], "-s") == 0)
      silent = 1;
    else if (strcmp(argv[i], "-l") == 0) {
      command[0] = "-l";
      nc = 1;
    } else if (strcmp(argv[i], "-lg") == 0) {
      command[0] = "-lg";
      nc = 1;
    } else if (strcmp(argv[i], "-lc") == 0) {
      command[0] = "-lc";
      if (i < (argc - 1)) {
        group = strtol (argv[i+1], NULL, 0);
        i++;
        nc = 1;
      }
    } else if (strcmp(argv[i], "-d") == 0)
      daemonise = 1;
    else if (strcmp(argv[i], "-fifo") == 0)
      daemon_route = 1;
    else if (strcmp(argv[i], "-null") == 0)
      daemon_route = 2;
    else if (strcmp(argv[i], "-nf") == 0)
      daemon_fork = 1;
    else if (strcmp(argv[i], "--version") == 0)
      PrintVersion();
    else if (argv[i][0] == '@')
      strcpy(host, &argv[i][1]);
    else if (argv[i][0] == '-' && (argv[i][1] < '0' || argv[i][1] > '9')
        && argv[i][1] != '.')
      USAGE(0);
    else
      command[nc++] = argv[i];
  }

  if (daemonise)
    Daemonise(daemon_route, daemon_fork);

  if (daemon_route || daemon_fork)
    USAGE(0);

  /* command given on comannd line */
  if (nc) {
    NetCmdConnect(host, silent, silent);

    if (strcmp(command[0], "-l") == 0)
      CommandList();

    if (strcmp(command[0], "-lg") == 0)
      CommandGroupList();

    if (strcmp(command[0], "-lc") == 0)
      CommandGroupListCommands(group);

    //normal command
    //TODO check CONFIRM for commands
    request[0] = t_link;
    request[1] = t_route;
    request[2] = ' ';
    request[3] = '\0';
    for (i = 0; i < nc; ++i) {
      strcat(request, command[i]);
      strcat(request, " ");
    }

    if (!silent) printf("Submitting %s\n\n", request);

    strcat(request, "\r\n");

    if (NetCmdTakeConn(silent)) {
      if (!silent) printf("Took the conn.\n");
      i = NetCmdSendAndReceive(request, silent);
      if (!silent) printf("%s\n", ack[i]);
      return i;
    } else {
      fprintf(stderr, "Unable to take the conn.\n");
      return 14;
    }
  } else USAGE(0);

  fprintf(stderr, "Unexpected trap in main. Stop.\n");
  return -1;
}
