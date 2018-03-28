/* blastcmd: groundstation BLAST command software
 *
 * This software is copyright (C) 2002-2013 D. V. Wiebe and others
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

#ifdef __SPIDER__
#define  PROGNAME   "spidercmd"
#else
#define  PROGNAME   "blastcmd"
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
#include <limits.h>
#include <ctype.h>

#include "daemon.h"
#include "netcmd.h"

double round(double x);

/* Include file containing the command definitions */
#include "command_list.h"

#ifndef DATA_ETC_DIR
#  define DATA_ETC_DIR "/data/etc/blastcmd"
#endif

#define INPUT_TTY "/dev/ttyCMD"
#define LOGFILE DATA_ETC_DIR "/blastcmd.log"

char err_message[ERR_MESSAGE_LEN];

int silent = 0;

/* If non-zero the selected link is disabled */
int link_disabled[256][2];

char host[1024] = "localhost";

#define ACK_COUNT 21
char *ack[ACK_COUNT] = {
  "Command transmitted.", /* 0 */
  "Unrecognised command.", /* 1 */
  "Unable to open output device.", /* 2 */
  "Parameter out of range.", /* 3 */
  "Science commanding disabled by GSE operator.", /* 4 */
  "Invalid SIP routing address.", /* 5 */
  "Selected SIP link not enabled.", /* 6 */
  "Unspecified error from GSE.", /* 7 */
  "GSE ACK == 0x0E.", /* 8 */
  "GSE ACK == 0x0F.", /* 9 */
  "Unexpected error in command definitions.", /* 10 */
  "Syntax error on command line.", /* 11 */
  "Command cancelled by user.", /* 12 */
  "Program timeout waiting for response from GSE.", /* 13 */
  "Unable to connect to daemon.", /* 14 */
  "Unable to start daemon.", /* 15 */
  "Connexion refused by daemon.", /* 16 */
  "Command parameter validation failed", /* 17 */
  "Error forwarding command", /* 18 */
  "Protocol error", /* 19 */
  "Channel disabled by command server", /* 20 */
};

void USAGE(int flag) {
  int i;
  
  printf(PROGNAME " [@host[:port]] [-v] [-f] [-s] "
      "[-los|-tdrss|-hf|-iridium|-pilot] \\\n"
      "                       [-com1|-com2] [-noelog] \\\n"
      "                       command [param00 [param01 [param02 [ ... ]]]]\n"
      PROGNAME " [-l|-lg|-lc group_num]\n"
      PROGNAME " -d [-nf] [-p port] [-fifo|-null] [-no LINK]...\\\n"
      "                       [-fw host1[:port1],host2[:port2]]\n"
      PROGNAME " --version\n\n"
      "Options:\n"
      "@host[:port]   Connect to the blastcmd daemon running on host\n"
      "                     (default localhost).\n"
      "          -f   Unused.  For backwards compatibility.\n"
      "          -s   Silent.\n"
      "          -v   Unused.  For backwards compatibility.\n"
      "        -los   Set link to Line of Sight.\n"
      "      -tdrss   Set link to TDRSS.\n"
      "    -iridium   Set link to Iridium packet.\n"
      "      -pilot   Set link to Iridium pilot.\n"
      "         -hf   A synonym for -iridium.  For backwards compatibility.\n"
      "       -com1   Set routing to COMM1.\n"
      "       -com2   Set routing to COMM2.\n"
      "     -noelog   Don't log this command to elog.\n"
      "          -l   List valid commands and parameters and exit.\n"
      "         -lg   List all command groups in order.\n"
      "         -lc   List all commands in group number given.\n"
      "          -d   Start the blastcmd daemon.\n"
      "         -nf   Don't fork into the background when daemonizing.\n"
      "         -fw   Use the specified hosts for command forwarding (Pilot)\n"
      "         -no   Disable a link/routing pair (specify L1, I2, P1, &c.)\n"
      "          -p   Listen on the specified port, instead of the default %i\n"
      "       -fifo   Route all commands through the local fifo.\n"
      "       -null   Route all commands through /dev/null.\n"
      "   --version   Show version and license information and exit.\n\n",
    SOCK_PORT);

    printf("Exit codes:\n"
        "    -1  Unexpected internal error.\n");

    for (i = 0; i < ACK_COUNT; ++i)
      if (ack[i][0])
        printf("    %2i  %s\n", i, ack[i]);

    printf("\nFor a list of valid commands use: " PROGNAME " -l\n");
    exit(11);
}

static const char *ParamTypeName(char c)
{
  switch (c) {
    case 'i':
      return "16-bit integer";
    case 'l':
      return "32-bit integer";
    case 'f':
      return "16-bit float";
    case 'd':
      return "32-bit float";
    case 's':
      return "string";
  }

  return "unknown type";
}

static void PrintMCommand(int i)
{
  int j, k;

  printf("  %s - %s\n", client_mcommands[i].name, client_mcommands[i].about);
  for (j = 0; j < client_mcommands[i].numparams; j++) {
    printf("    param%02i: %s -- %s\n", j, client_mcommands[i].params[j].name,
        ParamTypeName(client_mcommands[i].params[j].type));
    if (client_mcommands[i].params[j].nt)
      for (k = 0; client_mcommands[i].params[j].nt[k]; ++k)
        printf("        %i == %s\n", k, client_mcommands[i].params[j].nt[k]);
  }
  printf("\n");
}

void CommandList(void)
{
  int i;

  NetCmdGetCmdList();

  printf("\nCommand List Serial: %s\n\n", client_command_list_serial);
  
  printf("Valid Multiword Commands reported by server:\n");

  for (i = 0; i < client_n_mcommands; i++)
    PrintMCommand(i);

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
    if (GroupNames[i]) printf("%d:\t%s\n", i, GroupNames[i]);
  }

  exit(1);
}

void CommandGroupListCommands(int groupnum)
{
  int i;

  if (groupnum >= N_GROUPS || groupnum < 0)
  {
    fprintf(stderr, "Invalid group number requested\n");
    exit(-1);
  }

  NetCmdGetCmdList();

  printf("Valid mcommands in group %d (%s):\n", groupnum, GroupNames[groupnum]);
  for (i = 0; i < client_n_mcommands; i++) {
    if (client_mcommands[i].group & (1 << groupnum))
      PrintMCommand(i);
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

  unsigned short *f3fa = (unsigned short *)buf;
  if (counter > 2000)
    *i_ack = 0x10;
  else if (*f3fa == 0xf3fa)
    *i_ack = (unsigned int)buf[2];
  else
    *i_ack = 0x0f;
}

void ConfirmSend() {
  char tmp[5];

  printf("Send this command? [Y, n] ");

  if (fgets(tmp, 5, stdin) == NULL && ! feof(stdin)) {
    perror("fgets");
    printf("\nError reading response.\nCommand aborted.\n\n");
    exit(1);
  }

  if (tmp[1] == 0 || tmp[0] == 'Y' || tmp[0] == 'y')
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
  printf(PROGNAME ": Error in multiword command parameter.\n\n");

  PrintMCommand(mcmd);
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
  buffer[4] = scommands[i_cmd].command & 0xff;
  buffer[5] = 0xa0 | ((scommands[i_cmd].command >> 8) & 0x0f);
  buffer[6] = 0x03;

  if ((tty_fd = bc_setserial()) < 0) {
    perror("Unable to open serial port");
    exit(2);
  }

  WriteBuffer(sock, tty_fd, buffer, 7, i_ack);

  close(tty_fd);
}

void SendMcommand(int sock, int i_cmd, int t_link, int t_route, char *parms[],
    int np, unsigned int *i_ack)
{
  unsigned short dataq[DATA_Q_SIZE];
  int dataqsize = 0;
  float flote, max, min;
  long long ynt;
  char type;
  int packet_length = 6;
  unsigned char buffer[255];
  unsigned short *dataqbuffer;
  int i;

  char output[1024];
  int tty_fd;

  /* for the validator */
  double rvalues[MAX_N_PARAMS];
  int ivalues[MAX_N_PARAMS];
  char svalues[MAX_N_PARAMS][CMD_STRING_LEN];

  if (np != mcommands[i_cmd].numparams) {
    *i_ack = 0x111;
    return;
  }

  for (i = 0; i < np; i++) {
    min = mcommands[i_cmd].params[i].min;
    max = mcommands[i_cmd].params[i].max;
    type = mcommands[i_cmd].params[i].type;
    if (type == 'i') {
      /* 16 bit integer parameter */
      ynt = atoi(parms[i]);
      if (ynt < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld < %g)\r\n",
            i + 1, ynt, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > USHRT_MAX) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld > %u)\r\n",
            i + 1, ynt, USHRT_MAX);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld > %g)\r\n",
            i + 1, ynt, max);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      }
       dataq[dataqsize++] = (unsigned short)(ynt - min);
       ivalues[i] = ynt;
    } else if (type == 'l') {
      /* 32 bit integer parameter */
      ynt = atoi(parms[i]);
      if (ynt < min) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld < %g)\r\n",
            i + 1, ynt, min);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > UINT_MAX) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld > %u)\r\n",
            i + 1, ynt, UINT_MAX);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      } else if (ynt > max) {
        sprintf(output, ":::limit:::parameter %d out of range (%lld > %g)\r\n",
            i + 1, ynt, max);
        *i_ack = 0x103;
        send(sock, output, sizeof(output), MSG_NOSIGNAL);
        return;
      }
      ynt -= min;
      dataq[dataqsize++] = ynt & 0x0000ffff;         /* lower 16 bits */
      dataq[dataqsize++] = (ynt & 0xffff0000) >> 16; /* upper 16 bits */
      ivalues[i] = ynt;
    } else if (type == 'f') {
      /* 16 bit floating point parameter */
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
      dataq[dataqsize++] = round((flote - min) * USHRT_MAX / (max - min)); 
      rvalues[i] = flote;
    } else if (type == 'd') {
      /* 32 bit floating point parameter */
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
      ynt = round((flote - min) * UINT_MAX / (max - min)); 
      dataq[dataqsize++] = (ynt & 0xffff0000) >> 16;  /* upper 16 bits */
      dataq[dataqsize++] = ynt & 0x0000ffff;          /* lower 16 bits */
      rvalues[i] = flote;
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
      strncpy(svalues[i], parms[i], CMD_STRING_LEN);
      svalues[i][CMD_STRING_LEN - 1] = 0;
    } else {
      snprintf(err_message, ERR_MESSAGE_LEN,
          "Invalid parameter type '%c' for parameter %i of multicommand: %s",
          type, i, mcommands[i_cmd].name);
      *i_ack = 0x110;
      return;
    }
  }

  /* Pass this command through the parameter validator */
  if (mcom_validate(mcommands[i_cmd].command, ivalues, rvalues, svalues,
        ERR_MESSAGE_LEN, err_message))
  {
    *i_ack = 0x113;
    return;
  }

  // make sure there are enough 'parameters' for extended commanding.
  for (;dataqsize<11; dataqsize++)
    dataq[dataqsize] = 0;
  
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
  buffer[4] = (mcommands[i_cmd].command & 0xff);
  buffer[5] = 0x80 | ((unsigned char)(mcommands[i_cmd].command>>8) & 0x0f);

  /* Send parameters */
  for (i = 0; i < dataqsize; i++) {
    //dataq[i] &= 0x7fff; /* first bit must be a zero */
    dataqbuffer = (unsigned short *)(buffer + packet_length);
    *dataqbuffer = dataq[i];
    packet_length += 2;
  }

  /* Send command footer */
  buffer[packet_length++] = mcommands[i_cmd].command;
  buffer[packet_length++] = 0xc0;
  buffer[packet_length++] = 0x3;
  buffer[3] = packet_length - 5;

  WriteBuffer(sock, tty_fd, buffer, packet_length, i_ack);

  close(tty_fd);
}

void WriteLogFile(int count, const char *token[], int i_ack)
{
  FILE *f;
  time_t t;
  int n;

  f = fopen(LOGFILE, "a");
  if (f == NULL) {
    printf(PROGNAME ": could not open log file %s\n", LOGFILE);
    return;
  }


  if (count > 0) {
    t = time(NULL);
    fprintf(f, "%s", ctime(&t));

    for(n = 0; n < count; n++)
      fprintf(f, "Sent: %s\n", token[n]);
  }

  if (i_ack >= 0) {
    fprintf(f, "Ack: (0x%02x) %s\n", i_ack, ack[i_ack]);
  }

  if (err_message[0]) /* parameter validation failed */
    fprintf(f, "Error: %s\n", err_message);

  fprintf(f, "\n");

  fclose(f);

  chmod(LOGFILE, S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH | S_IWOTH);
}

void PrintVersion(void)
{
  printf(PROGNAME " " VERSION "  (C) 2002-2014 D. V. Wiebe and many others\n"
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
  int daemon_route = 0, daemon_fork = 0, daemon_port = SOCK_PORT;
  char daemon_fw[2048] = {0};
  char *daemon_pilot[2] = {daemon_fw, NULL};
  int daemonise = 0;
  char* command[200];
  int nc = 0;
  int group = -1;
  char request[1024];
  int no_elog = 0;

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
    else if (strcmp(argv[i], "-pilot") == 0)
      t_link = 'P';
    else if (strcmp(argv[i], "-com1") == 0)
      t_route = '1';
    else if (strcmp(argv[i], "-com2") == 0)
      t_route = '2';
    else if  (strcmp(argv[i], "-noelog") == 0)
      no_elog = 1;
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
    else if (strcmp(argv[i], "-no") == 0) {
      if (i < (argc - 1)) {
        int link = argv[i + 1][0];
        int route = argv[i + 1][1];
        if ((link == 'T' || link == 'I' || link == 'P' || link == 'L')
            && (route == '1' || route == '2'))
        {
          link_disabled[link][route - '1'] = 1;
        } else
          USAGE(0);
      } else
        USAGE(0);
    } else if (strcmp(argv[i], "-fw") == 0) {
      if (i < (argc - 1)) {
        char *ptr;
        strcpy(daemon_fw, argv[i + 1]);
        for (ptr = daemon_fw; *ptr; ++ptr) {
          if (*ptr == ',') {
            *ptr = 0;
            daemon_pilot[1] = ptr + 1;
            break;
          }
        }
        if (daemon_pilot[1] == NULL)
          USAGE(0);
        i++;
      } else
        USAGE(0);
    } else if (strcmp(argv[i], "-p") == 0) {
      if (i < (argc - 1)) {
        daemon_port = atoi(argv[i + 1]);
        i++;
        if (daemon_port <= 0 || daemon_port > 65535)
          USAGE(0);
      } else
        USAGE(0);
    } else if (strcmp(argv[i], "--version") == 0)
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
    Daemonise(daemon_route, daemon_fork, daemon_port, daemon_pilot);

  if (daemon_route || daemon_fork || daemon_pilot[1] ||
      daemon_port != SOCK_PORT)
  {
    USAGE(0);
  }

  /* command given on comannd line */
  if (nc) {
    int ret = NetCmdConnect(host, silent, silent);
    if (ret)
      return ret;

    if (strcmp(command[0], "-l") == 0)
      CommandList();

    if (strcmp(command[0], "-lg") == 0)
      CommandGroupList();

    if (strcmp(command[0], "-lc") == 0)
      CommandGroupListCommands(group);

    if (no_elog) { // encode 'no elog' in caps/small state of t_link.
      t_link = tolower(t_link);
    }
    
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

    if (!silent)
      printf("Submitting %s\n\n", request);

    strcat(request, "\r\n");

    if (NetCmdTakeConn(silent)) {
      char message[1024];
      if (!silent)
        printf("Took the conn.\n");
      i = NetCmdSendAndReceive(request, silent, 1024, message);
      if (!silent) {
        printf("%s\n", ack[i]);
        if (message[0])
          printf("Error: %s\n", message);
      }
      return i;
    } else {
      fprintf(stderr, "Unable to take the conn.\n");
      return 14;
    }
  } else
    USAGE(0);

  fprintf(stderr, "Unexpected trap in main. Stop.\n");
  return -1;
}
