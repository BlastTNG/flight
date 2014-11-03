/* defile: converts BLAST-type framefiles into dirfiles
 *
 * This software is copyright (C) 2004-2005 D. V. Wiebe
 * 
 * This file is part of defile.
 * 
 * defile is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * defile is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with defile; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#ifdef HAVE_CONFIG_H
#  include "config.h"
#endif

#include <stdlib.h>       /* ANSI C std library (atoi) */
#include <arpa/inet.h>    /* IP4 specification (inet_aton, inet_ntoa) */
#include <pthread.h>      /* POSIX threads (pthread_exit) */
#include <signal.h>       /* ANSI C signals (SIG(FOO), sigemptyset, &c.) */
#include <string.h>       /* ANSI C strings (strcat, strdup, &c.)  */
#include <unistd.h>       /* UNIX std library (read, write, close, sleep) */

#include "crc.h"
#include "quenya.h"
#include "channels.h"
#include "defile.h"
#include "blast.h"

#define QUENDI_PORT 44144

void ClientDone(int signo) {
  const char quit[] = "QUIT\r\n";

  if (rc.csock > 0)
    write(rc.csock, quit, strlen(quit));

  if (rc.csock >= 0)
    close(rc.csock);
  if (rc.dsock > 0)
    close(rc.dsock);

  ReaderDone(signo);
}

void ClientReconnect(void)
{
  /* clean up stuff that InitClient will malloc */
  if (rc.hostname)
    free(rc.hostname);
  if (rc.dsock)
    close(rc.dsock);
  rc.dsock = 0;

  /* pause */
  sleep(1);
  
  if (InitClient(NULL))
    bprintf(fatal,
        "Cannot continue after server disconnect in initialisation");
}

int InitClient(char* new_filename)
{
  char buffer[2000];
  int n;
  char *ptr1 = NULL, *ptr2;
  FILE* stream;

  if (new_filename == NULL) {
    rc.csock = MakeSock();

    printf("Connecting to %s:%i...\n", inet_ntoa(rc.addr.sin_addr),
        ntohs(rc.addr.sin_port));

    if ((n = connect(rc.csock, (struct sockaddr*)&rc.addr, sizeof(rc.addr)))
        != 0)
      berror(fatal, "Connect failed");

    switch (n = GetServerResponse(rc.csock, buffer)) {
      case -3:
        return -1;
      case QUENYA_RESPONSE_SERVICE_READY:
        for (ptr1 = buffer; *ptr1 != ' '; ++ptr1);
        *(ptr1++) = 0;
        for (ptr2 = ptr1; *ptr2 != '/'; ++ptr2);
        *(ptr2++) = 0;
        *(strchr(ptr2, ' ')) = 0;

        bprintf(info, "Connected to %s on %s speaking quenya version %s.\n",
            ptr1, buffer, ptr2);

        rc.hostname = strdup(buffer);
        break;
      default:
        bprintf(fatal, "Unexpected response from server on connect: %i\n", n);
    }

    strcpy(buffer, "IDEN defile\r\n");
    write(rc.csock, buffer, strlen(buffer));
    switch (n = GetServerResponse(rc.csock, buffer)) {
      case -3:
        return -1;
      case QUENYA_RESPONSE_ACCESS_GRANTED:
        break;
      default:
        bprintf(fatal, "Unexpected response from server after IDEN: %i\n", n);
    }

    if (OpenDataPort(rc.csock, &rc.addr, &rc.dsock))
      return -1;

    strcpy(buffer, "QNOW\r\n");
    write(rc.csock, buffer, strlen(buffer));
    switch (n = GetServerResponse(rc.csock, buffer)) {
      case -3:
        return -1;
      case QUENYA_RESPONSE_DATA_STAGED:
        for (ptr1 = buffer; *ptr1 != ':'; ++ptr1);
        *(ptr1++) = 0;
        *(strchr(ptr1, ' ')) = 0;
        rc.chunk = bstrdup(fatal, ptr1);
        rc.resume_at = atoi(buffer);
        break;
      case QUENYA_RESPONSE_NO_CUR_DATA:
        /* 451 is returned by the server if reading the curfile fails -
           It usually means that either the curfile doesn't exist or
           the dirfile it points to doesn't exist */
        bprintf(fatal,
            "Can't fetch data from server: no current data is available.\n");
        break;
      default:
        bprintf(fatal, "Unexpected response from server after QNOW: %i\n", n);
    }

    new_filename = ptr1;
  }

  /* networked data already has the suffix stripped */
  rc.sufflen = 0;

  if (rc.output_dirfile != NULL)
    strncpy(rc.dirfile, rc.output_dirfile, FILENAME_LEN);
  else
    GetDirFile(rc.dirfile, new_filename, rc.dest_dir, rc.resume_at);

  rc.chunk = (char*)balloc(fatal, FILENAME_LEN);
  sprintf(rc.chunk, "%s:%s", rc.hostname, new_filename);

  strcpy(buffer, "SPEC\r\n");

  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(rc.csock, buffer)) {
    case -3:
      return -1;
    case QUENYA_RESPONSE_SENDING_SPEC:
      break;
    default:
      bprintf(fatal, "Unexpected response from server after SPEC: %i\n", n);
  }

  if ((stream = fdopen(rc.dsock, "w+")) == NULL)
    berror(fatal, "fdopen");

  ReadSpecificationFile(stream);

  switch (n = GetServerResponse(rc.csock, buffer)) {
    case -3:
      return -1;
    case QUENYA_RESPONSE_TRANS_COMPLETE:
      break;
    default:
      bprintf(fatal, "Unexpected response from server after SPEC/2: %i\n", n);
  }

  strcpy(buffer, "CLOS\r\n");
  write(rc.csock, buffer, strlen(buffer));
  switch (n = GetServerResponse(rc.csock, buffer)) {
    case -3:
      return -1;
    case QUENYA_RESPONSE_OK:
      break;
    default:
      bprintf(fatal, "Unexpected response from server after CLOS: %i\n", n);
  }

  fclose(stream);

  MakeAddressLookups(NULL);
  bprintf(info, "Frame size: %i bytes\n", DiskFrameSize);

  if (OpenDataPort(rc.csock, &rc.addr, &rc.dsock))
    return -1;

  return 0;
}

void QuenyaClient(void)
{
  unsigned short* InputBuffer[INPUT_BUF_SIZE];
  struct sigaction action;
  int i, n, block_size = 0, bytes_read;
  unsigned long long block_count = 0;
  unsigned short crc;
  char buffer[2000];
  char* ptr1;
  int do_reconnect = -1;

  /* set up signal masks */
  sigemptyset(&signals);
  sigaddset(&signals, SIGHUP);
  sigaddset(&signals, SIGINT);
  sigaddset(&signals, SIGTERM);

  /* set up signal handlers */
  action.sa_handler = ClientDone;
  action.sa_mask = signals;
  sigaction(SIGTERM, &action, NULL);
  sigaction(SIGHUP, &action, NULL);
  sigaction(SIGINT, &action, NULL);

  /* enable signals -- they were blocked in main before this thread was
   * spawned */
  pthread_sigmask(SIG_UNBLOCK, &signals, NULL);

  for (;;) {
    if (do_reconnect) {
      if (do_reconnect == 1) {
        if (rc.auto_reconnect) {
          free(InputBuffer[0]);

          ClientReconnect();

          if (rc.output_dirfile == NULL)
            ri.dirfile_init = 0;
        } else {
          /* If we're not in reconnect mode, terminate nicely */
          raise(SIGTERM);
          sleep(100);
        }
      }

      /* initialisation for reader */
      InputBuffer[0] = (unsigned short*)balloc(fatal, DiskFrameSize
          * INPUT_BUF_SIZE);

      for (i = 1; i < INPUT_BUF_SIZE; ++i)
        InputBuffer[i] = (void*)InputBuffer[0] + i * DiskFrameSize;

      do_reconnect = 0;

      /* Initiate data transfer */
      strcpy(buffer, "DATA\r\n");

    }

    /* Get the block header */
    write(rc.csock, buffer, strlen(buffer));
    switch (n = GetServerResponse(rc.csock, buffer)) {
      case -3:
        do_reconnect = 1;
        continue;
      case QUENYA_RESPONSE_SENDING_DATA:
        block_size = atoi(buffer);
        break;
      case QUENYA_RESPONSE_STAGED_NEXT:
        for (ptr1 = buffer; *ptr1 != ':'; ++ptr1);
        *(ptr1++) = 0;
        *(strchr(ptr1, ' ')) = 0;
        rc.chunk = bstrdup(fatal, ptr1);
        rc.resume_at = atoi(buffer);

        /* refetch SPEC file */
        if (InitClient(ptr1))
          bprintf(fatal,
              "Cannot continue after server disconnect in initialisation");

        /* if the dirfile has changed, signal the writer to cycle */
        if (rc.output_dirfile == NULL)
          ri.dirfile_init = 0;

        /* restart data transmission */
        strcpy(buffer, "DATA\r\n");

        /* skip the block read until data has restarted */
        continue;
      default:
        bprintf(fatal, "Unexpected response from server after DATA: %i\n", n);
    }
    bytes_read = 0;

    /* read the block */
    while (bytes_read < block_size * DiskFrameSize) {
      n = read(rc.dsock, (void*)InputBuffer[0] + bytes_read,
          block_size * DiskFrameSize - bytes_read);

      bytes_read += n;

#if DEBUG_FASTSAMP
      printf("Read %i bytes for block %lli, giving %i\n", n, block_count,
          bytes_read);
#endif
    }

#if DEBUG_FASTSAMP
    for (i = 1; i < bytes_read / 2; ++i) {
      if (InputBuffer[0][i - 1] == 0xeb90 || InputBuffer[0][i - 1] == 0x146F) {
        printf("Candidate fastsamp at byte %i: %lu\n", i,
            *(unsigned long*)(&InputBuffer[0][i]));
      }
    }
#endif

    /* Get the block footer */
    switch (n = GetServerResponse(rc.csock, buffer)) {
      case -3:
        do_reconnect = 1;
        continue;
      case QUENYA_RESPONSE_BLOCK_CRC:
        break;
      default:
        bprintf(fatal, "Unexpected response from server while waiting for CRC: "
            "%i\n", n);
    }

    sscanf(buffer, "0x%4hx Block CRC", &crc);
    int oc = crc;
    crc -= CalculateCRC(CRC_SEED, InputBuffer[0], bytes_read);

    if (crc != 0) {
      /* Reget block */
      strcpy(buffer, "RTBK\r\n");
      bprintf(err, "CRC checksum mismatch for block %lli.  Refetching block.\n",
          block_count);
      printf("%04x, %04x\n", oc, oc - crc);
    } else {
      /* Defile block */
      for (i = 0; i < block_size; ++i) {
        /* push frame */
        PushFrame(InputBuffer[i]);

        /* increment counter */
        ri.read++;
      }

      /* Get next block */
      strcpy(buffer, "CONT\r\n");
      block_count++;
    }
  }

  exit(1);
}
