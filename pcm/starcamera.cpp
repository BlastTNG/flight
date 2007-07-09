/* mcp: the Spider master control program
 *
 * starcamera.cpp: star camera control and readout functions
 *     this file is a logical part of tx.c
 *     uses cpp to make calling class functions easier
 *
 * This software is copyright (C) 2002-2007 University of Toronto
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

#include <stdio.h>
#include <limits.h>
#include <sys/time.h>
#include <math.h>
#include <pthread.h>
#include <string>

extern "C" {
#include "blast.h"
#include "tx.h"
}
#include "channels.h"
#include "pointing_struct.h"
#include "mcp.h"
#include "camcommunicator.h"
#include "camstruct.h"
#include "command_struct.h"

//allow any host to be the star camera
#define CAM_SERVERNAME "any"

static CamCommunicator* camComm;
static pthread_t camcomm_id;

static void* camReadLoop(void* arg);
static string parseReturn(string rtnStr);

extern "C" {

/*
 * open a connection the the star camera computer
 * also creates the communications thread
 */
void openCamera()
{
  camComm = new CamCommunicator();
  pthread_create(&camcomm_id, NULL, &camReadLoop, NULL);
}

int sendCamCommand(const char *cmd)
{
  //this is okay unless I want to handle link dying during transmission
  return camComm->sendCommand(cmd);
}

}       //extern "C"

/*
 * wrapper for the read loop in CamCommunicator
 * mcp main should make a thread for this
 */
static void* camReadLoop(void* arg)
{
  if (camComm->openHost(CAM_SERVERNAME) < 0)
    berror(err, "Starcam: failed to accept star camera connection");
  else bprintf(startup, "Starcam: talking to star camera");

  while(true) {
    camComm->readLoop(&parseReturn);
    //readLoop returns when something bad happens; TODO handle
  }

  return NULL;
}

/*
 * function used be readloop that handles strings returned from the camera
 * it will write data to frames and log errors
 */
static string parseReturn(string rtnStr)
{
  //TODO this is temporary
  bprintf(info, "Starcam: return string: %s", rtnStr.c_str());
  return "";  //doesn't send a response back to camera
}

