/* palantir: BLAST status GUI
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of palantir.
 * 
 * palantir is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * palantir is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with palantir; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

// ***************************************************
// *  Programmed by Adam Hincks et al.               *
// *                                                 *
// *  Comments on classes & functions in .cpp file   *
// ***************************************************

#ifndef DECOMPOLL_H
#define DECOMPOLL_H

#include <qthread.h>

#define MAXPATHLENGTH 255

class DecomData
{
  public:
    DecomData();
    void setData(char*);
    int Status(void);
    double FrameLoss(void);
    double DataQuality(void);
    double DiskFree(void);
    char* DecomFile(void);

  private:
    unsigned long long int df;
    int status;
    int polarity;
    int decomUnlocks;
    double fs_bad;
    double dq_bad;
    char filename[256];
};

class DecomPoll : public QThread
{
  public:
    virtual void run();
};

extern char decomdHost[MAXPATHLENGTH];
extern int decomdPort;
extern bool pollDecomd;
extern int connectState;
extern class DecomData *theDecom;

#endif
