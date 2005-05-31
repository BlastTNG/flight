/* decompoller: decomd TCP client code
 *
 * This software is copyright (C) 2004-2005 University of Toronto
 * 
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

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
    unsigned long FrameCounter(void);
    double DataQuality(void);
    double DiskFree(void);
    char* DecomFile(void);

  private:
    unsigned long long int df;
    unsigned long int frame_counter;
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
    DecomPoll();
    void start(const char*, int);
    virtual void run();

    int connectState;
    class DecomData *theDecom;
    bool pollDecomd;

  private:
    char decomdHost[MAXPATHLENGTH];
    int decomdPort;
};

#endif
