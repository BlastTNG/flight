/* Owl: BLAST status GUI
 *
 * This file is part of Owl.
 *
 * Owl is written by Joshua Netterfield (joshua.netterfield@gmail.com)
 * and is copyright (C) 2011 University of Toronto.
 *
 * It is a successor to Palantir, which was written by Adam Hincks, et al.
 *
 * Owl is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * Owl is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Owl; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <QApplication>
#include <time.h>
#include "PMainWindow.h"
#include "PDotPal.h"
#include <QTime>
#include <qjson/serializer.h>
#include <python2.6/Python.h>   //you may need to change this...

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);

  time_t seconds= time (NULL);
  qsrand((seconds*1000+QTime::currentTime().msec())%RAND_MAX);  //for concurrent ids. do not remove this line.

  Py_Initialize();

  PMainWindow* palantir=new PMainWindow;
  palantir->setWindowTitle(_WINDOW_TITLE_);
  palantir->show();

  app.exec();

  return 0;
}
