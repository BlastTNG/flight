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
#include <iostream>
#ifdef __APPLE__
#include <python2.6/Python.h>
#else
#include <python2.7/Python.h>   //you may need to change this...
#endif

void usage(QString appname) {
  std::cout<<"usage: "<<qPrintable(appname)<<"[--webkey <key>] [--new | <filename>]"<<std::endl;
    exit(1);
}

int main(int argc, char* argv[]) {
  QApplication app(argc, argv);
  QString webkey;

  QString filename("__lastfile");

  if (app.arguments().size() > 4) {
      usage(app.arguments()[0]);
  }

  for (int i_arg=1; i_arg<app.arguments().size(); i_arg++) {
    if (app.arguments().at(i_arg)=="--webkey") {
      webkey = app.arguments().at(i_arg+1);
      i_arg++;
    } else if (app.arguments().at(i_arg) == "--new") {
      filename.clear();
    } else if (app.arguments().at(i_arg).startsWith('-')) {
      usage(app.arguments()[0]);
    } else {
      filename = app.arguments().at(i_arg);
    }
  }

  time_t seconds= time (NULL);
  qsrand((seconds*1000+QTime::currentTime().msec())%RAND_MAX);  //for concurrent ids. do not remove this line.

  Py_Initialize();
  PMainWindow::key = webkey;
  new PMainWindow(filename);

  app.exec();

  return 0;
}
