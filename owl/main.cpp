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
 * owl is also used by BIT!! 
 */

#include <QApplication>
#include <QStyleFactory>
#include <time.h>
#include "PMainWindow.h"
#include <QTime>
#include <iostream>
#include <QDebug>
#ifdef __APPLE__
#include <python2.6/Python.h>
#else
#include <python2.7/Python.h>   // Replace python2.7 with your version of Python
#endif

void usage(QString appname) {
  std::cout<<"usage: "<<qPrintable(appname)<<" [--fs <fontsize>] [--new | <filename>]"<<std::endl;
  exit(1);
}

int main(int argc, char* argv[]) {

  QApplication app(argc, argv);

  QString filename("__lastfile");

  app.setStyle(QStyleFactory::create("GTK+"));
  app.setWindowIcon(QIcon(":icons/icons/Owl0.svg"));


  if (app.arguments().size() > 6) {
      usage(app.arguments()[0]);
  }

  int font_size = app.font().pointSize();

  for (int i_arg=1; i_arg<app.arguments().size(); i_arg++) {
    if (app.arguments().at(i_arg) == "--new") {
      filename.clear();
    } else if (app.arguments().at(i_arg) == "--fs") {
      if (++i_arg <app.arguments().size()) {
        font_size = app.arguments().at(i_arg).toInt();
      } else {
        usage(app.arguments()[0]);
      }
    } else if (app.arguments().at(i_arg).startsWith('-')) {
      usage(app.arguments()[0]);
    } else {
      filename = app.arguments().at(i_arg);
    }
  }

  time_t seconds= time (NULL);
  qsrand((seconds*1000+QTime::currentTime().msec())%RAND_MAX); // For unique ids.

  Py_Initialize();
  PMainWindow *mw = new PMainWindow(font_size, filename);

  if (filename.isEmpty()) {
    mw->setMDIMinSize(1900, 1200);
  }

  app.exec();

  return 0;
}
