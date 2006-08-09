/* narsil: GUI commanding front-end
 *
 * This software is copyright (C) 2002-2004 University of Toronto
 * 
 * This file is part of narsil.
 * 
 * narsil is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * narsil is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with narsil; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

//|||***************************************************************************
//|||****
//|||****
//|||****     CLASS DoubleEntry -- a child class of Qt's TextEntry.  This is
//|||****          designed to work with doubles instead of just integers
//|||****          by tweaking some of the parent's functions.  Note: this
//|||****          is not a general class - it deals with narsil's defaults...
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************

//#include <qwidget.h>
#include "doubleentry.h"
#include "narsil.h"

//-------------------------------------------------------------
// DoubleEntry: constructor
//-------------------------------------------------------------

DoubleEntry::DoubleEntry(QWidget *parent, const char *name) : QLineEdit(parent, name) {
  command = 0;
  param = 0;
  SetMinMax(-1000000, 1000000);
}

//-------------------------------------------------------------
//
// RecordDefaults (public): fill defaults element
//
//-------------------------------------------------------------

void DoubleEntry::RecordDefaults() {
  QString v = value();
  defaults->Set(command, param, v);
}


//-------------------------------------------------------------
//
// SetMinMax (public): specify boundary values
//
//   mi: minumum value
//   ma: maximum value
//
//-------------------------------------------------------------

void DoubleEntry::SetMinMax(double mi, double ma) {
  rMin = mi;
  rMax = ma;
  iMin = (int)mi;
  iMax = (int)ma;
}

//-------------------------------------------------------------
//
// SetValue (public): use this function instead of the
//     parent's setValue as it will do the double conversion
//     for us
//
//   val: value to display
//
//-------------------------------------------------------------

void DoubleEntry::SetValue(double val) {
  if (type == 'i' || type == 'l') {
    if (val < iMin) val = iMin;
    if (val > iMax) val = iMax;
    setText(QString::number(val));
  } else {
    if (val < rMin) val = rMin;
    if (val > rMax) val = rMax;
    setText(QString::number(val));
  }
}

void DoubleEntry::SetStringValue(const char* str) { setText(str); }

void DoubleEntry::SetDefaultValue(int i, int j) {
  if (type == 'i' || type == 'l')
    setText(QString::number(defaults->asInt(i, j)));
  else if (type == 's')
    setText(defaults->asString(i, j));
  else
    setText(QString::number(defaults->asDouble(i, j)));
}

void DoubleEntry::SetType(char t) { type = t; }

//-------------------------------------------------------------
//
// SetParentField (public): assign this instance to a specific
//      command number and parameter number
//
//   com: command number
//   par: parameter number;
//
//-------------------------------------------------------------

void DoubleEntry::SetParentField(int com, int par) {
  command = com;
  param = par;
}

QString DoubleEntry::value() {
  if (type == 'i' || type == 'l') {
    int v = (int)text().toDouble();
    if (v < iMin) v = iMin;
    if (v > iMax) v = iMax;
    setText(QString::number(v));
  } else if (type == 's') {
    setText(text());
  } else {
    double v = text().toDouble();
    if (v < rMin) v = rMin;
    if (v > rMax) v = rMax;
    setText(QString::number(v));
  }
  return text();
}
