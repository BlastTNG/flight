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

extern double defaults[N_MCOMMANDS][MAX_N_PARAMS];


//-------------------------------------------------------------
//
// DoubleEntry: constructor
//
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
  defaults[command][param] = value();
  setText(QString::number(value()));
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
  Min = mi;
  Max = ma;
}


//-------------------------------------------------------------
//
// SetDoubleValue (public): use this function instead of the
//     parent's setValue as it will do the double conversion
//     for us
//
//   val: value to display
//
//-------------------------------------------------------------

void DoubleEntry::SetDoubleValue(double val) {
  if (val<Min) val = Min;
  if (val>Max) val = Max;
  setText(QString::number(val));
}


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

double DoubleEntry::value() {
  double v = text().toDouble();
  if (v<Min) v = Min;
  if (v>Max) v = Max;
  return(v);
}
