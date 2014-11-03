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

#ifndef DOUBLEE_H
#define DOUBLEE_H

#include <qlineedit.h>

class DoubleEntry : public QLineEdit {
  Q_OBJECT
public:
  DoubleEntry(QWidget *parent, const char *name);
  void SetMinMax(double mi, double ma);
  void SetValue(double val);
  void SetStringValue(const char* str);
  void SetType(char);
  void SetDefaultValue(int, int);
  void SetParentField(int com, int par);
  QString value();
  void RecordDefaults();
private:
  int command, param;
  double rMin, rMax;
  int iMin, iMax;
  char type;
};

#endif
