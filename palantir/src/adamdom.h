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

#ifndef ADAMDOM_H
#define ADAMDOML_H

#include <qdom.h>
#include <qfile.h>
#include <qptrlist.h>

class AdamDom
{
public:
  AdamDom(char *filename);
  AdamDom();
  bool LoadXML(char *filename);

  int CountEntries(QString entrylist, bool StartChild);
  int CountEntries(QString entrylist[], int numentries, bool StartChild);
	void AddBookMarks(int num);
	void SetBookMark(int index);
	void GoBookMark(int index);
	void GotoNextSib();
	void GotoFirstChild();
  bool NullEntry();
	bool GotoEntry(QString entrylist, int num, bool StartChild);
  bool GotoEntry(QString *entrylist, int numentries, int num, bool StartChild);
  bool GotoTag(QString entrylist, char *attrib, char *val, bool StartChild);
  QString GetAttribute(const char *attrib);
  QString GetTagName();

private:
  int RealCountEntries(QDomNode Node, QString *entrylist, int numentries,
			                 QString parents);
  int RealGotoEntry(QDomNode Node, QString *entrylist, int numentries,
			              QString parents, int num, int counter);
  bool RealGotoTag(QDomNode Node, QString entrylist, char *attrib,
			             QString val, QString parents);
  float QStringToFloat(QString str);
  int QStringToInt(QString str);
  bool QStringToBool(QString str);
  QDomNode StartNode(QDomElement e);
  QDomElement NextElement(QDomNode *n);

  QDomDocument *layout;
  QDomElement currElem;
	QList<QDomElement> BookMarks;
};

#endif
