#ifndef ADAMDOM_H
#define ADAMDOML_H

#include <qdom.h>
#include <qfile.h>
#include <qlist.h>

class AdamDom
{
public:
  AdamDom(char *filename);
  AdamDom();
  bool LoadXML(char *filename);

  int CountEntries(QString entrylist, bool StartChild);
  int CountEntries(QString entrylist[], int numentries, bool StartChild);
	void AddBookMarks(unsigned int num);
	void SetBookMark(unsigned int index);
	void GoBookMark(unsigned int index);
	void GotoNextSib();
	void GotoFirstChild();
  bool NullEntry();
	bool GotoEntry(QString entrylist, int num, bool StartChild);
  bool GotoEntry(QString *entrylist, int numentries, int num, bool StartChild);
  bool GotoTag(QString entrylist, char *attrib, char *val, bool StartChild);
  QString GetAttribute(char *attrib);
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
