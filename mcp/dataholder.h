#ifndef DATAHOLDER_H
#define DATAHOLDER_H

#include <stdio.h>
#include <qlist.h>
#include <qstring.h>

struct DataStruct_glob {
  char name[25];
  char type;
  char src[15];
  int framefreq;      // Times per frame.
  char numbits;
  char overflowsize;
  int divider;
  bool forcediv;
  int samplefreq;
  int fp;
  char datatype;
  long long minval;
  long long maxval;

  int mindex;
  int chnum;
  int wide;
};

class AdamDom;

class DataHolder
{
public:
  DataHolder(char *filename);
  DataHolder();
  bool LoadFromXML(char *filename);
  struct DataStruct_glob *firstSlow();
  struct DataStruct_glob *nextSlow();
  struct DataStruct_glob *firstFast();
  struct DataStruct_glob *nextFast();

  int MaxBitRate;
  int LoopLength;
  int SampleRate;
  float minover;
  float maxover;

private:
  void GetXMLInfo(char *tagname, QList<struct DataStruct_glob> *dest, 
                  AdamDom *InfoFile);
  void GetPredefined(QString name, struct DataStruct_glob *dest,
			               AdamDom *InfoFile);
  void GetValue(char *attrib, int *dest, int def, AdamDom *InfoFile);
  void GetValue(char *attrib, char *dest, char def, AdamDom *InfoFile);
  void GetValue(char *attrib, bool *dest, bool def, AdamDom *InfoFile);
  void GetValue(char *attrib, long long *dest, long long def,
                AdamDom *InfoFile);

  AdamDom *InfoFile;
  QList<struct DataStruct_glob> SlowInfo;
  QList<struct DataStruct_glob> FastInfo;
  struct DataStruct_glob DefaultInfo;

  int numpredefs;
  char **predefs;
};

#endif
