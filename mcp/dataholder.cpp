// **********************************************************
// *                                                        *
// *  Programmed by Adam Hincks                             *
// *                                                        *
// *  See also adamdom.cpp, which has a class used by this  *
// *  class.  The vocabulary (e.g., "tag", "attribute" used *
// *  in this file is explained there.                      *
// *                                                        *
// **********************************************************



//|||****_______________________________________________________________________
//|||***************************************************************************
//|||****
//|||**** 
//|||****     CLASS Buffer -- Makes use of AdamDom class to get information from
//|||****                     XML file, store it, and give it out when requested
//|||****
//|||****
//|||****_______________________________________________________________________
//|||***************************************************************************

#define DIFFERENTIAL   0
#define INT_PRESERVING 1
#define SINGLE         2
#define AVERAGE        3

#define NUM_BM         10
#define BM_FIRST       0
#define BM_RESERVE     9

#include <stdio.h>
#include <stdlib.h>
#include "adamdom.h"
#include "dataholder.h"


//-------------------------------------------------------------
// 
// DataHolder: constructors
//
//-------------------------------------------------------------

DataHolder::DataHolder(char *filename) {
  InfoFile = new AdamDom();

  SlowInfo.setAutoDelete(true);
  FastInfo.setAutoDelete(true);
  LoadFromXML(filename);
}

DataHolder::DataHolder() {
  InfoFile = new AdamDom();

  SlowInfo.setAutoDelete(true);
  FastInfo.setAutoDelete(true);
}


//-------------------------------------------------------------
// 
// LoadFromXML (public): read in contents of XML file
//
//   *filename: XML file
//
//   Returns: true if the file is read, false if it is not
//
//-------------------------------------------------------------

bool DataHolder::LoadFromXML(char *filename) {
  int i;
  
  if (!InfoFile->LoadXML(filename))
    return false;

  SlowInfo.clear();
  FastInfo.clear();

  // Get important variables about nature of downlink
  InfoFile->GotoEntry(".SETTINGS.GLOBALS", 0, false);
	// Maximum TDRSS downlink rate
  MaxBitRate = atoi(InfoFile->GetAttribute("maxbitrate"));
	// Length of the frame (seconds)
  LoopLength = atoi(InfoFile->GetAttribute("looplength"));
	// Rate at which mcp writes frames (Hz)
  SampleRate = atoi(InfoFile->GetAttribute("samplerate"));

  // Get variables about the nature of the compression
  InfoFile->GotoEntry(".SETTINGS.COMPRESSION", 0, false);
  maxover = atof(InfoFile->GetAttribute("maxover"));
  minover = atof(InfoFile->GetAttribute("minover"));

  // Set defaults
  DefaultInfo.framefreq = 1;
  DefaultInfo.numbits = 4;
  DefaultInfo.overflowsize = 4;
  DefaultInfo.divider = 2;
  DefaultInfo.forcediv = false;
  DefaultInfo.samplefreq = 1;
  DefaultInfo.minval = -2147483648LL;
  DefaultInfo.maxval = 4294967295LL;

  InfoFile->AddBookMarks(NUM_BM);

  i = InfoFile->CountEntries(".SETTINGS.PREDEFINED", false);
  InfoFile->AddBookMarks(i);
  predefs = (char **)malloc(i * sizeof(char *));
  numpredefs = i;
  InfoFile->GotoEntry(".SETTINGS", 0, false);
  i = 0;
  for (InfoFile->GotoFirstChild(); !InfoFile->NullEntry();
       InfoFile->GotoNextSib()) {
    if (InfoFile->GetTagName() == "PREDEFINED") {
      InfoFile->SetBookMark(NUM_BM + i);
      predefs[i] = (char *)malloc(35 * sizeof(char));
      strcpy(predefs[i], InfoFile->GetAttribute("name"));
      i++;
    }
  }
    
  GetXMLInfo(".SLOWDATA", &SlowInfo, InfoFile);
  GetXMLInfo(".FASTDATA", &FastInfo, InfoFile);

  return true;
}


//-------------------------------------------------------------
// 
// first, next (public): the information from the XML file is
//      stored in a QList of structures.  Calling functions can
//      obtain a pointer to a specific field by using these
//      functions
//
//   Returns: pointer to struct
//
//-------------------------------------------------------------

struct DataStruct_glob * DataHolder::firstSlow() {
  return SlowInfo.first();
}

struct DataStruct_glob * DataHolder::nextSlow() {
  return SlowInfo.next();
}

struct DataStruct_glob * DataHolder::firstFast() {
  return FastInfo.first();
}

struct DataStruct_glob * DataHolder::nextFast() {
  return FastInfo.next();
}


//-------------------------------------------------------------
// 
// GetXMLInfo (private): parses out information polled from
//      the InfoFile object (containing XML DOM information)
//
//   *dest: QList to fill
//   InfoFile: AdamDom object   
//
//-------------------------------------------------------------


void DataHolder::GetXMLInfo(char *tagname, QList<struct DataStruct_glob> *dest,
		                        AdamDom *InfoFile) {
  struct DataStruct_glob *currDatumInfo;
  char tmp[30];

	InfoFile->GotoEntry(tagname, 0, false);
  for (InfoFile->GotoFirstChild(); !InfoFile->NullEntry();
			 InfoFile->GotoNextSib()) {
    InfoFile->SetBookMark(BM_FIRST);
    dest->append(new DataStruct_glob);
    currDatumInfo = dest->current();
    
    if (InfoFile->GetTagName() == "DIFF")
      currDatumInfo->type = DIFFERENTIAL;
    if (InfoFile->GetTagName() == "INT")
      currDatumInfo->type = INT_PRESERVING;
    if (InfoFile->GetTagName() == "SINGLE")
      currDatumInfo->type = SINGLE;
    if (InfoFile->GetTagName() == "AVG")
      currDatumInfo->type = AVERAGE;

    strcpy(tmp, InfoFile->GetTagName());

    strcpy(currDatumInfo->src, InfoFile->GetAttribute("src"));
    strcpy(currDatumInfo->name, InfoFile->GetAttribute("name"));

    GetPredefined(InfoFile->GetAttribute("predefined"), currDatumInfo,
				          InfoFile);

    GetValue("perframe", &(currDatumInfo->framefreq), DefaultInfo.framefreq,
				     InfoFile);
    GetValue("numbits", &(currDatumInfo->numbits), DefaultInfo.numbits,
				     InfoFile);
    GetValue("overbits", &(currDatumInfo->overflowsize),
				     DefaultInfo.overflowsize, InfoFile);
    GetValue("divider", &(currDatumInfo->divider), DefaultInfo.divider,
				     InfoFile);
    GetValue("forcediv", &(currDatumInfo->forcediv), DefaultInfo.forcediv,
				     InfoFile);
    GetValue("samplefreq", &(currDatumInfo->samplefreq), DefaultInfo.samplefreq,
				     InfoFile);
    GetValue("minval", &(currDatumInfo->minval), DefaultInfo.minval, InfoFile);
    GetValue("maxval", &(currDatumInfo->maxval), DefaultInfo.maxval, InfoFile);
		InfoFile->GoBookMark(BM_FIRST);
  }
}


//-------------------------------------------------------------
// 
// GetValue (private): try to read a value from current tag
//
//   *attrib: value attribute name
//   *dest: where to write value to
//   def: default value if nothing is read
//   *InfoFile: AdamDom object
//
//-------------------------------------------------------------

void DataHolder::GetValue(char *attrib, int *dest, int def, AdamDom *InfoFile) {
  if (InfoFile->GetAttribute(attrib) != "")
    *dest = atoi(InfoFile->GetAttribute(attrib));
  else if (*dest == -1)
    *dest = def;
}

void DataHolder::GetValue(char *attrib, long long *dest, long long def,
                          AdamDom *InfoFile) {
  if (InfoFile->GetAttribute(attrib) != "")
    *dest = atoll(InfoFile->GetAttribute(attrib));
  else if (*dest == -1)
    *dest = def;
}

void DataHolder::GetValue(char *attrib, char *dest, char def,
		                      AdamDom *InfoFile) {
  if (InfoFile->GetAttribute(attrib) != "")
    *dest = atoi(InfoFile->GetAttribute(attrib));
  else if (*dest == -1)
    *dest = def;
}

void DataHolder::GetValue(char *attrib, bool *dest, bool def,
		                      AdamDom *InfoFile) {
  if (InfoFile->GetAttribute(attrib) != "") {
    if (InfoFile->GetAttribute(attrib) == "True" || InfoFile->GetAttribute(attrib) == "true")
      *dest = true;
    else
      *dest = false;
  }
  else if (*dest == false && def == true)
    *dest = true;
}


//-------------------------------------------------------------
// 
// GetPredefined (private): search through "PREDEFINED" tags
//      for the one with "name" and get its attributes
//
//   name: value of attribute "name"
//   *dest: where to write the info to
//   *InfoFile: AdamDom object
//
//-------------------------------------------------------------


void DataHolder::GetPredefined(QString name, struct DataStruct_glob *dest,
		                           AdamDom *InfoFile) {
  char predefname[35];
  int i;

	InfoFile->SetBookMark(BM_RESERVE);
  
	dest->framefreq = -1;
  dest->numbits = -1;
  dest->overflowsize = -1;
  dest->divider = -1;
  dest->forcediv = false;
  dest->samplefreq = -1;
  dest->minval = -1;
  dest->maxval = -1;
  
  strcpy(predefname, name);
  for (i = 0; i < numpredefs; i++) {
    if (strcmp(predefname, predefs[i]) == 0) {
      InfoFile->GoBookMark(i + NUM_BM);
      GetValue("perframe", &(dest->framefreq), -1, InfoFile);
      GetValue("numbits", &(dest->numbits), -1, InfoFile);
      GetValue("overbits", &(dest->overflowsize), -1, InfoFile);
      GetValue("divider", &(dest->divider), -1, InfoFile);
      GetValue("forcediv", &(dest->forcediv), false, InfoFile);
      GetValue("samplefreq", &(dest->samplefreq), -1, InfoFile);
      GetValue("minval", &(dest->minval), -1, InfoFile);
      GetValue("maxval", &(dest->maxval), -1, InfoFile);
      
      InfoFile->GoBookMark(BM_RESERVE);
      return;
    }
  }

  return;
}
