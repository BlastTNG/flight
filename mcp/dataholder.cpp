/*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*\
|*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
|*()                                                                        ()*|
|*() DATAHOLDER.CPP                                                         ()*|
|*()                                                                        ()*|
|*() Information on channels being compressed/decompressed by big/small.    ()*|
|*() Both big and small require this file to work in conjunction with each  ()*|
|*() other.  This file includes als the class AMLParser which parses AML    ()*|
|*() files containing channel information for big/small.                    ()*|
|*()                                                                        ()*|
|*() Adam Hincks, Summer 2004, Toronto                                      ()*|
|*()                                                                        ()*|
|*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
\*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dataholder.h"
#include "alice.h"


/******************************************************************************\
|*                                                                            *|
|* trim: remove leading and trailing spaces and new line characters           *|
|*                                                                            *|
|* *buf     - buffer to trim                                                  *|
|*                                                                            *|
\******************************************************************************/

void trim(char *buf) {
  int i;
  char tempbuf[strlen(buf)];

  for (i = 0; i < (signed int)strlen(buf); i++) {
    if (buf[i] != ' ' && buf[i] != '\n')
      break;
  }

  strcpy(tempbuf, buf + i);

  for (i = strlen(tempbuf) - 1; i >= 0; i--) {
    if (tempbuf[i] == ' ' || tempbuf[i] == '\n')
      tempbuf[i] = '\0';
    else
      break;
  }

  strcpy(buf, tempbuf);
}


/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                              CLASS AMLParser                           |**|
|**|                                                                        |**|
|**|                Parse AML (Alice Markup-Like Language) files.           |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/


/******************************************************************************\
|*                                                                            *|
|* AMLParser:  constructor                                                    *|
|*                                                                            *|
\******************************************************************************/

AMLParser::AMLParser(const char *filename) {
  CommonConstructor();
  LoadFile(filename);

  return;
}

AMLParser::AMLParser() {
  CommonConstructor();

  return;
}

void AMLParser::CommonConstructor() {
  numentries = 0;
  maxvalues = 0;
  maxdata = 0;
  currentry = -1;
  currdatum = -1;
  
  return;
}


/******************************************************************************\
|*                                                                            *|
|* LoadFile (public): load in an AML file and parse it.                       *|
|*                                                                            *|
|* *filename    - file to load                                                *|
|*                                                                            *|
|* Returns: true on successful load                                           *|
|*                                                                            *|
\******************************************************************************/

bool AMLParser::LoadFile(const char *filename) {
  FILE *f;
  int i, j, k;
  int entrycount;
  char linebuf[AML_LEN_LINE], tmpstr[AML_LEN_LINE], tmpstr2[AML_LEN_LINE];

  if ((f = fopen(filename, "r")) == NULL) {
    printf("AMLParser: couldn't open %s for openning.  Returning.\n", filename);
    return false;
  }

  // First time through, check for number of entries and maximum number of
  // values.
  i = 0;
  while (fgets(linebuf, AML_LEN_LINE, f) != NULL) {
    trim(linebuf);
    if (linebuf[0] == '+') {
      numentries++;
      if (GetNumValues(linebuf) > maxvalues)
        maxvalues = GetNumValues(linebuf);
      i = 0;
    }
    
    if (linebuf[0] == '-') {
      if (++i > maxdata)
        maxdata = i;
    }
  }

  // Allocate memory.
  entries = (char ****)malloc(numentries * sizeof(char ***));
  valuenames = (char ***)malloc(numentries * sizeof(char **));
  datanames = (char ***)malloc(numentries * sizeof(char **));
  datadefaults = (char ***)malloc(numentries * sizeof(char **));
  entrynames = (char **)malloc(numentries * sizeof(char *));
  level = (int *)malloc(numentries * sizeof(int));
  numvalues = (int *)malloc(numentries * sizeof(int));
  numdata = (int *)malloc(numentries * sizeof(int));
  parent = (int *)malloc(numentries * sizeof(int));
  for (i = 0; i < numentries; i++) {
    entries[i] = (char ***)malloc(maxvalues * sizeof(char **));
    valuenames[i] = (char **)malloc(maxvalues * sizeof(char *));
    datanames[i] = (char **)malloc(maxvalues * sizeof(char *));
    datadefaults[i] = (char **)malloc(maxvalues * sizeof(char *));
    entrynames[i] = (char *)malloc(AML_LEN_ENTRY * sizeof(char));
    level[i] = 0;
    numvalues[i] = 0;
    numdata[i] = 0;
    parent[i] = -1;
    for (j = 0; j < maxvalues; j++) {
      entries[i][j] = (char **)malloc(maxdata * sizeof(char *));
      valuenames[i][j] = (char *)malloc(AML_LEN_ENTRY * sizeof(char));
      for (k = 0; k < AML_LEN_ENTRY; k++)
        entries[i][j][k] = (char *)malloc(AML_LEN_ENTRY * sizeof(char));
    }
    for (j = 0; j < maxdata; j++) {
      datanames[i][j] = (char *)malloc(AML_LEN_ENTRY * sizeof(char));
      datadefaults[i][j] = (char *)malloc(AML_LEN_ENTRY * sizeof(char));
    }
  }

  // Now fill the array entries from the file.
  rewind(f);
  j = 0;
  entrycount = -1;
  for (i = 0; fgets(linebuf, AML_LEN_LINE, f) != NULL; i++) {
    trim(linebuf);

    switch(linebuf[0]) {
      case '+':
        entrycount++;
        for (j = 0; j < (signed int)strlen(linebuf) && linebuf[j] == '+'; j++);
        level[entrycount] = j;
        
        for (k = entrycount; k >= 0; k--) {
          if (level[k] == j - 1) {
            parent[entrycount] = k;
            break;
          }
        }

        numvalues[entrycount] = GetNumValues(linebuf);
        for (k = 0; k < numvalues[entrycount]; k++)
          GetValue(linebuf, k, valuenames[entrycount][k]);

        GetEntryName(linebuf, entrynames[entrycount]);
        ParseFullName(entrycount, tmpstr);

        for (k = 0; k < entrycount; k++) {
          ParseFullName(k, tmpstr2);
          if (!strcmp(tmpstr, tmpstr2)) {
            printf("AMLParser: found two instances of '%s' in the file '%s'.  "
                   "Using the first.\n", tmpstr, filename);
            entrycount--;
          }
        }
        break;

      case '-':
        j = 1;
        GetEntryName(linebuf, datanames[entrycount][numdata[entrycount]]);
        GetDataDefault(linebuf, datadefaults[entrycount][numdata[entrycount]]);
        for (i = 0; i < numdata[entrycount]; i++) {
          if (!strcmp(datanames[entrycount][numdata[entrycount]],
                      datanames[entrycount][i])) {
            ParseFullName(entrycount, tmpstr);
            printf("AMLParser: found two instances of '%s' in entry '%s' in "
                   "the file '%s'.  Using the first.\n", 
                   datanames[entrycount][i], tmpstr, filename);
            j = 0;
          }
        }
        
        if (!j)
          break;
        
        for (i = 0; i < numvalues[entrycount]; i++) {
          if (!GetValue(linebuf, i, 
                        entries[entrycount][i][numdata[entrycount]])) {
            ParseFullName(entrycount, tmpstr);
            printf("AMLParser: couldn't get datum for '%s' in column '%s' in "
                   "entry '%s' in the file '%s'.  Ignoring this line.\n",
                   datanames[entrycount][numdata[entrycount]], 
                   valuenames[entrycount][0], tmpstr, filename);
            j = 0;
            break;
          }
        }

        if (!j)
          break;
        numdata[entrycount]++;
        
        break; 
    }
  }
  
  return true;
}


/******************************************************************************\
|*                                                                            *|
|* ParseFullName (private): returns the full name of an entry                 *|
|*                                                                            *|
|* num          - index number of the entry                                   *|
|* *namebuf     - buffer in which to return the name                          *|
|*                                                                            *|
|* Returns: number of values                                                  *|
|*                                                                            *|
\******************************************************************************/


void AMLParser::ParseFullName(int num, char *namebuf) {
  int i;
  char tmpstr[AML_LEN_LINE];

  strcpy(namebuf, "");
  i = num;

  do {
    sprintf(tmpstr, "%s.%s", entrynames[i], namebuf);
    strcpy(namebuf, tmpstr);
    i = parent[i];
  } while (i >= 0);

  if (strlen(namebuf))
    namebuf[strlen(namebuf) - 1] = '\0';

  return;
}

/******************************************************************************\
|*                                                                            *|
|* GetNumValues (private): counts the number of values on a line of an AML    *|
|* file.                                                                      *|
|*                                                                            *|
|* *buf     - line of the file                                                *|
|*                                                                            *|
|* Returns: number of values                                                  *|
|*                                                                            *|
\******************************************************************************/

int AMLParser::GetNumValues(const char *buf) {
  int i, ret;
  char lastval;

  if (buf[0] != '+')  // Must be a valid line.
    return 0;

  lastval = '+';
  ret = 0;
  
  for (i = 1; i < (signed int)strlen(buf); i++) {
    if (buf[i] == ' ' && lastval != ' ')
      ret++;
    lastval = buf[i];
  }
 
  return ret;
}


/******************************************************************************\
|*                                                                            *|
|* GetEntryName (private): get the name of an entry in an AML file            *|
|*                                                                            *|
|* *buf     - line of the file                                                *|
|* *name   - buffer for return of value                                       *|
|*                                                                            *|
\******************************************************************************/

void AMLParser::GetEntryName(const char *buf, char *namebuf) {
  int i, j;

  for (i = 0; i < (signed int)strlen(buf) && (buf[i] == '+' || buf[i] == '-'); 
       i++);

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' ' && buf[j] != '@'; 
       j++)
    namebuf[j - i] = buf[j];
  namebuf[j - i] = '\0';
  
  return;
}


/******************************************************************************\
|*                                                                            *|
|* GetEntryName (private): get the name of an entry in an AML file            *|
|*                                                                            *|
|* *buf     - line of the file                                                *|
|* *name   - buffer for return of value                                       *|
|*                                                                            *|
\******************************************************************************/

void AMLParser::GetDataDefault(const char *buf, char *defaultbuf) {
  int i, j;

  for (i = 0; i < (signed int)strlen(buf) && (buf[i] == '+' || buf[i] == '-'); 
       i++);

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' ' && buf[j] != '@'; 
       j++);
 
  if (buf[j] != '@') {
    defaultbuf[0] = '\0'; 
    return;
  }

  i = j + 1;
  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' '; j++)
    defaultbuf[j - i] = buf[j];
  defaultbuf[j - i] = '\0';
  
  return;
}
 

/******************************************************************************\
|*                                                                            *|
|* GetValue (private): get a value from a list on a line of an AML file       *|
|*                                                                            *|
|* *buf     - line of the file                                                *|
|* num      - entry number                                                    *|
|* *value   - buffer for return of value                                      *|
|*                                                                            *|
|* Returns: false if value <num> does not exist; true otherwise               *|
|*                                                                            *|
\******************************************************************************/

bool AMLParser::GetValue(const char *buf, int num, char *valuebuf) {
  int i, j, count;
  char lastval;

  if (!(buf[0] == '+' || buf[0] == '-'))  // Must be a valid line.
    return false;
  
  lastval = '+';
  count = -1;
  
  for (i = 1; i < (signed int)strlen(buf); i++) {
    if (buf[i] == ' ' && lastval != ' ') {
      if (++count == num)
        break;
    }
    lastval = buf[i];
  }

  if (i == (signed int)strlen(buf))
    return false;

  for (; i < (signed int)strlen(buf) && buf[i] == ' '; i++);
  
  if (i == (signed int)strlen(buf))
    return false;

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' '; j++) 
    valuebuf[j - i] = buf[j];
  valuebuf[j - i] = '\0';
  
  return true;
}


/******************************************************************************\
|*                                                                            *|
|* FirstDatum (public):  go to the first datum for the requested entry.       *|
|*                                                                            *|
|* *fullentryname   - the name of the entry, in the format LEVEL1.LEVEL2...   *|
|*                                                                            *|
|* Returns: false if the entry does not exist or the entry contains no data   *|
|*                                                                            *|
\******************************************************************************/

bool AMLParser::FirstDatum(const char *fullentryname) {
  int i;
  char tmpstr[AML_LEN_LINE];

  for (i = 0; i < numentries; i++) {
    ParseFullName(i, tmpstr);
    if (!strcmp(fullentryname, tmpstr)) {
      if (numdata[i]) {
        currentry = i;
        currdatum = 0;
        return true;
      }
      else {
        currentry  = -1;
        currdatum = -1;
        return false;
      }
    }
  }

  currentry = -1;
  currdatum = -1;
  return false;
}


/******************************************************************************\
|*                                                                            *|
|* NumData (public):  get the number of data for the current entry.           *|
|*                                                                            *|
|* *fullentryname   - the name of the entry, in the format LEVEL1.LEVEL2...   *|
|*                                                                            *|
|* Returns: the number of rows of data for the current entry                  *|
|*                                                                            *|
\******************************************************************************/

int AMLParser::NumData(const char *fullentryname) {
  int i;
  char tmpstr[AML_LEN_LINE];

  for (i = 0; i < numentries; i++) {
    ParseFullName(i, tmpstr);
    if (!strcmp(fullentryname, tmpstr))
      return numdata[i];
  }

  return 0;
}


/******************************************************************************\
|*                                                                            *|
|* NextDatum (public):  go to the next datum for the current entry.           *|
|*                                                                            *|
|* Returns: false if there are no more data for the current entry.            *|
|*                                                                            *|
\******************************************************************************/

bool AMLParser::NextDatum() {
  if (currentry < 0 || currdatum < 0)
    return false;

  if (++currdatum >= numdata[currentry]) {
    currentry = -1;
    currdatum = -1;
    return false;
  }

  return true;
}


/******************************************************************************\
|*                                                                            *|
|* NoDatum (public):  check to see if the current datum is valid  .           *|
|*                                                                            *|
|* Returns: true if current datum is not valid                                *|
|*                                                                            *|
\******************************************************************************/

bool AMLParser::NoDatum() {
  if (currentry < 0 || currdatum < 0)
    return true;
  else
    return false;
}


/******************************************************************************\
|*                                                                            *|
|* Value (public):  return the value of the current datum for the requested   *|
|* column                                                                     *|
|*                                                                            *|
|* *valuename   - the column name                                             *|
|*                                                                            *|
|* Returns: the datum, or NULL if it does not exist                           *|
|*                                                                            *|
\******************************************************************************/

const char *AMLParser::Value(const char *valuename) {
  int i, j, k, l;
  
  if (currentry < 0 || currdatum < 0)
    return NULL;

  for (i = 0; i < numvalues[currentry]; i++) {
    // Look for the column requested.
    if (!strcmp(valuename, valuenames[currentry][i])) {

      // Look for a request for a default value or zero length string.
      if (!strcmp(entries[currentry][i][currdatum], "@")) {

        // Check to see if a default has been assigned.
        if (strlen(datadefaults[currentry][currdatum])) {
          j = currdatum;

          // Find the default value, recursively (a default value field could
          // itself have a default).  Because it is possible to assign defaults
          // in a circular way, don't let the recursion last forever -- cap it
          // at 50 recursions.
          for (l = 0; l < 50; l++) {
            for (k = 0; k < numdata[currentry]; k++) {
              if (!strcmp(datanames[currentry][k], 
                          datadefaults[currentry][j]))
                if (!strcmp(entries[currentry][i][k], "@")) {
                  if (strlen(datadefaults[currentry][k]))
                    j = k;
                  else
                    return "";
                }
                else
                  return entries[currentry][i][k];
            }
          }
          return "";
        }
        else
          return "";
      }
      else
        return entries[currentry][i][currdatum];
    }
  }
  
  return "";
}


/******************************************************************************\
|******************************************************************************|
|**.------------------------------------------------------------------------.**|
|**|                                                                        |**|
|**|                              CLASS DataHolder                          |**|
|**|                                                                        |**|
|**|    Store information on channels to be compressed/decompressed by      |**|
|**|    big/small.                                                          |**|
|**|                                                                        |**|
|**`------------------------------------------------------------------------'**|
|******************************************************************************|
\******************************************************************************/


/******************************************************************************\
|*                                                                            *|
|* DataHolder:  constructor                                                   *|
|*                                                                            *|
\******************************************************************************/

DataHolder::DataHolder() {
  allocated = false;
  return;
}

DataHolder::DataHolder(const char *filename) {
  allocated = false;
  LoadFromAML(filename);

  return;
}


/******************************************************************************\
|*                                                                            *|
|* LoadFromAML (public): read the channel data from an AML file               *|
|*                                                                            *|
|* *filename    - the full name of the AML file                               *|
|*                                                                            *|
|* Returns: false if unable to parse file                                     *|
|*                                                                            *|
\******************************************************************************/

bool DataHolder::LoadFromAML(const char *filename) {
  AMLParser *aml;

  aml = new AMLParser();

  if (!aml->LoadFile(filename))
    return false;

  if (!aml->FirstDatum("SETTINGS")) {
    printf("Fatal (DataHolder): %s contains no data under the SETTINGS "
           "entry.\n", filename);
    return false;
  }

  // Get settings.
  maxbitrate = atoi(aml->Value("maxbitrate"));
  if (!maxbitrate) {
    printf("Fatal (DataHolder): %s has maxbitrate = 0.\n", filename);
    return false;
  }
  looplength = atoi(aml->Value("looplength"));
  if (!looplength) {
    printf("Fatal (DataHolder): %s has looplength = 0.\n", filename);
    return false;
  }
  samplerate = atoi(aml->Value("samplerate"));
  if (!samplerate) {
    printf("Fatal (DataHolder): %s has samplerate = 0.\n", filename);
    return false;
  }
  minover = atof(aml->Value("minover"));
  if (!minover) {
    printf("Fatal (DataHolder): %s has minover = 0.\n", filename);
    return false;
  }
  maxover = atof(aml->Value("maxover"));
  if (!maxover) {
    printf("Fatal (DataHolder): %s has maxover = 0.\n", filename);
    return false;
  }
  
  // Allocate etc.
  numslows = aml->NumData("SLOWDATA.SINGLE") + aml->NumData("SLOWDATA.AVG");
  numfasts = aml->NumData("FASTDATA.DIFF") + aml->NumData("FASTDATA.INT");
  if (numslows == 0 && numfasts == 0)
    printf("Warning (DataHolder):  %s contains no channels.\n", filename);

  if (allocated) {
    slows = (struct DataStruct_glob *)realloc(slows, numslows * 
                                                     sizeof(DataStruct_glob));
    fasts = (struct DataStruct_glob *)realloc(fasts, numfasts * 
                                                     sizeof(DataStruct_glob));
  }
  else {
    slows = (struct DataStruct_glob *)malloc(numslows * 
                                             sizeof(DataStruct_glob));
    fasts = (struct DataStruct_glob *)malloc(numfasts * 
                                             sizeof(DataStruct_glob));
    allocated = true;
  }
 
  numslows = 0;
  for (aml->FirstDatum("SLOWDATA.SINGLE"); !aml->NoDatum(); aml->NextDatum()) {
    slows[numslows].type = COMP_SINGLE;
    PopulateDataStruct(slows + numslows, aml);
    numslows++;
  }
  for (aml->FirstDatum("SLOWDATA.AVG"); !aml->NoDatum(); aml->NextDatum()) {
    slows[numslows].type = COMP_AVERAGE;
    PopulateDataStruct(slows + numslows, aml);
    numslows++;
  }
  
  numfasts = 0;
  for (aml->FirstDatum("FASTDATA.DIFF"); !aml->NoDatum(); aml->NextDatum()) {
    fasts[numfasts].type = COMP_DIFFERENTIAL;
    PopulateDataStruct(fasts + numfasts, aml);
    numfasts++;
  }
  for (aml->FirstDatum("FASTDATA.INT"); !aml->NoDatum(); aml->NextDatum()) {
    fasts[numfasts].type = COMP_INT_PRESERVING;
    PopulateDataStruct(fasts + numfasts, aml);
    numfasts++;
  }
  
  return true;
}


/******************************************************************************\
|*                                                                            *|
|* FirstSlow (public): Go to the first slow channel                           *|
|*                                                                            *|
|* Returns: pointer to the first slow channel info structure                  *|
|*                                                                            *|
\******************************************************************************/

struct DataStruct_glob *DataHolder::FirstSlow() {
  currslow = 0;
  if (numslows)
    return slows;
  else
    return NULL;
}


/******************************************************************************\
|*                                                                            *|
|* FirstFast (public): Go to the first fast channel                           *|
|*                                                                            *|
|* Returns: pointer to the first fast channel info structure                  *|
|*                                                                            *|
\******************************************************************************/

struct DataStruct_glob *DataHolder::FirstFast() {
  currfast = 0;
  if (numfasts)
    return fasts; 
  else 
    return NULL;
}


/******************************************************************************\
|*                                                                            *|
|* NextSlow (public): Go to the next slow channel                             *|
|*                                                                            *|
|* Returns: pointer to the next slow channel info structure                   *|
|*                                                                            *|
\******************************************************************************/

struct DataStruct_glob *DataHolder::NextSlow() {
  if (++currslow >= numslows) {
    currslow = numslows;
    return NULL;
  }
  else
    return &(slows[currslow]); 
}


/******************************************************************************\
|*                                                                            *|
|* NextFast (public): Go to the next fast channel                             *|
|*                                                                            *|
|* Returns: pointer to the next fast channel info structure                   *|
|*                                                                            *|
\******************************************************************************/

struct DataStruct_glob *DataHolder::NextFast() {
  if (++currfast >= numfasts) {
    currfast = numfasts;
    return NULL;
  }
  else
    return &(fasts[currfast]); 
}


/******************************************************************************\
|*                                                                            *|
|* PopulateDataStruct (private): populate struct DataStruct_glob from AML     *|
|* file                                                                       *|
|*                                                                            *|
|* *s   - structure to populate                                               *|
|* *a   - AML file object                                                     *|
|*                                                                            *|
\******************************************************************************/

void DataHolder::PopulateDataStruct(struct DataStruct_glob *s, AMLParser *a) {
  strcpy(s->src, a->Value("src"));
  s->framefreq = atoi(a->Value("perframe"));
  s->numbits = atoi(a->Value("numbits"));
  s->overflowsize = atoi(a->Value("overbits"));
  s->divider = atoi(a->Value("divider"));
  if (!strcmp(a->Value("forcediv"), "true") ||
      !strcmp(a->Value("forcediv"), "True") ||
      !strcmp(a->Value("forcediv"), "TRUE"))
    s->forcediv = true;
  else
    s->forcediv = false;
  s->samplefreq = atoi(a->Value("samplefreq"));
  s->minval = atoll(a->Value("minval"));
  s->maxval = atoll(a->Value("maxval"));
  
  return;
}
