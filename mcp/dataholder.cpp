/* dataholder.cpp: mcp data buffer for TDRSS high-rate downlink
 *
 * This software is copyright (C) 2004 University of Toronto
 * 
 * This file is part of the BLAST flight code licensed under the GNU 
 * General Public License.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this software; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/*()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*\
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
 *()                                                                        ()*|
 *() DATAHOLDER.CPP                                                         ()*|
 *()                                                                        ()*|
 *() Information on channels being compressed/decompressed by big/small.    ()*|
 *() Both big and small require this file to work in conjunction with each  ()*|
 *() other.  This file includes als the class AMLParser which parses AML    ()*|
 *() files containing channel information for big/small.                    ()*|
 *()                                                                        ()*|
 *() Adam Hincks, Summer 2004, Toronto                                      ()*|
 *()                                                                        ()*|
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*|
 *()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()()*/


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dataholder.h"
#include "alice.h"
extern "C" {
#include "blast.h"
}

#define DEFAULT_AML ALICEFILE_DIR "0.aml"

/******************************************************************************\
 *                                                                            *|
 * trim: remove leading and trailing spaces and new line characters           *|
 *                                                                            *|
 * *buf     - buffer to trim                                                  *|
 *                                                                            *|
 ******************************************************************************/

void trim(char *buf)
{
  int i;
  char tempbuf[strlen(buf)];

  SMALL_TRACE("%p", buf);

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

  SMALL_RTN("");
}


/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                              CLASS AMLParser                           |**|
 **|                                                                        |**|
 **|                Parse AML (Alice Miniturisation List) files.            |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/


/******************************************************************************\
 *                                                                            *|
 * AMLParser:  constructor                                                    *|
 *                                                                            *|
 ******************************************************************************/

AMLParser::AMLParser(void)
{
  SMALL_TRACE("");

  numentries = 0;
  maxvalues = 0;
  maxdata = 0;
  currentry = -1;
  currdatum = -1;

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * LoadFile (public): load in an AML file and parse it.                       *|
 *                                                                            *|
 * *filename    - file to load                                                *|
 *                                                                            *|
 * Returns: true on successful load                                           *|
 *                                                                            *|
 ******************************************************************************/

bool AMLParser::LoadFile(char *filename)
{
  FILE *file;
  int i, j, k;
  int entrycount;
  char linebuf[AML_LEN_LINE], tmpstr[AML_LEN_LINE], tmpstr2[AML_LEN_LINE];

  SMALL_TRACE("%p", filename);

  if ((file = fopen(filename, "r")) == NULL) {
    berror(warning, "AMLParser: couldn't open %s for reading", filename);
    bprintf(warning, "AMLParser: trying %s instead.\n", DEFAULT_AML);
    strcpy(filename, DEFAULT_AML);
    if ((file = fopen(filename, "r")) == NULL)
      berror(tfatal, "AMLParser: couldn't open %s for reading", filename);
  }

  // First time through, check for number of entries and maximum number of
  // values.
  i = 0;
  while (fgets(linebuf, AML_LEN_LINE, file) != NULL) {
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
  SMALL_DEBUG("balloc(%i) as entries (%i of char ***)",
      numentries * sizeof(char ***), numentries);
  entries = (char ****)balloc(fatal, numentries * sizeof(char ***));
  SMALL_DEBUG("entries = %p\n", entries);

  SMALL_DEBUG("balloc(%i) as valuenames", numentries * sizeof(char **));
  valuenames = (char ***)balloc(fatal, numentries * sizeof(char **));

  SMALL_DEBUG("balloc(%i) as datanames", numentries * sizeof(char **));
  datanames = (char ***)balloc(fatal, numentries * sizeof(char **));

  SMALL_DEBUG("balloc(%i) as datadefaults", numentries * sizeof(char **));
  datadefaults = (char ***)balloc(fatal, numentries * sizeof(char **));

  SMALL_DEBUG("balloc(%i) as entrynames", numentries * sizeof(char *));
  entrynames = (char **)balloc(fatal, numentries * sizeof(char *));

  SMALL_DEBUG("balloc(%i) as level", numentries * sizeof(int));
  level = (int *)balloc(fatal, numentries * sizeof(int));

  SMALL_DEBUG("balloc(%i) as numvalues", numentries * sizeof(int));
  numvalues = (int *)balloc(fatal, numentries * sizeof(int));

  SMALL_DEBUG("balloc(%i) as numdata", numentries * sizeof(int));
  numdata = (int *)balloc(fatal, numentries * sizeof(int));

  SMALL_DEBUG("balloc(%i) as parent", numentries * sizeof(int));
  parent = (int *)balloc(fatal, numentries * sizeof(int));

  for (i = 0; i < numentries; i++) {
    SMALL_DEBUG("balloc(%i) as entries[%i] (%i of char**)",
        maxvalues * sizeof(char **), i, maxvalues);
    SMALL_DEBUG("entries = %p\n", entries);
    entries[i] = (char ***)balloc(fatal, maxvalues * sizeof(char **));
    SMALL_DEBUG("entries[%i] = %p\n", i, entries[i]);

    SMALL_DEBUG("balloc(%i) as valuenames[%i]", maxvalues * sizeof(char *), i);
    valuenames[i] = (char **)balloc(fatal, maxvalues * sizeof(char *));

    SMALL_DEBUG("balloc(%i) as datanames[%i]", maxvalues * sizeof(char *), i);
    datanames[i] = (char **)balloc(fatal, maxvalues * sizeof(char *));

    SMALL_DEBUG("balloc(%i) as datadefaults[%i]", maxvalues * sizeof(char *), i);
    datadefaults[i] = (char **)balloc(fatal, maxvalues * sizeof(char *));

    SMALL_DEBUG("balloc(%i) as entrynames[%i]", AML_LEN_ENTRY * sizeof(char), i);
    entrynames[i] = (char *)balloc(fatal, AML_LEN_ENTRY * sizeof(char));

    level[i] = 0;
    numvalues[i] = 0;
    numdata[i] = 0;
    parent[i] = -1;
    for (j = 0; j < maxvalues; j++) {
      SMALL_DEBUG("balloc(%i) as entries[%i][%i] (%i of char *)",
          maxdata * sizeof(char *), i, j, maxdata);
      SMALL_DEBUG("entries = %p\n", entries);
      SMALL_DEBUG("entries[%i] = %p\n", i, entries[i]);
      entries[i][j] = (char **)balloc(fatal, maxdata * sizeof(char *));
      SMALL_DEBUG("entries[%i][%i] = %p\n", i, j, entries[i][j]);

      SMALL_DEBUG("balloc(%i) as valuenames[%i][%i]",
          AML_LEN_ENTRY * sizeof(char), i, j);
      valuenames[i][j] = (char *)balloc(fatal, AML_LEN_ENTRY * sizeof(char));

      for (k = 0; k < AML_LEN_ENTRY; k++) {
        SMALL_DEBUG("balloc(%i) as entries[%i][%i][%i]",
            AML_LEN_ENTRY * sizeof(char), i, j, k);
        entries[i][j][k] = (char *)balloc(fatal, AML_LEN_ENTRY * sizeof(char));
      }
    }
    for (j = 0; j < maxdata; j++) {
      SMALL_DEBUG("balloc(%i) as datanames[%i][%i]",
          AML_LEN_ENTRY * sizeof(char), i, j);
      datanames[i][j] = (char *)balloc(fatal, AML_LEN_ENTRY * sizeof(char));

      SMALL_DEBUG("balloc(%i) as datadefaults[%i][%i]",
          AML_LEN_ENTRY * sizeof(char), i, j);
      datadefaults[i][j] = (char *)balloc(fatal, AML_LEN_ENTRY * sizeof(char));
    }
  }

  // Now fill the array entries from the file.
  rewind(file);
  j = 0;
  entrycount = -1;
  for (i = 0; fgets(linebuf, AML_LEN_LINE, file) != NULL; i++) {
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
            bprintf(err, 
                "AMLParser: found two instances of '%s' in the file '%s'.  "
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
            bprintf(err, 
                "AMLParser: found two instances of '%s' in entry '%s' in "
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
            bprintf(err, 
                "AMLParser: couldn't get datum for '%s' in column '%s' in "
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
  fclose(file);

  SMALL_RTN("%i", true);

  return true;
}


/******************************************************************************\
 *                                                                            *|
 * ParseFullName (private): returns the full name of an entry                 *|
 *                                                                            *|
 * num          - index number of the entry                                   *|
 * *namebuf     - buffer in which to return the name                          *|
 *                                                                            *|
 * Returns: number of values                                                  *|
 *                                                                            *|
 ******************************************************************************/


void AMLParser::ParseFullName(int num, char *namebuf)
{
  int i;
  char tmpstr[AML_LEN_LINE];

  SMALL_TRACE("%i, %p", num, namebuf);

  strcpy(namebuf, "");
  i = num;

  do {
    sprintf(tmpstr, "%s.%s", entrynames[i], namebuf);
    strcpy(namebuf, tmpstr);
    i = parent[i];
  } while (i >= 0);

  if (strlen(namebuf))
    namebuf[strlen(namebuf) - 1] = '\0';

  SMALL_RTN("");

  return;
}

/******************************************************************************\
 *                                                                            *|
 * GetNumValues (private): counts the number of values on a line of an AML    *|
 * file.                                                                      *|
 *                                                                            *|
 * *buf     - line of the file                                                *|
 *                                                                            *|
 * Returns: number of values                                                  *|
 *                                                                            *|
 ******************************************************************************/

int AMLParser::GetNumValues(const char *buf)
{
  int i, ret;
  char lastval;

  SMALL_TRACE("%p", buf);

  if (buf[0] != '+') {  // Must be a valid line.
    SMALL_RTN("%i", 0);
    return 0;
  }

  lastval = '+';
  ret = 0;

  for (i = 1; i < (signed int)strlen(buf); i++) {
    if (buf[i] == ' ' && lastval != ' ')
      ret++;
    lastval = buf[i];
  }

  SMALL_RTN("%i", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * GetEntryName (private): get the name of an entry in an AML file            *|
 *                                                                            *|
 * *buf     - line of the file                                                *|
 * *name   - buffer for return of value                                       *|
 *                                                                            *|
 ******************************************************************************/

void AMLParser::GetEntryName(const char *buf, char *namebuf)
{
  int i, j;

  SMALL_TRACE("%p, %p", buf, namebuf);

  for (i = 0; i < (signed int)strlen(buf) && (buf[i] == '+' || buf[i] == '-'); 
      i++);

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' ' && buf[j] != '@'; 
      j++)
    namebuf[j - i] = buf[j];
  namebuf[j - i] = '\0';

  SMALL_RTN("");

  return;
}


/******************************************************************************\
 *                                                                            *|
 * GetEntryName (private): get the name of an entry in an AML file            *|
 *                                                                            *|
 * *buf     - line of the file                                                *|
 * *name   - buffer for return of value                                       *|
 *                                                                            *|
 ******************************************************************************/

void AMLParser::GetDataDefault(const char *buf, char *defaultbuf)
{
  int i, j;

  SMALL_TRACE("%p, %p", buf, defaultbuf);

  for (i = 0; i < (signed int)strlen(buf) && (buf[i] == '+' || buf[i] == '-'); 
      i++);

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' ' && buf[j] != '@'; 
      j++);

  if (buf[j] != '@') {
    defaultbuf[0] = '\0'; 
    SMALL_RTN("");
    return;
  }

  i = j + 1;
  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' '; j++)
    defaultbuf[j - i] = buf[j];
  defaultbuf[j - i] = '\0';

  SMALL_RTN("");
  return;
}


/******************************************************************************\
 *                                                                            *|
 * GetValue (private): get a value from a list on a line of an AML file       *|
 *                                                                            *|
 * *buf     - line of the file                                                *|
 * num      - entry number                                                    *|
 * *value   - buffer for return of value                                      *|
 *                                                                            *|
 * Returns: false if value <num> does not exist; true otherwise               *|
 *                                                                            *|
 ******************************************************************************/

bool AMLParser::GetValue(const char *buf, int num, char *valuebuf)
{
  int i, j, count;
  char lastval;

  SMALL_TRACE("%p, %i, %p", buf, num, valuebuf);

  if (!(buf[0] == '+' || buf[0] == '-')) {  // Must be a valid line.
    SMALL_RTN("%i", false);
    return false;
  }

  lastval = '+';
  count = -1;

  for (i = 1; i < (signed int)strlen(buf); i++) {
    if (buf[i] == ' ' && lastval != ' ') {
      if (++count == num)
        break;
    }
    lastval = buf[i];
  }

  if (i == (signed int)strlen(buf)) {
    SMALL_RTN("%i", false);
    return false;
  }

  for (; i < (signed int)strlen(buf) && buf[i] == ' '; i++);

  if (i == (signed int)strlen(buf)) {
    SMALL_RTN("%i", false);
    return false;
  }

  for (j = i; j < (signed int)strlen(buf) && buf[j] != ' '; j++) 
    valuebuf[j - i] = buf[j];
  valuebuf[j - i] = '\0';

  SMALL_RTN("%i", true);
  return true;
}


/******************************************************************************\
 *                                                                            *|
 * FirstDatum (public):  go to the first datum for the requested entry.       *|
 *                                                                            *|
 * *fullentryname   - the name of the entry, in the format LEVEL1.LEVEL2...   *|
 *                                                                            *|
 * Returns: false if the entry does not exist or the entry contains no data   *|
 *                                                                            *|
 ******************************************************************************/

bool AMLParser::FirstDatum(const char *fullentryname)
{
  int i;
  char tmpstr[AML_LEN_LINE];

  SMALL_TRACE("%p", fullentryname);

  for (i = 0; i < numentries; i++) {
    ParseFullName(i, tmpstr);
    if (!strcmp(fullentryname, tmpstr)) {
      if (numdata[i]) {
        currentry = i;
        currdatum = 0;
        SMALL_RTN("%i", true);
        return true;
      } else {
        currentry  = -1;
        currdatum = -1;
        SMALL_RTN("%i", false);
        return false;
      }
    }
  }

  currentry = -1;
  currdatum = -1;
  SMALL_RTN("%i", false);
  return false;
}


/******************************************************************************\
 *                                                                            *|
 * NumData (public):  get the number of data for the current entry.           *|
 *                                                                            *|
 * *fullentryname   - the name of the entry, in the format LEVEL1.LEVEL2...   *|
 *                                                                            *|
 * Returns: the number of rows of data for the current entry                  *|
 *                                                                            *|
 ******************************************************************************/

int AMLParser::NumData(const char *fullentryname) {
  int i;
  char tmpstr[AML_LEN_LINE];

  SMALL_TRACE("%p", fullentryname);

  for (i = 0; i < numentries; i++) {
    ParseFullName(i, tmpstr);
    if (!strcmp(fullentryname, tmpstr)) {
      SMALL_RTN("%i", numdata[i]);
      return numdata[i];
    }
  }

  SMALL_RTN("%i", 0);
  return 0;
}


/*****************************************************************************\
 *                                                                            *|
 * NextDatum (public):  go to the next datum for the current entry.           *|
 *                                                                            *|
 * Returns: false if there are no more data for the current entry.            *|
 *                                                                            *|
 ******************************************************************************/

bool AMLParser::NextDatum(void)
{
  SMALL_TRACE("");

  if (currentry < 0 || currdatum < 0) {
    SMALL_RTN("%i", false);
    return false;
  }

  if (++currdatum >= numdata[currentry]) {
    currentry = -1;
    currdatum = -1;
    SMALL_RTN("%i", false);
    return false;
  }

  SMALL_RTN("%i", true);
  return true;
}


/******************************************************************************\
 *                                                                            *|
 * NoDatum (public):  check to see if the current datum is valid  .           *|
 *                                                                            *|
 * Returns: true if current datum is not valid                                *|
 *                                                                            *|
 ******************************************************************************/

bool AMLParser::NoDatum(void)
{
  bool ret = (currentry < 0 || currdatum < 0);

  SMALL_TRACE("");

  SMALL_RTN("%i", ret);

  return ret;
}


/******************************************************************************\
 *                                                                            *|
 * Value (public):  return the value of the current datum for the requested   *|
 * column                                                                     *|
 *                                                                            *|
 * *valuename   - the column name                                             *|
 *                                                                            *|
 * Returns: the datum, or NULL if it does not exist                           *|
 *                                                                            *|
 ******************************************************************************/

const char *AMLParser::Value(const char *valuename)
{
  int i, j, k, l;

  SMALL_TRACE("%p", valuename);

  if (currentry < 0 || currdatum < 0) {
    SMALL_RTN("%p", NULL);
    return NULL;
  }

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
                  else {
                    SMALL_RTN("\"\"");
                    return "";
                  }
                }
                else {
                  SMALL_RTN("%p", entries[currentry][i][k]);
                  return entries[currentry][i][k];
                }
            }
          }
          SMALL_RTN("\"\"");
          return "";
        } else {
          SMALL_RTN("\"\"");
          return "";
        }
      } else {
        SMALL_RTN("\"\"");
        return entries[currentry][i][currdatum];
      }
    }
  }

  SMALL_RTN("\"\"");
  return "";
}


/******************************************************************************\
 ******************************************************************************|
 **.------------------------------------------------------------------------.**|
 **|                                                                        |**|
 **|                              CLASS DataHolder                          |**|
 **|                                                                        |**|
 **|    Store information on channels to be compressed/decompressed by      |**|
 **|    big/small.                                                          |**|
 **|                                                                        |**|
 **`------------------------------------------------------------------------'**|
 ******************************************************************************|
 ******************************************************************************/


/******************************************************************************\
 *                                                                            *|
 * DataHolder:  constructor                                                   *|
 *                                                                            *|
 ******************************************************************************/

DataHolder::DataHolder(void)
{
  SMALL_TRACE("");

  allocated = false;

  SMALL_RTN("");
  return;
}

/******************************************************************************\
 *                                                                            *|
 * LoadFromAML (public): read the channel data from an AML file               *|
 *                                                                            *|
 * *filename    - the full name of the AML file                               *|
 *                                                                            *|
 * Returns: false if unable to parse file                                     *|
 *                                                                            *|
 ******************************************************************************/

bool DataHolder::LoadFromAML(char *filename) {
  AMLParser *aml;

  SMALL_TRACE("%p", filename);

  aml = new AMLParser();

  if (!aml->LoadFile(filename)) {
    SMALL_RTN("%i", false);
    return false;
  }

  if (!aml->FirstDatum("SETTINGS")) {
    bprintf(err, "Fatal (DataHolder): %s contains no data under the SETTINGS "
        "entry.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }

  // Get settings.
  maxbitrate = atoi(aml->Value("maxbitrate"));
  if (!maxbitrate) {
    bprintf(err, "Fatal (DataHolder): %s has maxbitrate = 0.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }
  looplength = atoi(aml->Value("looplength"));
  if (!looplength) {
    bprintf(err, "Fatal (DataHolder): %s has looplength = 0.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }
  samplerate = atoi(aml->Value("samplerate"));
  if (!samplerate) {
    bprintf(err, "Fatal (DataHolder): %s has samplerate = 0.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }
  minover = atof(aml->Value("minover"));
  if (!minover) {
    bprintf(err, "Fatal (DataHolder): %s has minover = 0.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }
  maxover = atof(aml->Value("maxover"));
  if (!maxover) {
    bprintf(err, "Fatal (DataHolder): %s has maxover = 0.\n", filename);
    SMALL_RTN("%i", false);
    return false;
  }

  // Allocate etc.
  numslows = aml->NumData("SLOWDATA.SINGLE") + aml->NumData("SLOWDATA.AVG");
  numfasts = aml->NumData("FASTDATA.DIFF") + aml->NumData("FASTDATA.INT");
  if (numslows == 0 && numfasts == 0)
    bprintf(warning,
        "Warning (DataHolder):  %s contains no channels.\n", filename);

  if (allocated) {
    slows = (struct DataStruct_glob *)reballoc(fatal, slows, numslows * 
        sizeof(DataStruct_glob));
    fasts = (struct DataStruct_glob *)reballoc(fatal, fasts, numfasts * 
        sizeof(DataStruct_glob));
  } else {
    slows = (struct DataStruct_glob *)balloc(fatal, numslows * 
        sizeof(DataStruct_glob));
    fasts = (struct DataStruct_glob *)balloc(fatal, numfasts * 
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

  SMALL_RTN("%i", true);
  return true;
}


/******************************************************************************\
 *                                                                            *|
 * FirstSlow (public): Go to the first slow channel                           *|
 *                                                                            *|
 * Returns: pointer to the first slow channel info structure                  *|
 *                                                                            *|
 ******************************************************************************/

struct DataStruct_glob *DataHolder::FirstSlow(void)
{
  SMALL_TRACE("");

  currslow = 0;
  if (numslows) {
    SMALL_RTN("%p", slows);
    return slows;
  }

  SMALL_RTN("%p", NULL);
  return NULL;
}


/******************************************************************************\
 *                                                                            *|
 * FirstFast (public): Go to the first fast channel                           *|
 *                                                                            *|
 * Returns: pointer to the first fast channel info structure                  *|
 *                                                                            *|
 ******************************************************************************/

struct DataStruct_glob *DataHolder::FirstFast(void)
{

  SMALL_TRACE("");
  currfast = 0;

  if (numfasts) {
    SMALL_RTN("%p", fasts);
    return fasts; 
  }

  SMALL_RTN("%p", NULL);
  return NULL;
}


/******************************************************************************\
 *                                                                            *|
 * NextSlow (public): Go to the next slow channel                             *|
 *                                                                            *|
 * Returns: pointer to the next slow channel info structure                   *|
 *                                                                            *|
 ******************************************************************************/

struct DataStruct_glob *DataHolder::NextSlow(void)
{
  SMALL_TRACE("");

  if (++currslow >= numslows) {
    currslow = numslows;
    SMALL_RTN("%p", NULL);
    return NULL;
  }

  SMALL_RTN("%p", &(slows[currslow]));
  return &(slows[currslow]); 
}


/******************************************************************************\
 *                                                                            *|
 * NextFast (public): Go to the next fast channel                             *|
 *                                                                            *|
 * Returns: pointer to the next fast channel info structure                   *|
 *                                                                            *|
 ******************************************************************************/

struct DataStruct_glob *DataHolder::NextFast(void)
{
  SMALL_TRACE("");

  if (++currfast >= numfasts) {
    currfast = numfasts;
    SMALL_RTN("");
    return NULL;
  }

  SMALL_RTN("%p", &(fasts[currfast]));
  return &(fasts[currfast]); 
}


/******************************************************************************\
 *                                                                            *|
 * PopulateDataStruct (private): populate struct DataStruct_glob from AML     *|
 * file                                                                       *|
 *                                                                            *|
 * *s   - structure to populate                                               *|
 * *a   - AML file object                                                     *|
 *                                                                            *|
 ******************************************************************************/

void DataHolder::PopulateDataStruct(struct DataStruct_glob *s, AMLParser *a)
{
  SMALL_TRACE("%p, %p", s, a);

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

  SMALL_RTN("");

  return;
}
