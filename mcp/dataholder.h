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

/******************************************************************************\
|* DATAHOLDER.H                                                               *|
|*                                                                            *|
|* Comments in dataholder.h                                                   *|
\******************************************************************************/

#ifndef DATAHOLDER_H
#define DATAHOLDER_H

#define DIFFERENTIAL    0
#define INT_PRESERVING  1
#define SINGLE          2
#define AVERAGE         3

#define AML_LEN_LINE    512
#define AML_LEN_ENTRY   32

#ifdef __MCP__
#  include "tdrss.h"
#else
#  include "big.h"
#endif
#include "channels.h"

struct value_struct {
  char (*entry)[AML_LEN_ENTRY];
  char name[AML_LEN_ENTRY];
};

struct data_struct {
  char name[AML_LEN_ENTRY];
  char dflt[AML_LEN_ENTRY];
};

struct AML_struct {
  struct value_struct* value;
  struct data_struct* datum;
  char entryname[AML_LEN_ENTRY];
  int level;
  int numvalues;
  int numdata;
  int parent;
};

struct DataStruct_glob {
  char type;
  char src[FIELD_LEN];
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

  int numsamples;
  struct ChannelStruct channel;
};

class AMLParser {
  public:
    AMLParser();
    bool LoadFile(char *filename);
    int NumData(const char *fullentryname);
    bool FirstDatum(const char *fullentryname);
    bool NextDatum();
    bool NoDatum();
    const char *Value(const char *valuename);

  private:
    int GetNumValues(const char *buf);
    void GetEntryName(const char *buf, char *namebuf);
    void GetDataDefault(const char *buf, char *defaultbuf);
    bool GetValue(const char *buf, int num, char *valuebuf);
    void ParseFullName(int num, char *namebuf);

    int numentries;
    int maxvalues;
    int maxdata;
    struct AML_struct *aml;
    int currentry;
    int currdatum;
};

class DataHolder {
  public:
    DataHolder();
    bool LoadFromAML(char *filename);
    struct DataStruct_glob *FirstSlow();
    struct DataStruct_glob *NextSlow();
    struct DataStruct_glob *FirstFast();
    struct DataStruct_glob *NextFast();

    int maxbitrate;
    int looplength;
    int samplerate;
    float minover;
    float maxover;

  private:
    void PopulateDataStruct(struct DataStruct_glob *s, AMLParser *a);

    int numslows;
    int numfasts;
    int currslow;
    int currfast;
    struct DataStruct_glob *slows;
    struct DataStruct_glob *fasts;
    bool allocated;
};

#endif
