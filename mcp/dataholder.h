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

struct DataStruct_glob {
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

class AMLParser {
  public:
    AMLParser(const char *filename);
    AMLParser();
    bool LoadFile(const char *filename);
    int NumData(const char *fullentryname);
    bool FirstDatum(const char *fullentryname);
    bool NextDatum();
    bool NoDatum();
    const char *Value(const char *valuename);

  private:
    void CommonConstructor();
    int GetNumValues(const char *buf);
    void GetEntryName(const char *buf, char *namebuf);
    void GetDataDefault(const char *buf, char *defaultbuf);
    bool GetValue(const char *buf, int num, char *valuebuf);
    void ParseFullName(int num, char *namebuf);
    
    int numentries;
    int maxvalues;
    int maxdata;
    char ****entries;
    char **entrynames;
    char ***valuenames;
    char ***datanames;
    char ***datadefaults;
    int *level;
    int *parent;
    int *numvalues;
    int *numdata;
    int currentry;
    int currdatum;
};

class DataHolder {
  public:
    DataHolder(const char *filename);
    DataHolder();
    bool LoadFromAML(const char *filename);
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
