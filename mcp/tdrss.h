// ***************************************************
// *  Programmed by Adam Hincks                      *
// *                                                 *
// *  Comments on classes & functions in .cpp file   *
// ***************************************************

#ifndef SMALL_H
#define SMALL_H

#include "small_c.h"

class AliceFile;
class DataHolder;

class Buffer
{
public:
  Buffer();
  ~Buffer();
  void Start(char filenum, unsigned int framenum);
  void Stop();
  void Introduce();
  void NoDataMarker();
  void RecordNumBytes();
  void EraseLastSection();
  void WriteTo(long long datum, char numbits, char oversize, bool hassign);
  short int Tester(int pos);
  int SectionSize();
  int CurrSize();
  int MaxSize();
  void SetSize(int s);

  int overnum;
private:
  void WriteChunk(char numbits, long long datum);

  unsigned char *buf;
  int size;
  int bytepos;
  char bitpos;
  char overflowsize;
  int startbyte;
};

class Alice
{
public:
  Alice();
  ~Alice();
  void CompressionLoop();

protected:

private:
  bool GetCurrentXML();


  double Differentiate(double *invals, int num, int divider);
  void Integrate(double *invals, int num);
  double Round(double num);

  void SendDiff(double *rawdata, int num, struct DataStruct_glob *currInfo,
			          float maxover, float minover);
  void SendInt(double *rawdata, int num, struct DataStruct_glob *currInfo,
			         float maxover, float minover);
  void SendSingle(double *rawdata, struct DataStruct_glob *currInfo);
  void SendAverage(double *rawdata, int num, struct DataStruct_glob *currInfo);

  int MaxFrameFreq();
  int FindPowTwo(int num, float threshold);
  int MaxPowTwo(int val, float threshold);

  AliceFile *DataSource;
  DataHolder *DataInfo;
  Buffer *sendbuf;

  int XMLsrc;
};

#endif
