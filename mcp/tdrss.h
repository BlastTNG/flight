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

class FrameBuffer
{
  public:
    FrameBuffer(unsigned int *mcpindex_in, unsigned short **fastin_in,
                unsigned short **slowin_in, int numframes_in);
    ~FrameBuffer();
    void Update();
    void Resize(int numframes_in);
    int NumFrames();
    int ReadField(double *returnbuf, const char *fieldname, int framenum_in,
                  int numframes_in);

  protected:

  private:
    unsigned int *mcpindex, lastmcpindex;
    unsigned short **fastdata, ***fastbuf;
    unsigned short **slowdata, ***slowbuf;
    int numframes;      // Number of frames in circular buffer.
    int framenum;       // Current frame in circ. buffer.
    int multiplexindex; // Alice defines frame as a 5 Hz frame. 
    int pseudoframe;    // Trick Alice into thinking that the buffer is linear
                        // by having a linear frame counter.

    bool memallocated, multiplexsynced;
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

  FrameBuffer *DataSource;
  DataHolder *DataInfo;
  Buffer *sendbuf;

  int XMLsrc;
};

#endif
