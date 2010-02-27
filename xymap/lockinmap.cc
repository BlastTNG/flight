#include <iostream>
#include <iomanip>
#include <fstream>
#include <string>
#include <vector>
#include <complex>
#include <cstring>
#include <cstdlib>
#include <cstdio>
#include <unistd.h>
#include <getdata/dirfile.h>
#include "iirfilter.h"

using std::cerr;
using std::cout;
using std::endl;
using std::string;
using std::vector;
using GetData::Dirfile;
typedef std::complex<double> complex; 

const double f_samp = 4.0E6/384.0/104.0;    //sample frequency
const double s2cm = 25.4/4000.0;	    //stage steps to cm
//TODO can use DELAY to acount for phase lags, seems to work fine without
const int DELAY = 0;

void Usage(char* name)
{
  cerr << name << " -f <file name> -d <detector field name>";
  cerr << "-r <first_frame:numframes>" << endl;
  cerr << "\t[ -x <x_field> ] [ -y <y_field> ] [ -c <chop_ref_field> ]" << endl;
  cerr << "Must specify at least -f -d -r" << endl;
  exit(1);
}

complex pll(double ref)
{
  static double period = 20.0;
  static double phase = 0.0;
  
  static double T = 0.0;

  static int ref_edge = 0;
  static int pll_edge = 0;
 
  static double ph = 0.0;
  static double dph = 0.0;
  
  static BesselHP1<double> fref(0.0125/f_samp);
  static BesselLP1<double> fperiod(0.15/f_samp);
  static BesselLP1<double> fphase(0.15/f_samp);
 
  double ref_ = fref(ref);
  ph += dph;
  T  += 1.0;  
  double pll_ref = cos(ph);
  
  if(ref_edge == 0  && ref_ >= 0.0) {
    period = fperiod(T);
    ref_edge = 1;
    dph = (2.0*M_PI)/period;
    T =  0.0; 
  } else if(ref_edge == 1 && ref_ < 0.0 && T > 1.0) {
    ref_edge = 0;
  }


  if(pll_edge == 0 && (pll_ref >= 0.0)) {
    phase = fphase(T*dph);
    pll_edge = 1;
  } else if(pll_edge == 1 && (pll_ref < 0.0)) {
    pll_edge = 0;
  }

  return complex( ((cos(ph+phase) > 0.0) ? 1 : -1),
	 ((sin(ph+phase) > 0.0) ? 1 : -1) );
}

int main(int argc, char* argv[])
{
  string filename = "", field = "", xfield = "", yfield = "", reffield = "";
  int fframe = -1, nframes = -1;
  int retval;

  /* parse command line arguments */
  while ( (retval = getopt(argc, argv, "f:d:r:x:y:c:")) != -1) {
    switch (retval) {
    case 'f':
      filename = optarg;
      break;
    case 'd':
      field = optarg;
      break;
    case 'x':
      xfield = optarg;
      break;
    case 'y':
      yfield = optarg;
      break;
    case 'c':
      reffield = optarg;
      break;
    case 'r':
      *index(optarg, ':') = ' ';
      if(sscanf(optarg, "%d %d", &fframe, &nframes) != 2)  Usage(argv[0]);
      break;
    }
  }
  
  if(filename[0] == '\0') Usage(argv[0]);
  if(field[0] == '\0') Usage(argv[0]);
  if (xfield[0] == '\0') xfield = "STAGE_X";
  if (yfield[0] == '\0') yfield = "STAGE_Y";
  if (reffield[0] == '\0') reffield = "CHOPPER";
  if(fframe < 0 || nframes <= 0) Usage(argv[0]);
  
  cerr << "Reading field \"" << field << "\" of: " << filename << endl;
  cerr << "for: " << nframes << " frames from: " << fframe
       << " to: " << fframe+nframes << endl;
  cerr << "Lock into reference \"" << reffield << "\", position fields: \""
       << xfield <<"\" and \"" << yfield << "\"" << endl;

  /* read required fields from the dirfile */
  Dirfile df(filename.c_str(), GD_RDONLY);
  if (df.Error()) {
    cerr << "Error opening dirfile: " << df.ErrorString() << endl;
    return 1;
  }

  unsigned int spf = df.SamplesPerFrame(field.c_str());
  if (df.Error()) {
    cerr << "Error sizing getdata storage: " << df.ErrorString() << endl;
    return 1;
  }
  if (spf != df.SamplesPerFrame(xfield.c_str())
      || spf != df.SamplesPerFrame(yfield.c_str())
      || spf != df.SamplesPerFrame(reffield.c_str())) {
    cerr << "Fixme: I don't support different sample rates for D, X, Y, ref\n";
    return 1;
  }
  unsigned int Dsize = nframes*spf;
  double* D = new double[Dsize];
  double* X = new double[Dsize];
  double* Y = new double[Dsize];
  double* ref = new double[Dsize];
  double* amp = new double[Dsize];

  df.GetData(field.c_str(), fframe, 0, nframes, 0, GetData::Float64, D);
  if (df.Error()) {
    cerr << "Error getting detector: " << df.ErrorString() << endl;
    return 1;
  }
  df.GetData(xfield.c_str(), fframe, 0, nframes, 0, GetData::Float64, X);
  if (df.Error()) {
    cerr << "Error getting X: " << df.ErrorString() << endl;
    return 1;
  }
  df.GetData(yfield.c_str(), fframe, 0, nframes, 0, GetData::Float64, Y);
  if (df.Error()) {
    cerr << "Error getting Y: " << df.ErrorString() << endl;
    return 1;
  }
  df.GetData(reffield.c_str(), fframe, 0, nframes, 0, GetData::Float64, ref);
  if (df.Error()) {
    cerr << "Error getting ref: " << df.ErrorString() << endl;
    return 1;
  }

  /* filter, lock-in, etc. */
  //define fileter functions HP for pre-lockin and LP for post lock-in
  BesselHP1<double> filt_d(0.15/f_samp);
  BesselLP4<complex> filt_li(0.39/f_samp);
  double max_amp = 0.0;
  double max_y = 0.0;
  for(unsigned int i = 0; i < Dsize; i++) {

    complex pll_ref = pll(ref[i]);
    double data = filt_d(D[i]);
    amp[i] = abs(filt_li(data*pll_ref));

    //find maxima, for normalizing output
    if (amp[i] > max_amp) max_amp = amp[i];
    if (Y[i] > max_y) max_y = Y[i];
  }

  /* output (a subset of results */
  cout << std::setprecision(10);
  for(unsigned int i = 2000; i < Dsize; i+= 64) {
    //output an INDEX field to compare results relative to dirfile
    //cout << fframe + (double)i/spf << " ";
    cout <<  X[i+DELAY]*s2cm << " " << (max_y - Y[i+DELAY])*s2cm 
	 << " " << amp[i]/max_amp << endl;
  }    

  delete[] D;
  delete[] X;
  delete[] Y;
  delete[] ref;
  delete[] amp;
  return 0;
}
