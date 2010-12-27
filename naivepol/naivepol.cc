#include <iostream>
#include <cctype>
#include <matpack.h>
#include <mpview.h>
#include <wordexp.h>
#include <string>
#include <algorithm>

#include <blasttools/blasttools.h>

using namespace std;
using namespace MATPACK;

#define FLAG      "_f_10_CBN"          // Flag extension to use
#define BOLO      "CALIB"        // Bolometer extention to use
#define POINTING  "FK5_23_MV_19_CS"
#define CALP      "_cp_21_mdt"
#define TOD_OFFS  0
#define ZERO_COLD_ENC 0.451545

#ifdef SCOFFSFILE
#undef SCOFFSFILE
#endif

#ifdef BOLO_TABLE
#undef BOLO_TABLE
#endif

#define SCOFFSFILE "sc_offs_mcm_19_MV_16_CS_modforblast06.txt"
#define BOLO_TABLE  "bolotable_blastpol_20101224_modforblast06.txt"
//#define SCOFFSFILE "sc_offs_kir.txt"
//#define BOLO_TABLE  "bolo_table_kir.txt"

#ifndef DATA_DIR
#define DATA_DIR "/data"
#endif

typedef struct {
  int f0, nf;
} range_t;

void Usage(char* name)
{
  cerr << name << " : bins up a naive map" << endl << endl;

  cerr << name << " [-F <file name>] [-f <first frame>]  [-l <last frame> | -n <numframes>]" << endl;
  cerr << "\t[-r <firstframe:numframes>] [-y <channel>] " << endl;
  cerr << "\t[-C <Colour(250, 350 or 500)>] [-o <fits file name>]" << endl;
  cerr << "\t[-s <ra(h):dec(deg):size(deg)>] [-c <smooth angle(arcsec)>]" << endl;
  cerr << "\t[-p <pixel angle(arcsec)>] [-B <bolo suffix>] [-K <flag suffix>]" << endl;
  cerr << "\t[-P <pointing suffix>] [-O <offs file>] [-D <yaw:pitch (asec)>" << endl;
  cerr << "\t[-h <freq (Hz)>] [-t <tod_offs (smaples)>] [-T] [-k] [-e] [-G] [-g] [-d] [-k]" << endl;

  cerr << endl;
  cerr << "Parameters:" << endl;
  cerr << "-F /data/blast_2005.z        : Data repository to use" << endl;
  cerr << "-f framenum                  : Starting frame" << endl; 
  cerr << "-l lastframe                 : End frame (as opposite to -n)" << endl; 
  cerr << "-n number of frames          : Number of frames to use (as opposite to -l)" << endl; 
  cerr << "-r framenum:number_of_frames : Frame range can be given also in this way " << endl;
  cerr << "-y channel                   : Channel name (can specify more than one. " << endl;
  cerr << "                                   Eg. -y N16C5 -y N15C10 -y ..." << endl; 
  cerr << "-C colour                    : use this option if you want to bin a full array." << endl;
  cerr << "                             : Eg. -C 250 for a full 250 map" << endl;
  cerr << "                                   this option is an alternative to a -y list of" << endl;
  cerr << "                                   channels to use" << endl; 
  cerr << endl;
  cerr << "-p ang(arcsec)               : Set the pixel size to ang (default 10\") " << endl;
  cerr << "-s ra(h):dec(deg):size(deg)  : If this option is set, map is drawn to have" << endl;
  cerr << "-A ra(h):dec(deg):Xc:Yc:npix_x:npix_y : If this option is set, map is drawn to have" << endl;
  cerr << "                                   ra,dec at pixel Xc, Yc and will be" << endl ;
  cerr << "                                   npix_x X npix_y pixels wide" << endl ;
  cerr << "-c ang(arcsec)               : Smooth the map with a gaussian of ang FWHM. " << endl;
  cerr << "-o filename                  : Output int filename. \".fits\" extention is added."<< endl;
  cerr << "                             :     Use \\!filename to override " << endl;
  cerr << "-B bolo_suffix               : defaults to " << BOLO << endl;
  cerr << "-K flag_suffix               : defaults to " << FLAG << endl;
  cerr << "-P pointing_suffix           : defaults to " << POINTING << endl;
  cerr << "-O offset filename           : defaults to " << SCOFFSFILE << endl;
  cerr << "-D yaw:pitch (arcsec)        : yaw and pitch (xel, el) pointing offset" << endl;
  cerr << "-h freq(Hz)                  : High pass the data at the given frequency." << endl;
  cerr << "-t tod_offs(samples)         : TOD to pointing solution offs (default " << TOD_OFFS << ")" << endl;
  cerr << "-L                           : Use Alt-Az coordinates" << endl;
  cerr << "-w                           : use weights - Each data stream is weighted" << endl;
  cerr << "                                   by 1/noise as listed in "<< endl;
  cerr << "                             :     " << BOLO_TABLE << endl;
  cerr << "-T                           : use telescope coordinates" << endl;
  cerr << "-k                           : Enable bolometer flagging" << endl;
  cerr << "-e                           : Enable pointing solution flagging" << endl;
  cerr << "-G                           : Use galactic coordinates" << endl;
  cerr << "-x                           : Calibrate with responsivity in " << BOLO_TABLE << endl;
  cerr << "-m                           : Calibrate with calpulse using " << CALP << endl;
  cerr << "-d                           : Despike Bolometers "  << endl;
  cerr << endl;
  cerr << "EXAMPLES" << endl;
  cerr << name << " -F /data/blast_2005.z -f 657000 -l 689000 -y N7C13  \n\t -o \\!CRL2688" << endl; 
  cerr << "   make a single pixel map of CRL2688 in CRL26688_flux.fits" << endl;
  cerr << endl;
  cerr << name << " -F /data/blast_2005.z -f 657000 -l 689000 -y N7C13 -y N7C14 -y N9C20  \n\t -o \\!CRL_2688" << endl; 
  cerr << "   make a 3 pixels map of CRL2688 in CRL26688_flux.fits" << endl;
  cerr << endl;
  cerr << name << " -F /data/blast_2005.z -f 657000 -l 689000 -C 250   \n\t  -o \\!CRL_2688 -p 10 -c 20" << endl; 
  cerr << "   make a full 250 map of CRL2688 with 10\" pixel, convolve with 20\" FWHM" << endl;
  cerr << endl;
  cerr << name << "  -F /mnt/blast_pol/map_maker_data/ -f 732000 -l 736704 -C 500 -L -B "" -h 0.3 -t -6 -d -o \\!test -A 7.385:-25.8:250:250:500:500" << endl;
  cerr << "   test map for naivepol" << endl;
  exit(0);
}

string to_lower(string s) {
  transform(s.begin(), s.end(), s.begin(), (int(*)(int))std::tolower);
  return s;
}


char* ShellExpand(const char* argument)
{
  char* ptr = NULL;
  wordexp_t expansion;

  /* shell expand the path, if necessary */
  if (wordexp(argument, &expansion, 0) != 0) {
    cerr << "unable to expand file path" << endl;
    exit(0);
  }

  if (expansion.we_wordc > 1) {
    cerr << "cannot handle multiple expansion of " <<  argument << endl;
    exit(0);
  }

  ptr = strdup(expansion.we_wordv[0]);

  /* clean up */
  wordfree(&expansion);

  return ptr;
}

int main(int argc, char *argv[])
{
  const string data_etc   = ShellExpand(DATA_DIR) + string("/etc/");

  int retval;
  
  string filename   = "";
  int fframe        =  0;
  int lframe        = -1;
  int nframes       = -1;

  string fits                = "";
  bool save                  = false;
  bool gauss_fit             = false;
  bool map_rel_source        = false;
  bool map_astrometry        = false;
  bool telescope_coordinates = false;
  bool use_pointing_flags    = false;
  bool use_bolo_flags        = false;
  bool bolo_despike          = false;
  bool galactic_coord        = false;
  bool altaz_coord           = false;
  bool weight_from_file      = false;
  bool calibrate             = false;
  bool calpulse              = false;
  double source_ra           = -100.0;
  double source_dec          = -100.0;
  double crpix_x             = -100.0;
  double crpix_y             = -100.0;
  
  double smooth_angle        = -1.0;
  double map_size            = DEG2RAD(14.0/60.0);
  int    npix_x              = 0;
  int    npix_y              = 0;
  double pixel_size          = ASEC2RAD(10.0);
  double highpassfrq         = -1.0;
  double d_pitch             = 0.0;
  double d_yaw               = 0.0;
  string bolo_suffix         = BOLO;
  string flag_suffix         = FLAG;
  string calpulse_suffix     = CALP;
  string pointing_suffix     = POINTING;
  string offs_filename       = data_etc+ShellExpand(SCOFFSFILE);
  int    tod_offs            = TOD_OFFS;
  
  const string bolo_table_filename = data_etc+BOLO_TABLE;
  BoloInfo binfo((char *)bolo_table_filename.c_str());
  
  list<range_t>::const_iterator irg;
  list<range_t> range;

  list<string>::const_iterator ich;
  list<string> channel;
  
  while ( (retval = getopt(argc, argv, "F:f:l:y:n:o:c:s:A:r:C:p:B:K:O:P:D:h:t:LmTkedgGwx")) != -1) {
    switch (retval) {
    case 'F':
      filename = ShellExpand(optarg);
      break;
    case 'f':
      fframe = atoi(optarg);
      break;
    case 'l':
      lframe = atoi(optarg);
      break;
    case 'n':
      nframes = atoi(optarg);
      break;
    case 'y':
      channel.push_back(optarg);
      break;
    case 'C':
      if(strncmp(optarg, "500", 3) == 0) channel = binfo.b500;
      else if(strncmp(optarg, "350", 3) == 0) channel = binfo.b350;
      else if(strncmp(optarg, "250", 3) == 0) channel = binfo.b250;
      break;
    case 'o': 
      save = true;
      fits = optarg;
      break;
    case 'g':
      gauss_fit = true;
      break;
    case 'c':
      smooth_angle = atof(optarg);
      break;
    case 'p':
      pixel_size = ASEC2RAD(atof(optarg));
      break;
    case 's':
      if(sscanf(optarg, "%lf:%lf:%lf", &source_ra, &source_dec, &map_size) != 3)  Usage(argv[0]);
      source_ra *= M_PI/12.0;
      source_dec *= M_PI/180.0;
      map_size *= M_PI/180.0;
      map_rel_source = true;
      break;
    case 'A':
      if(sscanf(optarg, "%lf:%lf:%lf:%lf:%d:%d", &source_ra, &source_dec, &crpix_x, &crpix_y, &npix_x, &npix_y) != 6)  
	Usage(argv[0]);
      source_ra  *= M_PI/12.0;
      source_dec *= M_PI/180.0;
      map_astrometry = true;
      break;
    case 'r':					
      range_t rtmp;
      if(sscanf(optarg, "%d:%d", &rtmp.f0, &rtmp.nf) != 2)  Usage(argv[0]);
      range.push_back(rtmp);
      break;
    case 'B':
      bolo_suffix = optarg;
      break;
    case 'K':
      flag_suffix = optarg;
      break;
    case 'P':
      pointing_suffix = optarg;
      break;
    case 'L':
      altaz_coord = true;
      break;
    case 'O':
      offs_filename = ShellExpand(optarg);
      break;
    case 'D':
      if(sscanf(optarg, "%lf:%lf", &d_yaw, &d_pitch) != 2)  Usage(argv[0]);
      d_yaw   *= M_PI/180.0/3600.0;
      d_pitch *= M_PI/180.0/3600.0;
      break;
    case 'T':
      telescope_coordinates = true;
      break;
    case 'd':
      bolo_despike = true;
      break;
    case 'k':
      use_bolo_flags = true;
      break;      
    case 'e':
      use_pointing_flags = true;
      break;      
    case 'G':
      galactic_coord = true;
      break;      
    case 'h':
      highpassfrq = atof(optarg);
      break;
    case 'w':
      weight_from_file = true;
      break;
    case 'x':
      calibrate = true;
      break;
    case 't':
      tod_offs = atoi(optarg);
      break;
    case 'm':
      calpulse = true;
      calpulse_suffix = CALP;
      break;
    }
  }
  
  if(filename[0] == 0) Usage(argv[0]);
  if(channel.size() == 0) Usage(argv[0]);
  if(range.size() == 0) {
    if(lframe == -1) {
      if(nframes == -1) nframes = GetFrames(filename) - fframe;
      lframe = fframe + nframes - 1;
    } else {
      nframes = lframe - fframe + 1;
    }
    range_t rtmp;
    rtmp.f0 = fframe; rtmp.nf = nframes;
    range.push_back(rtmp);
  }

  cerr << "Reading... " << endl;

  for(irg = range.begin(); irg != range.end(); irg++) {
    cerr << "\tfrom frame: " << irg->f0 << " to frame: " << irg->f0 + irg->nf - 1 ;
    cerr << " (" << irg->nf << ")." << endl;
  }

  cerr << "             Using bolos: " << bolo_suffix <<  endl;
  cerr << "             Pointing: " << pointing_suffix << endl;
  cerr << "             BoloTable: " <<  bolo_table_filename << endl;
  cerr << "             StarCameraOffset: " << offs_filename << endl;
  cerr << "             Using pointing flags: " << ((use_pointing_flags) ? "YES" : "NO") << endl;
  if(use_bolo_flags)
    cerr << "             BoloFlags: " << flag_suffix << endl;

  Map map;
  Data* Yaw;  
  Data* Pitch;
  Data* Roll;
  Data* Yaw_r;
  Data* Pitch_r;
  Data* Roll_r;
  char* PFlag = NULL;

  irg = range.begin();
  map.Info(filename, irg->f0, irg->nf, binfo.Color(*(channel.begin())));
  
  if(map_rel_source) {
    int type = (galactic_coord) ? MAP_IS_GAL : MAP_IS_RADEC;
    map.OpenPol(pixel_size, source_ra, source_dec, map_size, type);
  } else if(map_astrometry) {
    int type = (galactic_coord) ? MAP_IS_GAL : MAP_IS_RADEC;
    map.OpenPol(pixel_size, source_ra, source_dec, crpix_x, crpix_y, npix_x, npix_y, type);
  } else {
    cerr << "You must define a map opening method." << endl;
    Usage(argv[0]);
  }

  cerr << "Binning map ... " << endl;
  for(irg = range.begin(); irg != range.end(); irg++) {

    // Polarization angle = 2 * (theta-chi) - ro_roll + [0 1]*!pi/2 + PA
    Data* AngPol = new Data(filename, string("POT_HWPR"), irg->f0, irg->nf);
    *AngPol      = 4.0*M_PI*(ZERO_COLD_ENC - (*AngPol)); // 2 * (theta-theta0); chi has yet to be implemented
     AngPol->ReGrid(20);
    cerr << "Reading Pointing Solution ... " << flush;
    if(galactic_coord) {
      Yaw      = new Data(filename, string("GLON_")  + pointing_suffix, irg->f0, irg->nf);
      Pitch    = new Data(filename, string("GLAT_")  + pointing_suffix, irg->f0, irg->nf);
      Roll     = new Data(filename, string("GPHI_")  + pointing_suffix, irg->f0, irg->nf);
      *Yaw   *= M_PI/180.0;
      *Pitch *= M_PI/180.0;
      *Roll  *= M_PI/180.0;
    } else if (altaz_coord) {
      Yaw   = new Data(filename, string("AZ"), irg->f0, irg->nf);
      Pitch = new Data(filename, string("EL"), irg->f0, irg->nf);
      Roll  = new Data(Yaw->Lo(), Yaw->Hi(), 0.0);

      Data *Lst   = new Data(filename, string("LST"), irg->f0, irg->nf);
      Data *Lat   = new Data(filename, string("LAT"), irg->f0, irg->nf);

      *Yaw   *= M_PI/180.0;
      *Pitch *= M_PI/180.0;
      *Roll  *= M_PI/180.0;
      *Lst   *= M_PI/12.0;
      *Lat   *= M_PI/180.0;
	
      double ra_temp, dec_temp;
      for (int k = Yaw->Lo(); k <= Yaw->Hi(); k++) {
	RaDec((*Yaw)[k], (*Pitch)[k], (*Lat)[k / 20], (*Lst)[k / 20], &ra_temp, &dec_temp); //LST and LAT are slow channels
	(*Yaw)[k]   = ra_temp;
	(*Pitch)[k] = dec_temp;
        (*Roll)[k]  = Pa((*Yaw)[k], (*Pitch)[k], (*Lst)[k / 20], (*Lat)[k / 20]);
      }

      delete Lst;
      delete Lat;

    } else {
      Yaw      = new Data(filename, string("RA_")    + pointing_suffix, irg->f0, irg->nf);
      Pitch    = new Data(filename, string("DEC_")   + pointing_suffix, irg->f0, irg->nf);
      Roll     = new Data(filename, string("PHI_")   + pointing_suffix, irg->f0, irg->nf);
      *Yaw   *= M_PI/12.0;
      *Pitch *= M_PI/180.0;
      *Roll  *= M_PI/180.0;
    } 

    *AngPol += *Roll; // Add Roll to polarization angle.(if PA > 0 CCW and roll angle > 0 CW)
    // Parallactic angle = the angle as seen from the target between zenith and NCP, measured positive westward of meridian

    if(use_pointing_flags) {
      string flag = "FLAG_" + pointing_suffix;
      PFlag = new char [Yaw->Elements()];
      get_data(filename, irg->f0, 0, Yaw->Elements(), PFlag, flag, UInt8);
    }

    Yaw_r    = new Data(Yaw->Lo(), Yaw->Hi());
    Pitch_r  = new Data(Yaw->Lo(), Yaw->Hi());
    Roll_r   = new Data(Yaw->Lo(), Yaw->Hi());
        
    cerr << "done." << endl;
    
    for(ich = channel.begin(); ich != channel.end(); ich++) {
      cerr << "Reading channel " << setw(6) << *ich << "... " << flush;
      Data* Bolo   = new Data(filename, *ich + bolo_suffix, irg->f0, irg->nf);
      char *Flag   = new char [Bolo->Elements()];
     
      if(use_bolo_flags) {
	string flag = to_lower(*ich)+flag_suffix;
	get_data(filename, irg->f0, 0, Bolo->Elements(), Flag, flag, UInt8);
      } else {
	memset(Flag, 0, Bolo->Elements());
      }

      if(use_pointing_flags) {
	for(int idx = 0; idx < Bolo->Elements(); idx++)
	  Flag[idx] |= PFlag[idx];
      }

      if(bolo_despike) {
      	cerr << "despiking... ";
	Bolo->CRDespike(Flag, 4.5, 4.0, 5.0, 100);
	Bolo->FillSpike(Flag);
      }

      Flag -= Bolo->Lo();

      if(highpassfrq > 0.0) {
	
	cerr <<  "PoliSub... "<< flush;
	Vector* X = new Vector(Bolo->Lo(), Bolo->Hi(), 0.0);
	double* Y = new double [5];
	double x;
	int idx;

	for(idx = X->Lo(); idx <= X->Hi(); idx++) (*X)[idx] = (double)idx/(double)X->Elements();
	polyfit(X->Store(), Bolo->Store(), Bolo->Elements(), 5, Y, Flag);
	
	for(idx = Bolo->Lo(); idx <= Bolo->Hi(); idx++) {
	  x = (*X)[idx];
	  (*Bolo)[idx] -= (((Y[4]*x + Y[3])*x +Y[2])*x + Y[1])*x + Y[0];
	}
	delete X;
	delete Y;
      }

      if(calpulse) {
	cerr << "CalPulse... " << flush;
	string calp = to_lower(*ich)+calpulse_suffix;

	Data *CP = new 	Data(filename, calp, irg->f0, irg->nf);
	for(int idx = Bolo->Lo(); idx <= Bolo->Hi(); idx++)
	  (*Bolo)[idx] *= (*CP)[idx/FAST_PER_SLOW];
	delete CP;
      }

      if(highpassfrq > 0.0) {
	cerr << "prepare fft... " << flush; 
	Bolo->PrepareFFTVectorExtrap(0);
        //Bolo->PrepareFFTVector(100); // **LORENZO** 100 is the number of frames read before and after for padding
	cerr << "highpassing... " << flush; 
	Bolo->HighPass(highpassfrq);
	Bolo->RecoverFFTVector();
      }

      if(calibrate) {
	cerr << "calibrating... " << flush;
	Bolo->Calibrate( -binfo.Resp(*ich) , 0.0);
      } else {
	Bolo->Calibrate(-1.0, 0.0);
      }

      cerr << "Rotating PS... " << flush;
      double bs_yaw, bs_pitch;
      SCOffset(binfo.Color(*ich), irg->f0, bs_yaw, bs_pitch, offs_filename.c_str());
      binfo.SCOffset(bs_pitch+d_pitch, bs_yaw+d_yaw); 

      binfo.Rotate(*ich, *Yaw, *Pitch, *Roll, *Yaw_r, *Pitch_r, *Roll_r);
      
      double weight = (weight_from_file) ? 1.0/binfo.Noise(*ich) : 1.0;

      cerr << "binning (w = " << weight << ") ... " << flush;
      //map.ChannelAdd(*Bolo, Flag, *Yaw_r, *Pitch_r, *Roll_r, tod_offs, telescope_coordinates, weight);
      *AngPol += binfo.Ang(*ich)*M_PI/180.0; // Add detector grid angle (as per bolotable) to polarization angle.
      //cerr << binfo.Ang(*ich)*M_PI/180.0 << endl;
      map.ChannelAddPol(*Bolo, *AngPol, Flag, *Yaw_r, *Pitch_r, *Roll_r, tod_offs, telescope_coordinates, weight);

      cerr << "done." << endl;    
    
      delete[] Flag;
      delete Bolo;
    }

    delete AngPol;

    if (use_pointing_flags) delete[] PFlag;
    delete Yaw; delete Pitch; delete Roll;
    delete Yaw_r; delete Pitch_r; delete Roll_r;
  }
  map.ClosePol();

  cerr << "Variance in the map : " << map.Variance(ASEC2RAD(300)) << endl;

  if(smooth_angle > 0.0) {
    cerr << "Convolving whith a gaussian of " << smooth_angle << " arcsec fwhm\n";
    map.Convolve(ASEC2RAD(smooth_angle));
  }

  if(save) {
    cerr << "Writing to fits: " << fits << endl;
    map.WriteToFitsPol(fits, SAVE_FLUX);
    map.WriteToFitsPol(fits, SAVE_Q);
    map.WriteToFitsPol(fits, SAVE_U);
    map.WriteToFitsPol(fits, SAVE_NOISE);
    map.WriteToFitsPol(fits, SAVE_NOISE_Q);
    map.WriteToFitsPol(fits, SAVE_NOISE_U);
    map.WriteToFitsPol(fits, SAVE_HITS);
  }

  if(gauss_fit) {
    Vector param;
    GaussFit(map, param, 10);
    
    double fwhm = 2.354*sqrt(1.0/param[2]);

    double x_c = param[0];
    double y_c = param[1];

    map.xy2tp(x_c, y_c);
    tp2s(x_c, y_c, map.XiC(), map.EtaC());
    
    cerr << "Gaussian fit paramenters x0 y0 fwhm Amp offs | xi0 eta0 fwhm flux\n";
    cout << param[0] << " " << param[1] << " " << fwhm << " " << param[3] << " " << param[4] << " ";
    cout << " | " ;
    cout << RAD2DEG(x_c) << " " << RAD2DEG(y_c) << " " << RAD2ASEC(fwhm*map.XScale()) << " ";
    cout << map.Flux(1.5*fwhm) << endl;

    if(1) {
      int idx;
      double arg;
      ofstream outtxt;
      outtxt.open("gfit_yaw.dat");
      for(idx = map.Rlo(); idx <= map.Rhi(); idx++) {
        outtxt << map[idx][(int)nearbyint(param[1])] << " ";
	arg = 0.5 * (SQR(idx-param[0]) + SQR((int)param[1]-param[1])) * param[2];
	outtxt << param[3]*exp(-arg) + param[4] << " "; 
	outtxt << map.Hits(idx, (int)nearbyint(param[1])) << " "; 
	outtxt << map.Sd(idx, (int)nearbyint(param[1])) << endl; 
      }

      outtxt.close();
      outtxt.open("gfit_pitch.dat");
      for(idx = map.Clo(); idx <= map.Chi(); idx++) {
	outtxt << map[(int)nearbyint(param[0])][idx] << " ";
	arg = 0.5 * (SQR((int)param[0]-param[0]) + SQR(idx-param[1])) * param[2];
	outtxt << param[3]*exp(-arg) + param[4] << " ";
	outtxt << map.Hits((int)nearbyint(param[0]), idx) << " "; 
	outtxt << map.Sd((int)nearbyint(param[0]), idx) << endl; 
      }
      outtxt.close();
    }
    
    
  }
  
 

  return 0;
}
