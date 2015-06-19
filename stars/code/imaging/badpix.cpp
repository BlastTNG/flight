/*
 * Added by Krystle Sy to remove bad pixels.
 */

#include "badpix.h"
#include "../../shared/image/raw.h"
#include "../../tools/math.h"
#include "../../parameters/manager.h"
#include "../logger.h"


using namespace Solving::Finding;
using namespace boost::filesystem;
using std::vector;
using std::string;

#define shared_settings (*(Shared::Solving::settings.r))
#define shared_blobs (*(Shared::Image::blobs_solver_for_main.w))

Badpix::badpix(Parameters::Manager& params)
{
    image_width = params.general.image_width;
    image_height = params.general.image_height;
  //map = map_in;
  //xpix = xpix_in;
  //ypix = ypix_in;
  //bits = bits_in;
  //platescale = platescale_in;
  //npix = xpix*ypix;
  //
  //// Blob list pointers initialized to 0
  //numblobs = 0;
  //maxblobs = 0;
  //
  //firstblob = NULL;
  //lastblob = NULL;
  //thisblob = NULL;
  //
  //// Set the gridsearch array to NULL to check at the end if allocated
  //searchgrid = NULL;
  //meangrid = NULL;
  //
  //// Map statistics defaults
  //mapmean=0;
  //sigma=1;
  //satval = 16383;             // numerical value of a saturated pixel
  //satcol = new int[xpix];     // array contains # sat pixels each column
  //
  //gain=1;
  //readout_offset=0;
  //readout_noise=0;

  //// Defaults for search parameters
  //grid = 32;         // pixels/side in coarse grid search
  //threshold = 5;     // N sigma threshold criterion
  //disttol = 30*30;   // distance^2 tolerance for adjacent sources
  //
  //badpixels = NULL;

}

Badpix::~badpix(void)
{
	 delete[] badpixels;
}

int Badpix::load_badpix(char *fname) {
  FILE *badfile;
  Blob *firstbad, *thisbad, *nextbad, *lastbad;  // bad pixel list
  char thisline[81];
  int count;     // Number of elements read from the current line
  int i, x, y;   // coordinates of the bad pixel in this line
  
    
   badfile = housekeeping.load(system_complete(stars_dir + badpixlist).string());

  if( (badfile == NULL )
    return 0;
  
  // Read through file line-by-line reading in pixel coordinates.
  // Use the blob linked-list to dynamically add coordinates as we
  // read them in. At the end, put the array indices calculated from the
  // pixel coordinates into an array for quick access when we process
  // frames.
  
  numbadpix = 0;   
  firstbad = NULL; 
  
  //params.general.try_get("solver.blob_finder.badpixfilename", string("E:\stars\settings\badpixels_ISC.cam"));

  while( fgets(thisline,80,badfile) != NULL ) {
    count = sscanf(thisline,"%i %i",&x, &y);
    if( count == 2 ) { // If we read in a coordinate pair
      numbadpix ++;
      thisbad = new blob((double)x, (double)(ypix - y - 1),0);
      
      if( firstbad == NULL ) { // If first bad pixel in the list
        firstbad = thisbad;
      } else {                 // adding on new bad pixel to list
        lastbad->setnextblob(thisbad);
        thisbad->setprevblob(lastbad);
      }
      
      lastbad = thisbad;
    }
  }
  
  fclose(badfile);
  
  // Now traverse (and delete) linked list, calculating pixel coordinates
  
  badpixels = new unsigned int[numbadpix];
  thisbad = firstbad;  // start at the beginning of the list
  
  i = 0;
  
  //printf("badpixels: %i\n",numbadpix);
  while( thisbad != NULL ) {
    badpixels[i] = (unsigned int) thisbad->gety()*xpix + 
      (unsigned int) thisbad->getx();
    
    i++;
    nextbad = thisbad->getnextblob();  // get next blob
    delete thisbad;                    // delete the current one
    thisbad = nextbad;                 // move along
  }
  
  logger.log(format("Opened bad pixel file: %s with %i bad pixels.\n"
         % fname % numbadpix));

  return 1;
}



void Badpix::fix_badpix(MAPTYPE val) {
  int i;
  
  if( badpixels != NULL ) 
    for( i=0; i<(int)numbadpix; i++ ) { 
      map[badpixels[i]] = val;
    }
}

void Badpix::calc_mapstat() {
  unsigned int i, j, count,index;
  
  // initialize variables
  
  mapmean = 0;
  count = 0;
  memset(satcol,0,image_width*sizeof(int));
  
  for( j=0; j<image_height; j++ ) { // j=row
    index = j*image_width;         // inner loop columns to speed up addressing
    for( i=0; i<image_width; i++ ) {
      //printf("%i ",map[index]);
      
      if( map[index] > satval ) satcol[i]++;   // if bad pixel
      else {                // otherwise contributes to mean
        if( map[index] != satval ) { // pixel isn't bad
          mapmean += map[index];
          count ++;
        }
      }
      index++;
    }
  }
  
  if( count > 0 ) mapmean = mapmean / (double) count; // prevent  / zero
  if( mapmean < 1 ) mapmean = 1;                      // prevent 0 means
}

double Badpix::get_mapmean() {
  if( mapmean == 0 ) calc_mapstat();
  return mapmean;
}

void Badpix::setnextblob( Blob *nextblob_in ) {
  nextblob = nextblob_in;
}


void Badpix::setprevblob( bloblist *prevblob_in ) {
  prevblob = prevblob_in;
}

Blob *blob::getnextblob() {
  return nextblob;
}

Blob *blob::getprevblob() {
  return prevblob;
}