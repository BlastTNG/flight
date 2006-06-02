How to use the frameblob class
==============================

First create a frameblob object, and call the constructor:

frameblob Frameblob;
Frameblob.commonconstructor( (MAPTYPE *) QCFrame.pBuffer, xpix, ypix, 14, (double)0.00194 );

The QCFrame.pBuffer is simply the buffer where the image is stored (I
think I lifted this straight from the example in the QImaging
API). MAPTYPE is just an unsigned short int - this gets defined in the
frameblob header file. xpix and ypix are the number of pixels in the
x- and y-directions (1312 & 1024 for our camera), 14 is the number of
bits used, and the last number is the platescale (degrees / pixel
along a side, not the diagonal). I don't think I actually use these
last two parameters for anything... I think that I was originally
planning on having the pointing solution included in the frameblob
class.

Next set up the noise model. In order to estimate the significance of
individual pixels, we need some estimate of the SNR. I don't measure
the sample standard deviation for the entire image for two reasons:
(1) It can be a bit slow (first the total map mean must be measured,
then multiplication + subtraction for each pixel), and (2) if there
are lots of bright objects in the image, the total standard deviation
can easily be skewed upward.  Instead, I use a simple noise model that
is a function of the total map mean (just requires summing over all
the pixels in the map once). Assuming that most of the pixels have
noise that is drawn from the same parent Poisson distribution, the
standard deviation in a pixel (sigma) is just:

sigma = sqrt( G(m - O) )

G = gain in digitized units / e-
m = measured mean from all the pixels in the map
O = some offset (readout noise, amplifier offset, whatever)

I measure the constant G and O from a bunch of test images. Keeping
the offset / gain settings of your camera constant, take a bunch of
exposures from pure darkness (background limited regime) all the way
to saturation. I measured the standard deviation for each one of these
images in some small square (16 x 16 pixels say) in a relatively
"flat" looking region of the images. Then fitting for the two
constants was straight forward. I've included a plot to show you what
I mean (x-axis is the map mean, y-axis is the measured pixel standard
deviation - noisemodel.eps), and I derive G and O for our camera using
the preamp gain/offset settings we used in the BLAST test flight.

Frameblob.set_gain(0.677);
Frameblob.set_readout_offset(-209.2);
Frameblob.set_readout_noise(0);    // not used, but set it to 0

Next setup the search parameters. The basic algorithm is as follows:
Divide up the entire image into grid cells of size "grid" on a
side. Calculate the mean pixel value in each one of these cells which
we use as a local baseline correction. Check to see if any of the
(pixels - mean) have a SNR (where sigma is estimated from the _total_
map mean) greater than some threshold (5 for example). If the
brightest pixel in the cell is significant, flag that pixel. Once all
of the cells have been analyzed in this way, calculate a centroid in
some box (cenbox pixels on a side) around each of these significant
pixels (should be better than just taking the brightest pixel as the
center). This also re-calculates the baseline estimate for the source
by calculating the mean of the pixels around the perimeter of the
cenbox so that it is not affected by the flux in the blob.  Finally,
calculate the flux of each source in some aperture box (apbox). This
uses the improved baseline estimate, and also checks to see if the
flux _excluding_ the original bright pixel is significant - this is
how we check to see if the source is extended (real) or not.

// --- one-time setup ------------------------------

Frameblob.set_satval((MAPTYPE)16382);
Frameblob.set_threshold(5);
Frameblob.set_disttol(900);   // see notes on fix_multiple - this is the dist^2
Frameblob.set_grid(38);
Frameblob.set_cenbox(20);
Frameblob.set_apbox(5);

// we have a list of extra-noisey pixels that get ignored (just a table of x + y values)
Frameblob.load_badpix(badpixfilename);

// to ensure that we don't run out of memory, define a max number of blobs that may be
// detected
Frameblob.set_maxblobs(1000); 

// ---run this each time a new frame is exposed --------

Frameblob.calc_mapstat();      // calculate the map mean for all pixels < 16382
Frameblob.fix_badpix(Frameblob.get_mapmean()); // set all the noisey pixels to the map mean
Frameblob.calc_mapstat();      // redundant, but doesn't take any time
Frameblob.calc_searchgrid();   // identify bright pixels
Frameblob.calc_centroid();     // calculate centroids + improved baseline estimate
Frameblob.calc_flux();         // sum up the flux in the apbox, decide if extended or not

A slight problem with this algorithm is that is can detect a source
multiple times (especially if it happens to land part way between
severl of the original grid cells). I fix this up by searching the
full list of detected blobs (ignoring the point-sources) and checking
to see if several blobs are too close to eachother, and eliminating
all but the brightest in the group. This tolerance distance is
sqrt(disttol) - so I put 900 above, meaning that the algorithm
prevents multiple detections of a source within 30 pixels.

Frameblob.fix_multiple();      // blobs must be sqrt(disttol) pixels apart 

Finally I sort the list in order of descending flux to speed-up the pattern matching.

Frameblob.sortblobs();

The detected blobs are stored in a linked list. Here is a simple piece of code to show how you
get out the blob positions:

// get blob information
bloblist *blobs = Frameblob.getblobs();
int numextended = 0;   // count extended objects in the frame
int numpoint = 0;      // count point-like objects in the frame

double this_x, this_y, this_flux, this_sn;

while( blobs != NULL )
{	
  if( blobs->gettype() == 2 )
  {
    if( numextended < MAX_ISC_BLOBS )
    {
      this_x = blobs->getx();
      this_y = CCD_Y_PIXELS - blobs->gety() - 1; // want y increasing with elevation
      this_flux = blobs->getflux();
      this_sn = blobs->getsnr();

    }
    numextended++;  
  }
  else numpoint++;

  blobs = blobs->getnextblob();	
}


Basically you will need to figure out G and O for your camera. If that
is correct, you will probably only need to fiddle with the SNR
detection threshold. I usually use something between 5 and 10. The
other parameters I found by trial and error (i.e. grid, cenbox, apbox)
- you may very well find better parameters (in which case I'd like to
know!) but in general we were very pleased with the performance of
this algorithm using these parameters. Since significant pixels are
checked against there _local_ background (i.e. inside the grid cells)
it is pretty insensitive to gradients across the image (even though
the noise model uses the total map mean). Just as an example, we got
pointing solution looking out the Ft. Sumner high-bay door when nearly
1/2 of the image was dominated by reflected light from inside the
building because we looked too high and saw the wall.

If you need more help/explanation, email Edward Chapin:

echapin@mail.sas.upenn.edu
