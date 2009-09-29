// 
// Implementation of matching algorithm
//

#include <iostream>
#include <math.h>

#include <qvaluelist.h>
#include <qapplication.h>

#include "Match.h"
#include "AnnularRegion.h"
#include "HTMCatalog.h"
#include "Units.h"

Match::Match(const Frame &f, Catalog& c, const RegionGS& sr) :
  region(sr.clone()), catalog(c), frame(f) {

  setTolerance( Degree(0.04) ); // 1.2 arcminutes;
  setMatchFraction( 0.4 );      // 50% fraction
  quitFraction = 0.9;           // Return when this fraction is matched
  maxMatched = 0;
}
  
Match::~Match() {
  delete region;
}

QValueList<Frame> Match::match() {
  maxMatched = 0;
  
  // Frames to return
  QValueList<Frame> ret;
  
  if (!region) {
    qDebug("No region defined.  Returning with no match");
    return ret;
  }
  
  // For each star in search region
  QValueList<Star> found = catalog.find( *region );
  qDebug( "Match found %d stars to check as primary.", found.count());
  printf( "Match found %d stars to check as primary.", found.count());
  
  //emit primaryStars( found );
  
  //emit compareTotal( found.count() );
  int i = 0;
  QValueList<Star>::iterator it;
  for (it = found.begin(); it != found.end(); it++) {

    //qDebug( "Checking match %d", i);
    //qDebug( "Primary star is (Ra(H), Dec(deg), mag) = (%lg, %lg, %lg)", 
    //(*it).getRa( Hour() ), (*it).getDec( Degree() ),
    //(*it).getMagnitude() );
    //emit compareProgress(++i);
    //qApp->processEvents();
    //   Find all second stars with same spacing as the Frame 
    // This is a circle just a bit larger than the frame spacing
    MinorCircleRegion mcr( (*it), frame.getAxisLength() + 
			   Radian(tolerance()/2) );

    // Use the magnitude limit of the search region.
    if (region) mcr.setMagnitudeLimit( region->magnitudeLimit() );
    AnnularRegion ar( mcr, tolerance());
    ar.setMagnitudeLimit( mcr.magnitudeLimit() );
    QValueList<Star> second = catalog.find( ar );
    QValueList<Star>::iterator jt;
    
    frame.setPrimary( (*it) );

    //   For each second star
    qDebug( "Checking %d stars as secondary",  second.count() );

    for (jt = second.begin(); jt != second.end(); jt++) {
      //qDebug( "Secondary is (ra(H), dec(deg), mag) = (%lg, %lg, %lg)",
      //(*jt).getRa( Hour() ), 
      //(*jt).getDec( Degree() ),
      //(*jt).getMagnitude() );
      // Assign to frame
      frame.setSecondary( (*jt) );
      
      // Find all matches in the catalog
      QValueList<Star> blob = frame.getStars();
      QValueList<Star>::iterator kt;
      int matches = 0;
      int bCount = 1;
      for (kt = blob.begin(); kt != blob.end(); kt++) {
        //qDebug( "Blob %d maps to (ra(H), dec(deg)) = (%lg, %lg)", 
        //    bCount++,
        //    (*kt).getRa( Hour() ), 
        //    (*kt).getDec( Degree() ) );
        MinorCircleRegion tcr(*(kt), tolerance() );
        tcr.setMagnitudeLimit( region->magnitudeLimit() );
        QValueList<Star> pairing = catalog.find( tcr );
        if (pairing.size() > 0) {
	  // Assign the first star in this mini circle to the blob
	  
	  matches++;
        } 
      }
      
      //qDebug( "   Matched %d blobs", matches );
      if (matches > maxMatched) {
	maxMatched = matches;
	//emit newMax( maxMatched );
      }
      
      // Save this frame if it is good enough
      if ( matches / double(blob.count()) > matchFraction() ) {
        ret.append( frame ); // Make a copy of the frame as it stands now.
      }
      
      // Return if this is enough to satisfy the quit limit.
      if ( matches / double(blob.count()) > quitFraction ) {
	return ret;
      }
      
    } // End of loop over second stars
    
  } // End of loop over primary stars
  
  return ret;
} // End of method match()


void Match::setMagnitudeLimit( double limit ) {
  if (region) region->setMagnitudeLimit( limit );
}
