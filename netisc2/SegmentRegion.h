//
// SegmentRegion.h
//

// 
// Defines classes and methods for creating intersecting circular regions.
// Optimized to find if intersection of two SegmentRegions is empty or not.


#ifndef SEGMENT_REGION_H
#define SEGMENT_REGION_H

#include <list>
#include "Region.h"

// Class used to support the SegmentRegion
class Segment : public MinorCircleRegion {
 public:

  // Creates a complete segment from the given minor circle region
  Segment(const MinorCircleRegion &); 

  // Creates a segment from p1->p2 with a GreatCircle as the circle.
  Segment(const Point& p1, const Point& p2); 

  // Required by template used in SegmentRegion
  Segment();

  // Returns the starting point of the segment
  Point getStart() const; 

  // Returns the first point of intersection along the Segment, or
  // null if there is no intersection.  Ownership of the point is
  // given to the caller.
  Point* intersect(const Segment &) const;
  
  // Returns the angle at which the Point appears on the segment
  // meridian sweep.  Returns OFF_SEGMENT if the point is not on the
  // segment.
  Radian sweep(const Point&) const; 
  
  // Returs true if the Point is on the segment, or within the segment
  // wedge.  Returns false if the Point is on the MinorCircle but not
  // on the Segment.  Returns false if the Point is outside the wedge.
  bool inSegmentWedgeQ(const Point &) const;  
  
  // Larger than any possible theta returned from sweep.
  static const double OFF_SEGMENT; 

private:

  // Returns the point along circle at angle theta
  Point getPoint(Radian theta) const;  
  void init(); // Common initialization code

  Radian theta;  // Length of the segment along the circle
  Point cen;     // This is direction toward the center of the Segment
  Point start;   // This is direction from the center toward the start point
  Point toward;  // This is the direction orthogonal to the center and
		 // the start, and obeys t = c x s.

  // Together the above three make an orthonormal system (cen, start,
  // toward) and also the right hand orthonormal system (start,
  // toward, cen)
};

// Declare that we want to write the next function

class SegmentRegion : public RegionGS {
 public:
  SegmentRegion(const MinorCircleRegion &); // Makes a circluar region.
  SegmentRegion(const TriangleRegion &);    // Makes a Triangle region.
  
  // Methods required by RegionGS
  virtual Location location(const Point& p) const;
  virtual Size size() const; // Tells if the region is empty or not.
  virtual double lowerBound() const;
  virtual double upperBound() const;
  virtual RegionGS* clone() const;
  
  // Intersection Methods

  // Returns true if there is an intersection between the two segments
  bool intersectQ(const SegmentRegion&) const;  
  
 protected:

  // Creates an empty SegmentRegion
  SegmentRegion(); 

  // Creates a segment region from a segment list.
  SegmentRegion(std::list<Segment>); 
  
private:

  // These are the segments that make up this Region
	std::list<Segment> segment;  
  Size s;
  
  // This is here because the coding is incomplete.  Returns the
  // intersection of two Segment Regions
  SegmentRegion intersect(const SegmentRegion&) const;  
};

#endif // SEGMENT_REGION_H







