//
// SegmentRegion.cpp
//

// Implementation of SegmentRegion

#define _USE_MATH_DEFINES
#include <math.h>

#include "SegmentRegion.h"

using namespace std;

const double Segment::OFF_SEGMENT = 7;

// Segment
Segment::Segment(const MinorCircleRegion& mcr) :
  MinorCircleRegion(mcr), 
  cen(1,0,0), start(1,0,0), toward(1,0,0) {
  init();
}

Segment::Segment(const Point& p1, const Point& p2) :
  MinorCircleRegion( GreatCircleRegion(p1, p2) ), 
  cen(1,0,0), start(1,0,0), toward(1,0,0) {

  // The start direction is toward p1
  start = p1;
  cen = getDirection();
  toward = cen.cross( start );
  
  // The length of the segment
  theta = acos( p1.dot( p2 ));
}

Segment::Segment() :
  MinorCircleRegion( Point(0, 0, 1), Radian(0) ),
  cen(1,0,0), start(1,0,0), toward(1,0,0) {
  init();
}

void Segment::init() {
  cen = getDirection();
  
  // Check the sizes of the x and y components
  double x = cen.getX();
  double y = cen.getY();
  
  // Make an orthogonal vector, but be sure not to swap a pair of zeo
  // coordinates.
  Vector v = (fabs(x)<fabs(y)) ? Vector(0, cen.getZ(), -y) : 
    Vector(cen.getZ(), 0, -x);

  start = Point( v );
  toward = cen.cross( start );
  
  theta = 2*M_PI;
} 

Point Segment::getPoint(Radian t) const {
  double d = getDistance();
  double od = sqrt(1-d*d);
  double c = cos(t)*od;
  double s = sin(t)*od;
  //cout << "\t\tDistance: " << d << endl;
  //cout << "\t\tStart: " << start << endl;
  //cout << "\t\tToward: " << toward << endl;
  //cout << "\t\tEnd: " << cen << endl;
  return Point( c*start + s*toward + d*cen );
}

Point Segment::getStart() const {
  return getPoint( Radian(0) );
}

// This call is the meat of it all.  Detemining the point of
// intersection of two segments.
Point* Segment::intersect(const Segment &s2) const {
  // Find the line vector b and cross products of the planes
  Point n1 = getCenter();
  Point n2 = s2.getCenter();
  double p = n1.dot(n2);
  double d1 = getDistance();
  double d2 = s2.getDistance();
  double alpha = (d1-p*d2)/(1-p*p);
  double beta = (d2-p*d1)/(1-p*p);
  Vector b = alpha*n1 + beta*n2;
  
  // If length of b is greater than one, there is no intersection of
  // segments (note, there still may be intersection of the regions
  // via complete inclusion)
  
  double len = b.length();
  
  //cout << "\tThis segment starts at " << getStart() << endl;
  //cout << "\tThis segent has length " << Degree(theta) << endl;
  //cout << "\tThe comparison segment starts at " << s2.getStart() << endl;
  //cout << "\tThe comparison segent has length " << Degree(s2.theta) << endl;
  //cout << "\tIntersection parameter is " << len << endl;
  
  if (len > 1) return 0;
  
  // Here are the two possible points.  
  double clen = sqrt(1 - len*len); // Complement length
  Vector lvec = n1.cross(n2); // Line vector
  Point p1( b + clen*lvec );
  Point p2( b + -clen*lvec );
  
  // Look for the first intersection.
  Radian t1 = sweep(p1);
  Radian t2 = sweep(p2);
  Point pmin = (t1 < t2) ? p1 : p2;

  if (inSegmentWedgeQ( pmin ) && s2.inSegmentWedgeQ( pmin )) 
    return new Point(pmin);

  Point pmax = (t1 > t2) ? p1 : p2;
  if (inSegmentWedgeQ( pmax ) && s2.inSegmentWedgeQ( pmax )) 
    return new Point(pmax);

  // Points were not in wedges.
  return 0;
}

Radian Segment::sweep(const Point& p) const {

  // Find the projection of Point along the start and toward axes.
  double x = start.dot( p );
  double y = toward.dot( p );
  Radian angle = fmod( atan2(y,x) + 2*M_PI, 2*M_PI);
  if (angle > theta) return OFF_SEGMENT ;
  return angle;
}

bool Segment::inSegmentWedgeQ(const Point& p) const {
  if (sweep(p) == OFF_SEGMENT) return false;
  return true;
}


//  SegmentRegion

SegmentRegion::SegmentRegion(const MinorCircleRegion &mcr) {
  segment.push_back( Segment(mcr) );
  s = NonEmpty;
}

SegmentRegion::SegmentRegion(const TriangleRegion &tr) {
  // Append the segment from p0->p1
  segment.push_back( Segment( tr.getP0(), tr.getP1() ) );
  
  // Append the segment from p1->p2
  segment.push_back( Segment( tr.getP1(), tr.getP2() ) );
  
  // Append the segment from p2->p0
  segment.push_back( Segment( tr.getP2(), tr.getP0() ) );
  
  s = NonEmpty;
}

SegmentRegion::SegmentRegion() {
  s = Empty;
}

RegionGS::Location SegmentRegion::location(const Point& p) const {
  Location loc = Inside;
  
  // Check that the point is inside each segment
  std::list<Segment>::const_iterator it;
  for (it = segment.begin(); it != segment.end(); it++) {
    Location check = (*it).location( p );
    switch ( check ) {
    case Inside:
      break;
    case Outside:
      return Outside;
      break;
    case Border:
      loc = Border;
      break;
    }
  }
  
  return loc;
}

RegionGS::Size SegmentRegion::size() const {
  return s;
}

double SegmentRegion::lowerBound() const {
  // This could be more accurate.
  return 0;
}

double SegmentRegion::upperBound() const {
  // This could be more accurate.
  return CompleteArea;
}

RegionGS* SegmentRegion::clone() const {
  return new SegmentRegion(*this);
}

SegmentRegion SegmentRegion::intersect(const SegmentRegion& second) const {
  // Make algorithm symetric in a and b.
  const std::list<Segment> a = segment;
  const std::list<Segment> b = second.segment;
  
  // Flag to watch for intersections
  bool crossing = false;
  
  // Build up a segment chain while checking for a completed loop
  std::list<Segment> chain;
  Segment trail = a.front();
  
  // Check for intersection with all of the segments from the other chain
  double theta = Segment::OFF_SEGMENT;  // Large theta
  Point*  first_intersection = 0;  
  std::list<Segment>::const_iterator it;

  for (it = b.begin(); it != b.end(); it++) {
    Point *p = trail.intersect( *it );
    if (p) {
      crossing = true;
      // Check if this is 
      double sweep = trail.sweep( *p );
      if (sweep < theta) {
	first_intersection = p;
	theta = sweep;
      }
    }
    delete p;
  }
  
  // Now make the first segment of the new trail based on the above results
  
  
  // Dummy for comiler ### REPLACE ###
  return second;
}


// Fast testing to see if their is an intersection between two regions.
bool SegmentRegion::intersectQ(const SegmentRegion& second) const {

  // True if any of the endpoints of segments of this region are contained in 
  // the second region
  std::list<Segment>::const_iterator it;

  for (it = segment.begin(); it != segment.end(); it++) {
    if ( second.location( (*it).getStart() ) == Inside ) {
      //cout << "\tEndpoint of this region is containd in second region.\n";
      //cout << "\t" << (*it).getStart() << endl;
      return true;
    }
  }
  
  // True if any of the endpoints of segments of the second region are
  // contained in this region
  
  for (it = second.segment.begin(); it != second.segment.end(); it++) {
    if ( location( (*it).getStart() ) == Inside ) {
      //cout << "\tEndpoint of second segment is in this region\n";
      //cout << "\t" << (*it).getStart() << endl;
      return true;
    }
  }
  
  // True if any of the segments of this region intersect any of the segments 
  // of the second region
  std::list<Segment>::const_iterator jt;
  for (it = segment.begin(); it != segment.end(); it++) {
    for (jt = second.segment.begin(); jt != second.segment.end(); jt++) {
      Point* p = (*it).intersect( *jt );
      if (p) {
	delete p;
	//cout << "\tSegments of this region intersect segments of second
	//region\n";
	
	return true;
      }
    }
  }
  
  // Otherwise fasle
  return false;
}
