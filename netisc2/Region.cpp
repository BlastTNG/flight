//
//  IntersectRegion.cpp
//
//  Source and demo code for region.

#include <iostream>

#define _USE_MATH_DEFINES
#include <math.h>

#include "Region.h"

using namespace std;

const double RegionGS::CompleteArea = 4*M_PI;

RegionGS::~RegionGS() {}
bool RegionGS::insideQ(const Point &p) {
  if (location(p) == Inside) return true;
  return false;
}

int RegionGS::main(int, char *[]) { 
	cout << "Hello world from Region.cpp" << endl;
  
  // Create a set of regions and a set of points and intersect them.
  // Make a table that shows regions and points.  Dynamically updated
  // so the user can make tests of being inside.

  // ### TBD ###  
  // See example in Point.cpp for how a table can be setup.  Will need
  // to subclass to make signal/slot connections.
  
  return 0;
}

// Plane class
Plane::Plane(const Point& p, double dist) : Point(p), distance(dist) {}

Plane::~Plane() { }

double Plane::getDistance() const {return distance;}

Point Plane::getDirection() const {return *this;}

bool Plane::above(const Point &p) const {
  if ( (dot(p) - distance) > 0 ) return true;
  return false;
}

bool Plane::on(const Point &p) const {
  if ( (dot(p) - distance) == 0 ) return true;
  return false;
}

bool Plane::below(const Point &p) const {
  if ( (dot(p) - distance) < 0 ) return true;
  return false;
}


UnionRegion::UnionRegion(const RegionGS &ra, const RegionGS &rb) {
  r1 = ra.clone();
  r2 = rb.clone();
  s = RegionGS::Unknown;
  if (r1->size() == NonEmpty) s = RegionGS::NonEmpty;
  if (r2->size() == NonEmpty) s = RegionGS::NonEmpty;
}

UnionRegion::~UnionRegion() {
  delete r1;
  delete r2;
}

RegionGS::Location UnionRegion::location(const Point& p) const {
  RegionGS::Location l1;
  RegionGS::Location l2;

  if ((l1 = r1->location( p )) == RegionGS::Inside) return RegionGS::Inside;

  if (l1 == RegionGS::Outside) {
    if ((l2 = r2->location( p )) == RegionGS::Inside) return RegionGS::Inside;
    if (l2 == RegionGS::Outside) return RegionGS::Outside;
    if (l2 == RegionGS::Border) return RegionGS::Border;
  }

  if (l1 == RegionGS::Border) {
    if ((l2 = r2->location( p )) == RegionGS::Inside) return RegionGS::Inside;
    if (l2 == RegionGS::Outside) return RegionGS::Border;
    if (l2 == RegionGS::Border) return RegionGS::Border;
  }
  
  // This statement cannot be reached.  Included for safety and to
  // suppress compilers warnings.
  
  throw RegionGS::Error;
}

RegionGS::Size UnionRegion::size() const {
  return s;
}

double UnionRegion::lowerBound() const {
  double d1 = r1->lowerBound();
  double d2 = r2->lowerBound();
  if (d1 > d2) return d1;
  else return d2;
}

double UnionRegion::upperBound() const {
  double d1 = r1->upperBound();
  double d2 = r2->upperBound();
  return d1+d2;
}

RegionGS* UnionRegion::clone() const {
  UnionRegion *u = new UnionRegion(*r1, *r2);
  u->s = s; 
  
  return u;
}

ComplementRegion::ComplementRegion(const RegionGS& ra) {
  r1 = ra.clone();
  s = Unknown;
  if (r1->size() == Empty) s = NonEmpty;
}

ComplementRegion::~ComplementRegion() {
  delete r1;
}

RegionGS::Location ComplementRegion::location(const Point &p) const {
  RegionGS::Location l = r1->location(p);
  if (l == Inside) return Outside;
  
  // Point p is either Inside or on the border.  Size is NonEmpty;
  // Assignment has been depricated in favor of making the location
  // member const //s = NonEmpty;
  
  if (l == Outside) return Inside;
  if (l == Border) return Border;
  
  throw Error;  // Control should never reach here.
}

RegionGS::Size ComplementRegion::size() const {
  return s;
}

double ComplementRegion::lowerBound() const {
  return CompleteArea - r1->upperBound();
}

double ComplementRegion::upperBound() const {
  return CompleteArea - r1->lowerBound();
}

RegionGS* ComplementRegion::clone() const {
  return new ComplementRegion( *r1 );
}

CompleteRegion::CompleteRegion() {}

CompleteRegion::~CompleteRegion() {}

RegionGS::Location CompleteRegion::location(const Point &) const { 
  return Inside; 
}

RegionGS::Size CompleteRegion::size() const {
  return NonEmpty;
}

double CompleteRegion::lowerBound() const {return CompleteArea;}

double CompleteRegion::upperBound() const {return CompleteArea;}

RegionGS* CompleteRegion::clone() const  {return new CompleteRegion();}

EmptyRegion::EmptyRegion() {}

EmptyRegion::~EmptyRegion() {}

RegionGS::Location EmptyRegion::location(const Point &) const { 
  return Inside; 
}

RegionGS::Size EmptyRegion::size() const {return Empty;}

double EmptyRegion::lowerBound() const {return 0;}

double EmptyRegion::upperBound() const {return 0;}

RegionGS* EmptyRegion::clone()  const {return new EmptyRegion();}

GreatCircleRegion::GreatCircleRegion(const Point& pa, const Point& pb) : 
  p1( pa.cross(pb) ) {
}

GreatCircleRegion::GreatCircleRegion(const Point& pa) : p1(pa) {
}

GreatCircleRegion::~GreatCircleRegion() {}

RegionGS::Location GreatCircleRegion::location(const Point &p) const {
  double product = p1.dot( p );
  if (product > 0) return Inside;
  if (product == 0) return Border;
  else return Outside;
}

RegionGS::Size GreatCircleRegion::size() const {
  return NonEmpty;
}

double GreatCircleRegion::lowerBound() const {
  return CompleteArea/2;
}

double GreatCircleRegion::upperBound() const {
  return CompleteArea/2;
}

RegionGS* GreatCircleRegion::clone() const {
  return new GreatCircleRegion(p1);
}

Point GreatCircleRegion::getPole() const {
  return p1;
}

MinorCircleRegion::MinorCircleRegion(const Point& p, const Radian& r) : 
  Plane(p, cos(Radian(r))), p1(p), radius(r) {}

MinorCircleRegion::MinorCircleRegion(const Point& p, const Degree& r) : 
  Plane(p, cos(Radian(r))), p1(p), radius(r) {}

// This sum of three large cross product is used to avoid the
// normalization in Point::cross().

MinorCircleRegion::MinorCircleRegion(const TriangleRegion &t) : 
  Plane( t.center(), t.center().dot( t.getP0() )),
  p1( getDirection() ) {
  // Radius was probably miscalcualted previously
  radius = acos( getDistance() );
}

MinorCircleRegion::MinorCircleRegion(const GreatCircleRegion& gcr) :
  Plane( gcr.getPole(), 0 ),
  p1( getDirection() ) {
  radius = M_PI/2;
}

MinorCircleRegion::~MinorCircleRegion() {}

RegionGS::Location MinorCircleRegion::location(const Point& p) const {
  if (above(p)) return Inside;  // Inside when above the plane.
  if (below(p)) return Outside; // Outside when below the plane.
  if (on(p)) return Border;
  throw Error;  // Control should never reach here.
}

RegionGS::Size MinorCircleRegion::size() const { return NonEmpty; }

double MinorCircleRegion::lowerBound() const {
  return 2*M_PI*(1-cos(radius));
}

double MinorCircleRegion::upperBound() const {
  return 2*M_PI*(1-cos(radius));
}

RegionGS* MinorCircleRegion::clone() const {
  return new MinorCircleRegion(p1, radius);
}

// Triangle region
TriangleRegion::TriangleRegion(const Point& pa, const Point& pb, 
			       const Point& pc) : 
  p0(pa), p1(pb), p2(pc), 
  g12(p0, p1), g23(p1, p2), g31(p2, p0) {
  // All construction done above.
}

TriangleRegion::~TriangleRegion() {}

RegionGS::Location TriangleRegion::location(const Point& p) const {

  switch ( g12.location( p ) ) {

  case Inside:
    switch ( g23.location( p )) {

    case Inside:
      switch (g31.location( p )) {
      
      case Inside:  
	return Inside;
	break;
      
      case Outside:
	return Outside;
	break;

      case Border:
	return Border;
	break;
      }

    case Outside:
      return Outside;
      break;

    case Border:
      switch (g31.location( p )) {
	
      case Inside:  
	return Border;
	break;
	
      case Outside:
	return Outside;
	break;
	
      case Border:
	return Border;
	break;
      }
    }

  case Outside:
    return Outside;
    break;

  case Border:
    switch ( g23.location( p )) {

    case Inside:
      switch (g31.location( p )) {

      case Inside:  
	return Border;
	break;

      case Outside:
	return Outside;
	break;

      case Border:
	return Border;
	break;
      }

    case Outside:
      return Outside;
      break;
    
    case Border:
      switch (g31.location( p )) {
      
      case Inside:  
	return Border;
	break;

      case Outside:
	return Outside;
	break;

      case Border:
	return Border;
	break;
      }
    }
  }
  throw Error;  // Control should not reach here.
}

RegionGS::Size TriangleRegion::size() const {
  if (lowerBound() > 0) return NonEmpty;
  else return Empty;
}

double TriangleRegion::lowerBound() const {
  return 0;
}

double TriangleRegion::upperBound() const {
  return CompleteArea;
}

Point TriangleRegion::center() const {
  // Sum of three cross products.  Can't use Point::cross since there
  // is a call to normalize

  double x =
    p0.getY()*p1.getZ() - p0.getZ()*p1.getY() +
    p1.getY()*p2.getZ() - p1.getZ()*p2.getY() +
    p2.getY()*p0.getZ() - p2.getZ()*p0.getY();
  
  double y =
    p0.getZ()*p1.getX() - p0.getX()*p1.getZ() +
    p1.getZ()*p2.getX() - p1.getX()*p2.getZ() +
    p2.getZ()*p0.getX() - p2.getX()*p0.getZ();

  double z =
    p0.getX()*p1.getY() - p0.getY()*p1.getX() +
    p1.getX()*p2.getY() - p1.getY()*p2.getX() +
    p2.getX()*p0.getY() - p2.getY()*p0.getX();

  return Point(x,y,z);
}

RegionGS* TriangleRegion::clone() const {
  return new TriangleRegion(p0, p1, p2);
}

IntersectionRegion::IntersectionRegion(const RegionGS &ra, const RegionGS &rb)
{
  r1 = ra.clone();
  r2 = rb.clone();
  s = RegionGS::Unknown;
  
  // Look for shortcuts to intersection.  Hoping to find empty intersections.
  MinorCircleRegion * mc = 0;
  TriangleRegion * tr = 0;
  if ((mc = dynamic_cast<MinorCircleRegion *const>(r1))) {
    if ((tr = dynamic_cast<TriangleRegion *const>(r2))) {
      s = mc->intersectMinorCircle( MinorCircleRegion( *tr ));
      
      return;
    }
  }

  if ((mc = dynamic_cast<MinorCircleRegion *>(r2))) {
    if ((tr = dynamic_cast<TriangleRegion *>(r1))) {
      //cout << "\tChecking " << mc->getCenter() << " | " <<
      //	mc->getDistance() << endl;
      s = mc->intersectMinorCircle( MinorCircleRegion( *tr ));
      if (s == NonEmpty) {
	//cout << "\t" << mc->getCenter() << " intersects " << 
	//"\n\t" << tr->getP0() << "|" << tr->getP1() << "|" << 
	//tr->getP2() << endl;
      }
      return;
    }
  }
}


IntersectionRegion::IntersectionRegion(const TriangleRegion& rb, 
				       const MinorCircleRegion &ra) {

  r1 = ra.clone();  // r1 is the TriangleRegion
  r2 = rb.clone();  // r2 is the MinorCircleRegion
  s = ra.intersectMinorCircle( MinorCircleRegion( rb ));
}

IntersectionRegion::IntersectionRegion(const MinorCircleRegion &ra, const TriangleRegion &rb) {
  r1 = ra.clone();  // r1 is the TriangleRegion
  r2 = rb.clone();  // r2 is the MinorCircleRegion
  s = ra.intersectMinorCircle( MinorCircleRegion( rb ));
}

IntersectionRegion::~IntersectionRegion() {
  delete r1;
  delete r2;
}

RegionGS::Location IntersectionRegion::location(const Point& p) const {

  RegionGS::Location l1;
  RegionGS::Location l2;
  
  if ((l1 = r1->location( p )) == RegionGS::Outside) 
    return RegionGS::Outside;
  
  if (l1 == RegionGS::Inside) {
    if ((l2 = r2->location( p )) == RegionGS::Outside) 
      return RegionGS::Outside;
    // Depricated in favor of making location() member const
    //s = NonEmpty;
    if (l2 == RegionGS::Inside) return RegionGS::Inside;
    if (l2 == RegionGS::Border) return RegionGS::Border;
  }

  if (l1 == RegionGS::Border) {
    if ((l2 = r2->location( p )) == RegionGS::Outside) 
      return RegionGS::Outside;
    // Depricated in favor of making location() member const
    //s = NonEmpty;
    if (l2 == RegionGS::Inside) return RegionGS::Border;
    if (l2 == RegionGS::Border) return RegionGS::Border;
  }
  
  // This statement cannot be reached.  Included for safety and to
  // suppress compilers warnings.
  throw RegionGS::Error;
}

RegionGS::Size IntersectionRegion::size() const {
  return s;
}

double IntersectionRegion::lowerBound() const {
  return 0;
}

double IntersectionRegion::upperBound() const {
  double d1 = r1->upperBound();
  double d2 = r2->upperBound();
  if (d1 > d2) return d2;
  else return d1;
}

RegionGS* IntersectionRegion::clone() const {
  IntersectionRegion *i = new IntersectionRegion(*r1, *r2);
  i->s = s; 
  
  return i;
}


RegionGS::Size MinorCircleRegion::
intersectMinorCircle(const MinorCircleRegion& r2) const {

  const MinorCircleRegion& r1 = *this;  //  Add to symmetery of code below.

  // Check if the center of r is inside this.
  if ( r1.location( r2 ) == Inside ) {
    //cout << "\t\tr2 inside r1\n";
    return NonEmpty;
  }
  
  // Check if this center is inside the center of r
  if ( r2.location( r1 ) == Inside ) {
    //cout << "\t\tr1 inside r2\n";
    return NonEmpty;
  }
  
  // Check if the line of intersection of this and r is inside the sphere
  double d1 = r1.getDistance();
  double d2 = r2.getDistance();
  double p =  r1.dot( r2 );
  double b = (d1*d1 + d2*d2 - 2*p*d1*d2) / (1 - p*p);
  
  // Absolute value of intersection parameter less than one
  if (b <= 1 && b >= -1) {
    //cout << "\t\tIntersection parameter valid\n";
    //cout << "(d1,d2) = (" << d1 << ", " << d2 << ")\n";
    //cout << "(p,b) = (" << p << ", " << b << ")\n";
    return NonEmpty; 
  }
  
  //cout << "\t\tNo intersection\n";
  return Empty;
}
