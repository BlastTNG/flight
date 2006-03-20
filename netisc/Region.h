//
// Region.h
// 
// Interfaces for Region class.
//

#ifndef REGION_H
#define REGION_H

#include <math.h> 
//#define M_PI 3.14159265358979323846
#include "Point.h"
#include "Units.h"  // needed for Radian definition

// Interface class
class RegionGS {
 public:
  RegionGS() : magLimit(10) {}
    virtual ~RegionGS();
    
    // Leaf node methods
    virtual bool insideQ(const Point& p);  // Returns true if point is in
    
    enum Location { Inside, Border, Outside };
    enum Size { Empty, Unknown, NonEmpty };
    enum Status { Error };  // Used to throw things
    
    // Returns the location of point p.
    virtual Location location(const Point& p) const = 0;  
    
    // Returns roughest possible size of the region
    virtual Size size() const = 0;
    
    // Returns a lower bound on the size of the region.
    virtual double lowerBound() const = 0;
    
    // Returns an upper bound on the size of the region.  
    virtual double upperBound() const = 0;
    
    // This is intended to act like a virtual constructor.
    // Returns a cloned copy of the Region.  
    virtual RegionGS* clone() const = 0;  
    
    static const double CompleteArea; // Size of a complete region
    
    // Used for testing purposes ala java
    static int main(int argc, char *argv[]);  
    
    // Inserting implementation into interfacve.  Bad!  7/25/02
    double magnitudeLimit() const {return magLimit;}
    void setMagnitudeLimit(double lim) {magLimit = lim;}
    
 private:
    double magLimit;  // Magnitude limit of region.
    
};


// Helper class -- Plane.  Used to slice off a section of a circle.
class Plane : protected Point {
 public:
  
  Plane(const Point& p, double distance=0);
  virtual ~Plane();
  
  // Attriutes of the class (not setable)
  double getDistance() const;
  Point getDirection() const;
  
  // Tests for on, above and below the plane
  bool above(const Point &p) const; // On the opposite side of the
                    // plane as the origin
  bool on(const Point &p) const;    // On the plane
  bool below(const Point &p) const; // On the same side as the origin

 private: 
  double distance;
};

// Union class
class UnionRegion : public RegionGS {
 public:
  UnionRegion(const RegionGS& r1, const RegionGS& r2);
  virtual ~UnionRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the UnionRegion
  
 private:
  RegionGS *r1;
  RegionGS *r2;
  Size s;
};

// Complement class
class ComplementRegion : public RegionGS {
 public:
  ComplementRegion(const RegionGS& r1);
  virtual ~ComplementRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the region.
  
 private:
  RegionGS *r1;
  Size s;
};


// Complete Region
class CompleteRegion : public RegionGS {
 public:
  CompleteRegion();
  virtual ~CompleteRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the CompleteRegion
};


// Empty Region
class EmptyRegion : public RegionGS {
 public:
  EmptyRegion();
  virtual ~EmptyRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the EmptyRegion
};

// Great circle region
class GreatCircleRegion : public RegionGS {
 public:
  
  // Constructs the Great Circle Region at the half of the sphere
  // bounded by the great circle connecting p1 and p2 and containing
  // p1xp2.
  GreatCircleRegion(const Point& p1, const Point& p2); 
  
  // Constructs the Great Circle Region with p1 as the pole.
  GreatCircleRegion(const Point& p1); 

  virtual ~GreatCircleRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the GC Region
  
  Point getPole() const;
  
 private:
  Point p1; // Points to the pole.
};


// Triangular region
class TriangleRegion : public RegionGS
{
 public:
  
  // Forms a triangular region with the interior defined as the unit
  // outward normal of the interior defined by the right hand rule.
  TriangleRegion(const Point& p0, const Point& p1, const Point& p2); 

  virtual ~TriangleRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  Point getP0() const {return p0;}
  Point getP1() const {return p1;} 
  Point getP2() const {return p2;}
  Point center() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the TriangleRegion
  
 private:
  Point p0, p1, p2;
  GreatCircleRegion g12, g23, g31;
};


// Minor circle region
class MinorCircleRegion : public RegionGS, protected Plane {
 public:
  // Constructs the minor circle region about point p1 with radius
  // given in radians.
  MinorCircleRegion(const Point& p1, const Radian &radius);

  // Constructs the minor circle region about point p1 with radius
  // given in degrees.
  MinorCircleRegion(const Point& p1, const Degree &radius);

  //MinorCircleRegion(const Point& p1, double radius); Depricated in
  //favor of above.

  // Circular approximation to Triangle Region
  MinorCircleRegion(const TriangleRegion &triReg); 

  // A GreatCircle is special case of a minor circle
  MinorCircleRegion(const GreatCircleRegion &gcr); 
  virtual ~MinorCircleRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns a cloned copy of the Minor Cirlce
  
  Size intersectMinorCircle(const MinorCircleRegion& minorCirReg) const;
  Point getCenter() const {return p1;}
  double getDistance() const {return Plane::getDistance();}
  Radian getRadius() const {return radius;}

private:
  Point p1;       // Points to the center of the region.
  Radian radius;  // Radius of the region.
  
};


// Intersection class
class IntersectionRegion : public RegionGS {
 public:
  IntersectionRegion(const RegionGS& r1, const RegionGS& r2);
  IntersectionRegion(const TriangleRegion& r1, const MinorCircleRegion &r2); 
  IntersectionRegion(const MinorCircleRegion& r1, const TriangleRegion &r2); 
  virtual ~IntersectionRegion();
  
  virtual Location location(const Point& p) const;  
  virtual Size size() const;  
  
  virtual double lowerBound() const;
  virtual double upperBound() const;
  
  RegionGS* clone() const;  // Returns cloned copy of the intersection region
  
 private:
  RegionGS *r1;
  RegionGS *r2;
  Size s;
};

#endif
