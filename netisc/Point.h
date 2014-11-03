//
// Point.h
//
// Definitions for a spherical point.  
//

#ifndef POINT_H
#define POINT_H

//#include <iostream>

// Vector class for doing vector summations and scalings.
// Otherwise when doing a+b+c, a+b is normalized before adding c, which gives
// a different result.

class Vector {
 public:
  Vector(); // Zero vector
  Vector(double x, double y, double z);
  virtual ~Vector();
  
  virtual double getX() const;
  virtual double getY() const;
  virtual double getZ() const;
  
  virtual double setX(double x);
  virtual double setY(double y);
  virtual double setZ(double z);
  virtual void setXYZ(double x, double y, double z);
  
  virtual double dot(const Vector& p2) const;
  virtual Vector cross(const Vector& p2) const;
  virtual Vector operator+(const Vector& p2) const;
  virtual Vector operator-(const Vector& p2) const;
  virtual Vector operator*(double alpha) const;
  virtual double length() const;
  
  //friend std::ostream &operator<<(std::ostream &output, const Vector& p);
  friend Vector operator*(double alpha, const Vector& v);
  
 protected:
  double x, y, z;
};

class Point : public Vector {
 public:
  //Point(double x=1, double y=0, double z=0);
  Point(double x, double y, double z);
  Point(const Vector& v);
  virtual ~Point();
  
  virtual double setX(double x);
  virtual double setY(double y);
  virtual double setZ(double z);
  virtual void setXYZ(double x, double y, double z);
  
  virtual double dot(const Point& p2) const;
  virtual Point cross(const Point& p2) const;
  virtual Point operator+(const Point& p2) const;
  
  static int main(int argc, char *argv[]); 
  //friend std::ostream &operator<<(std::ostream &output, const Point& p);
  
 protected:
  void normalize();
  
};

#endif
