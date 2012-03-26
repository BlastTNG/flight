//
//  Point.cpp
//
// Implementation for a spherical point
//

#include <math.h>

#include "Point.h"

#include <iostream>
#include <list>

using namespace std;

// Vector class
// Output assistant

ostream &operator<<(ostream &output, const Vector& p) {
  return output << "[" << p.getX() << ", " << p.getY() << ", " << 
    p.getZ() << "]";
}

Vector::Vector(double a, double b, double c) : x(a), y(b), z(c) {
}

Vector::~Vector() {}

double Vector::getX() const {
    return x;
}
double Vector::getY() const {
    return y;
}
double Vector::getZ() const {
    return z;
}

double Vector::setX(double a) {
  x = a;
  return x;
}

double Vector::setY(double a) {
  y = a;
  return y;
}

double Vector::setZ(double a) {
  z = a;
  return z;
}

void Vector::setXYZ(double a, double b, double c) {
  x = a; y = b; z = c;
}

double Vector::dot(const Vector& p2) const {
  return x*p2.x + y*p2.y +z*p2.z;
}

Vector Vector::cross(const Vector& p2) const {
  double a, b, c;

  a = y*p2.z - z*p2.y;
  b = z*p2.x - x*p2.z;
  c = x*p2.y - y*p2.x;

  Vector p(a, b, c);

  return p; 
}

Vector Vector::operator+(const Vector& p2) const {
  return Vector(x+p2.x, y+p2.y, z+p2.z);
}

Vector Vector::operator-(const Vector& p2) const {
  return Vector(x-p2.x, y-p2.y, z-p2.z);
}

Vector Vector::operator*(double a) const {
  return Vector(a*x, a*y, a*z);
}

double Vector::length() const {
  return sqrt(x*x + y*y + z*z);
}

Vector operator*(double alpha, const Vector&v) {
  return v*alpha;
}

// Point class 
// Output assistant

ostream &operator<<(ostream &output, const Point& p) {
  return output << "[" << p.getX() << ", " << p.getY() << ", " << p.getZ() 
		<< "]";
}

Point::Point(double a, double b, double c) : Vector(a, b, c) {
  normalize();
}

Point::Point(const Vector& v) : Vector(v) {
  normalize();
}

Point::~Point() {}

double Point::setX(double a) {
  x = a;
  normalize();
  return x;
}

double Point::setY(double a) {
  y = a;
  normalize();
  return y;
}

double Point::setZ(double a) {
  z = a;
  normalize();
  return z;
}

void Point::setXYZ(double a, double b, double c) {
  x = a; y = b; z = c;
  normalize();
}

void Point::normalize() {
  double len = length();
  //Q_ASSERT( len > 0 );  // Check that the length is non-zero
  
  x /= len;
  y /= len;
  z /= len;
}

double Point::dot(const Point& p2) const {
  return x*p2.x + y*p2.y +z*p2.z;
}

Point Point::cross(const Point& p2) const {
  double a, b, c;
  a = y*p2.z - z*p2.y;
  b = z*p2.x - x*p2.z;
  c = x*p2.y - y*p2.x;
  Point p(a, b, c);
  p.normalize();
  return p; // Invokes default copy constructor for class;
}

Point Point::operator+(const Point& p2) const {
  double a, b, c;
  a = x + p2.x;
  b = y + p2.y;
  c = z + p2.z;
  
  Point p(a, b, c);
  p.normalize();
  return p;
}

class Triple {
public:
  double a, b, c;
};

int Point::main(int argc, char *argv[]) {
  cout << "Sample code for the Point class" << endl;
  
  int row = 2;
  
  // Array of triples over the cube.
  std::list<Triple> cube;
  std::list<Point> pList;
  
  for (int i=-1; i<=1; i++) {
    for (int j=-1; j<=1; j++) {
      for (int k=-1; k<=1; k++) {
        Triple t;
        t.a=i;
        t.b=j;
        t.c=k;
        if (i || j || k) { // Skip over zero point
          cube.push_back( t );
          Point p = Point(t.a, t.b, t.c);
          pList.push_back( p );
	  
          row++;
        }
      }
    }
  }
  
  // Now make them into Points and print them out.
  row = 2;
  std::list<Triple>::iterator it;
  std::list<Point>::iterator jt;
  jt = pList.begin();
  
  for (it=cube.begin(); it != cube.end(); it++, jt++) {
    Triple t = *it;
    Point p = *jt;
	cout << t.a << " " << t.b << " " << t.c << " -> " << p << endl;
    row++;
  }

  return 0;
}
