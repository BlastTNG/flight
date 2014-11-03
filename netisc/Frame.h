//
// Frame.h
//

// Class that can transform a set of blobs into a points on the sky,
// given an assignment to two blobs.


#ifndef FRAME_H
#define FRAME_H

#include "Units.h"
#include "Point.h"
#include "Star.h"
#include "qvaluelist.h"

class Frame {
public:
  Frame();
  Frame(const Frame&); // Copy constructor
  virtual ~Frame();
  
  void addBlob( const Degree& x, const Degree& y, double flux=0) ;
  void addPrimaryBlob(const Degree& x, const Degree& y, double flux=0) ; 
  void addSecondaryBlob(const Degree& x, const Degree& y, double flux=0) ; 
  void clear();
  int count() const {
    return (int) blob.count();
  }
  
  // Properties
  Radian getAxisLength(); 
  Point getPrimary( ) const;
  Point setPrimary( const Point& p1);

  Point getSecondary( ) const;
  Point setSecondary( const Point& s1);


  // Internal class to handle (x, y, flux) triplets.
  class Blob : public Point { 
  public: 
    Blob(const Degree& a = Degree(0), const Degree& b = 
	 Degree(0), double m = 0) :
      Point(Radian(a), Radian(b), sqrt(1-Radian(a)*Radian(a)-Radian(b) * 
				       Radian(b))), mag(m) {}
      double mag;
  };
  
  QValueList<Star> getStars();
  QValueList<Blob> getBlobs() {return blob;}

  Frame& operator=(const Frame&);  // Require explicit constr.. in assignment

 private:
  //Frame& operator=(const Frame&); // No implicit assignment constructor
  
  QValueList<Blob> blob;
  Blob *primaryBlob;
  Blob *secondaryBlob;
  Point *primaryPoint;
  Point *secondaryPoint;
  
  // Directions  (a->x, b->y, c->z)
  //Point *a, *b, *c;  // Points in the Blob set
  // Point *x, *y, *z;  // Points on the Sky
  double scale; // Size 
};

std::ostream &operator<<(std::ostream &output, Frame& s);

#endif // FRAME_H
