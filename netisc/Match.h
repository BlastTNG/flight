//
// Match.h
//
// Class for implementing matching algorithm
//


#ifndef MATCH_H
#define MATCH_H

#include "qvaluelist.h"
#include "qobject.h"

#include "Catalog.h"
#include "Frame.h"

// Algorithm class with poperties that can be set and queried to 
// control set of methods.  A Match object looks for instances of a 
// Frame pattern within a star Catalog.

class Match : public QObject {

  //    Q_OBJECT
 public:
  Match(const Frame& f, Catalog& c);
  Match(const Frame& f, Catalog& c, const RegionGS& sr);
  virtual ~Match();
  
  // Properties
  Radian tolerance() {return _tolerance;}
  double matchFraction() {return _matchFraction;}
  double magnitudeLimit();
  RegionGS* searchRegion();
  Frame getFrame() const {return frame;}
  
  //public slots:
  
  void setTolerance(Radian t) {_tolerance = t;}
  void setMatchFraction(double d) { _matchFraction = d; }
  void setRegion(const RegionGS& r) { region = r.clone(); }
  
  //void setFrame(const Frame& fr) { frame = fr; emit newFrame(frame);} 
  
  //public:
  Catalog* getCatalog() {return &catalog;}
  void setCatalog();
  void setMagnitudeLimit(double);

  void setSearchRegion( const RegionGS& sr);
  
  // Methods
  QValueList<Frame> match();
  
  int maxMatched;
  
  //  static int main(int argc, char *argv[]); // Main function
  
  //signals:
  //void compareTotal(int);
  //void compareProgress(int);
  //void newFrame( Frame & );
  //void newMax( int );
  //void primaryStars( QValueList<Star> );
  //void solution(Radian ra, Radian dec, Radian roll);
  
 private:
  RegionGS *region;
  Catalog& catalog;
  Frame frame;
  Radian _tolerance;
  double _matchFraction;
  double quitFraction;
};

#endif // MATCH_H
