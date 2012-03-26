//
//  HTMCatalog.h
//
//  Implementation of an HTM segmented catalog.
//

#ifndef HTM_CATALOG_H
#define HTM_CATALOG_H

#include "Star.h"
#include "Catalog.h"
#include "Region.h"
#include <list>
#include <string>
//#include "qptrlist.h"

using namespace std;

class HTMCatalog : public Catalog {
 public:
  HTMCatalog(const char *str);
  virtual ~HTMCatalog();
  void init();
  
  virtual void loadFromFile(const char *fileBase);
  virtual void sync();
  virtual bool add(const Star& star);  // Add a single Star.
  
  // Returns Stars in the given Region
  virtual std::list<Star> find(const RegionGS& region); 
  
  // Moved mag limit to Region
  //double getMagLimit() {return magLimit;}
  //double setMagLimit(double ml) {
  // magLimit = ml; 
  // return magLimit;
  //}
  //public slots:

  void addFile(string file);
  //void relayMessage(string mes);  // Causes signal message to be emitted.
  // Make sure there are no circular routes back to this 
  // slot that would cause an endless loop.


  // Prints the message via qDebug and via std::cout.  This is a
  // utility routine for applications that use an HTMCatalog as a
  // primary resource.
  void printMessage(string mes);  

  //signals:
  void message(string mes);
  
  //public:
  
  //virtual void add(const & std::list<Star> ls);  // Add a list of stars
  
  unsigned long print();
  
  static int main(int argc, char *argv[]);  // Main function to help build
  static int store(int argc, char *argv[]);  // Main function 

 protected:
  // Used to build children
  HTMCatalog(HTMCatalog *p, TriangleRegion *tr, const char *str);
  
  void makeChildren();
  void read( string fileName);
  
  //virtual void write();  // Write any stars associated with this node
  string filename();  
  
  void buildRoot();

 private:
  // An HTM catalog has four child nodes, each catalogs in their own right
  std::list<HTMCatalog*> children;

  int level;  // Level of this catalog. Level zero catalog no entries,
              // only entries from files.

  HTMCatalog *parent;  // Parent of this catalog

  // Internal filename lookup

  // Returns the name of the segment to a given level
  static string HTMSegment(const Radian& ra, const Radian& dec, int level=0);
  // Returns the name of the file 
  string fileFromCoords(const Radian &ra, const Radian& dec); 
  
  TriangleRegion *myTriangle;  // Region containted in this catalog
  
  string fileBase;
  string name;
  string fileName;
  
  // If multiple HTM's exist with other spacing, then these
  // need to change.
  static int dirSpacing;  //  Number of leters per directory
  static int fileLevel;   // Level on which file names are found
  static int threshold;   // Number of stars afterwhich a child is made.
  static double magLimit;
  static char *prefix;    // Prefix to the data filename
  static char *suffix;    // Suffix to the data filename
  
  std::list<Star> myStars;  // Stars found on this level.  Should be
			     // zero for nodes above the fileLevel.
  
  // Status of this node
  bool fileLoaded;
  bool childrenCreated;
  bool inSync;  // True unless a star has been added.
};

#endif HTM_CATALOG_H
