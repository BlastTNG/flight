#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <direct.h>

#include <iostream>
#include <list>
#include <algorithm>

#include <string>

#include "HTMCatalog.h"
#include "SegmentRegion.h"

using namespace std;

extern "C" {
#include "tables.h"
};


int HTMCatalog::dirSpacing;
int HTMCatalog::fileLevel;
int HTMCatalog::threshold;
double HTMCatalog::magLimit;
char * HTMCatalog::prefix;
char * HTMCatalog::suffix;

HTMCatalog::HTMCatalog(const char *str) {

  // Defaults for behavior,
  // these should be attributes and follow that pattern.
  myTriangle = 0;
  dirSpacing = 3;
  fileLevel = 5;
  threshold = 50;
  magLimit = 12;
  //children.setAutoDelete( true );
  
  prefix = "gsc";
  suffix = ".star";

  // Default constructor builds the root node with eight children.
  // All access should decend through the root node .
  loadFromFile(str);
}

void HTMCatalog::init() { 
  // Initialize with a given file root.
  
  //Having this in init allows messages to be seen after creation.
  buildRoot();
}

HTMCatalog::HTMCatalog(HTMCatalog *p, TriangleRegion* tr, const char *str) : 
  myTriangle(tr) {
  
  // Copy parent settings
  fileBase = p->fileBase;
  level = p->level + 1;
  name = p->name;
  fileLoaded = false;
  childrenCreated = false;
  
  //children.setAutoDelete( true );
  
  // Append the str onto the name.
  name += str;
  
  // Pass messages back to parents.
  //connect( this, SIGNAL(message(string)), p, SLOT(relayMessage(string)) );
  
  // If this below the file level, add more children
  if (level < fileLevel) {
      // Changed to delayed instaniation  06-29-02
      //makeChildren();

      //emit(  message( "Created node " + name ) );
  }
  
  // We are on a data node
  if (level == fileLevel) {
    childrenCreated = true;

	struct stat st;
    // Check that each level exists.
	if(stat(fileBase.c_str(),&st)!=0) _mkdir(fileBase.c_str());
    
	fileBase = fileBase + "/" + name.substr(0,2);
	if(stat(fileBase.c_str(),&st)!=0) _mkdir(fileBase.c_str());

    fileBase = fileBase + "/" + name.substr(2,3);
	if(stat(fileBase.c_str(),&st)!=0) _mkdir(fileBase.c_str());
    
    // Make directory and filename
    fileName = fileBase + "/" + name.substr(0,2) + "/" + name.substr(2, 3) +
      "/" + prefix + name + suffix;

    // Make directory and filename
    fileName = fileBase + "/" + prefix + name + suffix;
//    fileName = fileBase + "/" + name.substr(0,2) + "/" + name.substr(2, 3) +
//      "/" + prefix + name + suffix; 

    //emit(  message( "Created node " + name + " with fileName " + fileName));
  }
}

HTMCatalog::~HTMCatalog() {
  // 2002-03-15  Do not sync on destruction.  Overwrites files.
  // sync();
  
  delete myTriangle;
}

void HTMCatalog::buildRoot() {
  level = -1;
  name = "";
  
  // Make 8 triangle regions from the six cardinal points.
  // Initialization taken from http://www.sdss.jhu.edu/htm/doc/chap1.html
  Point v0 = Point(0, 0, 1);
  Point v1 = Point(1, 0, 0);
  Point v2 = Point(0, 1, 0);
  Point v3 = Point(-1, 0, 0);
  Point v4 = Point(0, -1, 0);
  Point v5 = Point(0, 0, -1);
  
  TriangleRegion* S0 = new TriangleRegion(v1, v5, v2);
  TriangleRegion* S1 = new TriangleRegion(v2, v5, v3);
  TriangleRegion* S2 = new TriangleRegion(v3, v5, v4);
  TriangleRegion* S3 = new TriangleRegion(v4, v5, v1);
  
  TriangleRegion* N0 = new TriangleRegion(v1, v0, v4);
  TriangleRegion* N1 = new TriangleRegion(v4, v0, v3);
  TriangleRegion* N2 = new TriangleRegion(v3, v0, v2);
  TriangleRegion* N3 = new TriangleRegion(v2, v0, v1);
  
  children.push_back( new HTMCatalog(this, S0, "S0") );
  children.push_back( new HTMCatalog(this, S1, "S1") );
  children.push_back( new HTMCatalog(this, S2, "S2") );
  children.push_back( new HTMCatalog(this, S3, "S3") );
  children.push_back( new HTMCatalog(this, N0, "N0") );
  children.push_back( new HTMCatalog(this, N1, "N1") );
  children.push_back( new HTMCatalog(this, N2, "N2") );
  children.push_back( new HTMCatalog(this, N3, "N3") );
  
  myTriangle = 0;
}

void HTMCatalog::makeChildren() {
  
  if (level == -1) return;  // Root node already has children
  
  // Clear any existing children
  std::list<HTMCatalog*>::iterator childit;
  for (childit = children.begin(); childit != children.end(); childit++) {
	delete *childit;
  }
  children.clear();

  // Use the side midpoints
  Point w0 = myTriangle->getP1() + myTriangle->getP2();
  Point w1 = myTriangle->getP0() + myTriangle->getP2();
  Point w2 = myTriangle->getP0() + myTriangle->getP1();
  
  TriangleRegion* t0 = new TriangleRegion( myTriangle->getP0(), w2, w1);
  TriangleRegion* t1 = new TriangleRegion( myTriangle->getP1(), w0, w2);
  TriangleRegion* t2 = new TriangleRegion( myTriangle->getP2(), w1, w0);
  TriangleRegion* t3 = new TriangleRegion( w0, w1, w2);

  children.push_back( new HTMCatalog(this, t0, "0") );
  children.push_back( new HTMCatalog(this, t1, "1") );
  children.push_back( new HTMCatalog(this, t2, "2") );
  children.push_back( new HTMCatalog(this, t3, "3") );

  childrenCreated = true;
}

// Read data into this node from fileName.
// Note that region checking is not done here. i.e. 
// file may not match Region of this node.

void HTMCatalog::read(string fileName) {

  FILE *fp = fopen(fileName.c_str(), "rb");

  if (fp == 0) {
    printf( "Error opening %s", fileName.c_str() ); 
    exit(-1);
  }
  
  //printf("Sizeof double is %d", sizeof(double));
  
  struct stat buf;
  int fh = fileno( fp );
  
  //int result = fstat( fh, &buf );
  fstat( fh, &buf );
  
  //printf("File size is %d", buf.st_size);
  
  double val;
  while ( fread( &val, sizeof(double), 1, fp) ) {
    
    // Read the other two doubles
    Degree ra = Degree(val);
    
    fread( &val, sizeof(double), 1, fp);
    Degree dec = Degree(val);
    
    fread( &val, sizeof(double), 1, fp);
    double mag = val;
    
    myStars.push_back( Star( Radian(ra), Radian(dec), mag) );
  }
  
  // ED
  //printf("Read %ld stars from %s into %s. ", 
  //      myStars.size(), fileName(), name());
  
  fclose( fp );
}


// Prints a summary of the catalog and 
// returns the number of stars in the catalog

unsigned long HTMCatalog::print() {

  // If I am at the file level, then I have stars
  if (level == fileLevel) {
    //if (myStars.size() > 0) {
    //    emit(  message( name + " " + QString::number(myStars.size()) )  );
    //}
    return (unsigned long)myStars.size();
  }
  
  // Else return the sum of the print of all of the chilren
  unsigned long ret = 0;
  HTMCatalog *child;
  std::list<HTMCatalog*>::iterator childit;
  for (childit = children.begin(); childit != children.end(); childit++) {
	child = *childit;
    ret += child->print();
  }
  return ret;
}

void HTMCatalog::loadFromFile(const char *str) {
  fileBase = str;
}

void HTMCatalog::sync() {
  // If there are any children, sync them also
  HTMCatalog *child;
  std::list<HTMCatalog*>::iterator childit;
  for (childit = children.begin(); childit != children.end(); childit++) {
	child = *childit;
    child->sync();
  }
  
  
  // Skip non-bottom nodes
  if (myStars.size() == 0) return;
  
  if (level < fileLevel) return;
  
  //emit(  message( "Syncing to " + fileName )  );
  // Write all stars and then write stars of any children.
  FILE *fp = fopen(fileName.c_str(),"w");
  if ( fp==0 ) {
    //emit(  message( "Error opening " + fileName )  );

  }

  std::list<Star>::iterator it;
  
  for (it = myStars.begin(); it != myStars.end(); ++it) {
    // Write value to file (in Decimal degrees)
    double ra = (*it).getRa( Degree() );
    fwrite( (const char *) &ra, sizeof(double), 1, fp )  ;
    
    double dec = (*it).getDec( Degree() );
    fwrite( (const char *) &dec, sizeof( double ), 1, fp );
    
    double mag = (*it).getMagnitude() ;
    fwrite( (const char *) &mag, sizeof( double ), 1, fp );
  }
  fclose(fp);
}

bool HTMCatalog::add(const Star& s) {

  //cout << "Adding(rad) (" << s.getRa(Radian()) << ", " <<
  //s.getDec(Radian()) << ")\n";

  // Put on a magnitude clip
  if (s.getMagnitude() > magLimit) return false;

  //cout << "Adding (" << s.getRa(Hour()) << ", " << s.getDec(DMS())
  //<< ", " << s.getMagnitude() << ") to "; cout << name << endl;

  // If this is level -1, then just pass on to all of the children
  if (level == -1) {
    HTMCatalog *child;
	std::list<HTMCatalog*>::iterator childit;
	int i;
	for (childit = children.begin(), i=0; i < 8 && (childit != children.end()); i++, childit++) {
		child = *childit;
		if (child->add( s )) return true;
    }
    return false;
  }
  
  // If star is inside our region, then add until one of our children accepts

  if ( myTriangle->insideQ(s) ) {

    if (!childrenCreated) { makeChildren(); }
    
    if (children.size() == 0) {
      
      // Load other stars first
      if (!fileLoaded) {
	struct stat st;

	if (stat(fileName.c_str(), &st)==0) {
        
	  //emit( message( "Opening " + fileName + " to read data." )  );
	  read( fileName );  // Loads myStars
	  fileLoaded = true;
	} else {
	  //emit( message( "File does not exist: " + fileName ) );
	  fileLoaded = false;
	}
      }

      //emit( message( "Adding " + s.asQString() ) );
      
      // cout << s << " is in " << myTriangle->getP0() << " " <<
      //     myTriangle->getP1() << " "  << myTriangle->getP2() << endl;
      
      // See if this star is already present
	  std::list<Star>::iterator starit;
	  starit = std::find(myStars.begin(), myStars.end(),s);
	  if (starit == myStars.end()) {
		myStars.push_back( s );
      } else {
	//emit( message("Star " + s.asQString() + " already in list") );
      }
      return true;
    }
    
    // Else pass onto the children
    HTMCatalog *child;
    
    // Loop until accepted or no more children
	std::list<HTMCatalog*>::iterator childit;
	for (childit = children.begin(); childit != children.end(); childit++) {
		child = *childit;
		if (!(child->add(s))) exit;
    }
    
    if (child == 0) {
      //cerr << "No child picked up star\n";
      //myStars.push_back( s );
    }
  }
  
  // Star is not in this triangle
  //cout << s << " Not in " << myTriangle->getP0() << " " <<
  //    myTriangle->getP1() << " "  << myTriangle->getP2() << endl;
  return false;
}

std::list<Star> HTMCatalog::find(const RegionGS& r) {

  // If the intersection between this and the triangle is empty,
  // return an empty list.  If there is an intersection, collect all
  // of the child responses.  

  //printf( "Looking for stars in " + name );
  
  std::list<Star> ret;
  if (myTriangle) {
    SegmentRegion mine( *myTriangle ); // Could be done once at node creation

    const MinorCircleRegion mc = dynamic_cast<const MinorCircleRegion &>(r);
    SegmentRegion samp(mc);

    //printf("Checking %s for overlap.", name.latin1() );
    //Star s0(myTriangle->getP0());
    //Star s1(myTriangle->getP1());
    //Star s2(myTriangle->getP2());
    
    //Star mcStar(mc.getCenter());

    //printf(" HTM has points (ra,dec) (%lg, %lg), (%lg, %lg), (%lg,
    //%lg)", s0.getRa(Hour()), s0.getDec(Degree()), s1.getRa(Hour()),
    //s1.getDec(Degree()), s2.getRa(Hour()), s2.getDec(Degree()) );
    
    //printf(" Overlap region centered on (ra, dec) (%lg, %lg)",
    //mcStar.getRa( Hour() ), mcStar.getDec( Degree() ) );

    if ( !mine.intersectQ(samp) ) {
      //printf( "No overlap in " + name);
      return ret;
    }
  } // fi (myTriangle)
  
  // Looking for region in
  //printf( "Overlap in " + name );
  
  // Here we have to find data, so load from file      
  if (!fileLoaded) {
    
    struct stat st;
    
	if (stat(fileName.c_str(), &st)==0) {
      //printf( "Opening " + fileName + " to read data." );
      read( fileName );  // Loads myStars
    }
    
    fileLoaded = true;
  }
  
  // Find the intersection with myStars.
  std::list<Star>::iterator it;
  for (it = myStars.begin(); it != myStars.end(); ++it) {
    if ( (* it).getMagnitude() <= r.magnitudeLimit() &&
	 r.location( *it ) == RegionGS::Inside) {
      ret.push_back( *it );
      //printf("Star (%lg, %lg, %lg) from %s is accepted", 
      
        //(*it).getRa( Hour() ), (*it).getDec( Degree() ),
        //(*it).getMagnitude(), name.latin1() );
    }
    //else {
    //printf("Star (%lg, %lg, %lg) from %s is rejected", (*it).getRa(
    //Hour() ), (*it).getDec( Degree() ), (*it).getMagnitude(),
    //name.latin1() ); }
  }
  
  if (myStars.size()) {
    //cout << " Checking location against " << myStars.size() << "
    //stars in " << name ;
  
    //cout << ". Found " << ret.size() << endl;
  }
  
  // Add stars present in the children, after checking for events
  if (!childrenCreated) { makeChildren(); }
  
  HTMCatalog *child;
  std::list<Star> found;
  std::list<HTMCatalog*>::iterator childit;
  std::list<Star>::iterator retit;
  for (childit = children.begin(); childit != children.end(); childit++) {
    child = *childit;
	found = child->find( r );
	retit = ret.end();
	ret.insert(retit,found.begin(),found.end());
  }
  // Return what we have collected 
  return ret;
}



int HTMCatalog::main(int , char *[]) {
  
  HTMCatalog catalog("E:\catalog\gsc1");
  
  // Make a small circular region
  Hour ra = 7;
  Degree dec = 65.;
  Star s(ra, dec, 8.76);
  
  //MinorCircleRegion mcr( Point(.5, .5, .5), Degree(0.2) );
  MinorCircleRegion mcr( s, Degree(0.2) );
  mcr.setMagnitudeLimit( 12.0 );
  //cout << "Finding stars near " << mcr.getCenter() << endl;
  //cout << "Sample star s " << s << endl;
  //cout << "Point of s " << Point(s) << endl;
  
  std::list<Star> star = catalog.find( mcr );
  std::list<Star>::iterator it;
  for (it = star.begin(); it != star.end(); it++) {
    cout << (*it) << endl;
  }
  
  return 0;
}



void HTMCatalog::addFile(string file) {
 
  //emit( message( "Adding stars from " + file ) );
  // Open GSC file  
  // Largest file on the first CD is 3588.gsc at 607680 bytes. (13289
  // entries)
  // Smallest is  5104.gsc at 23040 bytes. (258 entries)

  STAR gsc[MAX_STARS_PER_FILE];
  
  // Flag the Ra elements for later check
  for (int i=0; i<MAX_STARS_PER_FILE; i++) gsc[i].ra = -1;
    
  // Read out the header
  int status = read_table( file.c_str(), gsc, GSC_MODE);
  //emit( message( "Read " + QString::number(status) + " records." ) );
    
  // Print the first 10 stars
  int j;
  for (j=1; j<=status; j++) {    
    if (j <= 0) {
      string sp = " ";
      //emit ( message( QString::number(j) + sp + 
      //                QString::number(gsc[j].ra) + sp + 
      //                QString::number(gsc[j].dec) + sp + 
      //                QString::number(gsc[j].magnitude) ) );
    }    
  }

  int total = j-1;
  //emit( message( "Found stars from 1 to " + QString::number(j-1) ) ); 
    
  // Add all the stars to the catalog
  for (j=1; j<= status; j++) {
    // Make a Star from a STAR, if it was filled in
    if (gsc[j].ra != -1) {
      Star s = Star( Degree( gsc[j].ra ) , 
		     Degree( gsc[j].dec ) ,
		     gsc[j].magnitude );
      add( s );
    }
  }
  
  // Sync here needs to not overwrite since a given node may come from
  // multiple files.  If a second file overwrites, then stars will be
  // lost.
  //sync();
  
  //unsigned long cnt = print();
  //emit(  message( "Catalog holds " +  QString::number(cnt) )  ); 
}

//void HTMCatalog::relayMessage(QString mes)
//{
//    emit( message(mes) );
//}

void HTMCatalog::printMessage(string mes) {
	cout << mes << endl;
}


int HTMCatalog::store(int argc, char *argv[]) {  
  if (argc == 1) {
    cout << "Usage: 'demo file1 [file2 file3 ...]'" << endl
	      << "Reads GSC files and stores them into the HTMCatalog" << 
      endl;
    return -1;
  }
  
  HTMCatalog catalog("E:\catalog\gsc1");

  catalog.init();
  
  for (int i=1; i<argc; i++) {
	  cout << "Reading from " << argv[i] << endl;
    catalog.addFile( string(argv[i]) );
    cout << "Added " + string(argv[i]) << endl;
  }
  unsigned long cnt = catalog.print();
  //catalog.relayMessage( "Catalog holds " +  QString::number(cnt) );
  catalog.sync();
  return 0;
}


