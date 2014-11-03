#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <stdlib.h>

#include <iostream>
#include <list>

#include <qdir.h>
#include <qfile.h>
#include <qstring.h>
#include <qapplication.h>

#include "HTMCatalog.h"
#include "SegmentRegion.h"

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
  children.setAutoDelete( true );
  
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
  
  children.setAutoDelete( true );
  
  // Append the str onto the name.
  name += str;
  
  // Pass messages back to parents.
  //connect( this, SIGNAL(message(QString)), p, SLOT(relayMessage(QString)) );
  
  // If this below the file level, add more children
  if (level < fileLevel) {
      // Changed to delayed instaniation  06-29-02
      //makeChildren();

      //emit(  message( "Created node " + name ) );
  }
  
  // We are on a data node
  if (level == fileLevel) {
    childrenCreated = true;
    
    // Check that each level exists.
    QDir dir(fileBase);
    if (!dir.exists()) dir.mkdir(fileBase);
    
    dir.setPath(fileBase + "/" + name.left(2));
    if (!dir.exists()) dir.mkdir(fileBase + "/" + name.left(2));
    
    dir.setPath(fileBase + "/" + name.left(2) + "/" + name.mid(2, 3) );
    if (!dir.exists()) dir.mkdir(fileBase + "/" + name.left(2)  + "/" + 
				 name.mid(2, 3));
    
    
    // Make directory and filename
    fileName = fileBase + "/" + name.left(2) + "/" + name.mid(2, 3) +
      "/" + prefix + name + suffix;
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
  
  children.append( new HTMCatalog(this, S0, "S0") );
  children.append( new HTMCatalog(this, S1, "S1") );
  children.append( new HTMCatalog(this, S2, "S2") );
  children.append( new HTMCatalog(this, S3, "S3") );
  children.append( new HTMCatalog(this, N0, "N0") );
  children.append( new HTMCatalog(this, N1, "N1") );
  children.append( new HTMCatalog(this, N2, "N2") );
  children.append( new HTMCatalog(this, N3, "N3") );
  
  myTriangle = 0;
}

void HTMCatalog::makeChildren() {
  
  if (level == -1) return;  // Root node already has children
  
  // Clear any existing children
  children.clear();

  // Use the side midpoints
  Point w0 = myTriangle->getP1() + myTriangle->getP2();
  Point w1 = myTriangle->getP0() + myTriangle->getP2();
  Point w2 = myTriangle->getP0() + myTriangle->getP1();
  
  TriangleRegion* t0 = new TriangleRegion( myTriangle->getP0(), w2, w1);
  TriangleRegion* t1 = new TriangleRegion( myTriangle->getP1(), w0, w2);
  TriangleRegion* t2 = new TriangleRegion( myTriangle->getP2(), w1, w0);
  TriangleRegion* t3 = new TriangleRegion( w0, w1, w2);

  children.append( new HTMCatalog(this, t0, "0") );
  children.append( new HTMCatalog(this, t1, "1") );
  children.append( new HTMCatalog(this, t2, "2") );
  children.append( new HTMCatalog(this, t3, "3") );

  childrenCreated = true;
}

// Read data into this node from fileName.
// Note that region checking is not done here. i.e. 
// file may not match Region of this node.

void HTMCatalog::read(QString fileName) {

  FILE *fp = fopen(fileName, "rb");

  if (fp == 0) {
    qWarning( "Error opening %s", fileName.latin1() ); 
    exit(-1);
  }
  
  //qDebug("Sizeof double is %d", sizeof(double));
  
  struct stat buf;
  int fh = fileno( fp );
  
  //int result = fstat( fh, &buf );
  fstat( fh, &buf );
  
  //qDebug("File size is %d", buf.st_size);
  
  double val;
  while ( fread( &val, sizeof(double), 1, fp) ) {
    
    // Read the other two doubles
    Degree ra = Degree(val);
    
    fread( &val, sizeof(double), 1, fp);
    Degree dec = Degree(val);
    
    fread( &val, sizeof(double), 1, fp);
    double mag = val;
    
    myStars.append( Star( Radian(ra), Radian(dec), mag) );
  }
  
  // ED
  //qDebug("Read %ld stars from %s into %s. ", 
  //      myStars.count(), fileName.latin1(), name.latin1());
  
  fclose( fp );
}


// Prints a summary of the catalog and 
// returns the number of stars in the catalog

unsigned long HTMCatalog::print() {

  // If I am at the file level, then I have stars
  if (level == fileLevel) {
    //if (myStars.count() > 0) {
    //    emit(  message( name + " " + QString::number(myStars.count()) )  );
    //}
    return (unsigned long)myStars.count();
  }
  
  // Else return the sum of the print of all of the chilren
  unsigned long ret = 0;
  for (unsigned i=0; i<children.count(); i++) {
    ret += children.at(i)->print();
  }
  return ret;
}

void HTMCatalog::loadFromFile(const char *str) {
  fileBase = str;
}

void HTMCatalog::sync() {
  // If there are any children, sync them also
  HTMCatalog *child;
  child = children.first();
  for (child = children.first(); child; child=children.next()) {
    child->sync();
  }
  
  
  // Skip non-bottom nodes
  if (myStars.count() == 0) return;
  
  if (level < fileLevel) return;
  
  //emit(  message( "Syncing to " + fileName )  );
  // Write all stars and then write stars of any children.
  QFile file( fileName );
  if ( !file.open( IO_WriteOnly ) ) {
    //emit(  message( "Error opening " + fileName )  );
  }
  
  QValueList<Star>::iterator it;
  
  for (it = myStars.begin(); it != myStars.end(); ++it) {
    // Write value to file (in Decimal degrees)
    double ra = (*it).getRa( Degree() );
    file.writeBlock( (const char *) &ra, sizeof(double) )  ;
    
    double dec = (*it).getDec( Degree() );
    file.writeBlock( (const char *) &dec, sizeof( double )  );
    
    double mag = (*it).getMagnitude() ;
    file.writeBlock( (const char *) &mag, sizeof( double )  );
  }
  file.close();
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
    for (int i=0; i<8; i++) {
      if (children.at(i)->add( s )) return true;
    }
    return false;
  }
  
  // If star is inside our region, then add until one of our children accepts

  if ( myTriangle->insideQ(s) ) {

    if (!childrenCreated) { makeChildren(); }
    
    if (children.count() == 0) {
      
      // Load other stars first
      if (!fileLoaded) {
	QFile file( fileName );

	if (file.exists()) {
        
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
      if (myStars.contains( s ) == 0) {
	myStars.append( s );
      } else {
	//emit( message("Star " + s.asQString() + " already in list") );
      }
      return true;
    }
    
    // Else pass onto the children
    HTMCatalog *child = children.first();
    
    // Loop until accepted or no more children
    while ( child && !(child->add( s )) ) {
      child = children.next();
    }
    
    if (child == 0) {
      //std::cerr << "No child picked up star\n";
      //myStars.append( s );
    }
  }
  
  // Star is not in this triangle
  //std::cout << s << " Not in " << myTriangle->getP0() << " " <<
  //    myTriangle->getP1() << " "  << myTriangle->getP2() << endl;
  return false;
}

QValueList<Star> HTMCatalog::find(const RegionGS& r) {

  // If the intersection between this and the triangle is empty,
  // return an empty list.  If there is an intersection, collect all
  // of the child responses.  

  //qDebug( "Looking for stars in " + name );
  
  QValueList<Star> ret;
  if (myTriangle) {
    SegmentRegion mine( *myTriangle ); // Could be done once at node creation

    const MinorCircleRegion mc = dynamic_cast<const MinorCircleRegion &>(r);
    SegmentRegion samp(mc);

    //qDebug("Checking %s for overlap.", name.latin1() );
    //Star s0(myTriangle->getP0());
    //Star s1(myTriangle->getP1());
    //Star s2(myTriangle->getP2());
    
    //Star mcStar(mc.getCenter());

    //qDebug(" HTM has points (ra,dec) (%lg, %lg), (%lg, %lg), (%lg,
    //%lg)", s0.getRa(Hour()), s0.getDec(Degree()), s1.getRa(Hour()),
    //s1.getDec(Degree()), s2.getRa(Hour()), s2.getDec(Degree()) );
    
    //qDebug(" Overlap region centered on (ra, dec) (%lg, %lg)",
    //mcStar.getRa( Hour() ), mcStar.getDec( Degree() ) );

    if ( !mine.intersectQ(samp) ) {
      //qDebug( "No overlap in " + name);
      return ret;
    }
  } // fi (myTriangle)
  
  // Looking for region in
  //qDebug( "Overlap in " + name );
  
  // Here we have to find data, so load from file
  if (!fileLoaded) {
    
    QFile file( fileName );
    
    if (file.exists()) {
      //qDebug( "Opening " + fileName + " to read data." );
      read( fileName );  // Loads myStars
    }
    
    fileLoaded = true;
  }
  
  // Find the intersection with myStars.
  QValueList<Star>::iterator it;
  for (it = myStars.begin(); it != myStars.end(); ++it) {
    if ( (* it).getMagnitude() <= r.magnitudeLimit() &&
	 r.location( *it ) == RegionGS::Inside) {
      ret.append( *it );
      //qDebug("Star (%lg, %lg, %lg) from %s is accepted", 
      
      //  (*it).getRa( Hour() ), (*it).getDec( Degree() ),
      //  (*it).getMagnitude(), name.latin1() );
    }
    //else {
    //qDebug("Star (%lg, %lg, %lg) from %s is rejected", (*it).getRa(
    //Hour() ), (*it).getDec( Degree() ), (*it).getMagnitude(),
    //name.latin1() ); }
  }
  
  if (myStars.count()) {
    //cout << " Checking location against " << myStars.count() << "
    //stars in " << name ;
  
    //cout << ". Found " << ret.count() << endl;
  }
  
  // Add stars present in the children, after checking for events
  if (qApp) { qApp->processEvents(); }
  if (!childrenCreated) { makeChildren(); }
  
  HTMCatalog *child;
  child = children.first();
  for (child = children.first(); child; child=children.next()) {
    ret += child->find( r );
  }

  // Return what we have collected 
  return ret;
}



int HTMCatalog::main(int , char *[]) {
  
  HTMCatalog catalog("c:/catalog/gsc1");
  
  // Make a small circular region
  Hour ra = 7;
  Degree dec = 65.;
  Star s(ra, dec, 8.76);
  
  //MinorCircleRegion mcr( Point(.5, .5, .5), Degree(0.2) );
  MinorCircleRegion mcr( s, Degree(0.2) );
  mcr.setMagnitudeLimit( 12.0 );
  //std::cout << "Finding stars near " << mcr.getCenter() << std::endl;
  //std::cout << "Sample star s " << s << std::endl;
  //std::cout << "Point of s " << Point(s) << std::endl;
  
  QValueList<Star> star = catalog.find( mcr );
  QValueList<Star>::iterator it;
  for (it = star.begin(); it != star.end(); it++) {
    std::cout << (*it) << std::endl;
  }
  
  return 0;
}



void HTMCatalog::addFile(QString file) {
 
  //emit( message( "Adding stars from " + file ) );
  // Open GSC file  
  // Largest file on the first CD is 3588.gsc at 607680 bytes. (13289
  // entries)
  // Smallest is  5104.gsc at 23040 bytes. (258 entries)

  STAR gsc[MAX_STARS_PER_FILE];
  
  // Flag the Ra elements for later check
  for (int i=0; i<MAX_STARS_PER_FILE; i++) gsc[i].ra = -1;
    
  // Read out the header
  int status = read_table( file.latin1(), gsc, GSC_MODE);
  //emit( message( "Read " + QString::number(status) + " records." ) );
    
  // Print the first 10 stars
  int j;
  for (j=1; j<=status; j++) {    
    if (j <= 0) {
      QString sp = " ";
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

void HTMCatalog::printMessage(QString mes) {
  // Print message with qDebug
  qDebug(mes);
}


int HTMCatalog::store(int argc, char *argv[]) {  
  if (argc == 1) {
    std::cout << "Usage: 'demo file1 [file2 file3 ...]'" << std::endl
	      << "Reads GSC files and stores them into the HTMCatalog" << 
      std::endl;
    return -1;
  }
  
  HTMCatalog catalog("/mnt/win_d/catalog/gsc1");

  connect(&catalog, SIGNAL( message(QString) ), &catalog, 
	  SLOT( printMessage(QString) ) );

  catalog.init();
  
  for (int i=1; i<argc; i++) {
    std::cout << "Reading from " << argv[i] << endl;
    catalog.addFile( QString(argv[i]) );
    qDebug( "Added " + QString(argv[i]) );
  }
  unsigned long cnt = catalog.print();
  //catalog.relayMessage( "Catalog holds " +  QString::number(cnt) );
  catalog.sync();
  return 0;
}


