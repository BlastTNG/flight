SRC_DIR    = .
INCLUDE    = -I $(SRC_DIR)
LIBRARY    = -L $(SRC_DIR)
CPPFLAGS   = $(INCLUDE)
#CFLAGS = -O2 -Wall
#LDFLAGS = $(LIBRARY)
CFLAGS = -O0 -Wall -g   #for debugging
LDFLAGS = $(LIBRARY) -g
LD = gcc
CXX = g++
MOC = moc

########################################################
# Repository object/header path munging
#

SPIDER_ONLY = 1

EXPT_HEADERS = command_list.h command_struct.h pointing_struct.h
SHARED_OBJS = blast.o camcommunicator.o udp.o
SHARED_HEADERS = blast.h camcommunicator.h camstruct.h udp.h

include ../common/Makefile.shared

# now $(OBJS) and $(HEADERS) are defined, and $(CPPFLAGS) has been modified

################################################################################
# starcam makefile by Steve Benton
################################################################################
APP_NAME   = starcam
TEST_NAME  = testmain
USE_QT = 1           #change this to 0 to not use qt when its not available
                 #probably need to change bits of code too
O_APP      = csbigcam.o csbigimg.o mycam.o clensadapter.o bloblist.o \
frameblob.o blobimage.o pyramid.o svd.o $(OBJS)
O_APP_QT = imageviewer.o moc_imageviewer.o commandgui.o moc_commandgui.o
################################################################################

#different definitions for when Qt is and isn't available
ifeq ($(strip $(USE_QT)), 1)
APP_OBJ    = $(APP_NAME).o $(O_APP) $(O_APP_QT)
TEST_OBJ   = $(TEST_NAME).o $(O_APP) $(O_APP_QT)
QTLIB = -L/usr/share/qt3/lib -L/usr/X11R6/lib -lqt-mt -lXext -lX11
QTINC = -I/usr/share/qt3/mkspecs/default -I/usr/include/qt3
QTFLAGS = $(QTINC) -pipe -D_REENTRANT  -DQT_NO_DEBUG -DQT_THREAD_SUPPORT \
		-DQT_SHARED -DQT_TABLET_SUPPORT
else
APP_OBJ    = $(APP_NAME).o $(O_APP)
TEST_OBJ   = $(APP_NAME).o $(O_APP)
QTLIB =
QTINC =
QTFLAGS =
endif

################################################################################
#targets for building all or part of the main program
all:: cam comm view util test
cam: $(APP_NAME)
comm: commtest
view: scviewer
################################################################################
# APP_NAME
################################################################################

install: $(APP_NAME)
	install -m 755 $(APP_NAME) /usr/local/bin
	install -d -m 777 /data/etc
	install -d -m 777 /data/rawdir/
	[ -e /data/etc/init.txt ] || touch /data/etc/init.txt
	[ -e /data/etc/badpix.txt ] || touch /data/etc/badpix.txt


$(APP_NAME): $(APP_OBJ)
	  $(LD) $(LDFLAGS) $(QTLIB) -o $(APP_NAME) $(APP_OBJ) -lsbigudrv -lm -lpthread
	  chmod a+x $(APP_NAME)

starcam.o : starcam.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h clensadapter.h frameblob.h \
	camconfig.h clensadapterdefs.h csbigimgdefs.h pyramid.h svd.h imageviewer.h $(HEADERS)
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o starcam.o -c starcam.cpp

csbigcam.o : csbigcam.h csbigcam.cpp csbigimg.h camconfig.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o csbigcam.o -c csbigcam.cpp
	  
mycam.o : mycam.h mycam.cpp clensadapter.h blobimage.h camconfig.h csbigcam.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o mycam.o -c mycam.cpp

csbigimg.o : csbigimg.cpp csbigimg.h camconfig.h csbigimgdefs.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o csbigimg.o -c csbigimg.cpp
	  
clensadapter.o : clensadapter.h clensadapter.cpp camconfig.h clensadapterdefs.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o clensadapter.o -c clensadapter.cpp

bloblist.o : bloblist.h bloblist.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o bloblist.o -c bloblist.cpp

frameblob.o : frameblob.h frameblob.cpp bloblist.h camconfig.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o frameblob.o -c frameblob.cpp

blobimage.o : blobimage.h blobimage.cpp frameblob.h bloblist.h camconfig.h pyramid.h svd.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -o blobimage.o -c blobimage.cpp
	  
pyramid.o : pyramid.h pyramid.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -O0 -o pyramid.o -c pyramid.cpp
	  
svd.o : svd.h svd.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) -O0 -o svd.o -c svd.cpp
	  
################################################################################
#image viewer
VIEWER_OBJ=scviewer.o blobimage.o pyramid.o svd.o imageviewer.o moc_imageviewer.o csbigimg.o frameblob.o bloblist.o
scviewer: $(VIEWER_OBJ)
	  $(LD) $(LDFLAGS) $(QTLIB) -o scviewer $(VIEWER_OBJ) -lpthread
	  chmod a+x scviewer

scviewer.o : scviewer.cpp blobimage.h csbigimgdefs.h imageviewer.h
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o scviewer.o -c scviewer.cpp

imageviewer.o : imageviewer.h imageviewer.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o imageviewer.o -c imageviewer.cpp
	  
moc_imageviewer.cpp : imageviewer.h
	  $(MOC) imageviewer.h -o moc_imageviewer.cpp
	  
moc_imageviewer.o : imageviewer.h moc_imageviewer.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o moc_imageviewer.o -c moc_imageviewer.cpp

	  
################################################################################
#simple test program
test: testmain commtest

testmain : $(TEST_OBJ)
	  $(LD) $(LDFLAGS) $(QTLIB) -o $(TEST_NAME) $(TEST_OBJ) -lsbigudrv -lm \
			-lpthread
		chmod a+x $(TEST_NAME)
	  
testmain.o : testmain.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h \
	clensadapter.h frameblob.h camconfig.h clensadapterdefs.h csbigimgdefs.h \
	pyramid.h svd.h imageviewer.h $(HEADERS)
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o testmain.o -c testmain.cpp
	  
################################################################################
#test communicator app
COMMTEST_OBJ=testcomm.o $(OBJS) commandgui.o moc_commandgui.o
commtest : $(COMMTEST_OBJ)
	  $(LD) $(LDFLAGS) $(QTLIB) -o commtest $(COMMTEST_OBJ) -lpthread
	  chmod a+x commtest
	  
testcomm.o : testcomm.cpp camconfig.h commandgui.h $(HEADERS)
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o testcomm.o -c testcomm.cpp

commandgui.o : commandgui.h commandgui.cpp camconfig.h $(HEADERS)
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o commandgui.o -c commandgui.cpp
	  
moc_commandgui.cpp : commandgui.h
	  $(MOC) commandgui.h -o moc_commandgui.cpp
	  
moc_commandgui.o : commandgui.h moc_commandgui.cpp
	  $(CXX) $(CPPFLAGS) $(CFLAGS) $(QTFLAGS) -o moc_commandgui.o -c moc_commandgui.cpp

################################################################################
# utility programs that are not related to main functionality
util: powercycle findbadpixel fixbadpixel

# program to power cycle the camera with a parallel port dongle
powercycle: powercycle.c

# program to analyze bad pixel data
findbadpixel: findbadpixel.o blobimage.o pyramid.o svd.o frameblob.o bloblist.o csbigimg.o
	g++   findbadpixel.o blobimage.o pyramid.o svd.o frameblob.o bloblist.o csbigimg.o   -o findbadpixel
	chmod a+x findbadpixel

findbadpixel.o: findbadpixel.cpp blobimage.h frameblob.h bloblist.h \
	camconfig.h $(HEADERS) csbigimg.h csbigimgdefs.h
	g++  -O0 -I . -I ../common -L . -Wall -g     -c -o findbadpixel.o findbadpixel.cpp

# program to use a bad pixel file to correct a bad pixel
fixbadpixel: fixbadpixel.o blobimage.o pyramid.o svd.o frameblob.o bloblist.o csbigimg.o
	g++   fixbadpixel.o blobimage.o pyramid.o svd.o frameblob.o bloblist.o csbigimg.o   -o fixbadpixel
	chmod a+x fixbadpixel

fixbadpixel.o: fixbadpixel.cpp blobimage.h frameblob.h bloblist.h camconfig.h \
  csbigimg.h csbigimgdefs.h $(HEADERS)
	g++  -O0 -I . -I ../common -L . -Wall -g     -c -o fixbadpixel.o fixbadpixel.cpp

################################################################################

clean:
	rm -f *.o powercycle findbadpixel fixbadpixel scviewer testmain commtest starcam moc_*


