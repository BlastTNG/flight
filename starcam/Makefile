################################################################################
# starcam makefile by Steve Benton
################################################################################
APP_NAME   = starcam
TEST_NAME  = testmain
USE_QT = 1           #change this to 0 to not use qt when its not available
                 #probably need to change bits of code too
O_APP      = csbigcam.o csbigimg.o mycam.o clensadapter.o bloblist.o \
frameblob.o blobimage.o camcommunicator.o pyramid.o camcommserver.o
O_APP_QT = imageviewer.o moc_imageviewer.o commandgui.o moc_commandgui.o
################################################################################
SRC_DIR    = .
INCLUDE    = -I $(SRC_DIR)
LIBRARY    = -L $(SRC_DIR)
#CFLAGS = -O2 $(INCLUDE) $(LIBRARY) -Wall
CFLAGS = -O0 $(INCLUDE) $(LIBRARY) -Wall -g   #for debugging
CPPFLAGS = $(CFLAGS)
CC = g++
MOC = moc

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
all: cam comm view util
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
	  $(CC) $(QTLIB) $(CFLAGS) -o $(APP_NAME) $(APP_OBJ) -lsbigudrv -lm -lpthread
	  chmod a+x $(APP_NAME)

starcam.o : starcam.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h clensadapter.h frameblob.h \
	camconfig.h clensadapterdefs.h csbigimgdefs.h camcommunicator.h pyramid.h imageviewer.h camcommserver.h
	  $(CC) $(CFLAGS) $(QTFLAGS) -o starcam.o -c starcam.cpp

csbigcam.o : csbigcam.h csbigcam.cpp csbigimg.h camconfig.h
	  $(CC) $(CFLAGS) -o csbigcam.o -c csbigcam.cpp
	  
mycam.o : mycam.h mycam.cpp clensadapter.h blobimage.h camconfig.h csbigcam.h
	  $(CC) $(CFLAGS) -o mycam.o -c mycam.cpp

csbigimg.o : csbigimg.cpp csbigimg.h camconfig.h csbigimgdefs.h
	  $(CC) $(CFLAGS) -o csbigimg.o -c csbigimg.cpp
	  
clensadapter.o : clensadapter.h clensadapter.cpp camconfig.h clensadapterdefs.h
	  $(CC) $(CFLAGS) -o clensadapter.o -c clensadapter.cpp

bloblist.o : bloblist.h bloblist.cpp
	  $(CC) $(CFLAGS) -o bloblist.o -c bloblist.cpp

frameblob.o : frameblob.h frameblob.cpp bloblist.h camconfig.h
	  $(CC) $(CFLAGS) -o frameblob.o -c frameblob.cpp

blobimage.o : blobimage.h blobimage.cpp frameblob.h bloblist.h camconfig.h pyramid.h
	  $(CC) $(CFLAGS) -o blobimage.o -c blobimage.cpp
	  
camcommunicator.o : camcommunicator.h camcommunicator.cpp mycam.h blobimage.h camconfig.h
	  $(CC) $(CFLAGS) -o camcommunicator.o -c camcommunicator.cpp

camcommserver.o : camcommserver.h camcommserver.cpp camstruct.h camcommunicator.h
	  $(CC) $(CFLAGS) -o camcommserver.o -c camcommserver.cpp

pyramid.o : pyramid.h pyramid.cpp
	  $(CC) $(CFLAGS) -O0 -o pyramid.o -c pyramid.cpp
	  
################################################################################
#image viewer
VIEWER_OBJ=scviewer.o blobimage.o imageviewer.o moc_imageviewer.o csbigimg.o frameblob.o bloblist.o
scviewer: $(VIEWER_OBJ)
	  $(CC) $(QTLIB) $(CFLAGS) -o scviewer $(VIEWER_OBJ) -lpthread
	  chmod a+x scviewer

scviewer.o : scviewer.cpp blobimage.h csbigimgdefs.h imageviewer.h
	  $(CC) $(CFLAGS)  $(QTFLAGS) -o scviewer.o -c scviewer.cpp

imageviewer.o : imageviewer.h imageviewer.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o imageviewer.o -c imageviewer.cpp
	  
moc_imageviewer.cpp : imageviewer.h
	  $(MOC) imageviewer.h -o moc_imageviewer.cpp
	  
moc_imageviewer.o : imageviewer.h moc_imageviewer.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o moc_imageviewer.o -c moc_imageviewer.cpp

	  
################################################################################
#simple test program
test: testmain commtest

testmain : $(TEST_OBJ)
	  $(CC) $(CFLAGS) $(QTLIB) -o $(TEST_NAME) $(TEST_OBJ) -lsbigudrv -lm -lpthread
	  chmod a+x $(TEST_NAME)
	  
testmain.o : testmain.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h clensadapter.h frameblob.h \
				camconfig.h clensadapterdefs.h csbigimgdefs.h camcommunicator.h pyramid.h imageviewer.h
	  $(CC) $(CFLAGS) $(QTFLAGS) -o testmain.o -c testmain.cpp
	  
################################################################################
#test communicator app
COMMTEST_OBJ=testcomm.o camcommunicator.o commandgui.o moc_commandgui.o
commtest : $(COMMTEST_OBJ)
	  $(CC) $(CFLAGS) $(QTLIB) -o commtest $(COMMTEST_OBJ) -lpthread
	  chmod a+x commtest
	  
testcomm.o : testcomm.cpp camcommunicator.h camconfig.h commandgui.h
	  $(CC) $(CFLAGS)  $(QTFLAGS) -o testcomm.o -c testcomm.cpp

commandgui.o : commandgui.h commandgui.cpp camcommunicator.h camconfig.h
	  $(CC) $(CFLAGS) $(QTFLAGS) -o commandgui.o -c commandgui.cpp
	  
moc_commandgui.cpp : commandgui.h
	  $(MOC) commandgui.h -o moc_commandgui.cpp
	  
moc_commandgui.o : commandgui.h moc_commandgui.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o moc_commandgui.o -c moc_commandgui.cpp

################################################################################
# utility programs that are not related to main functionality
util: powercycle findbadpixel fixbadpixel

# program to power cycle the camera with a parallel port dongle
powercycle: powercycle.c

# program to analyze bad pixel data
findbadpixel: findbadpixel.o blobimage.o frameblob.o bloblist.o csbigimg.o

findbadpixel.o: findbadpixel.cpp blobimage.h frameblob.h bloblist.h camconfig.h camstruct.h \
  csbigimg.h csbigimgdefs.h

# program to use a bad pixel file to correct a bad pixel
fixbadpixel: fixbadpixel.o blobimage.o frameblob.o bloblist.o csbigimg.o

fixbadpixel.o: fixbadpixel.cpp blobimage.h frameblob.h bloblist.h camconfig.h camstruct.h \
  csbigimg.h csbigimgdefs.h

################################################################################

clean:
	rm -f *.o testmain commtest starcam moc_*


