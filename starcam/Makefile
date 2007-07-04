####################################################################################
# starcam makefile by Steve Benton
####################################################################################
APP_NAME   = starcam
TEST_NAME  = testmain
USE_QT = 1           #change this to 0 to not use qt when its not available
                 #probably need to change bits of code too
O_APP      = csbigcam.o csbigimg.o mycam.o clensadapter.o bloblist.o \
frameblob.o blobimage.o camcommunicator.o pyramid.o
O_APP_QT = imageviewer.o moc_imageviewer.o commandgui.o moc_commandgui.o
####################################################################################
SRC_DIR    = .
INCLUDE    = -I $(SRC_DIR)
LIBRARY    = -L $(SRC_DIR)
#the second CFLAGS variable is for use with valgrind while debugging
#CFLAGS = -O2 $(INCLUDE) $(LIBRARY) -Wall
CFLAGS = -O0 $(INCLUDE) $(LIBRARY) -Wall -g
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
####################################################################################
#targets for building all or part of the main program
all: cam comm
cam: $(APP_NAME)
comm: commtest
####################################################################################
# APP_NAME
####################################################################################

$(APP_NAME): $(APP_OBJ)
	  $(CC) $(QTLIB) $(CFLAGS) -o $(APP_NAME) $(APP_OBJ) -lsbigudrv -lm -lpthread
	  chmod a+x $(APP_NAME)

#main program may not depend on all of the headers
starcam.o : starcam.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h clensadapter.h frameblob.h \
			camconfig.h clensadapterdefs.h csbigimgdefs.h camcommunicator.h pyramid.h imageviewer.h
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

pyramid.o : pyramid.h pyramid.cpp
	  $(CC) $(CFLAGS) -O0 -o pyramid.o -c pyramid.cpp
	  
#don't try to compile classes for Qt based object when it isn't available
ifeq ($(strip $(USE_QT)), 1)
imageviewer.o : imageviewer.h imageviewer.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o imageviewer.o -c imageviewer.cpp
	  
moc_imageviewer.cpp : imageviewer.h
	  $(MOC) imageviewer.h -o moc_imageviewer.cpp
	  
moc_imageviewer.o : imageviewer.h moc_imageviewer.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o moc_imageviewer.o -c moc_imageviewer.cpp

commandgui.o : commandgui.h commandgui.cpp camcommunicator.h camconfig.h
	  $(CC) $(CFLAGS) $(QTFLAGS) -o commandgui.o -c commandgui.cpp
	  
moc_commandgui.cpp : commandgui.h
	  $(MOC) commandgui.h -o moc_commandgui.cpp
	  
moc_commandgui.o : commandgui.h moc_commandgui.cpp
	  $(CC) $(CFLAGS) $(QTFLAGS) -o moc_commandgui.o -c moc_commandgui.cpp
endif

	  
####################################################################################
#target for testing
test: testmain commtest

testmain : $(TEST_OBJ)
	  $(CC) $(CFLAGS) $(QTLIB) -o $(TEST_NAME) $(TEST_OBJ) -lsbigudrv -lm -lpthread
	  chmod a+x $(TEST_NAME)
	  
testmain.o : testmain.cpp csbigcam.h csbigimg.h mycam.h blobimage.h bloblist.h clensadapter.h frameblob.h \
				camconfig.h clensadapterdefs.h csbigimgdefs.h camcommunicator.h pyramid.h imageviewer.h
	  $(CC) $(CFLAGS) $(QTFLAGS) -o testmain.o -c testmain.cpp
	  
commtest : testcomm.o camcommunicator.o commandgui.o moc_commandgui.o
	  $(CC) $(CFLAGS) $(QTLIB) -o commtest testcomm.o camcommunicator.o \
		commandgui.o moc_commandgui.o -lpthread
	  chmod a+x commtest
	  
testcomm.o : testcomm.cpp camcommunicator.h camconfig.h commandgui.h
	  $(CC) $(CFLAGS)  $(QTFLAGS) -o testcomm.o -c testcomm.cpp

####################################################################################

clean:
	rm -f *.o testmain commtest starcam moc_*


