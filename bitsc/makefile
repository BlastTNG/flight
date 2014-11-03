cameraPointer : cameraAlignment.o camera.o cameraCommunicator.o pixelFixer.o locator.o
	gcc -Wall -g -lm -L/usr/lib/ -lueye_api -lpthread -lrt -ldl -o cameraPointer camera.o cameraCommunicator.o pixelFixer.o locator.o CameraAlignment.o

camera.o : camera.cpp camera.h pixelFixer.h stdafx.h
	gcc -Wall -c camera.cpp

cameraCommunicator.o : cameraCommunicator.cpp cameraCommunicator.h stdafx.h
	gcc -Wall -c cameraCommunicator.cpp

pixelFixel.o : pixelFixer.cpp pixelFixer.h stdafx.h
	gcc -Wall -c pixelFixer.cpp

locator.o : locator.cpp locator.h stdafx.h
	gcc -Wall -c locator.cpp

cameraAlignment.o : CameraAlignment.cpp cameraCommunicator.h stdafx.h
	gcc -Wall -c CameraAlignment.cpp
