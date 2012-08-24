// CameraAlignment.cpp : main project file.

#include "stdafx.h"
#include "cameraCommunicator.h"

int main()
{
	//Sets the thread priority to make this a real time application.
	struct timespec t;
	struct sched_param param;
	
	param.sched_priority = 49;
	if(sched_setscheduler(0, SCHED_FIFO, &param) == -1){
		printf("Setting scheduler fails. Ensure that you are root.\n");
	}

	//prefaults the stack
	if(mlockall(MCL_CURRENT|MCL_FUTURE) == -1){
		printf("Locking memory fails. Ensure that you are root.\n");
	}
	
	unsigned char dummy[8*1024];
	memset(dummy, 0, 8*1024);


	Camera camera = Camera();
	Locator locator = Locator(camera.getXSize(), camera.getYSize(), camera.getPixelSize(), 10, STAR_TRACKER, &camera);
	blob answer;
//	CameraCommunicator communicator = CameraCommunicator(&locator);
//	communicator.addConnection(std::string("bradoon"));

	timeval time1;
	timeval time2;
	timeval time3;
	timeval oldtime;


	gettimeofday(&time1, NULL);

	int xSubSize = 48;
	int ySubSize = 50;
	int xSubCenter = 1160;
	int ySubCenter = 1020;

	for(int i = 0; i< 100000; i++){
		clock_gettime(CLOCK_MONOTONIC, &t);
		//sleep(43200);
		//void* pointer = camera.capture();
		//camera.saveActiveData();
		//if(pointer != NULL){
			oldtime.tv_sec = time3.tv_sec;
			oldtime.tv_usec = time3.tv_usec;
			gettimeofday(&time3, NULL);
			//printf("%ld\n", 1000000*(time3.tv_sec - oldtime.tv_sec) + time3.tv_usec - oldtime.tv_usec);
			answer = locator.locate(NULL, xSubSize, ySubSize, xSubCenter, ySubCenter);
			answer.centroid.x += xSubCenter;
			answer.centroid.y += ySubCenter;

			xSubCenter = answer.centroid.x + (xSubSize/2);
			ySubCenter = answer.centroid.y + (ySubSize/2);
		//}
		printf("Blob found at %d, %d. Width: %d, height %d\n", answer.centroid.x, answer.centroid.y, answer.max.x - answer.min.x +1, answer.max.y - answer.min.y+1);
		t.tv_nsec += 5000000;
		if(t.tv_nsec >= 1000000000){
			t.tv_nsec -= 1000000000;
			t.tv_sec ++;
		}
		clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
	}
	
//	communicator.removeConnection(std::string("bradoon"));

	gettimeofday(&time2, NULL);
//	printf("elapsed time = %ld microseconds\n", 1000000*(time2.tv_sec - time1.tv_sec) + time2.tv_usec - time1.tv_usec);
	camera.closeCamera();
}
