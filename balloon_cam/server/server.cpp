//server.cpp

#include "server.h"

Server::Server(){
	init();
}

void Server::init(){
	addrinfo hints;
	addrinfo* serverInfo = (addrinfo *)malloc(sizeof(addrinfo));

	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;
	//gets the address info for the laptop at 192.168.1.104
	int status = getaddrinfo("192.168.1.119", DEFAULT_PORT, &hints, &serverInfo);

	if(status != 0){
		printf("getaddrinfo fails, error %s\n", gai_strerror(status));
		return;
	}
	//copies the data into the member variables
	servInfo = new addrinfo();

	memcpy(servInfo, serverInfo, sizeof(*serverInfo));

	servInfo->ai_addr = new sockaddr();

	memcpy(servInfo->ai_addr, serverInfo->ai_addr, sizeof(sockaddr));
	//calls socket to create an outgoing socket 
	int socketFD = socket(servInfo->ai_family, servInfo->ai_socktype, servInfo->ai_protocol);
	//frees the now useless addrinfo struct
	freeaddrinfo(serverInfo);

	if(socketFD == -1){
		printf("Socket has failed.\n");
	}

	socketFileDescriptor = socketFD;
	//sets a send timeout limit, otherwise it can hang perpetually
	timeval timeLim;
	timeLim.tv_sec = 30;
	timeLim.tv_usec = 0;

	status = setsockopt(socketFileDescriptor, SOL_SOCKET, SO_SNDTIMEO, (void *) &timeLim, sizeof(timeval));
	
	if(status == -1){
		printf("Setting the send timeout has failed. Error: %s\n",strerror(errno));
	}

	status = setsockopt(socketFileDescriptor, SOL_SOCKET, SO_RCVTIMEO, (void*) &timeLim, sizeof(timeval));
	
	if(status == -1){
		printf("Setting the recieve timeout has failed. Error: %s\n", strerror(errno));
	}

	//conects to the laptop with information given in the addrinfo struct
	status = connect(socketFileDescriptor, servInfo->ai_addr, servInfo->ai_addrlen);

	if(status != 0){
		printf("Connect has failed. Error: %s\n", strerror(errno));
	}

	lag = 0;
	lagTime.tv_usec = 0;
	lagTime.tv_sec = 0;
}

Server::~Server(){

	delete servInfo->ai_addr;
	delete servInfo;

}

void Server::runLoop(){

timeval timeValue;
char* data;
bool resend = false;
int x;
int y;
timeval returnedTime;


	while(true){

		//sleep(1);//for testng only. Otherwise it is way too fast
		if(!resend){
			//NOTE: TO BE REPLACED WITH SYSTEM CALL TO BLAST BUS SOMEHOW
			gettimeofday(&timeValue, NULL);//gets a time value
			
			if(abs(lagTime.tv_sec - timeValue.tv_sec) > 300){
				updateLag();
				gettimeofday(&timeValue, NULL);
			}
		}
		printf("sending data: %ld seconds, %ld microseconds\n", timeValue.tv_sec, timeValue.tv_usec);
		sendData(timeValue);//sends the query time to the laptop
		resend = false;		
		
		data = recieveData();//recieves the response

		if(!strcmp(data, "resend\n")){//if there was a resend request
			printf("Resending old data\n");
			resend = true;
			free(data);
			continue;
		}else{
			if(!strcmp(data, "")){//if there was an error
				printf("Exiting\n");
				free(data);
				return;
			}
			//extracts data from the string
			sscanf(data, "X=%d;Y=%d;s=%ld;us=%ld\n", &x, &y, &returnedTime.tv_sec, &returnedTime.tv_usec);
			passData(x, y, returnedTime);//passes data back to the blast bus
			free(data);
		}
	}
}

void Server::sendData(timeval time){
	
	char* buffer = (char*)malloc(100);
	sprintf(buffer,"%ld s; %ld us;\n\0", time.tv_sec, time.tv_usec);
	//sends data as a string
	int bytesSent = send(socketFileDescriptor, buffer, strlen(buffer), 0);
	//notifies that there was an error in sending. Does not retry, as this will 
	//screw up the retry request
	if(bytesSent != strlen(buffer)){
		printf("Not all data sent. Expect a retry request.\n");
	}
	free(buffer);

}

char* Server::recieveData(){
	//recieves data into a buffer
	char* buffer = (char*) malloc(1000);
	int status = recv(socketFileDescriptor, buffer, 1000, 0);
	//makes damn sure there is a null character in there somewhere
	*(buffer + 999) = '\0';
	*(buffer + status) = '\0';

	//NOTE: RAND IS HERE FOR TESTING PURPOSES. REMOVE IT BEFORE USE 
	if((status < 0)/*||(rand() %10 < 2)*/){//if there was an error
		printf("Recieve has failed. Error: %s\n", strerror(errno));
		free(buffer);
		timeval fakeTime;
		fakeTime.tv_sec = -2;
		sendData(fakeTime);//send a retry request
		return recieveData();
	}		
	if(status == 0){//if the connection has closed from the other end
		printf("Client has closed the connection\n");
		free(buffer);
		char* fakeString = new char();//fakes a return value
		return fakeString;
	}
	if(strchr(buffer, '\n') != NULL){//otherwise, if the string is properly formatted
		return buffer;
	}
}

//NOTE: DUMMY IMPLEMENTATION. TO BE REPLACED WITH SOME SORT OF SYSTEM CALL

void Server::passData(int x, int y, timeval t){

	printf("Data callibrated: X = %d, y = %d, s = %ld, us = %ld\n", x, y, t.tv_sec + ((lag/2)/1000000), t.tv_usec + ((lag/2) % 1000000));

}


void Server::updateLag(){
	timeval fakeTime;
	fakeTime.tv_sec = -3;
	fakeTime.tv_usec = 0;
	char* returnedData;
	timeval sysTime1;
	timeval sysTime2;
	lag = 0;
	for(int i = 0; i< 10; i++){
		
		gettimeofday(&sysTime1, NULL);
		sendData(fakeTime);
		returnedData = recieveData();
		if(strcmp(returnedData, "ping\n")){
			printf("Ping not returned, instead got: %s\n", returnedData);
		}

		gettimeofday(&sysTime2, NULL);
		lag += ((sysTime2.tv_sec - sysTime1.tv_sec)*1000000 + (sysTime2.tv_usec - sysTime1.tv_usec));
	}

	lag /= 10;
	printf("Lag found to be %d us\n", lag);
	lagTime = sysTime2;

}
