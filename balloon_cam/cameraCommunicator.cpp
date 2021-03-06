//cameraCommunicator.cpp
//a class that contains a linked list of network connection classes, and uses them to communicate over the network

#include "stdafx.h"
#include "cameraCommunicator.h"

CameraCommunicator::CameraCommunicator(Locator* locator){//constructor
	init(locator);
}

CameraCommunicator::~CameraCommunicator(){//destructor
	for(unsigned int i =0; i< list->size(); i++){//ensures that no threads are still running
		if((*list)[i]->isLooping()){
			(*list)[i]->endLoop();
		}
		delete (*list)[i];//deletes the struct
		(*list)[i] = NULL;
	}

	closesocket(socketFileDescriptor);//closes the sockets
	list->clear();
	delete list;
	delete servInfo->ai_addr;
	delete servInfo;
	WSACleanup();//mandatory system cleanup call
}

void CameraCommunicator::init(Locator* locator){

	list = new std::vector<NetworkConnection*>;
	numConnections = 0;
	this->locator = locator;
	WSADATA wsaData;
	int status;

	status = WSAStartup(MAKEWORD(2,2), &wsaData);//mandatory system startup call

	if(status != 0){
		printf("Wsa startup failed, error code %d\n", WSAGetLastError);
		return;
	}

	addrinfo hints;
	addrinfo* serverInfo = new addrinfo;
	memset(&hints, 0, sizeof(hints));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_flags = AI_PASSIVE;

	status = getaddrinfo(NULL, DEFAULT_PORT, &hints, &serverInfo);//gets address info for a local socket

	if(status != 0){
		printf("getaddrinfo fails for port %d, error code %d\n", DEFAULT_PORT, WSAGetLastError());
		return;
	}
	servInfo = new addrinfo();//copies the data into the member variables
	memcpy(servInfo, serverInfo, sizeof(addrinfo));

	SOCKET socketHandle = socket(serverInfo->ai_family, serverInfo->ai_socktype, serverInfo->ai_protocol);//makes a local socket
	if(socketHandle == INVALID_SOCKET){
		printf("Invalid socket returned. Error code %d\n", WSAGetLastError());
	}

	servInfo->ai_addr = new sockaddr();
	memcpy(servInfo->ai_addr, serverInfo->ai_addr, sizeof(sockaddr));

	freeaddrinfo(serverInfo);//frees the now useless addrinfo struct

	socketFileDescriptor = socketHandle;

	//sets the socket in reuse mode, this prevents crashes from requiring a restart to free the socket
	//then sets a timeout value for send(), otherwise it can hang perpetually
	bool value = true;
	DWORD timeLim = 30000;

	setsockopt(socketFileDescriptor, SOL_SOCKET, SO_REUSEADDR, (char *)&value, sizeof(bool));
	setsockopt(socketFileDescriptor, SOL_SOCKET, SO_SNDTIMEO, (char*) &timeLim, sizeof(DWORD));

	status = bind(socketHandle, servInfo->ai_addr, sizeof(sockaddr));//binds the socket to the local port specified above

	if(status != 0){
		printf("Bind fails, error code %d\n", WSAGetLastError());
		return;
	}

	status = listen(socketHandle, 15);//sets the socket to listening mode
    printf("Listening...\n");
	if(status != 0){
		printf("Listen has failed, error code %d\n", WSAGetLastError());
	}
}

int CameraCommunicator::addConnection(std::string hostname){//adds a new connection to the list of connections
	
	struct sockaddr address;
	int size = sizeof(address);
	printf("Accepting...\n");
	SOCKET status = accept(socketFileDescriptor, &address, &size);//accepts the connection
	printf("Accepted.\n");
	if(status == INVALID_SOCKET){
		printf("Accept fails. Error code %d\n", WSAGetLastError());
		return -1;
	}

	connection = new NetworkConnection(status, address, locator);//makes a new netowrk connection instance

	connection->loop();//sets the loop running
	//sets information about the connection in the classes
	connection->setHostId(hostname);

	(*list).push_back(connection);
	connection = NULL;
	numConnections ++;
	return 0;
}

void CameraCommunicator::removeConnection(std::string hostname){//removes a connection from the list of connections

	NetworkConnection* current; 
	bool found = false;

	for(unsigned int i = 0; i<list->size(); i++){//iterates through the list
		if((*list)[i]->getHostId() == hostname){//if this is the connection we want
			found = true;
			current = (*list)[i];
		}
		if(found && (i != list->size() -1)){//if we are past the connection
			(*list)[i] = (*list)[i+1];//moves everthing up one
		}
	}

	if(found){
		current->endLoop();//ends the connections loop
		delete current;
		if(numConnections == 1){
			list->clear();
		}else{
			list->pop_back();//removes the duplicate entry from the end of the list
		}
		numConnections --;
	}else{
		printf("connection not found\n");
	}
}

CameraCommunicator::NetworkConnection::NetworkConnection(SOCKET fileDescriptor, sockaddr addressInfo, Locator* locator){//constructor
	this->locator = locator;
	servInfo = new sockaddr();
	socketFileDescriptor = fileDescriptor;
	looping = false;
	memcpy(servInfo, &addressInfo, sizeof(addressInfo));
	pleaseStop = false; 
}

CameraCommunicator::NetworkConnection::~NetworkConnection(){//destructor
	if(this->isLooping()){
		endLoop();
	}
	while(this->isLooping()){
		Sleep(100);
	}
	delete servInfo;
}

int CameraCommunicator::NetworkConnection::loop(){

	//makes a new thread to handle the loop, the thread runs the communicate routine
	DWORD threadId;
	
	HANDLE threadHandle = CreateThread(NULL, 0,(LPTHREAD_START_ROUTINE) &CameraCommunicator::NetworkConnection::communicate, this, 0, &threadId);

	if(threadHandle == NULL){
		printf("Create thread has failed. Error code %d\n", GetLastError());
		return -1;
	}
	//sets varibles pertaining to the thread
	looping = true;

	this->threadId = threadId;

	this->threadHandle = threadHandle;

	return 0;

}

void CameraCommunicator::NetworkConnection::endLoop(){//tells the thread to end the loop
	if(!isLooping()){
		printf("Useless call to endLoop(), connection is not looping.\n");
		return;
	}

	pleaseStop = true;
}
//fake wrapper function for the thread. Basically, createThread() blows.
 DWORD WINAPI CameraCommunicator::NetworkConnection::communicate(void* instance){
	((CameraCommunicator::NetworkConnection*)instance)->communicate();
	return 0;
}

void CameraCommunicator::NetworkConnection::communicate(){

	timeval time;
	timeval localTime;
	FILETIME systemTime;
	int status;
	bool resend = false;
	timeval oldTime;
	blob data;
	DeleteFile((LPCTSTR)"laglog.txt");
	LARGE_INTEGER bigTimeInt;

	while(!pleaseStop){//as long as endLoop() hasn't been called
		//Sleep(1);//only for testing, otherwise it is too fast
		if(!resend){//if there was no resend request
			time = getData();//recieves the data
			GetSystemTimeAsFileTime(&systemTime);
			bigTimeInt.HighPart = systemTime.dwHighDateTime;
			bigTimeInt.LowPart = systemTime.dwLowDateTime;
			bigTimeInt.QuadPart -= _int64(116444736000000000);
			localTime.tv_sec = (long)(bigTimeInt.QuadPart/10000000);
			localTime.tv_usec = (bigTimeInt.QuadPart%10000000)/10;
			log(time, localTime);
		}

		if(time.tv_sec == -1){//if there was an unknown error, drop this timevalue and give up
			printf("Get data has failed.\n");
			oldTime = time;
			continue;
		}
		if(time.tv_sec == -2){//if there was a resend request
			printf("Server has requested the last data again\n");
			resend = true;
		}
		if(time.tv_sec == 0){//if the connection was closed from the other end
			printf("Server has closed the connection\n");
			return;
		}
		if(!resend){//if there was no resend request
			if(time.tv_sec != -3){
				data = locator->locate();
			}
			status = sendData(time, data);//send the new data
			oldTime = time;
		}else{//otherwise, resend the old data
			status = sendData(oldTime, data);
			resend = false;
		}

		if(status < 0){
			printf("Send data fails\n");
			continue;
		}
	}
	looping = false;
	return;
}

timeval CameraCommunicator::NetworkConnection::getData(){
	timeval fakeTime;
	//recieves the data in a buffer
	char buffer[100];
	int status = recv(socketFileDescriptor, buffer, 100, 0);
	
	if(status < 1){//if there was an error
		fakeTime.tv_sec = status;
		return fakeTime;
	}else{
		buffer[status] = '\0';
		//NOTE: RAND IS HERE FOR TEST PURPOSES ONLY. REMOVE BEFORE USE
		if((strchr(buffer, '\n') == NULL)/*||(rand() % 10 < 2)*/){//if the data does not resemble the data that was sent
			printf("%d bytes recieved in getData(). Retrying\n", status);
			blob fakeblob;
			fakeblob.size = 0;
			fakeTime.tv_sec = -2;
			status = sendData(fakeTime, fakeblob);//sends a retry request
			return getData();//recurses until the data looks right
		}else{//otherwise returns the data as a timeval	
			printf("data recieved: %s\n", buffer);
			timeval returnValue;
			sscanf_s((char*)buffer, "%ld s; %ld us;\n", &returnValue.tv_sec, &returnValue.tv_usec);
			return returnValue;
		}
	}
}

int CameraCommunicator::NetworkConnection::sendData(timeval time, blob data){
	//creates an output string
	char* outData = (char *)malloc(1000);

	if(time.tv_sec == -2){
		outData = "resend\n\0";
	}else if(time.tv_sec == -3){
		outData = "ping\n\0";
	}else{
		sprintf_s(outData, 1000, "X=%d;Y=%d;s=%ld;us=%ld\n", data.centroid.x, data.centroid.y, time.tv_sec, time.tv_usec);
	}
	int dataSize = (int) strlen(outData);
	printf("sending data: %s\n", outData);
	//sends the data to the server
	int bytesSent = send(socketFileDescriptor, outData, dataSize, 0);

	if(bytesSent < 0){//if there was an error
		printf("Send has failed. Error code %d\n", WSAGetLastError());
		return -1;
	}else{
		if(bytesSent != dataSize){//notifies that there was an error on this end. Does not resend, as this would screw up resend protocol
			printf("Not all bytes sent. Expect a retry request\n");
			return -1;
		}
	}
	return 0;
}

void CameraCommunicator::NetworkConnection::log(timeval networkTime, timeval localTime){

	FILE* file;
	fopen_s(&file, "laglog.txt", "a");
	int secondsDiff = localTime.tv_sec - networkTime.tv_sec;
	int usecDiff = localTime.tv_usec - networkTime.tv_usec;
	if(file != NULL){
		fprintf(file, "%d\n",(1000000* secondsDiff) + usecDiff);
		fclose(file);
	}
}