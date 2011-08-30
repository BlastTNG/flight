//server.h

#pragma once

#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <unistd.h>
#include <sys/time.h>

#define DEFAULT_PORT "31337"

class Server {

public:

	Server();

	~Server();

	void runLoop();

private:

	void sendData(timeval);

	char* recieveData();

	void init();

	void passData(int, int, timeval);

	addrinfo* servInfo;

	int socketFileDescriptor;

	timeval lagTime;

	int lag;

	void updateLag();
};
