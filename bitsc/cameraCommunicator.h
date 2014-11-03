//cameraCommunicator.h

#pragma once

#include "stdafx.h"
#include "locator.h"

#define DEFAULT_PORT "31337"
#define INVALID_SOCKET -1

class CameraCommunicator {

public:

	//CameraCommunicator(){};

	CameraCommunicator(Locator*);

	~CameraCommunicator();

	int addConnection(std::string);

	void removeConnection(std::string);

private:

	void init(Locator*);

	int numConnections;

	Locator* locator;

	class NetworkConnection{

	public:

		NetworkConnection(void*){};
		
		NetworkConnection(int, sockaddr, Locator *);

		~NetworkConnection();
		
		int loop();

		void endLoop();

		void setHostId(std::string data){hostId = data;};

		std::string getHostId(){return hostId;};

		bool isLooping(){return looping;};
		
		void* communicate();

		unsigned int getFileDescriptor(){return socketFileDescriptor;};

	private:

		timeval getData();

		int sendData(timeval, blob);

		unsigned int socketFileDescriptor;

		sockaddr* servInfo;

		std::string hostId;

		bool looping;

		HANDLE threadHandle;

		DWORD threadId;

		static void* communicateWrapper(void*);
	
		Locator* locator;

		bool pleaseStop;

		void log(timeval, timeval);

	};

	NetworkConnection* connection;

	std::vector<NetworkConnection*>* list;

	unsigned int socketFileDescriptor;

	addrinfo* servInfo;

};
