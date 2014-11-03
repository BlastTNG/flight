//cameraCommunicator.h

#pragma once

#include "stdafx.h"
#include "locator.h"

#define DEFAULT_PORT "31337"

class CameraCommunicator {

public:

	CameraCommunicator(){};

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

		NetworkConnection(){};
		
		NetworkConnection(SOCKET, sockaddr, Locator *);

		~NetworkConnection();
		
		int loop();

		void endLoop();

		void setHostId(std::string data){hostId = data;};

		std::string getHostId(){return hostId;};

		bool isLooping(){return looping;};
		
		void communicate();

	private:

		timeval getData();

		int sendData(timeval, blob);

		SOCKET socketFileDescriptor;

		sockaddr* servInfo;

		std::string hostId;

		bool looping;

		HANDLE threadHandle;

		DWORD threadId;
	
		static DWORD WINAPI communicate(void *);

		Locator* locator;

		bool pleaseStop;

		void log(timeval, timeval);

	};

	NetworkConnection* connection;

	std::vector<NetworkConnection*>* list;

	SOCKET socketFileDescriptor;

	addrinfo* servInfo;

};