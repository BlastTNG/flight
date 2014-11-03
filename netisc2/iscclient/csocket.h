#ifndef __CSOCKET_H
#define __CSOCKET_H

#include "stdio.h"
#include "winsock.h"

class CSocket
{
public:
	CSocket( char *szRemoteAddr, int iPort);
	CSocket( int iPort);
	CSocket( SOCKET Socket);

	~CSocket();

	CSocket * Accept( void);
	void Close( void);
	int Read( void * pData, unsigned int iLen);
	int Write( void * pData, unsigned int iLen);

private:
	SOCKET m_Socket;
	WSADATA m_WSAData;
	SOCKADDR_IN m_sockaddr;

	void Reset( unsigned int iPort);
};

class CSocketException
{
public:
	CSocketException( char * szText)
	{
		strcpy( m_szText, szText);
	}

	~CSocketException(){};

	char * getText(){ return( m_szText);}

private:
	char m_szText[ 128];
};

#endif //__CSOCKET_H