// This has been modified for BLAST...
// Ed Chapin    echapin@inaoep.mx  August 10, 2003


#include "csocket.h"

/**
 * @param iPort local listenning port
 * @throws CSocketException if server socket could not be created
 */
CSocket::CSocket( int iPort) {
  Reset( iPort);
  
  //printf("constructor\n");
  
  if( bind( m_Socket, ( SOCKADDR *)&m_sockaddr, sizeof( sockaddr)) != 0)
    throw CSocketException( "bind() failed");
  
  if( listen( m_Socket, 0) != 0)
    throw CSocketException( "accept() failed");
}

/**
 * @param szRemoteAddr Remote Machine Address
 * @param iRemotePort Server Listenning Port
 * @throws CSocketException if client socket could not be created
 */
CSocket::CSocket( char *szRemoteAddr, int iPort) {
  if( !szRemoteAddr)
    throw CSocketException( "Invalid parameters");
  
  Reset( iPort);
  
  // first guess => try to resolve it as IP@
  m_sockaddr.sin_addr.s_addr = inet_addr( szRemoteAddr);
  if( m_sockaddr.sin_addr.s_addr == INADDR_NONE) {  
    // screwed => try to resolve it as name
    LPHOSTENT lpHost = gethostbyname( szRemoteAddr);
    if( !lpHost)
      throw CSocketException( "Unable to solve this address");
    m_sockaddr.sin_addr.s_addr = **(int**)(lpHost->h_addr_list);
  }
  
  // actually performs connection
  if( connect( m_Socket, ( SOCKADDR*)&m_sockaddr, sizeof( sockaddr)) != 0)
    throw CSocketException( "connect() failed");
}

/**
 * Create a socket for data transfer (typically after Accept)
 * @param Socket the socket descriptor for this new object
 */
CSocket::CSocket( SOCKET Socket) {
  m_Socket = Socket;
}

/**
 * Destructor
 */
CSocket::~CSocket() {
  Close();
}

/**
 * Wait for incoming connections on server socket
 * @return CSocket new data socket for this incomming client. Can be NULL if anything went wrong
 */
CSocket * CSocket::Accept() {
  int nlen = sizeof( sockaddr);
  SOCKET Socket = accept( m_Socket, ( SOCKADDR *)&m_sockaddr, &nlen);
  
  if( Socket == -1)
    return( NULL);
  
  return( new CSocket( Socket));
}


/**
 * Close current socket
 */
void CSocket::Close() {
  if( m_Socket != INVALID_SOCKET)
    closesocket( m_Socket);
}

/**
 * Read data available in socket or waits for incomming informations
 * @param pData Buffer where informations will be stored
 * @param iLen Max length of incomming data
 * @return Number of bytes read or -1 if anything went wrong
 */
int CSocket::Read( void * pData, unsigned int iLen) {
  int ret;
  
  
  if( !pData || !iLen)
    return( -1);
  
  ret = recv( m_Socket, ( char *)pData, iLen, 0);
  
  // Debugging purposes - see if anything left in the input buffer
  
  //u_long val;
  //ioctlsocket( m_Socket, FIONREAD, &val );
  //printf("Remaining bytes in input buffer: %li packet size: %i\n",val,iLen);
  
  return ret;
}

/**
 * Initialisation common to all constructors
 */
void CSocket::Reset( unsigned int iPort) {
  // Initialize winsock
  if( WSAStartup( MAKEWORD(2,0), &m_WSAData) != 0)
    throw CSocketException( "WSAStartup() failed");
  
  // Actually create the socket
  m_Socket = socket( PF_INET, SOCK_STREAM, IPPROTO_TCP);
  if( m_Socket == INVALID_SOCKET)
    throw CSocketException( "socket() failed");
  
  // *** ADDED SO THAT WE CAN COMMUNICATE WITH LINUX - Ed ***
  int val=1;
  int n=setsockopt(m_Socket, IPPROTO_TCP, TCP_NODELAY, (char *)&val, 
		   sizeof(val));

  if(n==SOCKET_ERROR) {
    val = WSAGetLastError();
    printf("socket error: %i\n",val);
  }
  
  // sockaddr initialisation
  memset( &m_sockaddr, 0, sizeof( sockaddr));
  
  m_sockaddr.sin_family = AF_INET;
  m_sockaddr.sin_port = htons( iPort);
  m_sockaddr.sin_addr.s_addr = INADDR_ANY;
}

/**
 * @param pData Buffer to be sent
 * @param iSize Number of bytes to be sent from buffer
 * @return the number of sent bytes or -1 if anything went wrong
 */
int CSocket::Write( void * pData, unsigned int iSize) {
  if( !pData || !iSize)
    return( -1);
  
  return( ( int)send( m_Socket, ( LPCSTR)pData, iSize, 0));
}
