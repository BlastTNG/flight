#include <iostream>
#include <unistd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include "camcommserver.h"

#define CCSERV_DEBUG 0

CamCommServer::CamCommServer(short int port /* = DEFAULT_CAM_PORT */)
{
  serverFD = -1;
  first = last = NULL;
  count = 0;
  listenPort = port;
}

CamCommServer::~CamCommServer()
{
  while (last != NULL) remove(last);
  closeConnection();
}



//let connection run its read loop in a thread
//this takes a bit of a hack to allow calling the C++ member
struct servconn {
  CamCommServer* server;
  CamCommServer::camConnection* connection;
};
static void* runConnectionWrap(void* arg)
{
  servconn* sc = (servconn*)arg;
  sc->server->runConnection(sc->connection);
  return NULL;
}
void CamCommServer::runConnection(camConnection* conn)
{
  add(conn);
  cout << "New connection recieved from: " << conn->comm->target << endl;
  conn->comm->readLoop(conn->interpreter);

  cout << "Connection terminated from: " << conn->comm->target << endl;
  remove(conn);
}

//perpetually listen for new conenctions
//should be created in its own thread
int CamCommServer::startServer(string (*interpretFunction)(string))
{
  sockaddr_in servaddr;
  servaddr.sin_family = AF_INET;
  servaddr.sin_port = htons(listenPort);
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);

  serverFD = socket(PF_INET, SOCK_STREAM, 0);
  if (serverFD < 0) {
    return -1;
  }
  //enable address reuse (to restart without having to wait forever)
  int optval = 1;
  setsockopt(serverFD, SOL_SOCKET, SO_REUSEADDR, &optval, sizeof(optval));
  if (bind(serverFD, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
    closeConnection();
    return -2;
  }
  if (listen(serverFD, 2) < 0) {
    closeConnection();
    return -3;
  }

  while (1) {
    sockaddr_in cliaddr;
    socklen_t clilen = sizeof(cliaddr);
    int clifd = accept(serverFD, (sockaddr*)&cliaddr, &clilen);
    if (clifd < 0) {
#if CCSERV_DEBUG
      cerr << "[CCServer Debug]: failed to accept connection." << endl;
#endif
      continue;
    }
    //otherwise, got a good fd and have a new connection
    camConnection* conn = new camConnection();
    conn->comm = new CamCommunicator();
    conn->interpreter = interpretFunction;
    conn->comm->serverFD = -2;
    conn->comm->commFD = clifd;
    conn->comm->target = inet_ntoa(cliaddr.sin_addr);
    //hack to allow calling member in pthread
    servconn sc;
    sc.server = this;
    sc.connection = conn;
    pthread_create(&conn->thread, NULL, runConnectionWrap, (void*)&sc);
  }

  return 0;
}



//send a message over all connected sockets
int CamCommServer::sendAll(string msg)
{
  camConnection* conn = first;
  while (conn != NULL) {
    conn->comm->sendCommand(msg);
    conn = conn->next;
  }
  return 0;
}



void CamCommServer::closeConnection()
{
  if (serverFD >= 0) {
    close(serverFD);
    serverFD = -1;
  }
}



//add new connection to end of the list
void CamCommServer::add(camConnection* addee)
{
  addee->prev = last;
  addee->next = NULL;
  if (last != NULL) last->next = addee;
  else first = addee;     //last == NULL means first == NULL too
  last = addee;
  count++;
}

//remove connection from anywhere in the list
void CamCommServer::remove(camConnection* removee)
{
  if (removee->next == NULL)
    last = removee->prev;
  else removee->next->prev = removee->prev;
  if (removee->prev == NULL)
    first = removee->next;
  else removee->prev->next = removee->next;
  delete removee->comm;  //delete the CamCommunicator first
  delete removee;
  count--;
}

