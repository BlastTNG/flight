#ifndef CAMCOMMSERVER_H
#define CAMCOMMSERVER_H

#include <string>
#include <pthread.h>
#include "camstruct.h"
#include "camcommunicator.h"

/**
 * @author Steve Benton
 * Class which handles multiple connections on server side of CamCommunicator
 * Deprecates the openHost logic in CamCommunicator.
 */

class CamCommServer {
public:
  //struct that contains all information for a single connection
  struct camConnection {
    CamCommunicator* comm;
    camConnection *prev, *next;
    pthread_t thread;
    string (*interpreter)(string);
  };

  CamCommServer(short int port = DEFAULT_CAM_PORT);
  ~CamCommServer();
  void closeConnection();
  
  int startServer(string (*interpretFunction)(string));
  int sendAll(string msg);

  //accessors
  void setPort(short int newport) { listenPort = newport; }

  //run the connection in a thread
  void runConnection(camConnection* conn);

private:
  int serverFD;                //file descriptor for listening socket
  int listenPort;              //port to listen on
  camConnection *first, *last; //doubly-linked list of connections
  int count;                   //number of connections

  //maintaining the list
  void add(camConnection* addee);
  void remove(camConnection* removee);

};

#endif
