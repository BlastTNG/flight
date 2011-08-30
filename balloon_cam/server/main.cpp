#include "server.h"

int main(){

	srand(time(NULL));
	Server server;

	server.runLoop();

}
