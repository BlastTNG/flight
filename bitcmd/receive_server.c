/* On QNX: compile using:
 * 
 * gcc receive_server.c -lsocket -o receive_server
 * 
 */

#include <arpa/inet.h> // socket stuff
#include <netinet/in.h> // socket stuff
#include <stdio.h> // socket stuff
#include <sys/types.h> // socket stuff
#include <sys/stat.h>
#include <sys/socket.h> // socket stuff
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
//#include <sys/mman.h> // address mapping for I/O
//#include <hw/inout.h> // raw PC104 I/O routines
//#include <sys/neutrino.h> // Neutrino kernel/thread control

#define RECVBUFLEN 1024
#define GROUND_PORT 21212

int main()
{
	struct sockaddr_in cli_addr, my_addr;
	int sockfd;
	socklen_t slen;
	uint8_t recv_buf[RECVBUFLEN];
	
	slen = sizeof(cli_addr);

    if ((sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP))==-1) printf("Server : Socket() unsuccessful\n");
    //else
    //  printf("Server : Socket() successful\n");

	bzero(&my_addr, sizeof(my_addr));
	my_addr.sin_family = AF_INET;
	my_addr.sin_port = htons(GROUND_PORT);
	my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

	if (bind(sockfd, (struct sockaddr* ) &my_addr, sizeof(my_addr))==-1) perror("bind");
	
	printf("Waiting for client connection...\n");
	
	while (1)
	{
		if (recvfrom(sockfd, recv_buf, RECVBUFLEN, 0, (struct sockaddr*)&cli_addr, &slen)==-1) printf("Server : RecvFrom() failed\n");
		printf("Message from %s: %s\n",inet_ntoa(cli_addr.sin_addr),recv_buf);
	}
	
	close(sockfd);
	return 0;
}
