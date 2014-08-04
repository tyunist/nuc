/*
 * File: UdpServer.cpp
 * Last modified on Monday, 04 August  12:10 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides interface to communicate with the client in order to receive information.
 * Usage:  This server must be opened in advance to clients
 		first, create an UdpClient object:
						UdpServer server(port)
		then, open a port and create a socket
						server.connect();
		now, receive message from client if have:
						server.receive(int timeout); 
		finally, if want to close connection:
						server.disConnect();
 */
 
 /// Udp header files
#include <stdio.h>
#include <stdlib.h> /// exit()
#include <string.h>
#include <unistd.h> /// close()
#include <sys/socket.h>
#include <string>
#include <ctime>
#include <arpa/inet.h> /// inet_aton()
#include <sys/time.h> /// gettimeofday()
#include "UdpServer.h"

using namespace std;


/* Initialize a socket to have client to connect. This function use one variable:
 * port : port through which client is connected to the server. 
 */	 
UdpServer::	UdpServer(int & port)
{
	this->port = port;
	this->slen = sizeof(remaddr);
	this->buflen = 2048;
}
		
/* Start to create socket, bind the address */	

int UdpServer:: connect()
{
	/// create a UDP socket

	if ((this->fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
	}

	/* bind the socket to any valid IP address and a specific port */
	/// First, enable sharing port option with another program
	
	
	const int trueValue = 1;
	setsockopt(this->fd, SOL_SOCKET, SO_REUSEADDR, &trueValue, sizeof(trueValue) );
	
	memset((char *)&this->myaddr, 0, sizeof(this->myaddr));
	this->myaddr.sin_family = AF_INET;
	this->myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	this->myaddr.sin_port = htons(this->port);

	if (bind(this->fd, (struct sockaddr *)&this->myaddr, sizeof(struct sockaddr_in)) < 0) {
		perror("bind failed");
		return 0;
	}

			
			return 1;
 		}
 		
 		
/* Receive message from the client
 * timout is a variable that clarify whether receiver has timeout or not. Default is 0, meaning that 
 * server would wait forever. However, if timeout > 0 -> server just wait for a time = 
 * timeout miliseconds and then, out if there is no data. 
*/
char* UdpServer:: receive(int timeout)
{
	struct timeval tv; /// timeout struct
	tv.tv_sec = 0;
	tv.tv_usec = timeout;
    if(timeout > 0)   /// mean that from the second time of receving, just wait a few miliseconds
		setsockopt(this->fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval) );	
 	printf("waiting on port %d\n", this->port);
	this->recvlen = recvfrom(this->fd, this->buf, this->buflen, 0, 
							 (struct sockaddr *)&this->remaddr, &this->slen);
	if (this->recvlen > 0) {
		this->buf[this->recvlen] = 0;
		printf("Received message: \"%s\" (%d bytes)\n", this->buf, this->recvlen);
	}
	else 
	{
		printf("NO data");
		sprintf(this->buf, string("ERROR").c_str());
	}
	return this->buf;				
}
 		
 		
/* Close the socket */
 		void UdpServer:: disConnect()
 		{
 			close(this->fd);
 		}
