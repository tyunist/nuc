/*
 * File: UdpServer.cpp
 * Last modified on 31 August  10:30PM 2014 by tynguyen
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

/* Check whether buffer has data to read*/
bool UdpServer:: isReadable(int usec)
{
	fd_set socketReadSet;
	FD_ZERO(&socketReadSet);
	FD_SET(this->fd, &socketReadSet);

	// Setup timeout
	struct timeval tv;
	if( usec ) {
	tv.tv_sec   = usec / 1000000;
	tv.tv_usec = (usec % 1000000);
	} else {
	tv.tv_sec  = 0;
	tv.tv_usec = 0;
	} // end if



	// Select on our Socket Description
	if( select(this->fd+1,&socketReadSet,0,0,&tv) == -1 ) {
	perror("select");
	exit(1);
	}

	return FD_ISSET(this->fd,&socketReadSet) != 0;

} // end isReadable
 		
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
	this->recvlen = recvfrom(this->fd, this->buf, this->buflen, 0, (struct sockaddr *)&this->remaddr, &this->slen);
	if (this->recvlen > 0) {
		this->buf[this->recvlen] = 0;
		printf("Received message: \"%s\" (%d bytes)\n", this->buf, this->recvlen);
	}
	else 
	{
		perror("UDP Server - recvfrom() error");
		printf("NO data. Going to stop by server!!!");
		sprintf(this->buf, string("stop").c_str());

	}
	
	/* Read till the latest package */
	int k = 1;
	while(isReadable(2000)) 
	{
		printf("Can read continuously \n");
		this->recvlen = recvfrom(this->fd, this->buf, this->buflen, 0, (struct sockaddr *)&this->remaddr, &this->slen);
		if(this->recvlen > 0)
			printf("Add message %d is: %s \n", k++, this->buf);
		else printf("No add message! \n");
	}
		
	printf("No thing left to read \n");
	return this->buf;				
}
 	

/* Acknowledge package receiving to client*/
void UdpServer:: ack(int msgcnt)
{
	///Send acknowledge to client
	sprintf(reinterpret_cast<char*>(this->buf), "ack %d", msgcnt);
	printf("Sending response to client: \"%s\"\n", this->buf);
	if (sendto(this->fd, this->buf, strlen(reinterpret_cast<const char*>(this->buf) ), 0, (struct sockaddr *)&this->remaddr, this->slen) );

}



/* Close the socket */
void UdpServer:: disConnect()
{
	close(this->fd);
}
