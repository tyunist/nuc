/*
 * File: UdpServer.h
 * Last modified on Thursday 28 August  13:30 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides interface to communicate with the client in order to receive information.
 * Usage:  This server must be opened in advance to clients
 		first, create an UdpClient object:
						UdpServer server(port)
		then, open a port and create a socket
						server.connect();
		now, receive message from client if have:
						server.receive(); 
		finally, if want to close connection:
						server.disConnect();
 */
 
 #ifndef _UdpServer_h_
 #define _UdpServer_h_
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
 
using namespace std;
 
class UdpServer{
		
  private:
		
	struct sockaddr_in myaddr, remaddr;
	int fd, port, sentNum; /// fd: socket
	socklen_t slen;
	int buflen; /// Lenth of buffer
	char buf[2048];	/* message buffer */
	int recvlen;		/* # bytes in acknowledgement message */
			 
  public:
		
/* Initialize a socket to have client to connect. This function use one variable:
 * port : port through which client is connected to the server. 
 */	 
	UdpServer(int &port);
		
			
/* Start to create socket, bind the address */			
	int connect();
	
/* Check whether buffer has package to read*/
	bool isReadable(int usec);

/* Receive message from the client
 * timout is a variable that clarify whether receiver has timeout or not. Default is 0, meaning that 
 * server would wait forever. However, if timeout > 0 -> server just wait for a time = 
 * timeout miliseconds and then, out if there is no data.
 * msgcnt is a variable that shows the number of the package 
*/
	char* receive(int timeout = 0);
		

/* Acknowledge sent package to client */
	void ack(int msgcnt); 

		
/* Close the socket */
	void disConnect(); 
			
};

 
 #endif
