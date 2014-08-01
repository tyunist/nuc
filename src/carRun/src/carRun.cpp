/* carRun.cpp
   Version 2.0
	last edited 6:30AM_01/08/2014 by tynguyen
	tynguyen@unist.ac.kr

*/
/// udp server include files
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <netdb.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <string>
#include <iostream>
#include <ctime>
#include <unistd.h>
/// ROS include files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sstream>


#define BUFSIZE 2048
#define SERVICE_PORT 21234
using namespace std;

class Timer{
	public:
		ros::Time start, end;
		ros::Time elapsed;

		void start()
		{
			end = ros::Time::now();
		}
		
		double now()
		{
			return ros::Time::now().toSec();
		}

		double elapse()
		{
			ros::Time current = ros::Time::now()
			double elapsedTime = current.toSec() - elapsed.toSec();
			elapsed = current;
			return elapsedTime;
		}

		double stop()
		{
			end = ros::Time::now();
			double duration = end.toSec() - start.toSec();
			return duration;
		}
		
		void printTime(double duration)
		{
			ROS_INFO("Duration: %.3f", duration);
		}
};	

int main(int argc, char **argv)
{
	/* server setup */
	struct timeval tv; /// timeout struct
	tv.tv_sec = 0;
	tv.tv_usec = 10000;
	struct sockaddr_in myaddr;	/// our address
	struct sockaddr_in remaddr; ///remote address 
	socklen_t addrlen = sizeof(remaddr); ///length of addresses
	int recvlen;			/// bytes received 
	int fd;				/// our socket
	int msgcnt = 0;		/// count # of messages we received 
	char buf[BUFSIZE];	/// receive buffer 


	/// create a UDP socket

	if ((fd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
		perror("cannot create socket\n");
		return 0;
	}

	/// bind the socket to any valid IP address and a specific port

	memset((char *)&myaddr, 0, sizeof(myaddr));
	myaddr.sin_family = AF_INET;
	myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
	myaddr.sin_port = htons(SERVICE_PORT);

	if (bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0) {
		perror("bind failed");
		return 0;
	}

		
	/* Setup ROS control*/	
	ros::init(argc, argv, "carRun");

	ros::NodeHandle n;

	ros::Publisher velPub = n.advertise<std_msgs::Float64>("controlA", 64);
	ros::Publisher strPub = n.advertise<std_msgs::Float64>("controlS", 64);
	ros::Publisher idlePub = n.advertise<std_msgs::Float64>("controlI", 64);

	ros::Rate loop_rate(10);

	int count = 0;
	std_msgs::Float64 vel;
	std_msgs::Float64 ang;
	std_msgs::Float64 idle;
	vel.data = 0;
	ang.data = 0;
	idle.data = 1;
	loop_rate.sleep();
	strPub.publish(ang);	
	
	/// Variables to store received data from client
	float x, y, velX, velY, dirAngle, circleTime;
	double sentTime; 
	Timer timer;
	timer.start();

	/* now loop, receiving data of car's velocity and process them */
	for (;;) {
		double currTime = timer.stop();
		ROS_INFO("Cirle %d start!", count + 1);	
		if(count > 0)   /// mean that from the second time of receving, just wait a few miliseconds
			setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, (char *) &tv, sizeof(struct timeval) );
      
		printf("waiting on port %d\n", SERVICE_PORT);
		recvlen = recvfrom(fd, buf, BUFSIZE, 0, (struct sockaddr *)&remaddr, &addrlen);
		if (recvlen > 0) {
			buf[recvlen] = 0;
			printf("Received message: \"%s\" (%d bytes)\n", buf, recvlen);
			
			/// Extract infor from buf
			istringstream isBuf(buf);
         string startS, endS;
			isBuf >> startS >> x >> y >> velX >> velY >> dirAngle >> sentTime >> circleTime >>endS;
			ROS_INFO("Disembled data: x =  %f y = %f velX = %f velY = %f dirAngle = %f sentTime = %.3f circleTime = %.3f", x, y, velX, velY, dirAngle, sentTime, circleTime);
			 
			cout<<"Extracted information: "<<x<<endl;
			cout<<velX<<endl;
			cout<<sentTime<<endl;
			
			if(startS.compare(string("start") ) != 0 || endS.compare(string("end")) != 0 ) /// string is bad
			else 
			{
				/// Control car with new velocity

			}
		//// Test		
				
			cout +=1;
		}
		else
			printf("uh oh - something went wrong!\n");
		
		///Send acknowledge to client
		sprintf(reinterpret_cast<char*>(buf), "ack %d", msgcnt++);
		printf("Server response: \"%s\"\n", buf);

		if (sendto(fd, buf, strlen(reinterpret_cast<const char*>(buf) ), 0, (struct sockaddr *)&remaddr, addrlen) )
		{}	
		/// Delay for a while to see how'll happen

	}
	/* never exits */
	return 0;
}
