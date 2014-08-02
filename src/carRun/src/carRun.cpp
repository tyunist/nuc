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
#include <sstream> /// istringstream
#include "Timer.h"
#include "UdpServer.h"

#include <fstream>
#include <vector>
#define BUFSIZE 2048
#define SERVICE_PORT 21234
#define TIMEOUT = 10000 /// 10 miliseconds
using namespace std;


int main(int argc, char **argv)
{
	/* server setup */
	UdpServer server(SERVICE_PORT);
	server.connect();

		
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
	int errorNum = 0; /// Number of bad packages 
	int countNum = 0;
	Timer timer;
	timer.start();
	
	/// File and vector to store velocity
	ofstream fVelResponses("velResponse.csv");
	if(fVelResponses.fail())
    {
        cout<<"File ERROR!!! Could not open fVelResponses File!"<<endl;
        exit(1);
    }
	/// Write header
	fVelResponses << "X" << "\t" << "Y" << "\t" << "VELX" << "\t" << "VELY"
				  << "dirAngle" << "\t" << "sentTime" << "\t" << "circleTime";
	vector<vector<double> > velResponses;
	

	/* now loop, receiving data of car's velocity and process them */
	while(true) {
		double currSample = timer.stop();
		ROS_INFO("Cirle %d start!", count + 1);	
		/// Buffer to store information getting from the client
		char buf[2048];
	
		if(count > 0)   /// mean that from the second time of receving, just wait a few miliseconds
			buf = server.receive(timeout);
		else buf = server.receive();
		
		/// Extract infor from buf
		istringstream isBuf(buf);
	 	string startS, endS;
		isBuf >> startS >> x >> y >> velX >> velY >> dirAngle >> sentTime >> circleTime >>endS;
		ROS_INFO("Disembled data: x =  %f y = %f velX = %f velY = %f dirAngle = %f sentTime = %.3f circleTime = %.3f", x, y, velX, velY, dirAngle, sentTime, circleTime);
		/// Printout infor to std
		cout<<"Extracted information: "<<x<<endl;
		cout<<velX<<endl;
		cout<<"Sent time: "<<sentTime<<endl;
		double now = timer.now();
		cout<<"NUC time: ";
		timer.printTime(now);
		/// Save infor to vector
		vector<double> velResponse;
		velResponse.push_back(x, y, velX, velY, dirAngle, sentTime, circleTime);
		velResponses.push_back(velResponse);
		/// Save infor to file
		fVelResponses << x << "\t" << y << "\t" 
					  << velX << "\t" << velY <<"\t"
					  << dirAngle << "\t" << sentTime << "\t"
					  << circleTime; 
		
		if(startS.compare(string("start") ) != 0 || endS.compare(string("end")) != 0 ) /// string is bad
			{
				cout<<"Gotten string is bad!! It is: "<<isBuf<<endl;
				errorNum +=1;
			}
		/// Get velocity of previous period of time
		
		
		/// Control car with new velocity
		if(startS.compare(string("stop") ) == 0 )
		{
			vel.data = 0;
			velPub.publish(vel);
			ROS_INFO("%d: %f", count, vel.data);	
			cout<<"STOP by client!!!Stopping..."<<endl;
			break;
		}
		
		else 
		{
			if(vel.data == 0)
			{
				vel.data = 90;
				velPub.publish(vel);
				ros::Duration(0.2).sleep(); // sleep for half a second
				vel.data = 80;
				velPub.publish(vel);
				ros::Duration(0.2).sleep();
				vel.data = 1;
				velPub.publish(vel);
				ROS_INFO("%d: %f", count, vel.data);
			}
			else
			{
				vel.data = 1;
				velPub.publish(vel);
				ROS_INFO("%d: %f", count, vel.data);
			}
			countNum += 1;
		}
		
		
		/// update ros messages		
		ros::spinOnce();
		loop_rate.sleep();
		cout +=1;
	} /// end of while(true)
		
	
	/// Exit program
	vel.data = 0;
	ROS_INFO("%d: %f", count, vel.data);
	velPub.publish(vel);
	
	cout<<"****End of the program****"<<endl;
	cout<<"Successful frames: "<<countNum<<endl;
	cout<<"Error frames (bad gotten package from client): "<<errorNum<<endl;
	cout<<"Error rate ( Error / total): "<<errorNum/(errorNum + countNum)* 100<<"%"<<endl;
	return 0;
}
