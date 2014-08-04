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
#include <iomanip> /// Format cout<<
using namespace std;

/// Global constants
int SERVICE_PORT = 21234;

int main(int argc, char **argv)
{
	/* server setup */
	UdpServer server(SERVICE_PORT);
	server.connect();
   int timeout = 10000; /// 10 miliseconds
		
	/* Setup ROS control*/	
	ros::init(argc, argv, "carRun");

	ros::NodeHandle n;

	ros::Publisher velPub = n.advertise<std_msgs::Float64>("controlA", 64);
	ros::Publisher strPub = n.advertise<std_msgs::Float64>("controlS", 64);
	ros::Publisher idlePub = n.advertise<std_msgs::Float64>("controlI", 64);

	ros::Rate loop_rate(10);

	int circleNum = 0;
	std_msgs::Float64 vel;
	std_msgs::Float64 ang;
	std_msgs::Float64 idle;
	vel.data = 0;
	ang.data = 0;
	idle.data = 1;
	loop_rate.sleep();
	strPub.publish(ang);	

	/* Running condition: setpoints*/
	double velSet;

	if(argc < 2) 
		velSet = 5;
	else if (argc == 2)
	{
		sscanf(argv[1], "%lf", &velSet);
	}
	else 
	{
		ROS_INFO("Two many arguments");
		ROS_INFO("Right usage is: carRun velSet");
		return 0;
	}
	
	/// Variables to store received data from client
	double x, y, velX, velY, dirAngle, sentTime, circleTime;
	int errorNum = 0; /// Number of bad packages 
	int countNum = 0;
	Timer timer;
	timer.start();
	
	/// File and vector to store velocity
	ofstream fVelResponses;("velResponse.csv");
	fVelResponses.setf(ios::fixed); /// Floating point numbers are printed in fixed-point notation
	if(fVelResponses.fail())
    {
        cout<<"File ERROR!!! Could not open fVelResponses File!"<<endl;
        exit(1);
    }
	/// Write header
	fVelResponses << "X" << "\t" << "Y" << "\t" << "VELX" << "\t" << "VELY"
				  << "dirAngle" << "\t" << "sentTime" << "\t" << "circleTime"<< endl;
	vector<vector<double> > velResponses;
	

	/* now loop, receiving data of car's velocity and process them */
	while(true) {
		double currSample = timer.stop();
		ROS_INFO("Cirle %d start!", circleNum + 1);	
		/// Buffer to store information getting from the client
		char* buf = new char[2048];
	
		if(circleNum > 0)   /// mean that from the second time of receving, just wait a few miliseconds
			buf = server.receive(timeout);
		else buf = server.receive();
		double receiveTime = timer.now();

		circleNum += 1;
		/// Extract infor from buf
		istringstream isBuf(buf);
	 	string startS, endS;
		isBuf >> startS >> x >> y >> velX >> velY >> dirAngle >> sentTime >> circleTime >> endS;
		ROS_INFO("Disembled data: x =  %f y = %f velX = %f velY = %f dirAngle = %f sentTime = %.3f circleTime = %.3f", x, y, velX, velY, dirAngle, sentTime, circleTime);
		/// Printout infor to std
		ROS_INFO("Extracted information: X: %f", x);
		ROS_INFO("VELX: %f", velX);
		ROS_INFO("Sent time: %.3f", sentTime);
		ROS_INFO("NUC time when receiving: %.3f",receiveTime);
		
		/// If message is STOP, stop running
		if(startS.compare(string("stop") ) == 0 )
		{
			vel.data = 0;
			velPub.publish(vel);
			ROS_INFO("FORCE: %f",vel.data);	
			ROS_INFO("STOP by client!!!Stopping...");
			break;
		}

		/// If message is wrong, do not save, just continue running
		if(startS.compare(string("start") ) != 0 || endS.compare(string("end")) != 0 ) /// string is bad
		{
				cout<<"Gotten string is bad!! It is: "<<isBuf<<endl;
				errorNum +=1;
		
		/// Get velocity of previous period of time
		
		}

		/// Else, save infor to vector and file
		else
		{
		double arr[] = {x, y, velX, velY, dirAngle, sentTime, circleTime};
		vector<double> velResponse(arr, arr + 7);
		velResponses.push_back(velResponse);
		/// Save infor to file
		fVelResponses << x << "\t" << y << "\t" 
					  << velX << "\t" << velY <<"\t"
					  << dirAngle << "\t" << sentTime << "\t"
					  << circleTime << endl; 
		
		countNum += 1;
		}
		
		
		/// Control car with new velocity
	
		if(vel.data == 0)
		{
			vel.data = 90;
			velPub.publish(vel);
			ros::Duration(0.2).sleep(); // sleep for half a second
			vel.data = 80;
			velPub.publish(vel);
			ros::Duration(0.2).sleep();
			vel.data = velSet;
			velPub.publish(vel);
			ROS_INFO("FORCE: %f", vel.data);
		}
		else
		{
			vel.data = velSet;
			velPub.publish(vel);
			ROS_INFO("FORCE: %f", vel.data);
		}
		
	
		
		
		/// update ros messages		
		ros::spinOnce();
		loop_rate.sleep();
	} /// end of while(true)
		
	
	/// Exit program
	vel.data = 0;
	ROS_INFO("FORCE: %f", vel.data);
	velPub.publish(vel);
	
	cout<<"****End of the program****"<<endl;
	cout<<"Successful frames: "<<countNum<<endl;
	cout<<"Error frames (bad gotten package from client): "<<errorNum<<endl;
	cout<<"Error rate ( Error / total): "<<errorNum/(errorNum + countNum)* 100<<"%"<<endl;
	return 0;
}
