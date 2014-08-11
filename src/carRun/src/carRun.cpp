/* carRun.cpp
   Version 2.1
   	* Add pid controler
	last edited 10:00AM_09/08/2014 by tynguyen
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
#include "PID.h" 

#include <fstream>
#include <vector>
#include <iomanip> /// Format cout<<
using namespace std;

void adjustPID(PID &pid, double &kp, double &ki)
{
	pid.SetPGain(kp);
	pid.SetIGain(ki);
}


/// Global constants
int SERVICE_PORT = 21234;

int main(int argc, char **argv)
{
	/* server setup */
	UdpServer server(SERVICE_PORT);
	server.connect();
   int timeout = 4000000; /// 10 miliseconds
		
	/* Setup ROS control*/	
	ros::init(argc, argv, "carRun");

	ros::NodeHandle n;

	ros::Publisher velPub = n.advertise<std_msgs::Float64>("controlA", 64);
	ros::Publisher strPub = n.advertise<std_msgs::Float64>("controlS", 64);
	ros::Publisher idlePub = n.advertise<std_msgs::Float64>("controlI", 64);

	ros::Rate loop_rate(100);

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
	double velSet = 1;
	double kp = 22;
	double ki = 3;
	double akp = 2;
	double aki = 0.5; 
	
	if (argc == 4)
	{
		sscanf(argv[1], "%lf", &velSet);
		sscanf(argv[2], "%lf", &akp);
		sscanf(argv[3], "%lf", &aki);
	}
	else if(argc > 4)
	{
		ROS_INFO("Two many arguments");
		ROS_INFO("Right usage is: carRun velSet akp aki");
		return 0;
	}
	else if(argc == 3)
	{
		ROS_INFO("Less than 4 arguments.");
		sscanf(argv[1], "%lf", &velSet);
		sscanf(argv[2], "%lf", &akp);
		ROS_INFO("velSet is %f, akp is %f, aki is default = %f", velSet, akp, aki);
	}
	else if(argc == 2)
	{
		ROS_INFO("Less than 4 arguments.");
		sscanf(argv[1], "%lf", &velSet);
		ROS_INFO("velSet is %f, akp is default = %f, aki is default = %f", velSet, akp, aki);
	} 
	else 
	{
		ROS_INFO("Less than 4 arguments.");
		ROS_INFO("velSet is default = %f, akp is default = %f, aki is default = %f", velSet, akp, aki);
	}

	/* Create a PID controler */
	PID pid(kp, ki, 0);
	
	/// Variables to store received data from client
	double x, y, velX, velY, dirAngle, sentTime, circleTime;
	int errorNum = 0; /// Number of bad packages 
	int countNum = 0;
	Timer timer;
	timer.start();
	
	/// File and vector to store velocity
	ofstream fVelResponses("velResponse.csv");
	fVelResponses.setf(ios::fixed) ; /// Floating point numbers are printed in fixed-point notation
	if(fVelResponses.fail())
    {
        cout<<"File ERROR!!! Could not open fVelResponses File!"<<endl;
        exit(1);
    }
	/// Write header
	fVelResponses << "Sample" << "\t" << "velSet" << "\t" 
					  << "vel" << "\t" << "VelEstimated" << "\t"		  
				     << "X" << "\t" << "Y" << "\t" 
					  << "VELX" << "\t" << "VELY" << "\t"
				     << "dirAngle" << "\t" << "sentTime" << "\t" 
					  << "ReceiveTime" << "\t"<< "circleTime"<< endl;
	vector<vector<double> > velResponses;
	

	/* now loop, receiving data of car's velocity and process them */
	while(true) {
		ROS_INFO("Cirle %d start!", circleNum + 1);	
		/// Buffer to store information getting from the client
		char* buf = new char[2048];
		ROS_INFO("Start new frame at %.3f", timer.now() );	
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
    	ROS_INFO("Circle time: %.3f", circleTime);
		/// NUC circle time: (difference between two loops)
		double circleTime2 = timer.elapse();
		ROS_INFO("NUC circle time: %.3f", circleTime2);
		/// If message is STOP, stop running
		if(startS.compare(string("stop") ) == 0 )
		{
			vel.data = 0;
			velPub.publish(vel);
			ROS_INFO("FORCE: %f",vel.data);	
			ROS_INFO("STOP by client!!!Stopping...");
			break;
		}

	
		/* Now, with each received package, we need to estimate the current
		velocity based on sent velocity */
		double velEstimated;
		
		/// If message is wrong, do not save, just continue running
		if(startS.compare(string("start") ) != 0 || endS.compare(string("end")) != 0 ) /// string is bad
		{
				cout<<"Gotten string is bad!! It is: "<<isBuf<<endl;
				errorNum +=1;
		
				/// Get velocity of previous period of time
				if(velResponses.size() ) 
					velEstimated = velResponses.back()[3];
				else velEstimated = 0;
		}

		
		else
		{
			/* Compansate vision-based velocity calculation time delay */
			double currSample;
			double velEstimated;
			if(velResponses.size() >= 1) 
			{
				currSample = timer.now() - velResponses[0][10] ; /// - receiveTime 1st
				cout<<"Paramter to estimate vel:"<<velX<<" "<<currSample<<" "<<circleTime<<" "<<velResponses.back()[0]<<" "<<velResponses.back()[3]<<endl;
				velEstimated = velX + (currSample - circleTime - velResponses.back()[0])* (velX - velResponses.back()[3] ) / circleTime;
			/// In general, it should be vel rather than velX

			}	
			else
			{
				currSample = 0;
				velEstimated = 0; 			}
			
			/// Scheduled points
			if(currSample > 1.5)
				velSet = 0.5;
			if(currSample > 2)
				velSet = 1.5;
			if(currSample > 2.5)
				velSet = 0.5;

			/// Save infor to vector
			double arr[] = {currSample, velSet, velX, velEstimated, x, y, velX, velY,  dirAngle, sentTime, receiveTime, circleTime};
			vector<double> velResponse(arr, arr + 12);
			velResponses.push_back(velResponse);
			/// Save infor to file
			fVelResponses << currSample << "\t"<< velSet << "\t"
				        << velX << "\t" << velEstimated<<"\t"
						  << x << "\t"<< y << "\t" 
						  << velX << "\t" << velY <<"\t"
						  << dirAngle << "\t" << sentTime << "\t"   
						  << receiveTime << "\t" << circleTime << "\t"
						  << endl; 
			
			countNum += 1;
		}
		
		
		/* Control car with new velocity */
		/// Get command
		double velDelta = velEstimated - velSet;
		/// Adjust PID parameters after car's starting period
		if( (velEstimated >= velSet + 0.1) && currSample >= 1 )
			adjustPID(pid, akp, aki);
		double cmd = pid.Update(velDelta, circleTime);
		ROS_INFO("CONTROL AT: %.3f", timer.now() );	
		/// Imploy command to arduino
		///if(vel.data == 0)
		///{
		///	vel.data = 90;
		///	velPub.publish(vel);
		///	ros::Duration(0.2).sleep(); // sleep for half a second
		///	vel.data = 80;
		///	velPub.publish(vel);
		///	ros::Duration(0.2).sleep();
		///	vel.data = cmd;
		///	velPub.publish(vel);
		///	ROS_INFO("FORCE: %f", vel.data);
		///}
		///else
		///{
			vel.data = cmd;
			velPub.publish(vel);
			ROS_INFO("FORCE: %f", vel.data);
		///}
		
	
		
		
		/// update ros messages		
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("ENd control at: %.3f", timer.now() );
	} /// end of while(true)
		
	
	/// Exit program
	vel.data = 0;
	ROS_INFO("FORCE: %f", vel.data);
	velPub.publish(vel);
	
	cout<<"****End of the program****"<<endl;
	cout<<"Successful frames: "<<countNum<<endl;
	cout<<"Error frames (bad gotten package from client): "<<errorNum<<endl;
	cout<<"Error rate ( Error / total): "<<errorNum/(errorNum + countNum)* 100<<"%"<<endl;
	ROS_INFO("END time: %.3f", timer.now() );
	ROS_INFO("Running time: %.3f", timer.stop() );
	return 0;
}
