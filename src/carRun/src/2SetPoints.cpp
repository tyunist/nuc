/* 2SetPoints.cpp
   Version 1.0
   	* 2 setpont velocities
		* pid controler
   	* Calculate stable distance from setPoin1 to setPoint2 
	last edited 13:49PM_21/08/2014 by tynguyen
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
double stdErr = 0.25; /// standard velocity error to be considered a velocity
double stdTime = 0.8; /// standard time to be considered stable 

int main(int argc, char **argv)
{
	double addVel = 0.05;
	/* server setup */
	UdpServer server(SERVICE_PORT);
	server.connect();
   int timeout = 1000000; /// 1 second
		
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
	double velSet = 0;
	double setPoint1, setPoint2;
	double kp = 70;
	double ki = 65;
	double cmdPrev = 0;
	double Tstable2 = 0;
	double Tstable1 = 1.5; /// When to change from setpoin1 -> setpoint2
	if (argc == 5)
	{
		sscanf(argv[1], "%lf", &setPoint1);
		sscanf(argv[2], "%lf", &setPoint2);
		sscanf(argv[3], "%lf", &Tstable1);
		sscanf(argv[4], "%lf", &Tstable2);
	}

	else if(argc == 3)
	{
		sscanf(argv[1], "%lf", &setPoint1);
		sscanf(argv[2], "%lf", &setPoint2);
		ROS_INFO("No stable time input, no change time input -> no stable distance computation, start change setpoint at %lf!", Tstable1);
	}
	
	else if(argc == 4)
	{
		sscanf(argv[1], "%lf", &setPoint1);
		sscanf(argv[2], "%lf", &setPoint2);
		sscanf(argv[3], "%lf", &Tstable1);
		ROS_INFO("No stable time input -> no stable distance computation!");
	}

	else 
	{
		ROS_INFO("Two many or less arguments");
		ROS_INFO("Right usage is: carRun setPoint1 setPoint2 stableTime1 StableTime2");
		return 0;
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

	/// Stable distance 
	double Dstable = 0;
	bool calFlag1 = false, calFlag2 = false;
	ofstream fDstable;
	fDstable.open("Dstable.csv", ios::app);
	if(fDstable.fail())
	{
		cout<<"File ERROR!!! Could not open Dstable.csv File!"<<endl;
        exit(1);
    }
	

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

		ROS_INFO("Message implies a command to run");	
		/* Now, with each received package, we need to estimate the current
		velocity based on sent velocity */
		double velEstimated;
		double currSample;
		
		/// If message is wrong, do not save, just continue running
		if(startS.compare(string("start") ) != 0 || endS.compare(string("end")) != 0 ) /// string is bad
		{
				cout<<"Gotten string is bad!! It is: "<<isBuf<<endl;
				errorNum +=1;
				continue;
		}

		
		/* Compansate vision-based velocity calculation time delay */
		if(velResponses.size() >= 2) 
		{
			currSample = timer.now() - velResponses[0][10] ; /// - receiveTime 1st
			cout<<"Paramter to estimate vel:"<<velX<<" "<<currSample<<" "<<circleTime<<" "<<velResponses.back()[0]<<" "<<velResponses.back()[3]<<endl;
			
			double prevVelEstimated = velResponses.back()[3];
			double prevCmd = velResponses.back()[12];
			double prevVel = velResponses.back()[2];
			double sePrevCmd = velResponses[velResponses.size() - 2][12];
			if(sePrevCmd == 0)
				velEstimated = velX;
			else
				velEstimated = velX + circleTime2*(prevVelEstimated - prevVel)*prevCmd/sePrevCmd;
			/// In general, it should be vel rather than velX
			
		   /// Average velocity
			velEstimated = (velEstimated + prevVelEstimated)/ 2.0;
		}	
		else if(velResponses.size() == 0)
		{
			currSample = 0;
			velEstimated = 0; }
		else
		{
			currSample = timer.now() - velResponses[0][10] ; /// - receiveTime 1st
			double prevCmd = velResponses.back()[12];
			double prevVel = velResponses.back()[2];
			velEstimated = velX; 
		}
		
		ROS_INFO("Gonna go to scheduled points");
		/// Scheduled points
		if(currSample <= Tstable1)
			velSet = setPoint1;
		if(calFlag1 == false && currSample >= Tstable1) 
			{
				Dstable = x;
				calFlag1 = true;
				ROS_INFO("Start setPoint2 at %.3lf, pos: %f", timer.now(), x);
				velSet = setPoint2;
			}
		/// Calculate stable distance
		if(Tstable2 > 0 && currSample >= Tstable1 + Tstable2 && calFlag2 == false)
		{
			Dstable = x - Dstable;
			calFlag2 = true;
			ROS_INFO("Calculate stable distance at %.3lf, pos: %f, Tstable: %.3lf, Distance: %f", timer.now(), x, Tstable2, Dstable);
			fDstable << "-------2 setPoints--------" << endl;
			fDstable << setPoint1 << "\t" << setPoint2 << "\t" << Tstable2 << "\t" << Dstable << endl;;
		}

		
		/* Control car with new velocity */
		/// Get command
		double velDelta = velEstimated - velSet - 0.05;
		
		/// Adjust PID parameters after cmd <= 0
		if( currSample > 1.0)
		{};
			///adjustPID(pid, akp, aki);
		double cmdRaw = pid.Update(velDelta, circleTime2);
	
		/// Limit cmd (-100 < cmd < 100)
		double cmd = cmdRaw > 100? 100:(cmdRaw < -100? -100:cmdRaw);
		
		/// Compensate for delay
		if(velDelta > 0 && cmd < 0)
			cmd = cmd * 0.7;

		
		ROS_INFO("CONTROL AT: %.3f", timer.now() );	
		vel.data = cmd;
		velPub.publish(vel);
		ROS_INFO("FORCE: %f", vel.data);
		
		/// update ros messages		
		ros::spinOnce();
		loop_rate.sleep();
		ROS_INFO("ENd control at: %.3f", timer.now() );
		
	    /// Save infor to vector
		double arr[] = {currSample, velSet, velX, velEstimated, x, y, velX, velY,  dirAngle, sentTime, receiveTime, circleTime2, cmd};
		vector<double> velResponse(arr, arr + 13);
		velResponses.push_back(velResponse);
		/// Save infor to file
		fVelResponses << currSample << "\t"<< velSet << "\t"
			        << velX << "\t" << velEstimated<<"\t"
					  << x << "\t"<< y << "\t" 
					  << velX << "\t" << velY <<"\t"
					  << dirAngle << "\t" << sentTime << "\t"   
					  << receiveTime << "\t" << circleTime << "\t"
					  /// Add some more for testing
					  << circleTime2 <<"\t" << velDelta <<"\t"
					  << cmd
					  << endl; 
		
		countNum += 1;
	} /// end of while(true)
		
	
	/// Exit program
	vel.data = 0;
	ROS_INFO("FORCE: %f", vel.data);
	velPub.publish(vel);
	loop_rate.sleep();
	if(velX  >= 1.0)
	{
		vel.data = -20;
		ROS_INFO("FORCE: %f", vel.data);
		velPub.publish(vel);
		loop_rate.sleep();
		loop_rate.sleep();
		vel.data = 0;
		ROS_INFO("FORCE: %f", vel.data);
		velPub.publish(vel);
	}
	cout<<"****End of the program****"<<endl;
	cout<<"Successful frames: "<<countNum<<endl;
	cout<<"Error frames (bad gotten package from client): "<<errorNum<<endl;
	cout<<"Error rate ( Error / total): "<<errorNum/(errorNum + countNum)* 100<<"%"<<endl;
	ROS_INFO("END time: %.3f", timer.now() );
	ROS_INFO("Running time: %.3f", timer.stop() );
	return 0;
}
