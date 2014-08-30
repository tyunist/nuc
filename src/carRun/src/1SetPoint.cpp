/* 1SetPoint.cpp
   Version 1.0
		* Only one setpoint velocity
   	* pid controler
   	* stable distance calculation
	last edited 11:30PM_31/08/2014 by tynguyen
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
   	int timeout = 10000000; /// 10 000 000 miliseconds
	int msgcnt = 1; /// the number of packages	
	/* Setup ROS control*/	
	ros::init(argc, argv, "carRun");

	ros::NodeHandle n;

	ros::Publisher velPub = n.advertise<std_msgs::Float64>("controlA", 64);
	ros::Publisher strPub = n.advertise<std_msgs::Float64>("controlS", 64);
	ros::Publisher idlePub = n.advertise<std_msgs::Float64>("controlI", 64);

	ros::Rate loop_rate(70);

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
	double setPoint;
	double kp = 70;
	double ki = 65;
	double cmdPrev = 0;
	double Tstable = 0;
	if (argc == 3)
	{
		sscanf(argv[1], "%lf", &setPoint);
		sscanf(argv[2], "%lf", &Tstable);
	}
	
	else if (argc == 2)
	{
		sscanf(argv[1], "%lf", &setPoint);
		ROS_INFO("No stable time input, just run with Tstable = 0");
	}

	else 
	{
		ROS_INFO("Two many or less arguments");
		ROS_INFO("Right usage is: carRun setPoint StableTime");
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
	Timer stableTimer; 
	bool calFlag = false;
	int errorCount = 0; /// Count number of error in stable period of setPoint1
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
		else buf = server.receive(0);
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
				cout<<"Gotten string is bad!! It is: "<<startS << endS<<endl;
				errorNum +=1;
				/* Now, based on previous position, decide whether stop car or not*/
				if(velResponses.size() > 0 && velResponses.back()[4] >= 490)
					break;
				else continue;
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
		else /// Already one sample
		{
			currSample = timer.now() - velResponses[0][10] ; /// - receiveTime 1st
			double prevCmd = velResponses.back()[12];
			double prevVel = velResponses.back()[2];
			///velEstimated = velX + prevCmd/5*circleTime2; 
			velEstimated = velX;
		}
		
		ROS_INFO("Gonna go to scheduled points");
		/// Scheduled points
		velSet = setPoint;
		if(currSample == 0)
		{
			Dstable = x;
			ROS_INFO("Start setpoint2 at %.3lf and position: %f", timer.now(),x);
		}

		if(calFlag == false && currSample >= Tstable && Tstable > 0)
		{
			Dstable = x - Dstable; /// Generally, it should be sqrt(x*2 + y*2)
			ROS_INFO("Calculate at %.3lf, position: %f, after stableTime:%.3f  Dstable = %f", timer.now(), x, Tstable, Dstable);
		    fDstable << "-------1 setpoint----------"<<endl;
		    fDstable << 0 << "\t" << setPoint << "\t" << Tstable << "\t" << 	Dstable <<endl;
			calFlag = true;		
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
