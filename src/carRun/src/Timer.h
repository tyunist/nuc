/*
 * File: Timer.h
 * Last modified on Saturday, 02 August  07:10 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides timer counting with accuracy of milisecond
 * Usage: First, create an timer object
 									Timer timer;
 					then, start counting
 									timer.start();
 * 				At each elapse, call 
 									double duration = Timer::elapse(); 
 *				At the end, call 
 									double runningTime = Timer::stop();
 					Print time duration: 
 									Timer::timer.printTime(duration);
 */
#ifndef _Timer_h_
#define _Timer_h_
 
#include <iostream>
/// ROS include files
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>
#include <sstream>
using namespace std;

class Timer{
	public:
		ros::Time start, end;
		ros::Time elapsed;

		void start();
		
		double now();
	

		double elapse();
	

		double stop();
		
		
		void printTime(double duration);
	
};	


#endif
