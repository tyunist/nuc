/*
 * File: Timer.cpp
 * Last modified on Saturday, 02 August  07:10 2014 by tynguyen
 *     
 * -----------------------------------------------------
 * This interface provides timer counting with accuracy of milisecond
 * Usage: First, create an timer object
 									Timer timer;
 				then, start counting
 									timer.start();
 * 			    At each elapse, call 
 									double duration = Timer::elapse(); 
 *				At the end, call 
 									double runningTime = Timer::stop();
 * 				Print time duration: 
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
#include "Timer.h"
using namespace std;


void Timer::start()
{
	end = ros::Time::now();
}

double Timer:: now()
{
	return ros::Time::now().toSec();
}

double Timer::elapse()
{
	ros::Time current = ros::Time::now()
	double elapsedTime = current.toSec() - elapsed.toSec();
	elapsed = current;
	return elapsedTime;
}

double Timer::stop()
{
	end = ros::Time::now();
	double duration = end.toSec() - start.toSec();
	return duration;
}

void Timer::printTime(double duration)
{
	ROS_INFO("Duration: %.3f", duration);
}


#endif
