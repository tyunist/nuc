/* keyboardRun.cpp
	Version 4.0 works pretty well forward and backward
	* Just using keyboard to move the car
 */
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Bool.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <sstream>


/****************************************
  for using getch()
  ****************************************/
int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt); // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON); // disable buffering
	tcsetattr( STDIN_FILENO, TCSANOW, &newt); // apply new settings

	int c = getchar(); // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt); // restore old settings
	return c;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop");

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
	// done zero degree initiation

	while(true)
	{
		char c = getch();


		if(c == 'w'){
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
			else {
				vel.data +=1;
				velPub.publish(vel);
				velPub.publish(vel);
				ROS_INFO("%d: %f", count, vel.data);
			}
		}else if(c =='s'){
			if(vel.data == 0){
				idlePub.publish(idle);
				loop_rate.sleep();
				vel.data = -90;
				velPub.publish(vel);
				ros::Duration(0.2).sleep();
				vel.data = -10;
				velPub.publish(vel);
				ROS_INFO("%d: %f", count, vel.data);
			}else {
				vel.data -=1;
				ROS_INFO("%d: %f", count, vel.data);
				velPub.publish(vel);
			
			}
		}else if(c == 'l'){
				vel.data = 0;
				velPub.publish(vel);
				ROS_INFO("STOP by user! Stopping");
				loop_rate.sleep();
				ros::Duration(0.2).sleep();
				vel.data = -1;
				velPub.publish(vel);
				ros::Duration(0.05).sleep();
				vel.data = 0;
				ROS_INFO("%d: %f", count, vel.data);
						
		}else if(c == 'a'){
			if(ang.data+5>-45)
				ang.data -= 4;
			ROS_INFO("%d: %f", count, ang.data);
			strPub.publish(ang);
		}else if(c == 'd'){
			if(ang.data < 45)
				ang.data += 4;
			ROS_INFO("%d: %f", count, ang.data);
			strPub.publish(ang);		
		}else if(c == 'q'){
			break;
		}


		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	vel.data = 0;
	ROS_INFO("%d: %f", count, vel.data);

	velPub.publish(vel);

	return 0;
}
