#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <sstream>

int getch()
{
	static struct termios oldt, newt;
	tcgetattr( STDIN_FILENO, &oldt);           // save old settings
	newt = oldt;
	newt.c_lflag &= ~(ICANON);                 // disable buffering      
	tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings

	int c = getchar();  // read character (non-blocking)

	tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings
	return c;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "teleop");

	ros::NodeHandle n;

	ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("control", 64);
	ros::Publisher chatter_pub2 = n.advertise<std_msgs::Float64>("control2", 64);

	ros::Rate loop_rate(10);

	int count = 0;
	std_msgs::Float64 vel;
	std_msgs::Float64 ang;
	vel.data = 0;
	ang.data  = 0;
	loop_rate.sleep();
		chatter_pub2.publish(ang);

	while(true)
	{
		char c = getch();

		if(c == 'w'){
			if(vel.data < 0){
				vel.data = 4;
				chatter_pub.publish(vel);
				loop_rate.sleep();
				vel.data = 0;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}else if(vel.data < 4){
				vel.data = 4;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}else{
				vel.data+=0.1;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}
		}else if(c =='s'){
			if(vel.data > 0){
				vel.data = -8;
				chatter_pub.publish(vel);
				loop_rate.sleep();
				vel.data=0;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}else if(vel.data > -4){
				vel.data = -4;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}else{
				vel.data-=0.1;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}
		}else if(c == 'a'){
			if(ang.data+5>-45)
				ang.data -= 4;
		ROS_INFO("%d: %f", count, ang.data);
		chatter_pub2.publish(ang);
		}else if(c == 'd'){
			if(ang.data < 45)
				ang.data += 4;
		ROS_INFO("%d: %f", count, ang.data);
		chatter_pub2.publish(ang);
		}else if(c == ' '){
			vel.data=0;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
		}else if(c == 'q'){
			break;
		}else{
			if(vel.data>0){
				vel.data-=0.1;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}else if(vel.data<0){
				vel.data+=0.1;
		ROS_INFO("%d: %f", count, vel.data);
		chatter_pub.publish(vel);
			}
		}



		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	vel.data = 0;
	ROS_INFO("%d: %f", count, vel.data);

	chatter_pub.publish(vel);

	return 0;
}
