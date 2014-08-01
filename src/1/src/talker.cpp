#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>

#include <sstream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");

	ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::Float64>("control", 64);

  ros::Rate loop_rate(10);

  int count = 0;
	std_msgs::Float64 vel;
	vel.data = -5;
	loop_rate.sleep();
	for(int i = 0; i < 20; i++)
  {
    ROS_INFO("%d: %f", count, vel.data);

    chatter_pub.publish(vel);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }
	vel.data = 0;
    ROS_INFO("%d: %f", count, vel.data);

    chatter_pub.publish(vel);


  return 0;
}
