#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include "iiwa_msgs/JointPosition.h"
#include "TooN/TooN.h"
#define ngiun 7

TooN::Vector<ngiun> q;
int joint_init=1;

void Callback(const std_msgs::Float64MultiArray& msg)
{
	
   for (int i=0;i<ngiun;i++)
	{
	   q[i]=msg.data[i];
	}
	joint_init=0;
}

int main(int argc, char **argv)
{
  double T_s=0.002;    
  ros::init(argc, argv, "publisher_robot");
  ros::NodeHandle n;
  ros::Rate loop_rate(1/T_s);

  ros::Subscriber joint_sub = n.subscribe("/joints_position", 1, Callback);
  ros::Publisher pub_joint_command = n.advertise<iiwa_msgs::JointPosition>("/iiwa/command/JointPosition", 1);

   while(joint_init)
   {
    ros::spinOnce();
   }
   while (ros::ok())
   { 
	 iiwa_msgs::JointPosition robotPosition;
         
         robotPosition.position.a1 = q[0];
	 		robotPosition.position.a2 = q[1];
         robotPosition.position.a3 = q[2];
         robotPosition.position.a4 = q[3];
         robotPosition.position.a5 = q[4]; 
         robotPosition.position.a6 = q[5];
         robotPosition.position.a7 = q[6];

   
    pub_joint_command.publish(robotPosition);

    ros::spinOnce();
    loop_rate.sleep();
  }
  joint_init=1; 

}
