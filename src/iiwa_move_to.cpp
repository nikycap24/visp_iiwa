#include "ros/ros.h"
#include "TooN/TooN.h"
#include "Robots/LBRiiwa7.h"
#include "sensor_msgs/JointState.h"
#include <std_msgs/Float64MultiArray.h> 
#include "Traj_Generators/Vector_Independent_Traj.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include <iiwa_msgs/JointPosition.h>

using namespace std;
using namespace TooN;

Vector<7> q0,qf;
           								
double tf=20.0;
bool joint_ok_init=false;
iiwa_msgs::JointPosition current_joint_position;
	
void readJointPos(const iiwa_msgs::JointPosition& msg)
{
      if(!joint_ok_init){
      joint_ok_init=true;
      current_joint_position=msg;
      q0[0] = current_joint_position.position.a1;
      q0[1] = current_joint_position.position.a2;
      q0[2] = current_joint_position.position.a3;
      q0[3] = current_joint_position.position.a4;
      q0[4] = current_joint_position.position.a5;
      q0[5] = current_joint_position.position.a6;
      q0[6] = current_joint_position.position.a7; 
   }
}

int main(int argc, char*argv[]){

	ros::init(argc, argv, "iiwa_move_to");

    ros::NodeHandle nh;
    
    //Oggetto robot
	LBRiiwa7 iiwa;
	
	qf=makeVector(atof(argv[1]),atof(argv[2]),atof(argv[3]),atof(argv[4]),atof(argv[5]),atof(argv[6]),atof(argv[7]));
	 
   ros::Subscriber joint_states_sub= nh.subscribe("/iiwa/state/JointPosition", 1, readJointPos);
	ros::Publisher joint_states_pub = nh.advertise<std_msgs::Float64MultiArray>("/joints_position", 1);
	
	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
	
	//GENERAZIONE TRAIETTORIA
	Vector_Independent_Traj traj;	
	
	for(int i=0;i<iiwa.getNumJoints();i++){
	traj.push_back_traj(Quintic_Poly_Traj (tf, q0[i], qf[i]));
	}
	
	double hz = 1000.0;
   ros::Rate loop_rate(hz);

   double time_now = ros::Time::now().toSec();
    
   traj.changeInitialTime(time_now);
    
   std_msgs::Float64MultiArray joints_states_msg;
   joints_states_msg.data.resize(iiwa.getNumJoints()*2);
    
    
   while(ros::ok() && (!traj.isCompleate(time_now))){
    
    time_now = ros::Time::now().toSec();
    
    Vector<> position =traj.getPosition(time_now);
    Vector<> velocity =traj.getVelocity(time_now);
    
    vector<bool> check_joint_lim=iiwa.checkHardJointLimits(position);
    vector<bool>  check_vel_lim=iiwa.checkHardVelocityLimits(velocity);

    for(int i=0;i<iiwa.getNumJoints();i++)
    {
         if(check_joint_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite, valore "<<position[i]<<endl;
           return -1;
        }
           if(check_vel_lim[i])
        {
           cout<<" Il giunto "<<i+1<<" ha superato il limite di velocità, valore "<<velocity[i]<<endl;
           return -1;
        }
    }
    
    std_msgs::Float64MultiArray outmsg;
    outmsg.data.resize(iiwa.getNumJoints());

    for( int i = 0; i<iiwa.getNumJoints(); i++ ){
            outmsg.data[i] = position[i] ;
            //outmsg.velocity[i] = velocity[i];
        }
        
        
   joint_states_pub.publish(outmsg);
 
	
	loop_rate.sleep();
	
}

}
