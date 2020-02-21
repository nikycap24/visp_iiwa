#include "ros/ros.h"
#include "Robots/LBRiiwa7.h"
#include <iiwa_msgs/JointPosition.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include <std_msgs/Float64MultiArray.h>
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Traj_Generators/Vector_Independent_Traj.h"
#include <std_msgs/Bool.h>

#define _USE_MATH_DEFINES
#define NUM_JOINTS 7
#define dim_T 4
#define tf 15.0

bool joint_ok_init;

using namespace TooN;
using namespace std;

iiwa_msgs::JointPosition current_joint_position;
std_msgs::Bool stop_traj;

Vector<7> pos_i_Robot;

string object=" ";

void readJointPos(const iiwa_msgs::JointPosition& msg)
{
    current_joint_position=msg;
    if(!joint_ok_init){			
	 joint_ok_init=true;
      pos_i_Robot[0] = current_joint_position.position.a1;
      pos_i_Robot[1] = current_joint_position.position.a2;
      pos_i_Robot[2] = current_joint_position.position.a3;
      pos_i_Robot[3] = current_joint_position.position.a4;
      pos_i_Robot[4] = current_joint_position.position.a5;
      pos_i_Robot[5] = current_joint_position.position.a6;
      pos_i_Robot[6] = current_joint_position.position.a7;	
    }
}

int main(int argc, char **argv){

	ros::init(argc, argv, "force_position_control_node");
    ros::NodeHandle n = ros::NodeHandle();

    double T_s=0.001;
    ros::Rate loop_rate(1/T_s);


    ros::Subscriber sub = n.subscribe("iiwa/state/JointPosition", 1, readJointPos);
    ros::Publisher pub_joints_pose = n.advertise<geometry_msgs::PoseStamped>("/desired_pose", 1);
	 ros::Publisher pub_joints_twist = n.advertise<geometry_msgs::TwistStamped>("/desired_twist", 1);
	 ros::Publisher pub_stop = n.advertise<std_msgs::Bool>("stop_clik", 1);


	while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
	}
	
  Matrix<dim_T,dim_T> n_T_e=Data(   -1.0000,         0,   -0.0000,         0,
   											 0.0000,   -0.9659,   -0.2588,    0.0621,
   											-0.0000,   -0.2588,    0.9659,    0.0319,
         									 0      ,   0,         0,         1.0000);
            									
  LBRiiwa7 iiwa=LBRiiwa7(n_T_e,2.0,"kuka");

    Vector<> q_DH= iiwa.joints_Robot2DH(pos_i_Robot);
    Matrix<4,4> T_init = iiwa.fkine(q_DH);
    
   
    Vector<3> p_cur = transl(T_init);
    cout<<"p_cur: "<<p_cur[0]<<" "<<p_cur[1]<<" "<<p_cur[2]<<endl;

    /*Matrix<3,3> Rf= Data( 0,-1, 0,
                         -1, 0, 0,
                          0, 0,-1);

    Matrix<3,3> Rf_rot = Rf*rotx(15.0*M_PI/180.0);*/
        
    
    UnitQuaternion Qi(T_init);
    Vector<3> Qi_v=Qi.getV();
    double Qi_s=Qi.getS();
    /*UnitQuaternion Qf(Rf_rot);
    Vector<3> Qf_v=Qf.getV();
    double Qf_s=Qf.getS(); */

    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
       
    
    object=argv[1];
    
    double displacement_x=0.0;
    double displacement_z=0.0;
    
    if(object=="denkmit") {
    displacement_z = -0.0900;
    displacement_x = 0.0;
    }    
    else if(object=="balea") {
    displacement_z = -0.07744;
    displacement_x = 0.0;
    }
    else if(object=="finish") {
    displacement_x = 0.12414;
    displacement_z = 0.025;
    }   
    else if(object=="heitmann") {
    displacement_x = 0.1111;
    displacement_z = 0.02;
    }
    else{
    cout<<"Object name is not correct"<<endl;
    displacement_z = 0.0;
    displacement_x=0.0;
    }
       
    /*Vector<> pf_tilde=makeVector(0.0,0.0,displacement_z,1.0);   
    Vector<> pf_b_tilde=T_init*pf_tilde; 
    Vector<3> pf_b = pf_b_tilde.slice(0,3);
    
    cout<<"pf_b: "<<pf_b[0]<<" "<<pf_b[1]<<" "<<pf_b[2]<<endl;*/
    
    Line_Segment_Traj lin_traj( 
		                          p_cur,//pi
		                          p_cur + makeVector(displacement_x,0.0,displacement_z), //pf
		                          s_pos);

    /*Vector_Independent_Traj rot_traj;
    
  
  	 for(int i=0;i<3;i++){
		rot_traj.push_back_traj(Quintic_Poly_Traj (tf, Qi_v[i], Qi_v[i]));
    }
    rot_traj.push_back_traj(Quintic_Poly_Traj (tf, Qi_s, Qf_s));*/
	

    double time_now = ros::Time::now().toSec();
	 lin_traj.changeInitialTime(time_now);
   // rot_traj.changeInitialTime(time_now);

	while(ros::ok() && (!lin_traj.isCompleate(time_now)))// || (!rot_traj.isCompleate(time_now))))
	{
	
    ros::spinOnce();
    
    time_now = ros::Time::now().toSec();

    Vector<> final_position =lin_traj.getPosition(time_now);
	 cout<<"final_position: "<<final_position[0]<<" "<<final_position[1]<<" "<<final_position[2]<<endl;
    /*Vector<4> rot_position = rot_traj.getPosition(time_now);
    Vector<4> rot_position_normalized;
    for(int i=0; i<4; i++){
     			rot_position_normalized[i] = rot_position[i]/norm(rot_position);
     }*/

      geometry_msgs::PoseStamped posemsg;
      geometry_msgs::TwistStamped twistmsg;

      posemsg.pose.position.x=final_position[0];
		posemsg.pose.position.y=final_position[1];
		posemsg.pose.position.z=final_position[2];
		
		posemsg.pose.orientation.x=Qi_v[0];
	   posemsg.pose.orientation.y=Qi_v[1];
	   posemsg.pose.orientation.z=Qi_v[2];
	   posemsg.pose.orientation.w=Qi_s;

      twistmsg.twist.linear.x=0.0;
		twistmsg.twist.linear.y=0.0;
		twistmsg.twist.linear.z=0.0;
		
		twistmsg.twist.angular.x=0.0;
		twistmsg.twist.angular.y=0.0;
		twistmsg.twist.angular.z=0.0;    


      pub_joints_pose.publish(posemsg);
      pub_joints_twist.publish(twistmsg);

      loop_rate.sleep();           

    }
    for(int i=0; i<20; i++){
		stop_traj.data = 1; // sending signal to stop clik                         
		pub_stop.publish(stop_traj);
		ros::spinOnce();
    }
} 



