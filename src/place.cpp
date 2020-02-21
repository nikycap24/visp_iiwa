#include <ros/ros.h>
#include "TooN/TooN.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h> 
#include "sensor_msgs/JointState.h"
#include "Traj_Generators/Quintic_Poly_Traj.h"
#include "Traj_Generators/Line_Segment_Traj.h"
#include "Robots/LBRiiwa7.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/TwistStamped.h"
#include "geometry_msgs/Point.h"
#include "Traj_Generators/Vector_Independent_Traj.h"
#include <std_msgs/Bool.h>
#include "visp_common/ChooseObject.h"
#include <iiwa_msgs/JointPosition.h>
#include <fstream>
#include <iostream>

using namespace std;
using namespace TooN;

#define tf 20.0
#define NUM_JOINTS 7

std_msgs::Bool stop_traj;
Vector<NUM_JOINTS> q0; //initial configuration
bool joint_ok_init; //initial configuraton flag

iiwa_msgs::JointPosition current_joint_position;

/*--------------------CALLBACK---------------------------*/

void readJointPos(const iiwa_msgs::JointPosition& msg)
{
    current_joint_position=msg;
    if(!joint_ok_init){			
	 joint_ok_init=true;
      q0[0] = current_joint_position.position.a1;
      q0[1] = current_joint_position.position.a2;
      q0[2] = current_joint_position.position.a3;
      q0[3] = current_joint_position.position.a4;
      q0[4] = current_joint_position.position.a5;
      q0[5] = current_joint_position.position.a6;
      q0[6] = current_joint_position.position.a7;	
    }
}
/*-------------------------------------------------------*/


int main(int argc, char **argv){
    
   ros::init(argc, argv, "place_node");
   ros::NodeHandle n = ros::NodeHandle();
    

   double T_s=0.001;
   ros::Rate loop_rate(1/T_s);

   /*--------------------SUBSCRIBER & PUBLISHER---------------------------*/
   ros::Subscriber sub = n.subscribe("iiwa/state/JointPosition", 1, readJointPos);
   ros::Publisher pub_joints_pose = n.advertise<geometry_msgs::PoseStamped>("/desired_pose", 1);
   ros::Publisher pub_joints_twist = n.advertise<geometry_msgs::TwistStamped>("/desired_twist", 1);
   ros::Publisher pub_stop = n.advertise<std_msgs::Bool>("stop_clik", 1);
   /*--------------------------------------------------------------------*/

   
 while(ros::ok() && !joint_ok_init){
		ros::spinOnce();
   }
    
   
    Matrix<4,4> n_T_e=Data( -1.0000,         0,   -0.0000,         0,
   								  0.0000,   -0.9659,   -0.2588,    0.0621,
   								 -0.0000,   -0.2588,    0.9659,    0.0319,
         						  0      ,   0,         0,         1.0000);
            									
    LBRiiwa7 iiwa=LBRiiwa7(n_T_e,2.0,"kuka");

    Vector<> q_DH= iiwa.joints_Robot2DH(q0);
    Matrix<4,4> T_init = iiwa.fkine(q_DH);
    
    cout<<"T_init: "<<endl<<T_init<<endl;
    
    //return 0;

    Vector<3> pi=transl(T_init);
    

    Matrix<3,3> Rf= Data(0.999475, 0.00206915, 0.0323177,    
                         -0.0308376, -0.243968, 0.969262, 
                         0.00989062, -0.96975, -0.243776); 
                          
    UnitQuaternion Qi(T_init);
    Vector<3> Qi_v=Qi.getV();
    double Qi_s=Qi.getS();
    UnitQuaternion Qf(Rf);
    Vector<3> Qf_v=Qf.getV();
    double Qf_s=Qf.getS();
    
    cout<<"Quaternione iniziale: "<<endl<<Qi<<endl;
    cout<<"Quaternione finale: "<<endl<<Qf<<endl;


    Quintic_Poly_Traj s_pos(   
                            tf,//duration
                            0.0,//double initial_position,
                            1.0);//double final_position,
    
    Vector<3> pf1,pf_absolute;
    
    pf1=pi+makeVector(-0.12,0.0,0.3);  //cambiare pos finale 1 tratto
    
    Line_Segment_Traj lin_traj1( 
		                          pi,//pi
		                          pf1,//pf
		                          s_pos);
		                          
    string object = " ";
    object = argv[1];
        
    if(object == "balea"){
    	pf_absolute=makeVector(0.578196,0.320124,0.409613);
    }
    else if(object == "heitmann"){
      pf_absolute=makeVector(0.409501,0.393137,0.353213);
    }
    else if(object == "finish"){
      pf_absolute=makeVector(0.406169,0.326159,0.310224);
    }
    else if(object == "denkmit"){
      pf_absolute=makeVector(0.723309,0.332197,0.465324);
    }
    else{
    cout<<"Oggetto non riconosciuto"<<endl;
    	for(int i=0; i<20; i++){
			stop_traj.data = 1; // sending signal to stop clik                         
			pub_stop.publish(stop_traj);
			ros::spinOnce();
    	}
    }
    
        
    //pf_absolute=makeVector(0.633758,0.33991,0.473904);  //cambiare pos finale 2 tratto
    
    Line_Segment_Traj lin_traj2( 
		                          pf1,//pi
		                          pf_absolute,//pf
		                          s_pos);                         
                             

    double time_now = ros::Time::now().toSec();
    lin_traj1.changeInitialTime(time_now);
	 lin_traj2.changeInitialTime(time_now+tf);
	 s_pos.changeInitialTime(time_now);


	 while(ros::ok() && ((!lin_traj1.isCompleate(time_now)) || (!lin_traj2.isCompleate(time_now))))
	 {
	    
		 time_now = ros::Time::now().toSec();
       Vector<3> final_position;
		 UnitQuaternion Q_now=Qi.interp(Qf,s_pos.getPosition(time_now),false);
       
       if((!lin_traj1.isCompleate(time_now))){
		 final_position=lin_traj1.getPosition(time_now);
		 }
       
       if(lin_traj1.isCompleate(time_now) && (!lin_traj2.isCompleate(time_now))){
		 final_position=lin_traj2.getPosition(time_now);}
		 

		 geometry_msgs::PoseStamped posemsg;
		 geometry_msgs::TwistStamped twistmsg;

       cout<<"pf: "<<final_position[0]<<" "<<final_position[1]<<" "<<final_position[2]<<endl;
       
		 posemsg.pose.position.x=final_position[0];
		 posemsg.pose.position.y=final_position[1];
		 posemsg.pose.position.z=final_position[2];
		
	    posemsg.pose.orientation.x=Q_now.getV()[0];
	    posemsg.pose.orientation.y=Q_now.getV()[1];
	    posemsg.pose.orientation.z=Q_now.getV()[2];
	    posemsg.pose.orientation.w=Q_now.getS();
		
	   
		 twistmsg.twist.linear.x=0.0;
		 twistmsg.twist.linear.y=0.0;
		 twistmsg.twist.linear.z=0.0;
	
		 twistmsg.twist.angular.x=0.0;
		 twistmsg.twist.angular.x=0.0;
		 twistmsg.twist.angular.x=0.0;
		

		 pub_joints_pose.publish(posemsg);
		 pub_joints_twist.publish(twistmsg);

		 ros::spinOnce();
		 loop_rate.sleep();
    }
    
    for(int i=0; i<20; i++){
		stop_traj.data = 1; // sending signal to stop clik                         
		pub_stop.publish(stop_traj);
		ros::spinOnce();
    }
    

}

