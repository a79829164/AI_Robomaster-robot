#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;


class MakePlan{
	public:
		ros::NodeHandle* pnh;
		MoveBaseClient* ac; 
		ros::Publisher cancle_pub;
        move_base_msgs::MoveBaseGoal goal; 
        int goal_update;
		
		MakePlan(){
			pnh=new ros::NodeHandle("");
			cancle_pub= pnh->advertise<actionlib_msgs::GoalID>("move_base/cancel",1);
			ac=new MoveBaseClient("move_base", true);
            goal_update=0;
		};
		void cb_pose(const geometry_msgs::PoseConstPtr& msg);
};

void MakePlan::cb_pose(const geometry_msgs::PoseConstPtr& msg){
	
	goal.target_pose.header.frame_id = "odom"; 
    goal.target_pose.header.stamp = ros::Time::now(); 
    
    goal.target_pose.pose.position= msg->position;
    goal.target_pose.pose.orientation=msg->orientation;
	goal_update=1;
    ROS_INFO("*******************Get goal*************");
}


int main(int argc, char** argv)
{ 
	ros::init(argc, argv, "make_plan"); 
	ros::NodeHandle nh;
	
    MoveBaseClient ac("move_base", true);
    
	MakePlan mp;
//     move_base_msgs::MoveBaseGoal goal; 
//     goal.target_pose.header.frame_id = "odom"; 
//     goal.target_pose.header.stamp = ros::Time::now(); 
//     goal.target_pose.pose.position.x= 4;
//     goal.target_pose.pose.position.y= 2.5;
//     goal.target_pose.pose.orientation.x=0;
//     goal.target_pose.pose.orientation.y=0;
//     goal.target_pose.pose.orientation.z=0;
//     goal.target_pose.pose.orientation.w=1;
    
	ros::Subscriber sub_pose = nh.subscribe("goal_plan", 1, &MakePlan::cb_pose, &mp);
    ros::Rate rate(10);
    int k=0;
    while (ros::ok()){
        //mp.goal_update==1 &&
        if ( mp.goal_update==1 && ac.waitForServer(ros::Duration(5.0)))
        {
            ROS_INFO("****************************Set Goal*****************************");
            ac.sendGoal(mp.goal);
            mp.goal_update=0;
//             k=1;
        }
        ros::spinOnce();
        rate.sleep();
    }

}