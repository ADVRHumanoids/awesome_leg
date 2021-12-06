#include "ros/ros.h"

#include <sensor_msgs/JointState.h>
#include <xbot_msgs/JointCommand.h>
#include <vector>

// params: file for read commands, bridge_type=to_Xbot,to_Gazebo

ros::Publisher xbotcore_command_pub;

void CallBackFun(const sensor_msgs::JointState::ConstPtr& cartesio_sol)
{   
    int ctrl_mode_size=cartesio_sol->name.size()-1; //-1 to exclude the passive joint of the linear rail
    std::vector<int> ctrl_mode_aux;
    ctrl_mode_aux.assign(ctrl_mode_size,1); // control mode 1 for all the joints (position control)

    xbot_msgs::JointCommand xbot_command; 

    xbot_command.header.seq=cartesio_sol->header.seq; // access to cartesio_sol via -> because it is a pointer to the message; xbot_command can be directly accessed via ·
    xbot_command.header.stamp=cartesio_sol->header.stamp;
    xbot_command.header.frame_id=cartesio_sol->header.frame_id;
    xbot_command.name.assign((cartesio_sol->name.begin()+1),cartesio_sol->name.end()); // +1 to exclude the passive joint of the linear rail
    xbot_command.position.assign(cartesio_sol->position.begin()+1,cartesio_sol->position.end());
    xbot_command.velocity.assign(cartesio_sol->velocity.begin()+1,cartesio_sol->velocity.end());
    xbot_command.effort.assign(cartesio_sol->effort.begin()+1,cartesio_sol->effort.end());
    xbot_command.ctrl_mode.assign(ctrl_mode_aux.begin(),ctrl_mode_aux.end());

    xbotcore_command_pub.publish(xbot_command);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesio2gazebo_bridge");

  ros::NodeHandle n;

  ros::Subscriber cartesian_sol_sub = n.subscribe("/cartesian/solution", 1000, CallBackFun); 
  xbotcore_command_pub= n.advertise<xbot_msgs::JointCommand>("/xbotcore/command", 1000);
  ros::spin();
} 