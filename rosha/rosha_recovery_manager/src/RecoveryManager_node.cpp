/*
 * RecoverManager.cpp
 *
 *  Created on: May 8, 2014
 *      Author: dominik
 */

#include "../include/rosha_recovery_manager/RecoveryManager_node.h"

using namespace std;
using namespace value_iteration;

RecoveryManager* recM;
ros::Publisher* actionPublisher;

void Callback(const std_msgs::Int32::ConstPtr& msg)
{
  int stateId = msg->data;
    int actionId = recM->GetAction(stateId);

    std_msgs::String actionMsg;
    actionMsg.data = "test";
    actionPublisher->publish(actionMsg);
}


int main (int argc, char** argv)
{
  recM = new RecoveryManager();

  ros::init(argc, argv, "rosha_recovery_manager");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::String>("recovery_action", 1000);
  actionPublisher = &pub;

  ros::Subscriber sub = n.subscribe("diagnostic_info", 1000, Callback);

  try
  {
    ros::Rate pub_rate(1.0);
    while (ros::ok())
    {
      ros::spinOnce();

      //publish regularily data

      pub_rate.sleep();
    }
  }
  catch (exception& e)
  {
    ROS_FATAL("Recovery Manager node caught exception. Aborting. %s", e.what());
    ROS_BREAK();
  }


}

