/*****************************************************************************/
/*!
 *  @file   nodelet_manager.cc
 *  @date   2019.05.16
 *  @author     M. Treglia Akka
 *  @brief      Nodelet manager node
 *****************************************************************************/

#include <rosnodelets/manager/NodeletManager.h>
#include <ros/ros.h>

using rosnodelets::ManagerHandle;
using ros::init;
using ros::NodeHandle;
using ros::spin;
using ros::spinOnce;

int main(int argc, char **argv) {
  // Init ROS
  init(argc, argv, "PerceptionManager");
  // Get NodeHandle
  NodeHandle nh;
  // Instantiate Nodelet manager
  ManagerHandle manager(nh);
  // Spin
  spin();

  return 0;
}