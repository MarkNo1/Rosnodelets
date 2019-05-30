/*****************************************************************************/
/*!
 *  @file   NodeletManager.h
 *  @date   2019.05.16
 *  @author     M. Treglia Akka
 *  @brief      C++ Spawn Nodelet Manager
 *****************************************************************************/

#ifndef RENAULT_NODELET_MANAGER_
#define RENAULT_NODELET_MANAGER_

#include <string>
// ROS
#include <ros/ros.h>
// Nodelet
#include <nodelet/loader.h>

namespace rosnodelets {
// Class used
using Loader     = nodelet::Loader;
using NodeHandle = ros::NodeHandle;

/**
 * Class ManagerHandle
 */
class ManagerHandle {
 public:
  explicit ManagerHandle(NodeHandle& nh);
  ~ManagerHandle();

 private:
  Loader     manager_;
  NodeHandle nh_;
};
}  // namespace renault_nodelets

#endif  // RENAULT_NODELET_MANAGER_
