//
// Created by devel on 5/16/19.
//

#include <rosnodelets/manager/NodeletManager.h>

namespace rosnodelets {
ManagerHandle::ManagerHandle(NodeHandle &nh) : nh_(nh), manager_() {
  ROS_INFO("-- Nodelet Manager Created.");
}

ManagerHandle::~ManagerHandle() { ROS_INFO("-- Nodelet Manager Destructed."); }
}  // namespace rosnodelets
