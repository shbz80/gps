/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// Author: Stuart Glaser, Wim Meeussen
// Modfied by Shahbaz for YuMi ros control

#include "gps_agent_pkg/kdlchain.h"
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

namespace gps_control {

using namespace std;

//bool Chain::init(RobotState *robot_state, const std::string &root, const std::string &tip)
bool KdlChain::init(hardware_interface::EffortJointInterface* hw, const std::string &robot_descrption, const std::string &root, const std::string &tip)
{

  //robot_state_ = robot_state;
  //joints_ = joints;

  // Constructs the kdl chain
  KDL::Tree kdl_tree;
  if (!kdl_parser::treeFromString(robot_descrption, kdl_tree)){
    ROS_ERROR("Failed to construct kdl tree");
    return false;
  }

  bool res;
  try{
    res = kdl_tree.getChain(root, tip, kdl_chain_);
  }
  catch(...){
    res = false;
  }
  if (!res){
    ROS_ERROR("Could not extract chain between %s and %s from kdl tree",
              root.c_str(), tip.c_str());
    return false;
  }


  //Pulls out all the joint indices
  joints_.clear();
  for (size_t i=0; i<kdl_chain_.getNrOfSegments(); i++){
    if (kdl_chain_.getSegment(i).getJoint().getType() != KDL::Joint::None){
        joint_ = hw->getHandle(kdl_chain_.getSegment(i).getJoint().getName());  // throws on failure
        hardware_interface::JointHandle jnt = joint_;
        //TODO: implement some check here
        /*if (!jnt){
          ROS_ERROR("Joint '%s' is not found in joint state vector", kdl_chain_.getSegment(i).getJoint().getName().c_str());
          return false;
        }*/
        joints_.push_back(jnt);
    }
  }
  ROS_DEBUG("Added %i joints", int(joints_.size()));

  return true;
}

void KdlChain::getPositions(std::vector<double> &positions)
{
  positions.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    positions[i] = joints_[i].getPosition();
  }
}

void KdlChain::getVelocities(std::vector<double> &velocities)
{
  velocities.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    velocities[i] = joints_[i].getVelocity();
  }
}

void KdlChain::getEfforts(std::vector<double> &efforts)
{
  efforts.resize(joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
  {
    efforts[i] = joints_[i].getEffort();
  }
}

bool KdlChain::allCalibrated()
{
  //TODO: no calibration procedure for yumi
  // for (unsigned int i = 0; i < joints_.size(); ++i)
  // {
  //   if (!joints_[i]->calibrated_)
  //     return false;
  // }
  return true;
}

void KdlChain::toKDL(KDL::Chain &chain)
{
  chain = kdl_chain_;
}


void KdlChain::getPositions(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    a(i) = joints_[i].getPosition();
}

void KdlChain::getVelocities(KDL::JntArrayVel& a)
{
  assert(a.q.rows() == joints_.size());
  assert(a.qdot.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i){
    a.q(i) = joints_[i].getPosition();
    a.qdot(i) = joints_[i].getVelocity();
  }
}

void KdlChain::getEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    a(i) = joints_[i].getEffort();
}

void KdlChain::setEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    //joints_[i]->commanded_effort_ = a(i);
    joints_[i].setCommand(a(i)); // TODO: not sure about this
}

void KdlChain::addEfforts(KDL::JntArray& a)
{
  assert(a.rows() == joints_.size());
  for (unsigned int i = 0; i < joints_.size(); ++i)
    //joints_[i]->commanded_effort_ += a(i);
    joints_[i].setCommand(joints_[i].getEffort() + a(i)); // TODO: not sure about this
}


pr2_mechanism_model::JointState *KdlChain::getJoint(unsigned int actuated_joint_i)
{
  // if (actuated_joint_i >= joints_.size())
  //   return NULL;
  // else
  //   return joints_[actuated_joint_i];
  return NULL; // TODO check if this fuction is called anywhere
}



}
