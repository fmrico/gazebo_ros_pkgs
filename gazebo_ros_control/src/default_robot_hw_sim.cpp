/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/

#include <stdexcept>
#include <boost/foreach.hpp>
#include <transmission_interface/transmission_interface_loader.h>
#include <gazebo_ros_control/default_robot_hw_sim.h>

namespace gazebo_ros_control
{


bool DefaultRobotHWSim::initSim(
  const std::string& robot_namespace,
  ros::NodeHandle model_nh,
  gazebo::physics::ModelPtr parent_model,
  const urdf::Model *const urdf_model,
  std::vector<transmission_interface::TransmissionInfo> transmissions)
{
  // register hardware interfaces
  // TODO: Automate, so generic interfaces can be added
  registerInterface(&js_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  // populate hardware interfaces, bind them to raw Gazebo data
  namespace ti = transmission_interface;
  BOOST_FOREACH(const ti::TransmissionInfo& tr_info, transmissions)
  {
    BOOST_FOREACH(const ti::JointInfo& joint_info, tr_info.joints_)
    {
      BOOST_FOREACH(const std::string& iface_type, joint_info.hardware_interfaces_)
      {
        // TODO: Wrap in method for brevity?
        RwResPtr res;
        // TODO: A plugin-based approach would do better than this if chain
        // To do this, move contructor logic to init method, and unify signature
        if (iface_type == "hardware_interface/JointStateInterface")
        {
          res.reset(new internal::JointState());
        }
        else if (iface_type == "hardware_interface/PositionJointInterface")
        {
          res.reset(new internal::PositionJoint());
        }
        else if (iface_type == "hardware_interface/VelocityJointInterface")
        {
          res.reset(new internal::VelocityJoint());
        }

        // initialize and add to list of managed resources

        if (res)
        {
          try
          {
            res->init(joint_info.name_,
                      model_nh,
                      parent_model,
                      urdf_model,
                      this);
            rw_resources_.push_back(res);
            ROS_ERROR_STREAM("Registered joint '" << joint_info.name_ << "' in hardware interface '" <<
                             iface_type << "'."); // TODO: Lower severity to debug!
          }
          catch (const internal::ExistingResourceException&) {} // resource already added, no problem
          catch (const std::runtime_error& ex)
          {
            ROS_ERROR_STREAM("Failed to initialize gazebo_ros_control plugin.\n" <<
                             ex.what());
            return false;
          }
          catch(...)
          {
            ROS_ERROR_STREAM("Failed to initialize gazebo_ros_control plugin.\n" <<
                             "Could not add joint '" << joint_info.name_ << "' to hardware interface '" <<
                             iface_type << "'.");
            return false;
          }
        }

      }
    }
  }

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void DefaultRobotHWSim::readSim(ros::Time time, ros::Duration period)
{
  BOOST_FOREACH(RwResPtr res, rw_resources_)
  {
    res->read(time, period, e_stop_active_);
  }
}

void DefaultRobotHWSim::writeSim(ros::Time time, ros::Duration period)
{
  // TODO: Enforce joint limits

  BOOST_FOREACH(RwResPtr res, rw_resources_)
  {
    res->write(time, period, e_stop_active_);
  }
}

void DefaultRobotHWSim::eStopActive(const bool active)
{
  e_stop_active_ = active;
}
/*
// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void DefaultRobotHWSim::registerJointLimits(const std::string& joint_name,
                         const hardware_interface::JointHandle& joint_handle,
                         const ControlMethod ctrl_method,
                         const ros::NodeHandle& joint_limit_nh,
                         const urdf::Model *const urdf_model,
                         int *const joint_type, double *const lower_limit,
                         double *const upper_limit, double *const effort_limit)
{
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL)
  {
    const boost::shared_ptr<const urdf::Joint> urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL)
    {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN)
  {
    // Infer the joint type.

    if (limits.has_position_limits)
    {
      *joint_type = urdf::Joint::REVOLUTE;
    }
    else
    {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits)
  {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits)
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          ej_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          pj_limits_interface_.registerHandle(limits_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
          vj_limits_interface_.registerHandle(limits_handle);
        }
        break;
    }
  }
  else
  {
    switch (ctrl_method)
    {
      case EFFORT:
        {
          const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
          ej_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case POSITION:
        {
          const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
          pj_sat_interface_.registerHandle(sat_handle);
        }
        break;
      case VELOCITY:
        {
          const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
          vj_sat_interface_.registerHandle(sat_handle);
        }
        break;
    }
  }
}*/

} // namespace

PLUGINLIB_EXPORT_CLASS(gazebo_ros_control::DefaultRobotHWSim, gazebo_ros_control::RobotHWSim)
