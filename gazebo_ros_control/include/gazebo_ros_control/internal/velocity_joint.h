///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2015, PAL Robotics S.L.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of PAL Robotics S.L. nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

#ifndef _GAZEBO_ROS_CONTROL__INTERNAL__VELOCITY_JOINT_H_
#define _GAZEBO_ROS_CONTROL__INTERNAL__VELOCITY_JOINT_H_

#include <boost/shared_ptr.hpp>
#include <gazebo_ros_control/internal/joint_state.h>

namespace control_toolbox
{
  class Pid;
}

namespace gazebo_ros_control
{

namespace internal
{

class VelocityJoint : public JointState
{
public:
  VelocityJoint();

  virtual void init(const std::string&                        joint_name,
                    const ros::NodeHandle&                    nh,
                    boost::shared_ptr<gazebo::physics::Model> gazebo_model,
                    const urdf::Model* const                  urdf_model,
                    hardware_interface::RobotHW*              robot_hw);

  virtual void write(const ros::Time&     time,
                     const ros::Duration& period,
                     bool                 e_stop_active); // TODO: Invert to simply use active?

protected:
  double vel_cmd_;
//  double pos_min_; // TODO: Remove?
//  double pos_max_;
  double eff_max_;
  boost::shared_ptr<control_toolbox::Pid> pid_;
};

} // namespace

} // namespace

#endif
