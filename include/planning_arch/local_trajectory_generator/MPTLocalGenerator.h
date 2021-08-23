/*
MPTLocalGenerator
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <chrono>
#include <memory>
#include <vector>

#include <ros/ros.h>

#include <cpp_utils/io_utils.h>
#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>
#include <motion_primitive_tree/ListTree.h>
#include <ros_utils/ParameterUtils.h>
#include <planning_arch/LocalTrajectoryGenerator.h>
#include <primitive_vis_utils/MotionPrimitiveVisualization.h>


namespace planner {

class MPTLocalGenerator : public LocalTrajectoryGenerator {
public:
  MPTLocalGenerator() {};
  ~MPTLocalGenerator() {};

  bool initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private);

  bool generateTrajectory(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input);

  std::tuple<bool, bool> inputChanged(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input)
  {
    // default behavior is to just call generateTrajectory.
    bool plan_success = generateTrajectory(ref_state, ref_time, mapped_input);
    bool new_plan_generated = true;
    return {new_plan_generated, plan_success};
  }

private:

  std::deque<ForwardArcMotionPrimitives> final_trajectory_;

  MotionPrimitiveListTree tree_;
  std::string policy_;
  std::string primitive_type_;                  // linear velocity only or forward arc primitives
  std::string discretization_mode_;     // Discretization mode

  // Parameters
  double tf_;
  double yaw_rampup_duration_;
  double max_yaw_rate_;
  double omega_bound_;                  // Max bound on omega
  double z_bound_;                      // Max bound on z velocity
  double max_forward_velocity_;         // Max linear velocity

  // Uniformly discretized input arrays in order to compute joystick to motion
  std::vector<double> omega_range_, z_vel_range_, x_vel_range_, speed_range_;
  unsigned int num_lin_vel_;            //Number of linear vel discretizations
  unsigned int num_omega_;              // Number of omega discretizations
  unsigned int num_z_;                  // Number of z velocity discretizations
  // Visualizations
  visualization_msgs::MarkerArray marker_array_;
  visualization_msgs::MarkerArray endpoints_array_;
  ros::Publisher mp_vis_pub_;
  ros::Publisher mpl_vis_pub_;
  ros::Publisher endpoints_vis_pub_;

  // Visualizer
  MotionPrimitiveVisualization visualization_;
  ros::Publisher local_traj_vis_pub_;

  // record
  bool record_;

};

} // namespace planner
