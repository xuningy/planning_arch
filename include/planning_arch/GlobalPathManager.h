/*
GlobalPathManager
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
#include <nav_msgs/Odometry.h>

#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>
#include <time_optimal_primitives/TimeOptimalPrimitives.h>
#include <ros_utils/ParameterUtils.h>
#include <planning_representations/Trajectory.h>
#include <trajectory_utils/PathUtils.h>
#include <sensor_msgs/Joy.h>
#include <vis_utils/BaseVisualization.h>

#include <planning_arch/Motion.h>

namespace planner {

struct GlobalPathInfo {
  bool global_path_set_ = false;
  ForwardArcMotionPrimitives global_trajectory_;
  std::vector<FlatState> global_path_;
  FlatState global_start_state_;
  FlatState global_end_state_;

  double global_traj_duration_;
  ros::Time global_traj_start_time_;

  void clear()
  {
    global_path_set_ = false;
    global_path_.clear();
    global_traj_duration_ = 0;
    ros::Time global_traj_start_time_ = ros::Time();
  };


  void print()
  {
    ROS_INFO("Global:");
    ROS_INFO("\tglobal_path_set_: %d", global_path_set_);
    ROS_INFO("\tglobal_start_state_: pos (%.3f, %.3f, %.3f), vel (%.3f, %.3f, %.3f), acc (%.3f, %.3f, %.3f)",
    global_start_state_.pos(0), global_start_state_.pos(1), global_start_state_.pos(2), global_start_state_.vel(0), global_start_state_.vel(1), global_start_state_.vel(2), global_start_state_.acc(0), global_start_state_.acc(1), global_start_state_.acc(2));
    ROS_INFO("\tglobal_end_state_: pos (%.3f, %.3f, %.3f), vel (%.3f, %.3f, %.3f), acc (%.3f, %.3f, %.3f)",
    global_end_state_.pos(0), global_end_state_.pos(1), global_end_state_.pos(2), global_end_state_.vel(0), global_end_state_.vel(1), global_end_state_.vel(2), global_end_state_.acc(0), global_end_state_.acc(1), global_end_state_.acc(2));
    ROS_INFO("\tglobal_traj_duration_: %.3f s", global_traj_duration_);
    ROS_INFO("\tglobal_traj_start_time_: %.3f s", global_traj_start_time_.toSec());
  }
};

class GlobalPathManager {
public:
  GlobalPathManager() {};
  ~GlobalPathManager() {};

  bool initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private);

  bool intentionChanged(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input);

  bool replanGlobalPath(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input, double duration);

  void setMotion(const Motion& motion)
  { motion_ = motion; };

  void setFsmTeleop(bool flag)
  { teleop_enabled_ = flag;  };

  std::shared_ptr<GlobalPathInfo> global_path_info_;

private:
  // flags
  Motion motion_; // trajectory planners enabled only when motion is in NAVIGATION mode
  bool teleop_enabled_;

  FlatState findClosestGlobalReference(const ForwardArcMotionPrimitives& mp, const Eigen::Vector3d& pos);

  // get odom (theoretically 1000Hz)
  ros::Subscriber odom_sub_;
  void odomCallback(const nav_msgs::Odometry& msg);

  // position based regeneration checking at 10Hz
  ros::Timer near_end_timer_;
  void nearEndCallback(const ros::TimerEvent& event);

  // get joy
  ros::Subscriber joy_sub_;
  void rawJoystickCallback(const sensor_msgs::Joy& msg);
  double value_from_angular_velocity_joystick_raw_;
  double value_from_linear_velocity_joystick_raw_;

  // timing parameters
  double replan_dist_threshold_;
  double global_check_rate_;
  double global_replan_dist_;

  // Input parameters
  double max_z_velocity_;
  double max_angular_velocity_;
  double max_forward_velocity_;
  double max_yaw_rate_;
  double stopping_trajectory_duration_;
  double braking_trajectory_duration_;
  double yaw_rampup_duration_;
  double tf_;

  // model
  std::string global_model_;

  Eigen::Vector3d curr_pos_;
  Eigen::Vector4d current_input_;
  Eigen::Vector4d filtered_input_; // for filtering of simple_filter
  double lambda_; // lambda for simple_filter

  // params
  double global_traj_duration_param_; // a duration that is the length of the primitive to be generated.

  // Visualizer
  ros::Publisher mp_vis_pub_;

  // Debug
  bool debug_vis_only_; // vis only?
  bool verbose_;

  // debug special mode: one input only.
  bool first_input_ = true;

};

} // namespace planner
