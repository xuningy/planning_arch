/*
PlanManager
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
#include <pluginlib/class_loader.h>
#include <visualization_msgs/MarkerArray.h>

#include <codetimer_catkin/codetimer.h>
#include <cpp_utils/io_utils.h>
#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>
#include <joystick_handler/JoystickHandler.h>
#include <joystick_handler/JoystickValues.h>
#include <ros_utils/ParameterUtils.h>
#include <planning_representations/FlatState.h>

#include <planning_control_interface/ControlArchInterface.h>

#include <planning_arch/Motion.h>
#include <planning_arch/GlobalPathManager.h>
#include <planning_arch/LocalTrajectoryManager.h>

namespace planner {

class PlanManager {
public:
  PlanManager();
  ~PlanManager();

  bool initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private);

  bool checkAndPublishBasicMotion(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time);

private:
  // Define callback functions
  void joystickCallback(const joystick_handler::JoystickValues::ConstPtr& msg);
  void callTrajectoryGeneration(const ros::TimerEvent& event);

  // Functions that process joystick input
  // bool shouldReplan(const Eigen::Vector4d& new_input);
  // bool newInputReceived(const Eigen::Vector4d& input);
  // bool regenerateWithPreviousInput(const Eigen::Vector4d& input);


  // Trajectory publishing
  void publishWaypoints(const std::vector<FlatState>& traj_wpts, const ros::Time& publish_time, float traj_duration);
  void publishPolynomial(const ros::Time& publish_time,
    const ForwardArcMotionPrimitives& mp);
  void publishPureYaw(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time);
  void publishPureZ(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time);
  void publishPureStop(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time);
  void publishEmergencyBrake(const FlatState& ref_state, const ros::Time& ref_time);

  // misc utilities
  void updateAvgComputeTime(const std::chrono::duration<double>& t_callback);

  // states
  State getReplanState(const Eigen::Vector4d& input);
  Motion getMotion(const Eigen::Vector4d& input);

  State state_;
  Motion motion_;

  // Subscribers
  ros::Subscriber joy_sub_;

  // Timers
  ros::Timer joy_timer_;

  // FSM interfacing
  std::string name_;                            // Node name
  ControlArchInterface control_io_;             //

  // Joystick mapping
  Eigen::Vector4d joy_input_;        // Raw joystick values in [-1, 1]
  Eigen::Vector4d previous_joy_input_; // previous input for continued input
  Eigen::Vector4d mapped_input_;     // Scaled input from raw joystick values to real velocities
  enum { FORWARD, YAW, Z, SIDE }; // Indices into the above x_input_ Vec4s

  // Input parameters
  double max_z_velocity_;
  double max_angular_velocity_;
  double max_forward_velocity_;
  double max_yaw_rate_;
  double stopping_trajectory_duration_;
  double braking_trajectory_duration_;
  double yaw_rampup_duration_;
  double tf_;

  bool side_velocity_enabled_;
  bool aligned_heading_;

  // Trajectory flags
  std::string mode_;
  bool regen_with_prev_input_;  // a flag for continuing previous input;
  ros::Time traj_start_time_; // the time the trajectory was actually executed
  float traj_duration_; // the duration of the actual trajectory published
  double traj_gen_rate_;                // Trajectory generation rate
  float lookahead_ = 0.1;

  // Global and Local trajectory generators
  std::unique_ptr<GlobalPathManager> global_planner_;
  std::unique_ptr<LocalTrajectoryManager> local_planner_;

  std::shared_ptr<GlobalPathInfo> global_path_info_;
  std::shared_ptr<LocalTrajInfo> local_traj_info_;

  // Debug
  bool debug_vis_only_; // vis only?
  ros::Timer print_timer_;
  void printCallback(const ros::TimerEvent& event);
  void printGlobalLocalStatus();
  bool verbose_;

  // Record
  bool record_;
  std::string saveto_directory_;
  std::string datetime_folder_name_;

  // Lookahead parameters
  float avg_traj_compute_time_ = 0;
  unsigned int num_traj_callbacks_  = 0;

};

} // namespace planner
