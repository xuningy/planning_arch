/*
LocalTrajectoryManager
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
#include <collision_checker/CollisionChecker.h>
#include <ros_utils/ParameterUtils.h>
#include <primitive_vis_utils/MotionPrimitiveVisualization.h>
#include <planning_arch/Evaluation.h>
#include <vis_utils/BaseVisualization.h>

#include <planning_arch/Motion.h>
#include <planning_arch/GlobalPathManager.h>
#include <planning_arch/LocalTrajectoryGenerator.h>
#include <planning_arch/local_trajectory_generator/MPTLocalGenerator.h>
// #include <stopping_trajectory/StoppingTrajectory.h>
#include <planning_control_interface/ControlArchInterface.h>

namespace planner {

struct LocalTrajInfo {
  bool local_trajectory_set_ = false;
  std::vector<FlatState> local_path_;

  double local_traj_duration_;
  ros::Time local_traj_start_time_;
  FlatState local_start_state_;
  FlatState local_end_state_;

  void clear()
  {
    local_trajectory_set_ = false;
    local_path_.clear();
    local_traj_duration_ = 0;
    ros::Time local_traj_start_time_ = ros::Time();
  }

  void print()
  {
    ROS_INFO("Local:");
    ROS_INFO("\tlocal_trajectory_set_: %d", local_trajectory_set_);
    ROS_INFO("\tlocal_start_state_: pos (%.3f, %.3f, %.3f), vel (%.3f, %.3f, %.3f), acc (%.3f, %.3f, %.3f)",
    local_start_state_.pos(0), local_start_state_.pos(1), local_start_state_.pos(2), local_start_state_.vel(0), local_start_state_.vel(1), local_start_state_.vel(2), local_start_state_.acc(0), local_start_state_.acc(1), local_start_state_.acc(2));
    ROS_INFO("\tlocal_end_state_: pos (%.3f, %.3f, %.3f), vel (%.3f, %.3f, %.3f), acc (%.3f, %.3f, %.3f)",
    local_end_state_.pos(0), local_end_state_.pos(1), local_end_state_.pos(2), local_end_state_.vel(0), local_end_state_.vel(1), local_end_state_.vel(2), local_end_state_.acc(0), local_end_state_.acc(1), local_end_state_.acc(2));
    ROS_INFO("\tlocal_traj_duration_: %.3f s", local_traj_duration_);
    ROS_INFO("\tlocal_traj_start_time_: %.3f s", local_traj_start_time_.toSec());
  }
};

class LocalTrajectoryManager {
public:
  LocalTrajectoryManager() {};
  ~LocalTrajectoryManager() {};

  bool initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private);

  bool replanLocalTrajectory(const Eigen::Vector4d& mapped_input);
  bool replanLocalTrajectory()
  {
    // call using default input.
    // TODO: last_executed_mapped_input_ is set by joystickChanged
    replanLocalTrajectory(last_executed_mapped_input_);
  }

  void publishTrajectory(const ros::Time& ref_time);

  void setMotion(const Motion& motion)
    { motion_ = motion; };

  bool linkGlobalPathInfo(std::shared_ptr<GlobalPathInfo> global_path_info)
    { global_path_info_ = global_path_info; };

  void setFsmTeleop(bool flag)
    { teleop_enabled_ = flag;  };

  // this function is called by plan_manager when joystick input has changed.
  // global plan's function intentionChanged() will be called prior to calling this function.
  // Intention and the global path may be changed prior to this function call.
  void joystickChanged(const Eigen::Vector4d& mapped_input);

  std::shared_ptr<LocalTrajInfo> local_traj_info_;
  std::shared_ptr<GlobalPathInfo> global_path_info_;


private:
  Motion motion_; // sets the motion type; trajectory planners enabled only when motion is in NAVIGATION mode
  bool teleop_enabled_;

  // Collision callbacks
  ros::Timer collision_check_timer_;
  ros::Timer near_end_timer_;
  void collisionCheckCallback(const ros::TimerEvent& event);
  void nearEndCallback(const ros::TimerEvent& event);

  // Stopping Trajectory
  // planner::StoppingTrajectory stopping_traj;
  // ros::Timer stopping_traj_timer_;
  // void callStoppingTrajChecker(const ros::TimerEvent& event);

  void updateAvgComputeTime(const std::chrono::duration<double>& t_callback);


  // FSM interfacing
  std::string name_;                            // Node name
  ControlArchInterface control_io_;             //

  // Parameters
  std::string mode_;
  bool direct_motion_primitive_enabled_;
  double global_dist_threshold_; // stop local plan if within threshold to global goal
  double local_replan_dist_threshold_; // dist before local end to replan local
  double global_regen_cost_threshold_; // cost for discrete frechet distance to regenerate a current trajectory if a new global trajectory is published.
  std::string lookahead_mode_;
  double local_lookahead_ = 0.0;
  double collision_check_rate_;
  double near_end_check_rate_;

  // compute time
  double avg_traj_compute_time_ = 0.0;
  int num_traj_callbacks_ = 0;

  // Default input
  Eigen::Vector4d last_executed_mapped_input_ = Eigen::Vector4d(1, 0, 0, 0);

  // Collision Checker
  std::shared_ptr<CollisionChecker> collision_checker_;

  // Trajectory generator
  std::shared_ptr<MPTLocalGenerator> local_trajectory_generator_;
  // std::unique_ptr<pluginlib::ClassLoader<planner::LocalTrajectoryGenerator>> trajectory_generator_plugin_loader_ptr_;
  // boost::shared_ptr<LocalTrajectoryGenerator> local_trajectory_generator_;

  // // VIsualization
  ros::Publisher local_traj_vis_pub_;
  ros::Publisher global_segment_vis_pub_, local_segment_vis_pub_;

  // Debug
  bool debug_vis_only_; // vis only?
  bool verbose_;
};

} // namespace planner
