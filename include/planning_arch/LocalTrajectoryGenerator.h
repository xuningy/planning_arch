/*
LocalTrajectoryGenerator
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

namespace planner {

class LocalTrajectoryGenerator {
public:
  LocalTrajectoryGenerator() {};
  ~LocalTrajectoryGenerator() {};

  virtual bool initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private) = 0;

  bool initializeMap(const std::shared_ptr<CollisionChecker>& collision_checker)
  {
    collision_checker_ = collision_checker;
    ROS_WARN("[LocalTrajectoryGenerator] map initialized!");
  };


  void setGlobalTrajectory(const std::vector<FlatState>& global_segment)
  {
    current_global_segment_ = global_segment;
    global_segment_available_ = true;
    ROS_INFO("[LocalTrajectoryGenerator] global segment set in the local trajectory generator.");
  }

  void setLocalTrajectory(const std::vector<FlatState>& local_segment)
  {
    previous_local_segment_ = local_segment;
    local_segment_available_ = true;
    ROS_INFO("[LocalTrajectoryGenerator] local segment set in the local trajectory generator.");
  }

  virtual bool generateTrajectory(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input) = 0;

  virtual std::tuple<bool, bool> inputChanged(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input)
  {
    // default behavior is to just call generateTrajectory.
    bool plan_success = generateTrajectory(ref_state, ref_time, mapped_input);
    bool new_plan_generated = true;
    return {new_plan_generated, plan_success};
  }

  double getDuration() { return traj_duration_; };

  std::vector<FlatState> getTrajectory() { return trajectory_; };

protected:

  double traj_duration_; // must be set by LTG
  ros::Time traj_start_;

  std::vector<FlatState> current_global_segment_; // should be called to set by LTM
  std::vector<FlatState> previous_local_segment_; // should be called to set by LTM
  std::vector<FlatState> trajectory_; // must be set by LTG

  bool global_segment_available_ = false;
  bool local_segment_available_ = false;

  std::shared_ptr<CollisionChecker> collision_checker_;

};

} // namespace planner
