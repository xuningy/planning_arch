/*
Evaluation
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <limits>
#include <deque>
#include <vector>
#include <stdexcept>

#include <planning_representations/FlatState.h>
#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>
#include <forward_arc_primitives/ForwardArcPrimitiveTrajectory.h>
#include <trajectory_utils/PathUtils.h>
#include <trajectory_utils/Similarity.h>

namespace traj_evaluation
{

  // Closeness: provides a closeness metric to another based on relative weighting.
  inline double Closeness(const std::vector<planner::FlatState>& traj1, const std::vector<planner::FlatState>& traj2)
  {
    // compute the distance between each trajectory to the global trajectory.
    // Using Dynamic Time Warping Distnace
    // c_global = similarity::DynamicTimeWarpingDistance(seg, global);

    // Using Discrete Frechet Distance
    // limit the number of points to max 50 points.
    int num_points = std::min(50, (int)std::max(traj1.size(), traj2.size()));
    auto rediscretized_traj1 = path_utils::discretizePath(traj1, num_points);
    auto rediscretized_traj2 = path_utils::discretizePath(traj2, num_points);

    double cost = similarity::DiscreteFrechetDistance(rediscretized_traj1, rediscretized_traj2);

    return cost;
  }


  // Closeness: provides a closeness metric to N trajectories based on relative weighting.
  inline double Closeness(const std::vector<std::vector<planner::FlatState>>& set_of_traj_to_compare_to, const std::vector<double> weights, const std::deque<planner::ForwardArcMotionPrimitives>& traj_to_evaluate)
  {
    double cost = 0;
    static double LARGE = 10e6;
    size_t N = weights.size();

    std::cout << "Number of trajectories " << N << " with weights: ";
    for (auto w : weights) std::cout <<  w << ", ";
    std::cout << "end." << std::endl;

    std::cout << "cost (w*cost): ";
    for (size_t i = 0; i++; i < N)
    {
      auto comparison_traj = set_of_traj_to_compare_to[i];
      auto w = weights[i];

      if (comparison_traj.size() < 2)
      {
        std::cout << "Trajectory comparison candidate " << i << "has only 1 point! cannot compute discrete frechet, setting cost for this trajectory to LARGE." << std::endl;
        cost += LARGE;
        continue;
      }

      // If the trajectory contains more than 2 segments, continue.

      // Get the duration for comparison trajectory for truncation
      double T_comparison = (comparison_traj.size() < 2) ? 0.0 : comparison_traj.back().t - comparison_traj.front().t;

      // Get the sample size for comparison trajectories, assuming they are evenly discretized
      double dt_comparison = (comparison_traj.size() < 2) ? 0.0 : comparison_traj[1].t - comparison_traj[0].t;

      // truncate the trajectory to have corresponding segments as the global trajectories.
      std::vector<planner::FlatState> traj_sample = planner::forward_arc_primitive_trajectory::samplePath(traj_to_evaluate, dt_comparison);

      auto [duration, seg] = path_utils::getSegmentWithLength(traj_sample, traj_sample.front().pos, T_comparison);

      // compute the distance between each trajectory to the global trajectory.
      // Using Dynamic Time Warping Distnace
      // c_global = similarity::DynamicTimeWarpingDistance(seg, global);

      // Using Discrete Frechet Distance
      // limit the number of points to max 50 points.
      int num_points = std::min(50, (int)std::max(seg.size(), comparison_traj.size()));
      auto traj1 = path_utils::discretizePath(seg, num_points);
      auto traj2 = path_utils::discretizePath(comparison_traj, num_points);

      double c = similarity::DiscreteFrechetDistance(traj1, traj2);

      cost += w*c;

      std::cout << w << "*" << c;
      if (i < N-1) std::cout << " + ";
    }

    std::cout << " = " << cost << std::endl;
    return cost;
  }


}
