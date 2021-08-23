#include <planning_arch/PlanManager.h>

using Clock = std::chrono::high_resolution_clock;

namespace planner {

void PlanManager::callTrajectoryGeneration(const ros::TimerEvent& event)
{
  // Get the reference to use for the trajectory
  ros::Time ref_time;
  FlatState ref_state;

  // Check if teleop is enabled, or a trajectory should be generated
  bool teleop_enabled = control_io_.FSMModeEnabled("teleop");
  if (!teleop_enabled) return;
  global_planner_->setFsmTeleop(teleop_enabled);
  local_planner_->setFsmTeleop(teleop_enabled);

  // Check if a reference state is available for planning
  if (!control_io_.getControlReference(ref_state, ref_time, lookahead_))
  {
    ROS_ERROR("[PlanManager::callTrajectoryGeneration] reference not found; trajectory not generated");
    return;
  }

  state_ = getReplanState(joy_input_);
  motion_ = getMotion(joy_input_);

  global_planner_->setMotion(motion_);
  local_planner_->setMotion(motion_);

  if (state_ == State::DO_NOTHING) return;
  if (state_ == State::REGENERATE && motion_ == Motion::NAVIGATION) return;
  if (state_ == State::REGENERATE && motion_ == Motion::ZERO) return;

  // Record joystick inputs for posterity.
  if (state_ == State::INPUT_CHANGED)
  {
    auto t_start = Clock::now();

    CodeTimer::record("[RecordingJoystickInputFreq] TOTAL NEW INPUTS", t_start);

    if (motion_ == Motion::PURE_YAW)
      CodeTimer::record("[RecordingJoystickInputFreq] PURE_YAW", t_start);
    if (motion_ == Motion::PURE_Z)
      CodeTimer::record("[RecordingJoystickInputFreq] PURE_Z", t_start);
    if (motion_ == Motion::ZERO)
      CodeTimer::record("[RecordingJoystickInputFreq] ZERO", t_start);
    if (motion_ == Motion::NAVIGATION)
      CodeTimer::record("[RecordingJoystickInputFreq] NAVIGATION", t_start);
  }

  switch (motion_)
  {
    case Motion::PURE_YAW:
    {
      ROS_INFO("[FSM: PURE_YAW]");
      publishPureYaw(joy_input_, ref_state, ref_time);
      break;
    }
    case Motion::PURE_Z:
    {
      ROS_INFO("[FSM: PURE_Z]");
      publishPureZ(joy_input_, ref_state, ref_time);
      break;
    }
    case Motion::ZERO:
    {
      if (state_ == State::INPUT_CHANGED)
      {
        ROS_INFO("[FSM: INPUT_CHANGED, ZERO]");
        publishPureStop(joy_input_, ref_state, ref_time);
      }
      break;
    }
    case Motion::NAVIGATION:
    {
      if (state_ == State::INPUT_CHANGED)
      {
        ROS_INFO("[INPUT_CHANGED, NAVIGATION] Change intention: (v, omega, z, side): (%.2f, %.2f, %.2f, %.2f)", mapped_input_(FORWARD), mapped_input_(YAW), mapped_input_(Z), mapped_input_(SIDE));

        // Update global trajectory with the full input regardless of the side velocity factor
        global_planner_->intentionChanged(ref_state, ref_time, joy_input_);
        local_planner_->joystickChanged(mapped_input_);

        traj_duration_ = global_path_info_->global_traj_duration_;
        traj_start_time_= global_path_info_->global_traj_start_time_;

      }
      break;
    } // end case
  } // end switch

  if (verbose_) printGlobalLocalStatus();

  return;
}



void PlanManager::publishPureYaw(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time)
{
  // Check for pure yaw

  traj_duration_ = tf_;
  traj_start_time_ = ref_time;

  auto pure_yaw_mp = ForwardArcMotionPrimitives(ref_state, mapped_input_, tf_, tf_, true);
    control_io_.publishTrajectory(ref_time, pure_yaw_mp);
  ROS_INFO("Pure yaw input provided; publishing pure yaw");
}

void PlanManager::publishPureZ(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time)
{

  traj_duration_ = tf_;
  traj_start_time_ = ref_time;

  auto pure_z_mp = ForwardArcMotionPrimitives(ref_state, mapped_input_, tf_, tf_, true);
  control_io_.publishTrajectory(ref_time, pure_z_mp);
  ROS_INFO("Pure z input provided; publishing thrust");
}

void PlanManager::publishPureStop(const Eigen::Vector4d& joy_input, const FlatState& ref_state, const ros::Time ref_time)
{

  traj_duration_ = braking_trajectory_duration_;
  traj_start_time_ = ref_time;

  control_io_.publishZeroTrajectory(ref_state, ref_time, braking_trajectory_duration_);

  ROS_INFO("Zero input provided; publishing stopping trajectory");
}

void PlanManager::publishEmergencyBrake(const FlatState& ref_state, const ros::Time& ref_time)
{
  if (ref_state.vel.norm() < 0.02)
  {
    ROS_INFO("[PlanManager::publishEmergencyBrake] reference is zero! No stopping trajectory published.");
  } else {
    ROS_WARN("[PlanManager::publishEmergencyBrake] Publish stopping trajectory for nonzero end velocity: ref vel: %.5f, %.5f, %.5f norm: %.5f", ref_state.vel(0), ref_state.vel(1), ref_state.vel(2), ref_state.vel.norm());

    // publish stopping trajectory
    control_io_.publishZeroTrajectory(ref_state, ref_time, stopping_trajectory_duration_);


    traj_duration_ = stopping_trajectory_duration_;
    traj_start_time_ = ref_time;
  }
}

} // namespace planner
