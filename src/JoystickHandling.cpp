#include <planning_arch/PlanManager.h>

namespace planner {

void PlanManager::joystickCallback(const
  joystick_handler::JoystickValues::ConstPtr& msg)
{
  joy_input_(FORWARD) = msg->v_x;
  joy_input_(YAW) = msg->omega;
  joy_input_(Z) = msg->v_z;
  joy_input_(SIDE) = msg->v_side;
}

State PlanManager::getReplanState(const Eigen::Vector4d& input)
{
  bool trajectory_input = (std::abs(input(FORWARD)) +
                        std::abs(input(Z)) +
                        std::abs(input(YAW)) +
                        std::abs(input(SIDE))) > 0.00005;
  // This threshold was relaxed in order to accommodate for sensitive
  // joysticks. Experimentally tested to be an OK value.
  bool change_in_input = (input - previous_joy_input_).norm() > 0.05;

  // If no change in input and it's zero, do not replan. else, handle as usual
  if (input.norm() < 0.0001 && !change_in_input)
  {
    // ROS_WARN("[PlanManager::at_end] DO_NOTHING:  input.norm() < 0.0001 && !change_in_input");
    return State::DO_NOTHING;
  }

  if (change_in_input) {
    ROS_WARN("[PlanManager::getReplanState (at_end)] INPUT_CHANGED. Previous input is: (%.2f, %.2f, %.2f, %.2f), updating to: (%.2f, %.2f, %.2f, %.2f)", previous_joy_input_(FORWARD), previous_joy_input_(YAW), previous_joy_input_(Z), previous_joy_input_(SIDE), input(FORWARD), input(YAW), input(Z), input(SIDE));
    previous_joy_input_ = input;
    return State::INPUT_CHANGED;
  }

  auto time_passed_since_traj_start = ros::Time::now() - traj_start_time_;
  auto time_to_replan = ros::Duration(std::max(traj_duration_ - lookahead_, (float)0.0));

  if (time_passed_since_traj_start >= time_to_replan)
  {
    // ROS_WARN("[PlanManager::getReplanState (at_end)] REGENERATE. time_passed: %.3f, time_to_replan %3f", time_passed_since_traj_start.toSec(), time_to_replan.toSec());
    return State::REGENERATE;
  }

  return State::DO_NOTHING;
}

Motion PlanManager::getMotion(const Eigen::Vector4d& input)
{
  if (std::abs(input(FORWARD)) <= 0.01 &&
      std::abs(input(YAW)) > 0 &&
      std::abs(input(Z)) <= 0.01 &&
      std::abs(input(SIDE)) <= 0.01)
  {
    mapped_input_(FORWARD) = 0.0;
    mapped_input_(YAW) = joy_input_(YAW) * max_yaw_rate_;
    mapped_input_(Z) = 0.0;
    mapped_input_(SIDE) = 0.0;

    return Motion::PURE_YAW;
  }
  else if (std::abs(input(FORWARD)) <= 0.01 &&
      std::abs(input(Z)) > 0 &&
      std::abs(input(SIDE)) <= 0.01)
  {
    mapped_input_(FORWARD) = 0.0;
    mapped_input_(YAW) = joy_input_(YAW) * max_angular_velocity_;
    mapped_input_(Z) = joy_input_(Z) * max_z_velocity_;
    mapped_input_(SIDE) = 0.0;

    return Motion::PURE_Z;
  }
  else if (std::abs(input(FORWARD)) <= 0.01 &&
      std::abs(input(YAW)) <= 0.01 &&
      std::abs(input(Z)) <= 0.01 &&
      std::abs(input(SIDE)) <= 0.01)
  {
    mapped_input_(FORWARD) = 0.0;
    mapped_input_(YAW) = 0.0;
    mapped_input_(Z) = 0.0;
    mapped_input_(SIDE) = 0.0;
    return Motion::ZERO;
  }
  else
  {
    mapped_input_(FORWARD) = joy_input_(FORWARD) * max_forward_velocity_;
    mapped_input_(YAW) = joy_input_(YAW) * max_angular_velocity_;
    mapped_input_(Z) = joy_input_(Z) * max_z_velocity_;
    mapped_input_(SIDE) = side_velocity_enabled_ ? joy_input_(SIDE) * max_forward_velocity_ : 0.0;
    return Motion::NAVIGATION;
  }
}
// bool PlanManager::newInputReceived(const Eigen::Vector4d& input)
// {
//   // If no change in input and it's zero, do not replan. else, handle as usual
//   if (input.norm() < 0.0001) return false;
//
//   if (regeneration_scheme_ == "fixed_rate")
//   {
//     return true;
//   }
//   else if (regeneration_scheme_ == "at_end")
//   {
//     // This threshold was relaxed in order to accommodate for sensitive
//     // joysticks. Experimentally tested to be an OK value.
//     bool change_in_input = (input - previous_joy_input_).norm() > 0.05;
//
//     if (change_in_input)
//     {
//       previous_joy_input_ = input;
//       return true;
//     } else return false;
//   }
//   return false;
// }
//
// bool PlanManager::regenerateWithPreviousInput(const Eigen::Vector4d& input)
// {
//   // If no change in input and it's zero, do not replan. else, handle as usual
//   if (input.norm() < 0.0001) return false;
//
//   if (regeneration_scheme_ == "at_end")
//   {
//     float lookahead = avg_traj_compute_time_ == 0? 0.8 : avg_traj_compute_time_;
//
//     auto time_passed_since_traj_start = ros::Time::now() - traj_start_time_;
//     auto time_left = ros::Duration(std::max(traj_duration_ - lookahead, (float)0.0));
//
//     if (time_passed_since_traj_start > time_left)
//       return true;
//   }
//   return false;
// }

} // namespace planner
