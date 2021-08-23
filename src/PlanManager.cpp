#include <planning_arch/PlanManager.h>

namespace planner {

PlanManager::PlanManager() {}
PlanManager::~PlanManager()
{
  if (record_)
  {
    std::string codetimer_filename = saveto_directory_ + datetime_folder_name_ + std::string("/timing.csv");
    std::ofstream codetimer_file_;
    codetimer_file_.open(codetimer_filename);
    CodeTimer::printStats(codetimer_file_);
    codetimer_file_.close();
  }
  else
  {
    CodeTimer::printStats();
  }
}

bool PlanManager::initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private)
{
  name_ = ros::names::append(n.getNamespace(), "PlanManager");

  // Initialize fsm flags
  ros::NodeHandle nh(n);
  ros::NodeHandle nh_private(n_private);

  control_io_.initialize(n, nh);

  // Motion primitive Parameters
  if (!param_utils::get("teleoperation/max_z_velocity", max_z_velocity_)) return false;
  if (!param_utils::get("teleoperation/max_angular_velocity", max_angular_velocity_))
    return false;
  if (!param_utils::get("teleoperation/max_yaw_rate", max_yaw_rate_)) return false;
  param_utils::get("teleoperation/max_forward_velocity", max_forward_velocity_, 0.5);
  param_utils::get("teleoperation/trajectory_duration", tf_, 1.0);
  param_utils::get("teleoperation/stopping_trajectory_duration",
    stopping_trajectory_duration_, tf_ * 0.8);
  param_utils::get("teleoperation/braking_trajectory_duration",
    braking_trajectory_duration_, tf_);
  param_utils::get("teleoperation/yaw_rampup_duration", yaw_rampup_duration_, 0.5);

  param_utils::get("teleoperation/side_velocity_enabled", side_velocity_enabled_, false);
  param_utils::get("teleoperation/aligned_heading", aligned_heading_, false);
  param_utils::get("teleoperation/lookahead", lookahead_, (float)0.1);

  // Debug parameters
  param_utils::get("debug/vis_only", debug_vis_only_, false);
  param_utils::get("debug/verbose", verbose_, false);
  param_utils::get("record", record_, false);
  if (record_)
  {
    if (!param_utils::get("saveto_directory", saveto_directory_))
      saveto_directory_ = "~/.ros/";
  }

  // Initialize callbacks
  joy_sub_ = nh.subscribe("joy_filtered", 1,
    &PlanManager::joystickCallback, this);

  // Start timer
  joy_timer_ = n.createTimer(ros::Duration(traj_gen_rate_),
      &PlanManager::callTrajectoryGeneration, this);

  // Initialize regeneration flag.
  regen_with_prev_input_ = false;
  traj_start_time_ = ros::Time(1e6); // This is initialized to a ridiculously large number until the first trajectory is published.
  traj_duration_ = 0.0;

  joy_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  previous_joy_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);

  local_traj_info_ = std::make_shared<LocalTrajInfo>();

  // Initialize Global and Local trajectory generators
  global_planner_ = std::make_unique<GlobalPathManager>();
  global_planner_->initialize(n, n_private);
  global_path_info_ = global_planner_->global_path_info_;

  local_planner_ = std::make_unique<LocalTrajectoryManager>();
  local_planner_->initialize(n, n_private);
  local_planner_->linkGlobalPathInfo(global_planner_->global_path_info_);
  local_traj_info_= local_planner_->local_traj_info_;

  state_ = State::DO_NOTHING;
  motion_ = Motion::ZERO;

  // Debug function that prints what global and local planners are doing
  if (verbose_)
  {
    print_timer_ = n.createTimer(ros::Duration(0.5),
        &PlanManager::printCallback, this);
  }

  // Get folder name for filewriting
  if (record_)
  {
    datetime_folder_name_ = io_utils::DateTime();
  }

  // print parameters

  std::cout << "=================================================" << std::endl;
  std::cout << "        PlanManager Parameters " << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Mode: \t\t\t" << mode_ << std::endl;
  std::cout << "---" << std::endl;
  std::cout << "Max forward velocity: \t\t" << max_forward_velocity_ << " m/s"<< std::endl;
  std::cout << "Max angular velocity: \t\t" << max_angular_velocity_ << " rad/s"<< std::endl;
  std::cout << "Max pure yaw rate: \t\t" << max_yaw_rate_ << " rad/s"<< std::endl;
  std::cout << "Trajectory duration: \t\t" << tf_ << " s" << std::endl;
  std::cout << "Stopping trajectory duration: \t" << stopping_trajectory_duration_ << " s" << std::endl;
  std::cout << "Yaw ramp-up duration: \t\t" <<yaw_rampup_duration_ << " s" << std::endl;
  std::string ha_string = (aligned_heading_) ? "YES" : "NO";
  std::cout << "Heading aligned w/ trajectory:  " << ha_string << std::endl;
  std::cout << "Side velocity enabled:  " << side_velocity_enabled_ << std::endl;
  std::cout << "---" << std::endl;
  std::cout << "Lookahead: \t\t" << lookahead_ << std::endl;

  // Debug parameters
  std::cout << "---" << std::endl;
  if (debug_vis_only_) std::cout << "Visualizing trajectories ONLY, no trajectories published to the control_arch!" << std::endl;
  std::cout << "verbose: \t\t" << verbose_ << std::endl;
  std::cout << "record: \t\t" << record_ << std::endl;
  if (record_)
  std::cout << "files saved to: " << saveto_directory_.c_str() << datetime_folder_name_ << std::endl;
  std::cout << "=================================================" << std::endl;

  return true;
}

void PlanManager::printCallback(const ros::TimerEvent& event)
{
  printGlobalLocalStatus();
}

void PlanManager::printGlobalLocalStatus()
{
  std::vector<std::string> state_str_{"INPUT_CHANGED", "REGENERATE", "DO_NOTHING" };
  std::vector<std::string> motion_str_ {"PURE_YAW", "PURE_Z", "ZERO", "NAVIGATION"};
  ROS_INFO("================================================");
  ROS_INFO("State: %s", state_str_[static_cast<int>(state_)].c_str());
  ROS_INFO("Motion: %s", motion_str_[static_cast<int>(motion_)].c_str());

  global_path_info_->print();
  local_traj_info_->print();
  ROS_INFO("================================================");
}


} // namespace planner
