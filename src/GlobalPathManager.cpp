#include <planning_arch/GlobalPathManager.h>
#include <codetimer_catkin/codetimer.h>

using Clock = std::chrono::high_resolution_clock;


namespace planner {


bool GlobalPathManager::initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private)
{
  ros::NodeHandle nh(n);
  ros::NodeHandle nh_private(n_private);

  // Parameters
  if (!param_utils::get("teleoperation/max_z_velocity", max_z_velocity_)) return false;
  if (!param_utils::get("teleoperation/max_angular_velocity", max_angular_velocity_))
    return false;
  param_utils::get("teleoperation/max_forward_velocity", max_forward_velocity_, 0.5);
  param_utils::get("teleoperation/trajectory_duration", tf_, 1.0);
  param_utils::get("teleoperation/yaw_rampup_duration", yaw_rampup_duration_, 0.5);

  param_utils::get("global/near_end_check_rate", global_check_rate_, 0.1);
  param_utils::get("global/replan_dist_threshold", global_replan_dist_, 0.5);

  if (!param_utils::get("global/model", global_model_)) global_model_ = std::string("single_line_only");
  param_utils::get("global/simple_filter_lambda", lambda_, 0.5);
  param_utils::get("global/global_traj_duration_param", global_traj_duration_param_, 5.0);

  param_utils::get("debug/vis_only", debug_vis_only_, false);
  param_utils::get("debug/verbose", verbose_, false);

  // Suscribers
  odom_sub_ = nh_private.subscribe("odom", 50,
      &GlobalPathManager::odomCallback, this);
  joy_sub_ = nh_private.subscribe("joy", 50,
          &GlobalPathManager::rawJoystickCallback, this); //added to get directionality on the angular velocity pad

  // Timers
  near_end_timer_ = n.createTimer(ros::Duration(global_check_rate_),
      &GlobalPathManager::nearEndCallback, this);

  // Publishers
  mp_vis_pub_ = nh.advertise<visualization_msgs::Marker>("vis_global_path", 10);

  // Initialize variables
  motion_ = Motion::ZERO;
  global_path_info_ = std::make_shared<GlobalPathInfo>();

  // Initialize simple_filter
  filtered_input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  value_from_linear_velocity_joystick_raw_ = 0.0;
  value_from_angular_velocity_joystick_raw_ = 0.0;

  std::cout << "=================================================" << std::endl;
  std::cout << "        GlobalPathManager Parameters " << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Model: \t\t\t" << global_model_.c_str() << std::endl;
  std::cout << "simple filter lambda: \t\t" << lambda_ << std::endl;
  std::cout << "global_traj_duration parameter: " << global_traj_duration_param_ << std::endl;
  // Replan parameters
  std::cout << "---" << std::endl;
  std::cout << "replan_dist_threshold: \t\t" << global_replan_dist_ << " m" << std::endl;
  std::cout << "near_end_check_rate: \t\t" << global_check_rate_ << " s\t"
                          << 1.0/global_check_rate_ << " Hz" << std::endl;
  // Debug parameters
  std::cout << "---" << std::endl;
  if (debug_vis_only_) std::cout << "Visualizing trajectories ONLY, no trajectories published to the control_arch!" << std::endl;
  std::cout << "verbose: \t\t\t" << verbose_ << std::endl;
  std::cout << "=================================================" << std::endl;
  return true;
}

bool GlobalPathManager::intentionChanged(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input)
{
  if (motion_ != Motion::NAVIGATION) return false;

  ROS_INFO("[GlobalPathManager::intentionChanged] updating...");

  // start timer
  auto t_start = Clock::now();

  bool success = false;
  // Decide what to do based on the type of model selected
  if (global_model_ == "single_straight_line")
  {
    // HACK: mode for teleoperating just a single global path. The gist is that it just continues in a line forever.
    if (!first_input_ ) return true;

    // straight line
    Eigen::Vector4d hack_input = Eigen::Vector4d(mapped_input(0), 0, 0, 0);

    FlatState hack_state;
    hack_state.pos = ref_state.pos;
    hack_state.yaw = ref_state.yaw;

    // must set current_input_ for replanning.
    current_input_ = hack_input;

    success = replanGlobalPath(hack_state, ref_time, hack_input, global_traj_duration_param_);

    first_input_ = false;
  }
  else if (global_model_ == "direct_motion_primitive")
  {
    FlatState hack_state;
    hack_state.pos = ref_state.pos;
    hack_state.vel = ref_state.vel;
    hack_state.acc = ref_state.acc;
    hack_state.yaw = ref_state.yaw;

    // Generate a single guiding motion primitive for the kinodynamic plan
    // TODO: currently scaling it by a fixed number N of the already mapped input. This is to circumvent the problem with increased duration, the trajectories dont look as nice and curved. Leave this for now.
    double vx = mapped_input(0)*4*cos(mapped_input(1));
    double vy = mapped_input(0)*4*sin(mapped_input(1));
    Eigen::Vector4d hack_input = Eigen::Vector4d(vx, 0, 0, vy);

    // must set current_input_ for replanning.
    current_input_ = hack_input;

    success = replanGlobalPath(hack_state, ref_time, hack_input, global_traj_duration_param_);
  }
  else if (global_model_ == "simple_filter")
  {
    // A simple low pass filter on previous inputs to update the global path.
    FlatState hack_state;
    hack_state.pos = ref_state.pos;
    hack_state.vel = ref_state.vel;
    hack_state.acc = ref_state.acc;
    hack_state.yaw = ref_state.yaw;

    // filtered_input_ = lambda_ * filtered_input_ + (1 - lambda_) * Eigen::Vector4d(value_from_linear_velocity_joystick_raw_, 0, 0, value_from_angular_velocity_joystick_raw_);

    ROS_INFO("[GlobalPathManager::intentChanged] filtered input: %.2f, %.2f, %.2f, %.2f", filtered_input_(0), filtered_input_(1), filtered_input_(2), filtered_input_(3));

    // must set current_input_ for replanning.
    current_input_ = filtered_input_;
    success = replanGlobalPath(hack_state, ref_time, filtered_input_, global_traj_duration_param_);
  }
  else
  {
    ROS_ERROR("[GlobalPathManager::intentChanged] Global models specified '%s' does not exist. Check planning.yaml for available models");
  }

  CodeTimer::record("[GlobalPathManager::intentChanged]", t_start);

  ROS_INFO("[GlobalPathManager::intentChanged] ...complete.");
  return success;
}


void GlobalPathManager::nearEndCallback(const ros::TimerEvent& event)
{
  if (motion_ != Motion::NAVIGATION) return;
  if (!teleop_enabled_) return;
  if (!global_path_info_->global_path_set_) return;

  FlatState closest_state = findClosestGlobalReference(global_path_info_->global_trajectory_, curr_pos_);

  double dist_to_goal = (closest_state.pos - global_path_info_->global_end_state_.pos).norm();

  // regenerate using the same input using the closest_state
  if (dist_to_goal <= global_replan_dist_)
  {
    auto t_start = Clock::now();
    ROS_INFO("[GlobalPathManager::nearEndCallback] Regenerate global reference...");

    ros::Time new_start_time = global_path_info_->global_traj_start_time_ + ros::Duration(closest_state.t);

    replanGlobalPath(closest_state, new_start_time, current_input_, global_traj_duration_param_);
    CodeTimer::record("[GlobalPathManager::nearEndCallback]", t_start);
    ROS_INFO("[GlobalPathManager::nearEndCallback] ...complete.");
  }

  return;
}

bool GlobalPathManager::replanGlobalPath(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input, double duration)
{
  if (motion_ != Motion::NAVIGATION) return false;
  global_path_info_->clear();

  ROS_INFO("[GlobalPathManager::replanGlobalPath] replanGlobalPath...");

  // mock a long trajectory with a long duration of 10s
  // TODO this has only position continuity
  auto mp = ForwardArcMotionPrimitives(ref_state, mapped_input, duration, yaw_rampup_duration_, true);

  vis_utils::visualizeLine(mp.samplePathEquidistant(0.05), vis_utils::grey_, 0, mp_vis_pub_);

  // Sample points
  static constexpr double ds = 0.01;
  global_path_info_->clear();
  global_path_info_->global_trajectory_ = mp;
  global_path_info_->global_path_ = mp.samplePathEquidistant(ds);
  global_path_info_->global_start_state_ = ref_state;
  global_path_info_->global_end_state_ = mp.getFinalWorldPose();
  global_path_info_->global_traj_duration_ = mp.duration();
  global_path_info_->global_traj_start_time_ = ref_time;
  global_path_info_->global_path_set_ = true;

  ROS_INFO("[GlobalPathManager::replanGlobalPath] ...complete.");
  return true;
}

FlatState GlobalPathManager::findClosestGlobalReference(const ForwardArcMotionPrimitives& mp, const Eigen::Vector3d& pos)
{
  std::vector<FlatState> path = mp.samplePathEquidistant(0.01);
  auto [idx, closest_state] = path_utils::findClosestPointAlongPath(path, pos);
  return closest_state;
}

void GlobalPathManager::rawJoystickCallback(const sensor_msgs::Joy& msg)
{
  // HACK: directly interfacing joystick inputs.
  value_from_linear_velocity_joystick_raw_ = msg.axes[1];
  value_from_angular_velocity_joystick_raw_ = msg.axes[3];

  if (global_model_ == "simple_filter")
  {
    filtered_input_ = lambda_ * filtered_input_ + (1 - lambda_) * Eigen::Vector4d(value_from_linear_velocity_joystick_raw_, 0, 0, value_from_angular_velocity_joystick_raw_);
  }
}


void GlobalPathManager::odomCallback(const nav_msgs::Odometry& msg)
{
  if (motion_ != Motion::NAVIGATION) return;
  if (!teleop_enabled_) return;
  if (!global_path_info_->global_path_set_) return;

  curr_pos_ = Eigen::Vector3d(msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z);

  return;
}



} // namespace planner
