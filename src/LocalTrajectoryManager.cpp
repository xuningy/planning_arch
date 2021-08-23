#include <planning_arch/LocalTrajectoryManager.h>

#include <cpp_utils/print_utils.h>
#include <codetimer_catkin/codetimer.h>

using Clock = std::chrono::high_resolution_clock;

namespace print = print_utils;

namespace planner {

bool LocalTrajectoryManager::initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private)
{
  ros::NodeHandle nh(n);
  ros::NodeHandle nh_private(n_private);

  param_utils::get("local/mode", mode_);
  param_utils::get("local/enable_direct_motion_primitives", direct_motion_primitive_enabled_, true);
  param_utils::get("local/replan_dist_threshold", local_replan_dist_threshold_, 1.0);
  param_utils::get("local/global_goal_dist_threshold", global_dist_threshold_, 1.0);
  param_utils::get("local/global_regen_cost_threshold", global_regen_cost_threshold_, 3.0);
  param_utils::get("local/near_end_check_rate", collision_check_rate_, 0.03);
  param_utils::get("local/near_end_check_rate", near_end_check_rate_, 0.01);
  param_utils::get("local/lookahead_mode", lookahead_mode_);
  if (lookahead_mode_ == "fixed")
  {
    param_utils::get("local/lookahead", local_lookahead_, 0.3);
  }

  param_utils::get("debug/vis_only", debug_vis_only_, false);
  param_utils::get("debug/verbose", verbose_, false);

  control_io_.initialize(n, nh);

  // Start timers
  near_end_timer_ = n.createTimer(ros::Duration(near_end_check_rate_),
  &LocalTrajectoryManager::nearEndCallback, this);

  collision_check_timer_ = n.createTimer(ros::Duration(collision_check_rate_),
  &LocalTrajectoryManager::collisionCheckCallback, this);

  // Initialize variables
  motion_ = Motion::ZERO;
  local_traj_info_ = std::make_shared<LocalTrajInfo>();

  // initialize Collision Checker
  collision_checker_ = std::make_shared<CollisionChecker>();
  if (!collision_checker_->initialize(n_private))
  {
    ROS_ERROR("[LocalTrajectoryManager] unable to initialize collision checker, exiting!");
    return false;
  }

  // Initialize LocalTrajectoryGenerator

  // local_trajectory_generator_ = std::make_shared<TopoLocalGenerator>();
  local_trajectory_generator_ = std::make_shared<MPTLocalGenerator>();

  // first initialize map
  local_trajectory_generator_->initializeMap(collision_checker_);

  // then initialize everything else
  if (!local_trajectory_generator_->initialize(n, n_private))
  {
    ROS_ERROR("[LocalTrajectoryManager] initialization of local trajectory generator failed!");
    return false;
  }

  // initialize stopping trajectory timer
  // stopping_traj.initialize(n);
  // stopping_traj_timer_ = n.createTimer(ros::Duration(0.03),
      // &LocalTrajectoryManager::callStoppingTrajChecker, this);

  // Setup additional publishers
  local_traj_vis_pub_ =
    nh.advertise<visualization_msgs::Marker>("vis_local_traj", 10);

  global_segment_vis_pub_ =
    nh.advertise<visualization_msgs::Marker>("vis_current_global_segment", 10);
  local_segment_vis_pub_ =
    nh.advertise<visualization_msgs::Marker>("vis_current_local_segment", 10);

   // print parameters

  std::cout << "=================================================" << std::endl;
  std::cout << "        LocalTrajectoryManager Parameters " << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Mode: \t\t\t" << mode_ << std::endl;
  std::cout << "Enable direct motion primitive: " << direct_motion_primitive_enabled_ << std::endl;
  std::cout << "---" << std::endl;
  std::cout << "replan_dist_threshold: \t\t" << local_replan_dist_threshold_ << " m" << std::endl;
  std::cout << "global_goal_dist_threshold: \t" << global_dist_threshold_ << " m" << std::endl;
  std::cout << "global_regen_cost_threshold: \t" << global_regen_cost_threshold_ << " m" << std::endl;
  std::cout << "collision_check_rate: \t\t" << collision_check_rate_ << " s\t"
                          << 1.0/collision_check_rate_ << " Hz" << std::endl;
  std::cout << "near_end_check_rate: \t\t" << near_end_check_rate_ << " s\t"
                          << 1.0/near_end_check_rate_ << " Hz" << std::endl;

  std::cout << "---" << std::endl;
  std::cout << "Lookahead mode: \t\t" << lookahead_mode_.c_str() << std::endl;
  if (lookahead_mode_ == "fixed")
    std::cout << "\tLookahead: \t\t" << local_lookahead_ << std::endl;

  // Debug parameters
  std::cout << "---" << std::endl;
  if (debug_vis_only_) std::cout << "Visualizing trajectories ONLY, no trajectories published to the control_arch!" << std::endl;
  std::cout << "verbose: \t\t" << verbose_ << std::endl;
  std::cout << "=================================================" << std::endl;

  return true;
}

bool LocalTrajectoryManager::replanLocalTrajectory(const Eigen::Vector4d& mapped_input)
{
  if (motion_ != Motion::NAVIGATION) return false;
  if (!teleop_enabled_) return false;

  ROS_INFO("[LocalTrajectoryManager::replanLocalTrajectory] begin...");

  // get the following values
  FlatState ref_state;
  ros::Time ref_time;

  // Check if a reference state is available for planning
  if (!control_io_.getControlReference(ref_state, ref_time, local_lookahead_))
  {
    ROS_ERROR("[LocalTrajectoryManager::replanLocalTrajectory] reference not found; trajectory not generated");
    return false;
  }

  bool success = false;

  /****************** Check of a single motion primitive is collision free, in which case directly execute!  ***************/

  if (direct_motion_primitive_enabled_)
  {
    auto t_mp = Clock::now();

    auto mp = ForwardArcMotionPrimitives(ref_state, mapped_input, 2, 0.3, true);

    constexpr static double ds = 0.01;
    std::vector<FlatState> states = mp.samplePathEquidistant(ds);
    // vis_utils::visualizeLine(states, vis_utils::red_, 0, local_traj_vis_pub_);

    float dist;
    bool safe = collision_checker_->checkTrajectorySafeKDTree(mp.samplePath(0.01), dist);

    if (safe)
    {
      ROS_WARN("[LocalTrajectoryManager::replanLocalTrajectory] Direct motion primitive is safe, publishing motion primitive with input (%.1f, %.1f, %.1f, %.1f)", mapped_input(0), mapped_input(1), mapped_input(2), mapped_input(3));

      // set motion primitive directly as path
      local_traj_info_->clear();
      local_traj_info_->local_path_ = mp.samplePath(0.01);
      local_traj_info_->local_traj_duration_ = mp.duration();
      local_traj_info_->local_start_state_ = ref_state;
      local_traj_info_->local_end_state_ = mp.getFinalWorldPose();
      local_traj_info_->local_traj_start_time_ = ref_time;
      local_traj_info_->local_trajectory_set_ = true;

      // immediately publish
      if (debug_vis_only_) return true;

      control_io_.publishTrajectory(local_traj_info_->local_traj_start_time_,
        local_traj_info_->local_path_,
        local_traj_info_->local_traj_duration_);

      CodeTimer::record("[LocalTrajectoryManager::replanLocalTrajectory] direct mp", t_mp);

      ROS_INFO("[LocalTrajectoryManager::replanLocalTrajectory] direct mp OK, publishing. Exiting replan.");

      // bypass local trajectory generator, as we are using motion primitive directly as the trajectory.
      return true;
    }
  }

  auto t_start = Clock::now();

  /************************* calling local trajectory generator ***************/
  constexpr static double local_traj_length_ = 5;


  // Set global and local traj segment for the planner

  // Set the global path segment
  if (!global_path_info_->global_path_.empty())
  {
    auto [global_idx, global_segment] =  path_utils::getSegmentWithLength(global_path_info_->global_path_, ref_state.pos, local_traj_length_);

    local_trajectory_generator_->setGlobalTrajectory(global_segment);

    // visualize the segment for debug purposes
    vis_utils::visualizeLine(global_segment, vis_utils::magenta_, 0, global_segment_vis_pub_);
  }

  // Set the local path segment
  if (!local_traj_info_->local_path_.empty())
  {
    auto [local_idx, local_segment] =  path_utils::getSegmentWithLength(local_traj_info_->local_path_, ref_state.pos, local_traj_length_);

    local_trajectory_generator_->setLocalTrajectory(local_segment);

    // visualize the segment for debug purposes
    vis_utils::visualizeLine(local_segment, vis_utils::cyan_, 0, local_segment_vis_pub_);
  }

  if(mapped_input.norm() == 0) {
    // control_io_.publishZeroTrajectory(ref_state, ref_time, stopping_trajectory_duration_);

    ROS_WARN("[LocalTrajectoryManager::replanLocalTrajectory] mapped_input.norm() is zero, calling stopping_traj.generateCollisionFreeWaypoints!");
    // stopping_traj.generateCollisionFreeWaypoints(ref_state, ref_time);
    return true;
  }

  // Call local trajectory generator.
  success = local_trajectory_generator_->generateTrajectory(ref_state, ref_time, mapped_input);

  // post-generation processing
  if (success) {
    // Publish the said trajectory
    auto traj = local_trajectory_generator_->getTrajectory();
    local_traj_info_->clear();
    local_traj_info_->local_path_ = traj;
    local_traj_info_->local_traj_duration_ = local_trajectory_generator_->getDuration();
    local_traj_info_->local_start_state_ = ref_state;
    local_traj_info_->local_end_state_ = traj.back();
    local_traj_info_->local_traj_start_time_ = ref_time;
    local_traj_info_->local_trajectory_set_ = true;
  }
  else
  {
    ROS_ERROR("[LocalTrajectoryManager::replanLocalTrajectory] did not find successful trajectory, retrying!");

    return false;
  }
  // else
  // {
  //   ROS_WARN("[LocalTrajectoryManager::replanLocalTrajectory] no successful trajectory found using motion primitive trees, regenerating using a single motion primitive!");
  //
  //   Eigen::Vector4d mapped_input(1, 0, 0, 0);
  //
  //   auto mp = ForwardArcMotionPrimitives(ref_state, mapped_input, 3, 0.3, true);
  //
  //   constexpr static double ds = 0.01;
  //   std::vector<FlatState> states = mp.samplePathEquidistant(ds);
  //   // vis_utils::visualizeLine(states, vis_utils::red_, 0, local_traj_vis_pub_);
  //
  //   local_traj_info_->clear();
  //   local_traj_info_->local_path_ = mp.samplePath(0.01);
  //   local_traj_info_->local_traj_duration_ = mp.duration();
  //   local_traj_info_->local_start_state_ = ref_state;
  //   local_traj_info_->local_end_state_ = mp.getFinalWorldPose();
  //   local_traj_info_->local_traj_start_time_ = ref_time;
  //   local_traj_info_->local_trajectory_set_ = true;
  // }

  /*******************end calling local planner*************************/

  std::chrono::duration<double> t_callback = Clock::now() - t_start;
  updateAvgComputeTime(t_callback);
  success = true;

  // If successful, publish new trajectory. else, do nothing
  if (success)
  {
    ROS_INFO("[LocalTrajectoryManager::replanLocalTrajectory] Found successful trajectory, publishing. Exiting replan.");

    if (debug_vis_only_) return true;
    control_io_.publishTrajectory(local_traj_info_->local_traj_start_time_,
      local_traj_info_->local_path_,
      local_traj_info_->local_traj_duration_);
  }
  // else
  // {
    // ROS_ERROR("[LocalTrajectoryManager::replanLocalTrajectory] did not find successful trajectory, retrying!");
  // }

  CodeTimer::record("[LocalTrajectoryManager::replanLocalTrajectory] local trajectory generator invoked", t_start);

  return true;
}

void LocalTrajectoryManager::collisionCheckCallback(const ros::TimerEvent& event)
{
  // Check collision for the current trajectory
  if (motion_ != Motion::NAVIGATION) return;
  if (!local_traj_info_->local_trajectory_set_) return;
  if (!teleop_enabled_) return;

  float dist;

  bool safe = collision_checker_->checkTrajectorySafeKDTree(local_traj_info_->local_path_, dist);
  if (!safe) {
    auto t_start = Clock::now();

    ROS_WARN("[LocalTrajectoryManager::collisionCheckCallback] Current trajectory unsafe; dist to collision: %.3f, replanning!", dist);

    bool success = replanLocalTrajectory();

    CodeTimer::record("[LocalTrajectoryManager::collisionCheckCallback] collision detected", t_start);

  }

  return;
}

void LocalTrajectoryManager::nearEndCallback(const ros::TimerEvent& event)
{
  // Check collision for the current trajectory
  if (motion_ != Motion::NAVIGATION) return;
  if (!local_traj_info_->local_trajectory_set_) return;
  if (!global_path_info_->global_path_set_) return;
  if (!teleop_enabled_) return;

  // get the current pose
  FlatState curr_state;
  ros::Time curr_time;

  // Check if a reference state is available for planning
  if (!control_io_.getControlReference(curr_state, curr_time, 0))
  {
    ROS_ERROR("reference not found!");
    return;
  }

  // TODO get current pos from motion manager
  Eigen::Vector3d cur_pos = curr_state.pos;

  // get dist to global goal
  double dist_to_global_end = (cur_pos - global_path_info_->global_end_state_.pos).norm();

  // get dist to local goal
  double dist_to_local_end = (cur_pos - local_traj_info_->local_end_state_.pos).norm();

  if (verbose_)
  {
    ROS_INFO("[LocalTrajectoryManager::nearEndCallback] dist to global goal: %.3f dist to local end: %.3f", dist_to_global_end, dist_to_local_end);
  }

  // Check if near global goal
  if (dist_to_global_end < global_dist_threshold_)
  {
    ROS_WARN("[LocalTrajectoryManager::nearEndCallback] close to global goal (%.3f), stopping!", dist_to_global_end);
    // global_trajectory_set_ = false;
    return;
  }

  // check if near local replan time
  if (dist_to_local_end < local_replan_dist_threshold_)
  {
    ROS_WARN("[LocalTrajectoryManager::nearEndCallback] dist to local end (%.3f) is less than allowable local replan distance (%.4f), replanning!", dist_to_local_end, local_replan_dist_threshold_);

    auto t_start = Clock::now();

    // TODO Replan
    bool success = replanLocalTrajectory();

    CodeTimer::record("[LocalTrajectoryManager::nearEndCallback] end reached", t_start);
  }

  return;
}

// this function is called by plan_manager when joystick input has changed.
// global plan's function intentionChanged() will be called prior to calling this function.
// Intention and the global path may be changed prior to this function call.
void LocalTrajectoryManager::joystickChanged(const Eigen::Vector4d& mapped_input)
{
  if (motion_ != Motion::NAVIGATION) return;
  if (!global_path_info_->global_path_set_) return;
  if (!teleop_enabled_) return;
  ROS_INFO("[LocalTrajectoryManager::joystickChanged] begin evaluation...");

  // Since the shared ptr is linked, the data should be updated. Here we should just deal with the fact that the global path been changed and evaluate if a new trajectory should be generated.
  //TODO: check if a new trajectory needs to be generated instead of directly calling replan.
  // allow two modes here:
  // 1. allow input to be directly passed to replanLocalTrajectory.
  // 2. allow evaluation of the current global plan and see if anything needs updating.

  auto t_start = Clock::now();

  // First, evaluate if a new trajectory needs to be generated.

  bool regeneration = false;

  constexpr static double length_to_evaluate = 3.0;

  if (direct_motion_primitive_enabled_)
  {
    regeneration = true;
  }
  else if (!global_path_info_->global_path_.empty() &&
      !local_traj_info_->local_path_.empty())
  {

    // get the following values
    FlatState ref_state;
    ros::Time ref_time;

    // Check if a reference state is available for planning
    if (!control_io_.getControlReference(ref_state, ref_time, local_lookahead_))
    {
      ROS_ERROR("[LocalTrajectoryManager::joystickChanged] reference not found; not able to compare trajectories.");
      return;
    }

    // find the local segment for the next 5s, or whatever length available:
    auto [local_duration, local_segment] =  path_utils::getSegmentWithLength(local_traj_info_->local_path_, ref_state.pos, length_to_evaluate );

    // use the same duration for the global path
    auto [global_duration, global_segment] =  path_utils::getSegmentWithLength(global_path_info_->global_path_, ref_state.pos, length_to_evaluate );

    auto cost = traj_evaluation::Closeness(global_segment, local_segment);

    std::cout << "regeneration evaluation: cost is " << cost << std::endl;
    // check bound on cost
    if (cost > global_regen_cost_threshold_) regeneration = true;
  }
  else if (!global_path_info_->global_path_.empty() &&
            local_traj_info_->local_path_.empty())
  {
    regeneration = true;
  }

  if (regeneration)
  {
    bool success = replanLocalTrajectory(mapped_input);
    last_executed_mapped_input_ = mapped_input;

    CodeTimer::record("[LocalTrajectoryManager::joystickChanged] joystick regeneration called", t_start);
  }

  ROS_INFO("[LocalTrajectoryManager::joystickChanged] end.");

  return;
}


void LocalTrajectoryManager::updateAvgComputeTime(const std::chrono::duration<double>& t_callback)
{
  // Compute average compute time depending on the compute time.
  avg_traj_compute_time_ = (avg_traj_compute_time_ * num_traj_callbacks_ + t_callback.count()) / (num_traj_callbacks_ + 1);
  num_traj_callbacks_++;

  // Update lookahead time (dynamically computed based on each function call)
  if (lookahead_mode_ == "dynamic") local_lookahead_ = avg_traj_compute_time_;
}

// void LocalTrajectoryManager::callStoppingTrajChecker(const ros::TimerEvent& event)
// {
  // stopping_traj.commandStop(event);
// }

} // namespace planner
