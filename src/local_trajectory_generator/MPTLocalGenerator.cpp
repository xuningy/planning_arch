#include <planning_arch/local_trajectory_generator/MPTLocalGenerator.h>

using Clock = std::chrono::high_resolution_clock;

namespace print = print_utils;

namespace planner {

bool MPTLocalGenerator::initialize(const ros::NodeHandle& n, const ros::NodeHandle& n_private)
{

  std::cout << "initializing MPT Local generator" << std::endl;
  ros::NodeHandle nh(n);
  ros::NodeHandle nh_private(n_private);

    if (!param_utils::get("teleoperation/max_yaw_rate", max_yaw_rate_)) return false;
    param_utils::get("teleoperation/trajectory_duration", tf_, 1.0);
    param_utils::get("teleoperation/yaw_rampup_duration", yaw_rampup_duration_, 0.5);
    if (!param_utils::get("teleoperation/max_z_velocity", z_bound_)) return false;
    if (!param_utils::get("teleoperation/max_angular_velocity", omega_bound_))
      return false;
    param_utils::get("teleoperation/max_forward_velocity", max_forward_velocity_, 0.5);

    if (!param_utils::get("motion_primitives/discretization_mode", discretization_mode_)) return false;
    if (discretization_mode_ == "manual") {
      if (!param_utils::get("motion_primitives/num_z", num_z_)) return false;
      if (!param_utils::get("motion_primitives/num_omega", num_omega_)) return false;
      if (!param_utils::get("motion_primitives/num_lin_vel", num_lin_vel_)) return false;
    } else {
      double discretization = 0.3;
      if (discretization_mode_ == "ultra-fine") {
        discretization = 0.01;
      } else if (discretization_mode_ == "fine") {
        discretization = 0.05;
      } else if (discretization_mode_ == "medium") {
        discretization = 0.15;
      } else if (discretization_mode_ == "coarse") {
        discretization = 0.3;
      }
      num_omega_ = std::ceil( (omega_bound_ * 2) / discretization);
      num_z_ = std::ceil( (z_bound_ * 2 ) / discretization);
      num_lin_vel_ = std::ceil( (max_forward_velocity_ * 2 ) / discretization);

    }

    // Ensure that the discretizations entered are odd

    num_omega_ = (num_omega_%2 == 0) ? num_omega_+1 : num_omega_;
    num_z_ = (num_z_%2 == 0) ? num_z_+1 : num_z_;
    num_lin_vel_ = (num_lin_vel_ == 0) ? num_lin_vel_ + 1 : num_lin_vel_;

    // Generate the underlying array of of finely discretized motion primitives

    if (num_omega_ == 1) {
      omega_range_ = lu::Linspace<double>(0.0, 0.0, 1);
    }
    else {
      omega_range_ = lu::Linspace<double>(-omega_bound_, omega_bound_, num_omega_);
    }

    if (num_z_ == 1) {
      z_vel_range_ = lu::Linspace<double>(0.0, 0.0, 1);
    }
    else {
      z_vel_range_ = lu::Linspace<double>(-z_bound_, z_bound_, num_z_);
    }

    if (num_lin_vel_ == 1) {
      x_vel_range_ = lu::Linspace<double>(max_forward_velocity_,
        max_forward_velocity_, 1);
      speed_range_ = lu::Linspace<double>(max_forward_velocity_,
        max_forward_velocity_, 1);
    }
    else {
      x_vel_range_ = lu::Linspace<double>(-max_forward_velocity_,
        max_forward_velocity_, num_lin_vel_);
      // speed_range_ = lu::Linspace<double>(0, max_forward_velocity_,
      //     num_lin_vel_ * 0.5 + 1);
      speed_range_ = lu::Linspace<double>(0, max_forward_velocity_,
          num_lin_vel_);
    }

    param_utils::get("active_replan/policy", policy_);

    auto duration_range = lu::Linspace<double>(0.6, tf_, 5);

    // initialize tree
    tree_.initialize(n_private, speed_range_, omega_range_, z_vel_range_, duration_range, collision_checker_);


    param_utils::get("record", record_, false);
    if (record_) {
      std::string saveto_directory;
      param_utils::get("saveto_directory", saveto_directory);
      std::string saveto_folder = saveto_directory + io_utils::DateTime();
      tree_.setUpFileWriting(saveto_folder);
    }

    // {
    //   ROS_ERROR("[MPTLocalGenerator] motion primitive tree object not initialized, exiting!");
    //   return false;
    // }

    mpl_vis_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("moprim_library_vis", 1);
    endpoints_vis_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("endpoints_vis", 1);
    mp_vis_pub_ =
      nh.advertise<visualization_msgs::Marker>("moprim_vis", 1);

  // Setup additional publishers
  local_traj_vis_pub_ =  nh.advertise<visualization_msgs::Marker>("vis_local_traj", 10);


  std::cout << "=================================================" << std::endl;
  std::cout << "        MPTLocalGenerator Parameters " << std::endl;
  std::cout << "=================================================" << std::endl;
  std::cout << "Max forward velocity: \t\t" << max_forward_velocity_ << " m/s"<< std::endl;
  std::cout << "Max angular velocity: \t\t" << omega_bound_ << " rad/s"<< std::endl;
  std::cout << "Max pure yaw rate: \t\t" << max_yaw_rate_ << " rad/s"<< std::endl;
  std::cout << "Trajectory duration: \t\t" << tf_ << " s" << std::endl;
  std::cout << "Yaw ramp-up duration: \t\t" <<yaw_rampup_duration_ << " s" << std::endl;
  std::cout << "Discretization mode: \t\t" << discretization_mode_ << std::endl;
  std::cout << "Number of primitives: \t" << num_omega_*num_z_*num_lin_vel_ << std::endl;
  std::cout << "     linear velocity: \t" << num_lin_vel_ << std::endl;
  std::cout << "    angular velocity: \t" << num_omega_ << std::endl;
  std::cout << "          z velocity: \t" << num_z_ << std::endl;
  std::cout << "=================================================" << std::endl;

  return true;
}

bool MPTLocalGenerator::generateTrajectory(const FlatState& ref_state, const ros::Time& ref_time, const Eigen::Vector4d& mapped_input)
{
  bool success = false;

  // ***********************calling local planner****************************//

  tree_.clear();

  std::deque<ForwardArcMotionPrimitives> traj;
  std::vector<std::deque<ForwardArcMotionPrimitives>> trajectories;

  auto t_start = Clock::now();

  // Depending if we're in autonomous mode vs teleoperation, build the tree differently.
  if (policy_ == "autonomous_forward") {
    // Build tree & add timing.

    std::cout << "Building tree for autonomous forward" << std::endl;
    if (!tree_.biasedTree(ref_state)) {
      std::cout << "No feasible trajectory found -- stuck." << std::endl;
      return false;
    }

    // Update tree compute time
    CodeTimer::record("[MotionPrimitiveTreeTeleop] trajectory generation", t_start);

    auto t_policy = Clock::now();
    trajectories = tree_.getAllTrajectories(ref_state);

    if (trajectories.empty()) return true;

    // Publish a random trajectory
    traj = tree_.getMinCostTrajectory(ref_state);
    success = true;

    CodeTimer::record("[MotionPrimitiveTreeTeleop] trajectory_selection", t_policy);

  } // end if policy == "autonomous_forward"
  else if (policy_ == "joystick")
  {
    std::cout << "Building tree for joystick" << std::endl;

    // current_input_ = joy_input_;
    // auto ref_input = current_input_;
    // ref_input(1) = 0.6*ref_input(1); // Just a small buffer to tone down the spread

    // Visualize the cost trajectory
    // ForwardArcMotionPrimitives cost_primitive(ref_state, mapped_input, tf_, yaw_rampup_duration_, true);

    // visualization_.visualizePrimitive(cost_primitive, visualization_.yellow_, mp_vis_pub_);


    if (!tree_.biasedTree(ref_state, mapped_input)) {
      std::cout << "No feasible trajectory found -- stuck." << std::endl;
      return false;
    }

    // Update tree compute time
    CodeTimer::record("[MotionPrimitiveTreeTeleop] tree: generation", t_start);

    auto t_policy = Clock::now();

    // First, build a tree
    trajectories = tree_.getAllTrajectories(ref_state);

    if (trajectories.empty()) return true;

    success = true;

    // Pick the trajectory that's the closest to the operator's input.
    // traj = tree_.getMinCostTrajectory(ref_state);

    // or, Pick a trajectory that is lowest cost to the global and local trajectories.
    traj = tree_.pickBestGlobalLocalTrajectory(trajectories, current_global_segment_, previous_local_segment_);

    CodeTimer::record("[MotionPrimitiveTreeTeleop] tree: selection", t_policy);

  } // end if policy == "joystick"

  CodeTimer::record("[MotionPrimitiveTreeTeleop] tree: generation + selection", t_start);

  // Visualize all the trajectories
  visualization_.clearMarker(mp_vis_pub_);
  visualization_.clearMarkerArray(mpl_vis_pub_);

  marker_array_.markers.clear();
  endpoints_array_.markers.clear();

  unsigned int i = 0;
  for (auto& tr : trajectories) {
    visualization_msgs::Marker endpoint_marker;
    marker_array_.markers.push_back(visualization_.visualizeMultiPrimitive(tr, i, visualization_.purple_, endpoint_marker));
    endpoints_array_.markers.push_back(endpoint_marker);
    i++;
  }

  mpl_vis_pub_.publish(marker_array_);
  endpoints_vis_pub_.publish(endpoints_array_);

  // Visualize the selected trajectory
  visualization_.visualizeMultiPrimitive(traj, visualization_.yellow_, mp_vis_pub_);

  if (success)
  {
    // successful, set class variable to be executed by the local trajectory manager.

    // Compute duration of the entire set of waypoints.
    traj_duration_ = 0;
    for (auto &mp : traj) traj_duration_ += mp.duration();

    // Set final trajectory
    final_trajectory_ = traj; // this is the std::deque<FAMP>
    trajectory_ = forward_arc_primitive_trajectory::samplePath(traj, 0.05);

  }

  return success;
}


} // namespace planner
