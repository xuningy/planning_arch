#include <planning_arch/PlanManager.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_manager");
  ros::NodeHandle node("");
  ros::NodeHandle node_private("~");

  planner::PlanManager plan_manager;

  if (!plan_manager.initialize(node, node_private))
  {
    ROS_ERROR("%s: failed to initialize plan_manager",
        ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
