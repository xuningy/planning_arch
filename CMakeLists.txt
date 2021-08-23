cmake_minimum_required(VERSION 3.1)
project(planning_arch)

set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
      roscpp
      codetimer_catkin
      collision_checker
      cpp_utils
      forward_arc_primitives
      joystick_handler
      motion_primitive_tree
      trajectory_utils
      # stopping_trajectory
      ros_utils
			planning_control_interface
)

catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME}
      CATKIN_DEPENDS
        roscpp
        codetimer_catkin
        collision_checker
        cpp_utils
        forward_arc_primitives
        joystick_handler
        motion_primitive_tree
        trajectory_utils
        stopping_trajectory
        ros_utils
				planning_control_interface
      DEPENDS
)

include_directories(include
      ${codetimer_catkin_INCLUDE_DIRS}
      ${catkin_INCLUDE_DIRS}
      # ${kdtree_INCLUDE_DIRS}
)

link_directories(
      ${codetimer_catkin_LIBRARY_DIRS}
      ${catkin_LIBRARY_DIRS}
      # ${kdtree_INCLUDE_DIRS}
)

add_definitions(
      ${catkin_DEFINITIONS}
)

add_library(${PROJECT_NAME}
      src/GeneratePlan.cpp
      src/GlobalPathManager.cpp
      src/JoystickHandling.cpp
      src/LocalTrajectoryManager.cpp
      src/PlanManager.cpp
      # src/local_trajectory_generator/TopoLocalGenerator.cpp
      src/local_trajectory_generator/MPTLocalGenerator.cpp
)

target_link_libraries(${PROJECT_NAME}
      ${catkin_LIBRARIES}
      # ${kdtree_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}
	${catkin_EXPORTED_TARGETS}
)

# message(WARNING "planning_arch " ${PROJECT_NAME})
# message(WARNING "catkin_LIBRARIES " ${catkin_LIBRARIES})
# message(WARNING "catkin_LIBRARY_DIRS " ${catkin_LIBRARY_DIRS})

add_executable(planning_node
      src/planning_node.cpp
)

set_target_properties(planning_node
    PROPERTIES OUTPUT_NAME ${PROJECT_NAME}
)

target_link_libraries(planning_node
      ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} planning_node
      ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
      RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/{PROJECT_NAME}/
      DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
      FILES_MATCHING PATTERN "*.h"
)
