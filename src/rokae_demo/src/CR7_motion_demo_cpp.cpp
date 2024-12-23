#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <iostream>
#include <tf/LinearMath/Quaternion.h>

/// @brief 
/// @param argc 
/// @param argv 
/// @return 


double total_angle_change (std::vector<double> j1, std::vector<double> j2){
  int i;
  double sum = 0;
  for(i=0; i<5; i++){
    sum = sum + abs(j1[i] - j2[i]);
  }
  return sum;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "MoveGroupInterface_To_Noetic");\

  ros::NodeHandle node_handle;

  ros::AsyncSpinner spinner(1);

  spinner.start();

  static const std::string PLANNING_GROUP = "motionplanning";

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

  move_group.setPlanningPipelineId("ompl");

  move_group.setPlannerId("FMT");

  move_group.setMaxVelocityScalingFactor(0.1);

  move_group.setMaxAccelerationScalingFactor(0.1);

  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);

  moveit_msgs::DisplayTrajectory display_trajectory;

  const robot_state::JointModelGroup* joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

  // Go to home position; these are not needed when the current position of arm is not hugely different from home position
  // std::vector<double> home_pose = {0, M_PI/6, -M_PI/2, 0, -M_PI/3, 0};
  // move_group.setJointValueTarget(home_pose);
  // move_group.move();
  // sleep(15);

  // Define the workspace
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  
  // Define a Obstacle
  moveit_msgs::CollisionObject obstacle;
  obstacle.header.frame_id = "world";
  obstacle.id = "box1";
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.25;
  primitive.dimensions[primitive.BOX_Y] = 0.20;
  primitive.dimensions[primitive.BOX_Z] = 0.5;
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.5;
  box_pose.position.y = 0.325;
  box_pose.position.z = 0.25;
  obstacle.primitives.push_back(primitive);
  obstacle.primitive_poses.push_back(box_pose);
  obstacle.operation = obstacle.ADD;

  // Define a manipulated object 
  moveit_msgs::CollisionObject grip_object;
  grip_object.header.frame_id = "world";
  grip_object.id = "box2";
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = 0.05;
  primitive.dimensions[primitive.BOX_Y] = 0.05;
  primitive.dimensions[primitive.BOX_Z] = 0.05;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.7;
  box_pose.position.y = -0.2;
  box_pose.position.z = 0.31;
  grip_object.primitives.push_back(primitive);
  grip_object.primitive_poses.push_back(box_pose);
  grip_object.operation = grip_object.ADD;

  // Add these two objects to the workspace
  collision_objects.push_back(obstacle);
  collision_objects.push_back(grip_object);
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Define a pose goal
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 0;
  target_pose1.orientation.x=  0;
  target_pose1.orientation.y = 1;
  target_pose1.orientation.z = 0;
  target_pose1.position.x = 0.70;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.35;
  move_group.setPoseTarget(target_pose1);
 
  // Move the end effector to a pose position
  moveit::planning_interface::MoveGroupInterface::Plan plan1;
  move_group.plan(plan1);
  sleep(3);
  move_group.execute(plan1);
  move_group.stop();
  move_group.clearPoseTargets();

  // Wait until the robot completes previous movement
  // double joint_change = 0.0;
  // double thres = 0.001;
  // joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // std::vector<double> prev_joint_position, current_joint_position;
  // moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  // current_state->copyJointGroupPositions(joint_model_group, prev_joint_position);
  // while(joint_change < thres){
  //   sleep(1);
  //   current_state = move_group.getCurrentState();
  //   current_state->copyJointGroupPositions(joint_model_group, current_joint_position);
  //   joint_change = total_angle_change(current_joint_position, prev_joint_position);
  //   prev_joint_position = current_joint_position;
  // }
  // while(joint_change > thres){
  //   sleep(1);
  //   current_state = move_group.getCurrentState();
  //   current_state->copyJointGroupPositions(joint_model_group, current_joint_position);
  //   joint_change = total_angle_change(current_joint_position, prev_joint_position);
  //   prev_joint_position = current_joint_position;
  // }

  // attach the object to the end effector of the robot
  move_group.attachObject(grip_object.id, move_group.getEndEffectorLink());

  // Move the robot to a joint goal position
  move_group.setPlannerId("FMT");
  std::vector<double> current_joint_position;
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
  current_state->copyJointGroupPositions(joint_model_group, current_joint_position);
  std::vector<double> goal_position = current_joint_position;
  goal_position[0] += 110 * M_PI / 180;
  move_group.setJointValueTarget(goal_position);
  moveit::planning_interface::MoveGroupInterface::Plan plan2;
  char action2 = 'a';
  while(action2 != 'e'){
    move_group.plan(plan2);
    printf("Enter 'e' to execute, press enter to replan: \n ");
    scanf("%c", &action2);
  }
  printf("Plan accepted, executing \n");
  move_group.execute(plan2);
  move_group.stop();

  // Wait until the robot completes previous movement
  // joint_change = 0.0;
  // joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  // current_state = move_group.getCurrentState();
  // current_state->copyJointGroupPositions(joint_model_group, prev_joint_position);
  // while(joint_change < thres){
  //   sleep(1);
  //   current_state = move_group.getCurrentState();
  //   current_state->copyJointGroupPositions(joint_model_group, current_joint_position);
  //   joint_change = total_angle_change(current_joint_position, prev_joint_position);
  //   prev_joint_position = current_joint_position;
  // }
  // while(joint_change > thres){
  //   sleep(1);
  //   current_state = move_group.getCurrentState();
  //   current_state->copyJointGroupPositions(joint_model_group, current_joint_position);
  //   joint_change = total_angle_change(current_joint_position, prev_joint_position);
  //   prev_joint_position = current_joint_position;
  // }

  // detach object
  move_group.detachObject(grip_object.id);
  sleep(1.5);

  // remove obstacle
  std::vector<std::string> object_ids;
  object_ids.push_back(obstacle.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  sleep(3);

  // Path planning with several waypoints
  std::vector<geometry_msgs::Pose> waypoints;
  geometry_msgs::Pose waypoint1;
  waypoint1.position.x = 0.3;
  waypoint1.position.y = 0.3;
  waypoint1.position.z = 0.6;
  waypoint1.orientation.y = 1;
  waypoints.push_back(waypoint1);
  waypoint1.position.x = 0.56;
  waypoint1.position.y = -0.15;
  waypoint1.position.z = 0.41;
  waypoints.push_back(waypoint1);

  moveit_msgs::RobotTrajectory trajectory;

  double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0, trajectory);

  sleep(3.0);

  move_group.execute(trajectory);
  
  sleep(5.0);
 
  ros::shutdown();

  return 0;
  
}
