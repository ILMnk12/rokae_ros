#!/usr/bin/env python3
from __future__ import print_function
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import numpy as np

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))

from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list


def all_close(goal, actual, tolerance):
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        
        d = dist((x1, y1, z1), (x0, y0, z0))
        
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPythonInterfaceTutorial(object):
    def __init__(self):
        # Initialization
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()

        scene = moveit_commander.PlanningSceneInterface()

        group_name = "rokae_arm"

        move_group = moveit_commander.MoveGroupCommander(group_name)

        move_group.set_planning_pipeline_id("ompl") # Set the planning algorithm
        move_group.set_planner_id("RRT") 

        move_group.set_max_velocity_scaling_factor(0.05)# Set the limitation of velocity and acceleration
        move_group.set_max_acceleration_scaling_factor(0.05)
        
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        planning_frame = move_group.get_planning_frame()

        eef_link = move_group.get_end_effector_link()

        group_names = robot.get_group_names()

        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_home_position(self):
        move_group = self.move_group
        move_group.set_joint_value_target([0, pi/6, 0, pi/3, 0, pi/2, 0])
        move_group.go()
        rospy.sleep(15)
    
    def go_to_joint_state(self,j1,j2=0,j3=0,j4=0,j5=0,j6=0):
        # move to a joint goal
        move_group = self.move_group

        move_group.set_planner_id("FMT")

        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] += j1
        joint_goal[1] += j2
        joint_goal[2] += j3
        joint_goal[3] += j4
        joint_goal[4] += j5
        joint_goal[5] += j6  

        move_group.set_joint_value_target(joint_goal)

        a = 'a' # Path planning, an appropriate plan could be chosen to execute manually
        while(a!='e'):
           plan_success, plan, planning_time, error_code = move_group.plan()
           a = input("Enter 'e' to execute, press enter to replan: \n ")

        print("Plan accepted, executing")

        move_group.execute(plan, wait = True)

        move_group.stop()

        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)
    
    def go_to_pose_goal(self, t_x, t_y, t_z):
        # go to a pose goal
        move_group = self.move_group

        move_group.set_planner_id("RRT")

        pose_goal = move_group.get_current_pose().pose 
        pose_goal.orientation.x = 0
        pose_goal.orientation.y = 1
        pose_goal.orientation.z = 0
        pose_goal.orientation.w = 0
        pose_goal.position.x = t_x
        pose_goal.position.y = t_y
        pose_goal.position.z = t_z

        move_group.set_pose_target(pose_goal)

        # Path planning, an appropriate plan could be chosen to execute manually if obstacle avoidance is needed
        # a = 'a'
        # while(a!='e'):
        #    plan_success, plan, planning_time, error_code = move_group.plan()
        #    a = input("Enter 'e' to execute, press enter to replan: \n ")
        # print("Plan accepted, executing")

        plan_success, plan, planning_time, error_code = move_group.plan() # If not, the generated path need not be selected

        rospy.sleep(3.0)

        move_group.execute(plan, wait = True)

        move_group.stop()

        move_group.clear_pose_targets()

        current_pose = self.move_group.get_current_pose().pose

        return all_close(pose_goal, current_pose, 0.01)

    def plan_cartesian_path(self, scale=1):
        # Plan a path with several waypoints
        move_group = self.move_group

        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.x = 0.3
        wpose.position.y = 0.3
        wpose.position.z = 0.6
        waypoints.append(copy.deepcopy(wpose))
        wpose.position.x = 0.563
        wpose.position.y = 0.0
        wpose.position.z = 0.4324
        waypoints.append(copy.deepcopy(wpose))

        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  
        )  

        return plan, fraction

    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        
        display_trajectory_publisher.publish(display_trajectory)

    def execute_plan(self, plan):
        move_group = self.move_group

        move_group.execute(plan, wait=True)

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):

        box_name = self.box_name
        scene = self.scene

        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():

            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            is_known = box_name in scene.get_known_object_names()

            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            rospy.sleep(0.1)
            seconds = rospy.get_time()

        return False

    def add_box(self, bxname, lx, ly, lz, px, py, pz, timeout=4):
        # Add a box to the work space
        box_name = self.box_name
        scene = self.scene

        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = pz  
        box_pose.pose.position.y = py 
        box_pose.pose.position.x = px  
        box_name = bxname
        scene.add_box(box_name, box_pose, size=(lx, ly, lz))

        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Attach a object to the end effector of the robot
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        scene.attach_box(eef_link, "box2") 

        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Detach the object
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        scene.remove_attached_object(eef_link, name="box2")

        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, bx_name, timeout=4):
        # Remove the object
        box_name = self.box_name
        scene = self.scene

        scene.remove_world_object(bx_name)

        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )
    
    def wait_for_movement_finished(self):
        move_group = self.move_group

        thres = 0.01
        joint_change = 0
        prev_joint_position = move_group.get_current_joint_values()
        while(joint_change < thres):
            rospy.sleep(0.5) 
            current_joint_position = move_group.get_current_joint_values()
            joint_change = sum(abs(np.array(prev_joint_position) - np.array(current_joint_position)))
            prev_joint_position = current_joint_position   
        while(joint_change > thres):
            rospy.sleep(0.5) 
            current_joint_position = move_group.get_current_joint_values()
            joint_change = sum(abs(np.array(prev_joint_position) - np.array(current_joint_position)))
            prev_joint_position = current_joint_position
        


def main():
    try:
        tutorial = MoveGroupPythonInterfaceTutorial()

        # Go to home position; this is not needed when the current position of arm is not hugely different from home position
        # tutorial.go_to_home_position() 

        # Add an obstacle in the work space
        tutorial.add_box("box1", 0.25, 0.20, 0.5, 0.5, 0.325, 0.25)
        tutorial.add_box("box2", 0.05, 0.05, 0.05, 0.5, -0.3, 0.32)

        # Move Rokae arm to a pose goal with obstacle avoidance
        tutorial.go_to_pose_goal(0.5, -0.3, 0.35)
        # tutorial.wait_for_movement_finished()
        tutorial.attach_box()
       
        # Move Rokae arm to a joint state goal with obstacle avoidance
        tutorial.go_to_joint_state(140 * pi / 180)
        # tutorial.wait_for_movement_finished()

        # Detach the box
        tutorial.detach_box()

        # Remove all the box
        # tutorial.remove_box("box2")
        tutorial.remove_box("box1")
        rospy.sleep(3.0)

        # Plan a path which passes several waypoints, this function can not avoid obstacles for the moment
        cartesian_plan, fraction = tutorial.plan_cartesian_path()
        rospy.sleep(3.0)
        tutorial.display_trajectory(cartesian_plan)
        tutorial.execute_plan(cartesian_plan)





    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

