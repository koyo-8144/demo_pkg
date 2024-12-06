#!/usr/bin/env python3

import numpy as np
import threading
import tf
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import String  # Import the String message for communication



class GraspPlannerNode():
    def __init__(self):
        # Initialize parameters
        self.init_params()    

        # Initialize planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")

        # Set robot arm's speed and acceleration
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)

        # Targe grasp position
        # self.target_pose = PoseStamped()
        self.target_pose = PoseStamped()
        self.target_pose.pose.position.x = 0

        self.received_pose = False

        # Go to start position immediately upon initialization
        rospy.loginfo("Moving to start position")
        self.go_sp()

    def init_params(self):
        pass

    ####### lister for transformation between base_link and grasp #######

    # This is open-loop system so I do not need to run constantly
    def lister_callback(self):
        listener = tf.TransformListener()

        while not rospy.is_shutdown():
            try:
                # Lookup transform for the currently selected object
                # base_link → d435_depth_optical_frame → grasp
                (trans, rot) = listener.lookupTransform('base_link', '/grasp', rospy.Time(0))

                # Once we have the transformation, exit the loop
                print("Translation: ", trans)
                print("Rotation: ", rot)

                self.target_pose.pose.position.x = trans[0]
                self.target_pose.pose.position.y = trans[1]
                self.target_pose.pose.position.z = trans[2]

                self.target_pose.pose.orientation.x = rot[0]
                self.target_pose.pose.orientation.y = rot[1]
                self.target_pose.pose.orientation.z = rot[2]
                self.target_pose.pose.orientation.w = rot[3]

                if self.target_pose.pose.position.x != 0.0 and self.target_pose.pose.position.y != 0.0 and self.target_pose.pose.position.z != 0.0:
                    # Set the flag to indicate pose has been received
                    self.received_pose = True
                    rospy.loginfo("Pose received: {}".format(self.target_pose))
                    # Break the loop after getting the values
                    break  # Exits the loop after the first successful transformation retrieval

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue


    ####### grasp planning #######

    def start_grasp_planning(self):
        rospy.loginfo("Waiting for pose...")
        # Wait for pose to be received
        while not self.received_pose and not rospy.is_shutdown():
            rospy.sleep(0.1)

        rospy.loginfo("Pose received. Starting grasp planning...")
        self.execute_grasp_sequence()

    def execute_grasp_sequence(self):
        """
        Execute the grasp sequence as a separate method.
        This method waits for the object pose to be detected before proceeding.
        """
        rospy.loginfo('Starting grasp sequence')

        # self.go_sp()
        # self.gripper_move(0.6)

        # 1. Grasp the object
        self.grasp_obj()

        # 2. Read grasp width from gg_values.txt
        filepath = "/home/chart-admin/koyo_ws/langsam_grasp_ws/data/gg_values.txt"
        width = self.read_gg_values(filepath)

        # 3. Move gripper based on the grasp width
        self.gripper_move(3.6 * width)

        # 4. Place the object at the target location
        self.place_obj()
        
        self.go_sp()

        rospy.loginfo("Grasp completed successfully.")

    def grasp_obj(self):
        """
        Grasp object based on object detection.
        """
        rospy.loginfo("Grasping object at position: {}".format(self.target_pose.pose.position))

        # self.target_pose.pose.position.x -= 0.055
        # self.target_pose.pose.position.z += 0.05

        self.arm_group.set_pose_target(self.target_pose.pose)
        self.arm_group.go()
        self.plan_cartesian_path(self.target_pose.pose)
        # self.target_pose.pose.position.z += 0.05
        self.plan_cartesian_path(self.target_pose.pose)
        self.arm_group.set_pose_target(self.target_pose.pose)
        self.arm_group.go()

    def plan_cartesian_path(self, target_pose):
        """
        Cartesian path planning
        """
        waypoints = []
        waypoints.append(target_pose)  # Add the pose to the list of waypoints

        # Set robot arm's current state as start state
        self.arm_group.set_start_state_to_current_state()

        try:
            # Compute trajectory
            (plan, fraction) = self.arm_group.compute_cartesian_path(
                waypoints,   # List of waypoints
                0.01,        # eef_step (endpoint step)
                0.0,         # jump_threshold
                False        # avoid_collisions
            )

            if fraction < 1.0:
                rospy.logwarn("Cartesian path planning incomplete. Fraction: %f", fraction)

            # Execute the plan
            if plan:
                self.arm_group.execute(plan, wait=True)
            else:
                rospy.logerr("Failed to compute Cartesian path")
        except Exception as e:
            rospy.logerr("Error in Cartesian path planning: %s", str(e))

    def read_gg_values(self, filepath):
        with open(filepath, 'r') as file:
            lines = file.readlines()

        width = None
        for line in lines:
            if 'width:' in line:
                width = float(line.split('width:')[1].split(',')[0].strip())
                break  # Stop searching once width is found
        return width

    def place_obj(self):
        self.target_pose.pose.position.z += 0.07
        # self.plan_cartesian_path(self.target_pose.pose)

        self.arm_group.set_joint_value_target([-1.1923993012061151, 0.7290586635521652,
                                               -0.7288901499177471, 1.6194515338395425,
                                               -1.6699862200379725, 0.295133228129065])
        self.arm_group.go()

        self.gripper_move(0.6)

    def go_sp(self):
        self.arm_group.set_joint_value_target([-0.08498747219394814, -0.2794001977631106,
                                               0.7484180883797364, -1.570090066123494,
                                               -2.114137663337607, -1.6563429070772748])
        self.arm_group.go(wait=True)

    def gripper_move(self, width):
        gripper_joints_state = self.gripper_group.get_current_joint_values()
        gripper_joints_state[2] = width
        self.gripper_group.set_joint_value_target(gripper_joints_state)
        self.gripper_group.go()
 

def main():
    rospy.init_node('grasp_planning', anonymous=True)
    rospy.loginfo('Start Grasp Demo')
    grasp_planner_node = GraspPlannerNode()

    # Start the listener in a separate thread
    rospy.loginfo('Starting listener for grasp planning ...')
    listener_thread = threading.Thread(target=grasp_planner_node.lister_callback)
    listener_thread.start()

    grasp_planner_node.start_grasp_planning()
    rospy.spin()
 
if __name__ == "__main__":
    main()