#!/usr/bin/env python3
import rospy
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose

CONSTANT_TARGET_UPDATE = 1

class GraspPlannerNode():
    def __init__(self):
        # Initialize planning group
        self.robot = moveit_commander.robot.RobotCommander()
        self.arm_group = moveit_commander.move_group.MoveGroupCommander("arm")
        self.gripper_group = moveit_commander.move_group.MoveGroupCommander("gripper")

        # Set robot arm's speed and acceleration
        self.arm_group.set_max_acceleration_scaling_factor(1)
        self.arm_group.set_max_velocity_scaling_factor(1)

        # self.listener = tf.TransformListener()
        # # We add this sleep to allow the listener time to be initialised
        # # Otherwise the lookupTransform could give not found frame issue.
        # time.sleep(1)

        # Subscribe to the object position topic
        self.obj_pose_sub = rospy.Subscriber("/object_position_pose", Pose, self.pose_callback)
        # Targe grasp position
        self.target_pose = PoseStamped()
        self.target_pose_update = True


    ####### pose_callback from listener #######

    def pose_callback(self, msg):
        """
        This function gets called whenever a message is received on /objection_position_pose topic.
        It updates the object pose and sets find_enable to False indicating that the object was found.
        """
        if self.target_pose_update:
            self.target_pose = msg
        

    ####### grasp planning #######

    def start_grasp_planning(self):
        rospy.loginfo("Starting grasp planning...")
        rospy.loginfo("Moving to start position")
        self.go_sp()
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
        width = self.read_gripper_width(filepath)

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

        if CONSTANT_TARGET_UPDATE:
            self.target_pose_update = True
        else:
            self.target_pose_update = False

        self.arm_group.set_pose_target(self.target_pose.pose)
        self.arm_group.go()
        # self.plan_cartesian_path(self.target_pose.pose)
        # self.target_pose.pose.position.z += 0.05
        # self.plan_cartesian_path(self.target_pose.pose)
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

    def read_gripper_width(self, filepath):
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
    grasp_planner_node = GraspPlannerNode()
    grasp_planner_node.start_grasp_planning()
    rospy.spin()
 
if __name__ == "__main__":
    main()