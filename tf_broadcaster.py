#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
import os
import io
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

ROTATE = 1


class MultiTfBroadcast:
    def __init__(self):
        rospy.init_node('broadcaster')

        # Load gg_values.txt from a local file path
        self.filepath = "/home/chart-admin/koyo_ws/graspnet/graspnet-baseline/output/gg_values.txt"

        # # Ensure the file exists before proceeding
        # if not os.path.exists(filepath):
        #     rospy.logerr("File gg_values.txt not found at {}".format(filepath))
        #     speak_text("gg_values.txt file not found. Please ensure the file exists.")
        #     rospy.signal_shutdown("gg_values.txt file missing")
        #     return

        self.broadcaster = tf.TransformBroadcaster()

        self._rate = rospy.Rate(10)


    def read_gg_values(self, filepath):
        print("Start reading gg values")
        with open(filepath, 'r') as file:
            lines = file.readlines()

        poses = {}
        translation = None
        rotation = []

        for i, line in enumerate(lines):
            if 'translation:' in line:
                translation_str = line.split('[')[1].split(']')[0]
                translation = [float(num) for num in translation_str.split()]
                poses['translation'] = translation
            elif 'rotation:' in line:
                rotation = []
                for j in range(3):
                    rotation_line = lines[i + 1 + j]
                    rotation_row = [float(num) for num in rotation_line.strip().strip('[]').split()]
                    rotation.append(rotation_row)
                poses['rotation'] = rotation

        return poses

    def broadcast_tf(self, translation, rotation_matrix):
        print("Start tf broadcasting")

        while not rospy.is_shutdown():
            # Using the translation values read from the file
            translation = tuple(translation)

            # Using the rotation matrix values read from the file
            rotation_matrix = np.array(rotation_matrix).reshape((3, 3))

            # Convert the rotation matrix to a quaternion
            quaternion_original = tf.transformations.quaternion_from_matrix(
                np.vstack((np.hstack((rotation_matrix, [[0], [0], [0]])), [0, 0, 0, 1]))
            )

            quaternion_x180 = tf.transformations.quaternion_about_axis(np.pi, (1, 0, 0))
            combined_quaternion_x = tf.transformations.quaternion_multiply(quaternion_original, quaternion_x180)

            # Quaternion for a 90 degree rotation around the y-axis
            quaternion_y90 = tf.transformations.quaternion_about_axis(np.pi / 2, (0, 1, 0))

            # Quaternion for a 90 degree rotation around the z-axis
            quaternion_z90 = tf.transformations.quaternion_about_axis(np.pi / 2, (0, 0, 1))

            # Combine the quaternions: first apply the rotation around y, then around z
            combined_quaternion_y = tf.transformations.quaternion_multiply(quaternion_original, quaternion_y90)
            combined_quaternion_z = tf.transformations.quaternion_multiply(combined_quaternion_y, quaternion_z90)


            if ROTATE:
                # grasp frame is located at a specific position and orientation relative to the depth camera's optical frame.
                self.broadcaster.sendTransform(translation, # These values do not change
                                               combined_quaternion_z, # These values do not change
                                               rospy.Time.now(), 
                                               'grasp', 
                                               'd435_depth_optical_frame'
                )
            else:
                self.broadcaster.sendTransform(translation, # These values do not change
                                               quaternion_original, # These values do not change
                                               rospy.Time.now(), 
                                               'grasp', 
                                               'd435_depth_optical_frame'
                )

            self._rate.sleep()

    def start(self):
        # Read translation and rotation matrix values from gg_values.txt
        self.poses = self.read_gg_values(self.filepath)
        print("Obtained pos from gg_values.txt: ", self.poses)
        self.broadcast_tf(self.poses['translation'], self.poses['rotation'])



def main():
    multi_tf_broadcast_obj = MultiTfBroadcast()
    try:
        multi_tf_broadcast_obj.start()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()


# The pose from graspnet, 'grasp', is in reference to 'd435_depth_optical_frame'
# But we need to know the 'grasp' frame in reference to 'base_link' for moveit grasp planning
# So we need broadcasting between 'base_link' and 'grasp'

# Here is my though
# 'grasp' is in reference to 'd435_depth_optical_frame'
# The 'grasp' value does not change, but it can be used as long as it referrs to 'd435_depth_optical_frame'
# However, during grasp planning, 'd435_depth_optical_frame' moves
# Therefore, we need to keep broadcasting between 'd435_depth_optical_frame' and 'grasp'
# , which is used to broadcast between 'base_link' and 'grasp' during grasp planning