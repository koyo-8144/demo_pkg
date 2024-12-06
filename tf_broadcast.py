#!/usr/bin/env python
#-*- coding: utf-8 -*-
import rospy
import tf
import numpy as np
import os
import io
import speech_recognition as sr
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped

# def speak_text(text):
#     try:
#         command = 'espeak "{}"'.format(text)
#         rospy.loginfo("Executing command: %s", command)  # Log the espeak command
#         os.system(command)
#     except Exception as e:
#         rospy.logerr("Error with TTS engine: {}".format(e))  # Log the error


class StaticTfBroadcaster:
    def __init__(self):
        rospy.init_node('static_tf2_broadcaster')

        # Publisher to send grasp command to another node
        self.grasp_pub = rospy.Publisher('/grasp_command', String, queue_size=10)

        # Load gg_values.txt from a local file path
        filepath = "/home/chart-admin/koyo_ws/langsam_grasp_ws/data/gg_values.txt"

        # # Ensure the file exists before proceeding
        # if not os.path.exists(filepath):
        #     rospy.logerr("File gg_values.txt not found at {}".format(filepath))
        #     speak_text("gg_values.txt file not found. Please ensure the file exists.")
        #     rospy.signal_shutdown("gg_values.txt file missing")
        #     return

        self.broadcaster = tf.TransformBroadcaster()

        # Read translation and rotation matrix values from gg_values.txt
        self.poses = self.read_gg_values(filepath)

        print("Obtained pos from gg_values.txt: ", self.poses)

        # Start grasping loop
        self.broadcast_static_tf(self.poses['translation'], self.poses['rotation'])

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


    def broadcast_static_tf(self, translation, rotation_matrix):
        print("Start broadcasing static tf")
        broadcaster = tf.TransformBroadcaster()
        rate = rospy.Rate(10)  # 10 Hz

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

            # Broadcasting the transform
            # transformation from camera to grasp
            # grasp frame is located at a specific position and orientation relative to the depth camera's optical frame.
            broadcaster.sendTransform(
                translation, combined_quaternion_z, rospy.Time.now(), 'grasp', 'd435_depth_optical_frame'
            )

if __name__ == '__main__':
    broadcaster = StaticTfBroadcaster()
    rospy.spin()