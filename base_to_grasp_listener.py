#!/usr/bin/env python3
import numpy as np
import tf
import rospy
import time
from geometry_msgs.msg import PoseStamped, Pose

class BaseToGraspListener():
    def __init__(self):

        self.listener = tf.TransformListener()
        # We add this sleep to allow the listener time to be initialised
        # Otherwise the lookupTransform could give not found frame issue.
        time.sleep(1)
        self._rate = rospy.Rate(10.0)

        self.update_target_pub = rospy.Publisher('/object_position_pose', Pose, queue_size=10)

    
    def update_target_pose(self, t, o):
        """Publishes the position and orientation of the object"""
        p = Pose()
        p.position.x = t[0]
        p.position.y = t[1]
        p.position.z = t[2]

        p.orientation.x = o[0]
        p.orientation.y = o[1]
        p.orientation.z = o[2]
        p.orientation.w = o[3]

        self.update_target_pub.publish(p)

    def start_loop(self):
        while not rospy.is_shutdown():
            try:
                (trans, rot) = self.listener.lookupTransform('base_link', '/grasp', rospy.Time(0))
                self.update_target_pose(trans, rot)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF Exception: {e}")
            self._rate.sleep()

def main():
    rospy.init_node('tf_listener')
    listener_obj = BaseToGraspListener()
    try:
        listener_obj.start_loop()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()