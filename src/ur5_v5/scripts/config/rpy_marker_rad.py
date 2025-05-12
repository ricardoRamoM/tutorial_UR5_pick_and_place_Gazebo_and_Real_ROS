#!/usr/bin/env python3

import rospy
import tf
from visualization_msgs.msg import Marker
import math

def publish_rpy():
    rospy.init_node('rpy_marker_rad', anonymous=True)
    marker_pub = rospy.Publisher('rpy_marker_rad', Marker, queue_size=10)
    listener = tf.TransformListener()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        try:
            (trans, rot) = listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            
            text = (
                f"x: {trans[0]:<7.3f} roll: {roll:<6.2f}\n"
                f"y: {trans[1]:<7.3f} pitch: {pitch:<6.2f}\n"
                f"z: {trans[2]:<7.3f} yaw: {yaw:<6.2f}\n"
            )

            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = rospy.Time.now()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = trans[0]
            marker.pose.position.y = trans[1]
            marker.pose.position.z = trans[2] + 0.35
            marker.scale.z = 0.05
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = text

            marker_pub.publish(marker)
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue

        rate.sleep()

if __name__ == '__main__':
    try:
        publish_rpy()
    except rospy.ROSInterruptException:
        pass