#!/usr/bin/env python3

import rospy
import subprocess
import time

def main():
    rospy.init_node('delayed_spawn')
    rospy.loginfo("Esperando 5 segundos antes de lanzar spawn_objects.launch...")
    time.sleep(5)
    rospy.loginfo("Lanzando spawn_objects.launch...")
    subprocess.call(['roslaunch', 'ur5_v5', 'spawn_objects.launch'])

if __name__ == '__main__':
    main()