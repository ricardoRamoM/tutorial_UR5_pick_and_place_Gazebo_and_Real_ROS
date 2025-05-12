#!/usr/bin/env python3

import rospy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

def move_gripper(position, duration=2.0):
    # Inicializa el nodo
    rospy.init_node('robotiq_gripper_commander')

    # Publicador al controller
    pub = rospy.Publisher('/gripper_controller/command', JointTrajectory, queue_size=10)

    # Espera al publisher
    rospy.sleep(1)

    # Prepara mensaje de trayectoria
    traj = JointTrajectory()
    traj.joint_names = ['robotiq_85_left_knuckle_joint']

    point = JointTrajectoryPoint()
    point.positions = [position]  # Solo mandamos el knuckle joint
    point.time_from_start = rospy.Duration(duration)

    traj.points.append(point)

    # Publica el comando
    rospy.loginfo(f"Moviendo gripper a {position} rad")
    pub.publish(traj)

    rospy.sleep(duration + 1)

if __name__ == '__main__':
    try:
        # Ejemplo: cerrar a 0.8 rad, esperar, luego abrir a 0.0 rad
        move_gripper(0.4)
        rospy.sleep(3)
        #move_gripper(0.0)
    except rospy.ROSInterruptException:
        pass