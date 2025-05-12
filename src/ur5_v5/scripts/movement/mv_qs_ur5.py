#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
from math import pi

def main():
    # Inicializa moveit_commander y nodo ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('move_ur5_simple_node', anonymous=True)

    # Crea objetos de MoveIt
    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()
    group_name = "manipulator"  # Nombre del grupo de joints para el UR5
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Muestra información del robot
    rospy.loginfo("Grado de libertad disponibles: %s", move_group.get_joints())

    # Define una posición objetivo (valores en radianes para cada articulación)
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -pi/2
    joint_goal[2] = pi/2
    joint_goal[3] = -pi/2
    joint_goal[4] = 0
    joint_goal[5] = 0

    # Mueve el brazo a esa posición
    move_group.go(joint_goal, wait=True)

    # Detiene cualquier movimiento residual
    move_group.stop()

    # Finaliza MoveIt
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass