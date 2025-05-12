#!/usr/bin/env python3

import sys
import rospy
import moveit_commander
import geometry_msgs.msg
import copy
import tf  # ← importante para la conversión

# Solo le das el punto final y ya te mueve el robot solo a esa posicion, osea hace cinematica inversa

def main():
    # Inicializa MoveIt Commander y ROS node
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('pick_and_place_node', anonymous=True)

    # Inicializa el grupo de planificación (asegúrate que el nombre coincida con tu config)
    arm_group = moveit_commander.MoveGroupCommander("manipulator")
    arm_group.set_planner_id("RRTConnectkConfigDefault")
    arm_group.set_planning_time(10)

    # Pose inicial (home)
    arm_group.set_named_target("home")
    arm_group.go(wait=True)

    # Pose de 'pick'
    pick_pose = geometry_msgs.msg.Pose()

    # Define ángulos RPY en radianes
    # 90 grados → 1.57 radianes
    roll = -1.57
    pitch = 0.0
    yaw =  0.0

    # Convierte RPY a cuaterniones
    quaternion = tf.transformations.quaternion_from_euler(roll, pitch, yaw)

    # Asigna posición y orientación
    pick_pose.position.x = 0.85 
    pick_pose.position.y = 0.45
    pick_pose.position.z = 0.0
    pick_pose.orientation.x = quaternion[0]
    pick_pose.orientation.y = quaternion[1]
    pick_pose.orientation.z = quaternion[2]
    pick_pose.orientation.w = quaternion[3]

    arm_group.set_pose_target(pick_pose)
    arm_group.go(wait=True)

    rospy.sleep(1)  # simula cerrar gripper (aquí agregarías control de pinza real)

    # Pose de 'place'
    place_pose = copy.deepcopy(pick_pose)
    place_pose.position.x = 0.7
    place_pose.position.y = 0.5
    place_pose.position.z = 0.5

    arm_group.set_pose_target(place_pose)
    arm_group.go(wait=True)

    rospy.sleep(1)  # simula abrir gripper

    # Regresa a 'home'
    arm_group.set_named_target("home")
    arm_group.go(wait=True)

    # Limpia
    arm_group.stop()
    arm_group.clear_pose_targets()
    moveit_commander.roscpp_shutdown()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass