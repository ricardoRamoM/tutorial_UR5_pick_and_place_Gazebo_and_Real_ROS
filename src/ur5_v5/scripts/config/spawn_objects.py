#!/usr/bin/env python3
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose

def spawn_sdf_model(spawn_model_prox, model_name, model_path, x, y, z):
    with open(model_path, 'r') as f:
        model_xml = f.read()

    pose = Pose()
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z

    try:
        spawn_model_prox(model_name, model_xml, '', pose, 'world')
        rospy.loginfo(f"Modelo {model_name} cargado.")
    except rospy.ServiceException as e:
        rospy.logerr(f"Error al cargar {model_name}: {e}")

if __name__ == '__main__':
    rospy.init_node('spawn_objects')

    # Esperar al servicio de spawn
    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)

    models = {
        'mesa': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/mesa.sdf',
        'dropbox': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/dropbox.sdf',
        'dropbox2': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/dropbox2.sdf',
        'cubo1': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/cube1.sdf',
        'cubo2': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/cube2.sdf',
        'cubo3': '/home/gazebo-ros/catkin_ws_7/src/ur5_v5/worlds/sdf/cube3.sdf',
    }

    # Ajusta las posiciones de cada modelo
    positions = {
        'mesa': (0, 0.3, 0),
        'dropbox': (-0.7, 0.3, 0.985),
        'dropbox2': (-0.7, 0.65, 0.985),
        'cubo1': (0.6, 0.8, 1.01),
        'cubo2': (0.9, 0.4, 1.01),
        'cubo3': (0.65, 0.05, 1.01),
    }

    for name, path in models.items():
        x, y, z = positions[name]
        spawn_sdf_model(spawn_model_prox, name, path, x, y, z)