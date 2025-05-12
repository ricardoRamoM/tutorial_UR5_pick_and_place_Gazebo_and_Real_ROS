# {Tutorial: Simulación y Ejecución de un Pick and Place con el Brazo UR5 y Gripper Robotiq usando ROS Noetic, Gazebo y MoveIt}

Este tutorial te guía paso a paso para simular y ejecutar una tarea de pick and place utilizando el brazo robótico UR5 y el gripper Robotiq 2F-85, integrando herramientas como Gazebo, MoveIt, RViz y Python en ROS Noetic sobre Ubuntu 20.04. Comenzarás configurando un entorno de simulación funcional y terminarás controlando el robot físico desde una computadora remota, aplicando los mismos scripts desarrollados en el entorno virtual. Ideal para quienes buscan unir teoría, simulación y práctica real en robótica colaborativa.

---

<h2 id="indice">📑 Índice</h2>

1. [📋 I - Requisitos Previos](#-i--requisitos-previos)
2. [📖 II - Introducción](#-ii--introducción)
3. [💾 III - Instalación del Software Necesario](#-iii--instalación-del-software-necesario)
   - [1. Gazebo](#1-gazebo-simulador-3d-para-ros)
   - [2. MoveIt](#2-moveit-para-la-planificación-de-trayectorias)
   - [3. RViz](#3-instalación-de-rviz)
   - [4. Herramientas adicionales](#4-instalación-de-herramientas-adicionales)
   - [5. Python y dependencias](#5-python-y-dependencias-ros-para-python)
   - [6. Plugin Mimic para el Gripper](#6-instalación-del-plugin-mimic-para-gazebo-robotiq-gripper)
4. [🛠️ IV - Configuración del Entorno](#-iv--configuración-del-entorno)
   - [1. catkin_ws](#1-creación-y-configuración-del-catkin_ws)
   - [2. Repositorios necesarios](#2-clonado-de-repositorios-ur5-robotiq-moveit-config-etc)
   - [3. Compilación con catkin_make](#3-compilación-con-catkin_make)
   - [4. Sourcing del workspace](#4-sourcing-del-workspace)
   - [5. Crear tu propio paquete](#5-crear-tu-propio-paquete-package)
   - [6. Probar simulación básica](#6-probar-simulación-básica-ur5-y-gripper)
5. [🧪 V - Simulación del Pick and Place](#-v--simulación-del-pick-and-place)
   - [1. Visualización del robot con XACRO](#1-visualizar-el-robot-en-rviz-con-archivo-xacro)
   - [2. Crear Launch para RViz](#2-crear-launch-para-mostrar-el-robot-en-rviz)
   - [3. Configurar visualización](#3-configurar-visualización-en-rviz-y-guardar-configuración)
   - [4. Configurar controladores](#4-configurar-controladores-del-robot)
   - [5. Crear modelos SDF](#5-crear-modelos-sdf-de-objetos)
   - [6. Script para spawnear objetos](#6-crear-script-para-spawnear-objetos)
   - [7. Crear Launches de Simulación y Planificación](#7-crear-launches-de-simulación-y-planificación)
   - [8. Spawnear Objetos Desde Launch](#8-spawnear-objetos-desde-launch)
   - [9. Añadir Delay Antes de Spawnear Objetos](#9-añadir-delay-antes-de-spawnear-objetos)
   - [10. Establecer Pose Inicial del Robot con Python](#10-establecer-pose-inicial-del-robot-con-python)
   - [11. Lanzar Simulación y Spawneo de Objetos](#11-lanzar-simulación-y-spawneo-de-objetos)
   - [12. Ver Posición y Orientación en RPY en RViz](#12-ver-posición-y-orientación-en-rpy-en-rviz)
   - [13. Ver Articulaciones q1-q6 en RViz](#13-ver-articulaciones-q1-q6-en-rviz)
   - [14. Incluir Todos los Scripts en el Launch de MoveIt + RViz](#14-incluir-todos-los-scripts-en-el-launch-de-moveit--rviz)
   - [15. Añadir Plugin Mimic en el archivo URDF del gripper](#15-añadir-plugin-mimic-en-el-archivo-urdf-del-gripper)
   - [16. Verificar movimiento de los dedos del gripper en RViz](#16-verificar-movimiento-de-los-dedos-del-gripper-en-rviz)
   - [17. Incluir URDF del gripper en el XACRO principal del UR5](#17-incluir-urdf-del-gripper-en-el-xacro-principal-del-ur5)
   - [18. Incluir el gripper en la configuración de MoveIt](#18-incluir-el-gripper-en-la-configuración-de-moveit)
   - [19. Lanzar el robot con gripper en Gazebo](#19-lanzar-el-robot-con-gripper-en-gazebo)
   - [20. Visualizar el robot con gripper en MoveIt y RViz](#20-visualizar-el-robot-con-gripper-en-moveit-y-rviz)
   - [21. Crear archivo de controladores para el gripper](#21-crear-archivo-de-controladores-para-el-gripper)
   - [22. Conectar los controladores del gripper en el archivo de control](#22-conectar-los-controladores-del-gripper-en-el-archivo-de-control)
   - [23. Ajustar rotación inicial del gripper en eef.xacro](#23-ajustar-rotación-inicial-del-gripper-en-eefxacro)
   - [24. Crear launch file para spawn del robot y objetos en Gazebo](#24-crear-launch-file-para-spawn-del-robot-y-objetos-en-gazebo)
   - [25. Crear script en Python para mover el UR5](#25-crear-script-en-python-para-mover-el-ur5)
6. [🔌 VI - Conexión con el Robot Físico UR5](#-vi--conexión-con-el-robot-físico-ur5)
   - [1. Configuración de red y comunicación con el UR5](#1-configuración-de-red-y-comunicación-con-el-ur5)
   - [2. Lanzar el robot real con MoveIt](#2-lanzar-el-robot-real-con-moveit)
   - [3. Adaptar y ejecutar el script en el robot físico](#3-adaptar-y-ejecutar-el-script-en-el-robot-físico)
7. [🧩 VII - Estructura del Código y Explicación del Script](#-vii--estructura-del-código-y-explicación-del-script)
   - [1. Desglose del script Python](#1-desglose-del-script-python)
   - [2. Comunicación de nodos y control de movimientos](#2-comunicación-de-nodos-y-control-de-movimientos)
8. [✅ VIII - Conclusión](#-viii--conclusión)
9. [🚀 IX - Mejoras Futuras](#-ix--mejoras-futuras)
10. [⚠️ X - Advertencia](#-x--advertencia)
11. [📚 XI - Recursos Adicionales](#-xi--recursos-adicionales)
12. [👥 XII - Autores del Proyecto](#-xii--autores-del-proyecto)
13. [📬 XIII - Contacto](#-xiii--contacto)
---

## 📋 I-Requisitos Previos

🖥️ **Hardware mínimo recomendado**

- Procesador de 2 núcleos o más
- 4 GB de RAM mínimo (se recomienda 8 GB o más para una experiencia fluida con Gazebo)
- Tarjeta gráfica dedicada o integrada compatible con OpenGL (recomendado para simulación en Gazebo)
- Al menos 20 GB de espacio libre en disco

💻 **Entorno y sistema operativo**

- Ubuntu 20.04 LTS (recomendado como sistema principal o en una partición dedicada)
- Evitar máquinas virtuales si se trabajará con simulación en Gazebo, ya que pueden generar problemas de rendimiento y compatibilidad gráfica
- Tener ROS Noetic ya instalado sobre Ubuntu 20.04 (idealmente con desktop-full).

📚 **Conocimientos técnicos sugeridos**

- Uso básico de la terminal de Linux
- Conocimientos fundamentales de ROS: Nodos, tópicos, servicios y catkin
- Conceptos básicos de MoveIt y RViz
- Programación básica en Python
- Conocimiento básico de cinemática de robots (opcional pero útil)

---

## 📖  II-Introducción

En este tutorial aprenderás a simular, planificar trayectorias y controlar una tarea de pick and place utilizando el brazo robótico UR5 con el gripper Robotiq 2F-85, integrando herramientas del ecosistema de ROS Noetic sobre Ubuntu 20.04.

El flujo del proyecto abarca desde la simulación completa del sistema en el entorno virtual de Gazebo, la planificación y ejecución de movimientos mediante MoveIt, la visualización y prueba de trayectorias en RViz, hasta la conexión con el robot físico UR5 desde una computadora remota usando scripts en Python y MoveIt Commander.

Este proyecto combina varios componentes clave del ecosistema ROS:

🧩 Gazebo: Simulador 3D que permite modelar entornos físicos realistas para probar comportamientos del robot antes de llevarlos al mundo real.

🦾 MoveIt: Framework de planificación de movimiento que considera cinemática, obstáculos, límites articulares y más.

👁️ RViz: Herramienta de visualización 3D utilizada para monitorear, planificar y validar los movimientos del robot en tiempo real.

🐍 Python + ROS: Scripts personalizados para automatizar secuencias de pick and place y controlar el gripper mediante MoveIt Commander.

Una vez validado el sistema en simulación, se procede a establecer comunicación con el UR5 físico mediante conexión por red, permitiendo ejecutar exactamente la misma lógica desarrollada en el entorno virtual.

Este tutorial está diseñado para estudiantes, investigadores y entusiastas de la robótica que deseen aprender a integrar simulación y hardware real usando ROS, enfocándose en aplicaciones prácticas como la automatización de procesos mediante pick and place.


## 💾 III-Instalación del Software Necesario
### 1. Gazebo (Simulador 3D para ROS)

Si instalaste ros-noetic-desktop-full, ya tienes Gazebo 11 instalado, por lo tanto no necesitas instalarlo aparte.

En caso de que hayas instalado otra versión de ROS Noetic sin Gazebo, desde la terminal, usa:

    sudo apt install ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control

### 2. MoveIt (para la planificación de trayectorias)
MoveIt no viene incluido por defecto, incluso si usaste desktop-full. Debes instalarlo manualmente desde la terminal con:

    sudo apt install ros-noetic-moveit

### 3. Instalación de RViz
RViz sí viene incluido por defecto en ros-noetic-desktop-full. Solo deberías instalarlo desde la terminal si usaste una versión ligera de ROS con:

    sudo apt install ros-noetic-rviz

### 4. Instalación de herramientas adicionales
Asegúrate de tener estas herramientas auxiliares para simulación y control del robot. 
Solo ejecuta en terminal estas instrucciones:

    sudo apt install ros-noetic-joint-state-publisher ros-noetic-robot-state-publisher
    sudo apt install ros-noetic-controller-manager ros-noetic-industrial-core

### 5. Python y Dependencias ROS para Python:
Ejecuta en la terminal estos comandos:

    sudo apt install python3-pip
    pip3 install -U rospy moveit_commander

### 6. Instalación del Plugin Mimic para Gazebo (Robotiq Gripper)
El gripper Robotiq 2F-85 utiliza joints que deben moverse de forma sincronizada. Para ello, es necesario el plugin mimic_joint_plugin de Gazebo.

🔍 Verificar si ya está instalado ejecutando desde la terminal:

    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"

Si no aparece ningún resultado, sigue los pasos a continuación para instalarlo.

🔧 Opción 1 – Instalar globalmente en el sistema (recomendado). Únicamente ejecuta desde la terminal los siguientes comandos:

		cd ~
        git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
		cd roboticsgroup_gazebo_plugins
		mkdir build && cd build
		cmake ..
		make
		sudo make install    

🔧 Opción 2 – Instalarlo dentro de tu workspace de ROS (si no tienes permisos sudo). 
Esto no lo podremos hacer en este instante. Una vez que creemos nuestro workspace de ROS ya podríamos regresar a realizar este paso y desde la terminal ejecutar los siguientes comandos:

        cd ~/catkin_ws_1/src
        git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
        cd ..
        catkin_make

Para verificar la instalación correcta se ejecuta desde la terminal:

    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"

Configurar Gazebo para encontrar el plugin:

    echo 'export GAZEBO_PLUGIN_PATH=$GAZABO_PLUGIN_PATH:/usr/local/lib' >> ~/.bashrc
    source ~/.bashrc


<h2 id="configuracion-entorno">🛠️ IV - Configuración del Entorno</h2>

### 1. Creación y configuración del catkin_ws_1
Si aún no tienes un workspace de ROS configurado, sigue estos pasos:

    mkdir -p ~/catkin_ws_1/src
    cd ~/catkin_ws_1/
    catkin_make

Agrega el workspace a tu entorno de shell para que ROS lo reconozca:

    echo "source ~/catkin_ws_1/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

### 2. Clonado de repositorios (UR5, Robotiq, MoveIt config, etc.)

Dentro de la carpeta src, clona los siguientes paquetes necesarios para el UR5 y el gripper Robotiq. 

#### A) Repositorio del UR5 directo de Universal Robots

	sudo apt-get install ros-noetic-universal-robots
	cd src
	git clone -b noetic-devel https://github.com/ros-industrial/universal_robot.git
	cd ..
	rosdep update
	rosdep install --rosdistro noetic --ignore-src --from-paths src
	catkin_make

#### B) Instalar el plugin mimic (para articulaciones sincronizadas del gripper)

Este plugin es necesario para simular correctamente el movimiento sincronizado de los dedos del gripper. Solo es necesario si no lo instalaste globalmente:

    cd ~/catkin_ws_1/src
    git clone https://github.com/roboticsgroup/roboticsgroup_gazebo_plugins.git
    cd ..
    catkin_make

Si lo instalaste globalmente, puedes verificarlo con:

    find /usr/ -name "libroboticsgroup_gazebo_mimic_joint_plugin.so"

#### C) Agregar el gripper Robotiq 2F-85
En este tutorial usaremos una versión simplificada del modelo del gripper:

    cd ~/catkin_ws_1/src
    git clone https://github.com/LearnRoboticsWROS/robotiq_description.git
    mv robotiq_description robotiq_gripper
    cd ..
    catkin_make

Nota: La version completa está en este github: https://github.com/philwall3/UR5-with-Robotiq-Gripper-and-Kinect/tree/master.
Para simulación basta con la carpeta: robotiq_85_gripper-master/robotiq_85_description, pero este tutorial usa una versión simplificada.

### 3. Compilación con catkin_make
Una vez descargados los paquetes:

    cd ~/catkin_ws_1
    rosdep install --from-paths src --ignore-src -r -y
    catkin_make

Si todo se compila sin errores, ¡ya tienes tu entorno base configurado!

### 4. Sourcing del workspace
Para no tener que hacer esto cada vez:

    source ~/catkin_ws_1/devel/setup.bash

Automatízalo añadiéndolo a tu .bashrc con la ayuda de este comando:

    echo "source ~catkin_ws_1/devel/setup.bash" >> ~/.bashrc

Si ya hiciste esto último, en automatico hará el sourcing en cada nueva terminal.

### 5. Crear tu propio paquete (package)
Crea un paquete donde guardarás tus archivos de control, simulación y scripts ejecutando las siguientes instrucciones en la terminal:

    cd ~/catkin_ws_1/src
    catkin_create_pkg ur5_v1 controller_manager joint_state_controller robot_state_publisher roscpp rospy std_msgs urdf
    cd ..
    catkin_make

Esto te generará la estructura básica en catkin_ws_1/src/ur5_V1, donde colocarás tus archivos .launch, URDFs y scripts Python.

Nota: Recordemos que para crear un package, se debe de seguir la siguiente estructura:

    catkin_create_pkg  <name_of_package> <dependencies of package>


### 6. Probar simulación básica (UR5 y gripper)
Para verificar la simuación del UR5 en Gazebo y RVIZ. Abre tres terminales para ejecutar lo siguiente:

**Terminal 1** – Lanzar UR5 en Gazebo:

    roslaunch ur_gazebo ur5_bringup.launch

**Terminal 2** – Lanzar MoveIt:

    roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true

**Terminal 3** – Visualizar en RViz:

    roslaunch ur5_moveit_config moveit_rviz.launch config:=true

En RViz:
- Cambia Fixed Frame a base_link. Gloal Options -> Fixed Frame -> base_link
- Añade RobotModel desde el botón Add. Add -> RobotModel
- Añade Motion PLanning desde el botón Add para poder mover el robot. Add -> Motion PLanning

![UR5_Gazebo_Rviz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/ur5_gazebo_rviz.png)

Para verificar la simulación del gripper en RVIZ en una nueva terminal ejecutamos esto: 

    roslaunch robotiq_gripper spawn_robotiq_85_gripper.launch

![Gripper](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/gripper_rviz.png)


Nota: Para terminal la ejecución, presiona en cada terminal las teclas: ctrl + C


## 🧪 V-Simulación del Pick and Place

### 1) Visualizar el Robot en RViz con Archivo XACRO
- Crear la carpeta urdf (Unified Robot Description Format) dentro de la ruta ~/catkin_ws_1/src/ur5_v1

- Ejecutar en una terminal: 

    roslaunch moveit_setup_assistant setup_assistant.launch

![moveit_assitant](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/moveit_assitant.png)


    - Dar click en -> Edit Existing MoveIt Configuration Package
    - Poner esta ruta: /home/gazebo-ros/catkin_ws_1/src/universal_robot/ur5_moveit_config
    - Darle click a Load Files
    - Ir a la parte de "Simulation" y copiar todo el texto
    - Cerrar Movit Assistant

- Crear, dentro de la carpeta "urdf", el archivo ur5_1.xacro y pegar ahi lo copiado
- Modificar la interfaz de los joints, cambiando en el archivo ur5_1.xacro los "PositionJointInterface" por "EffortJointInterface"

- Añadir el joint fijo entre world y base_link. Se arregla abajo de la linea 145, o abajo de esto:

		<link name="base_link_inertia">
		...
		</link>:	
	
    - Se hace pegando lo siguiente en ese lugar: 

            <!-- Fix the cobot to the world -->
            <link name="world"/>


            <joint name="fixed" type="fixed">
                <parent link="world"/>
                <child link="base_link"/>   
            </joint>


### 2) Crear Launch para Mostrar el Robot en RViz
- Crear la carpeta launch dentro de catkin_ws_1/src/ur5_v1. 
- Crear dentro, el archivo "rviz_ur5.launch"
- En ese archivo se coloca lo siguiente, revisar que donde dice find si aparezca el nombre que tu tienes de tu carpeta y que el archivo xacro igual se llame igual que el que tu tienes: 

        <?xml version="1.0"?>
        <launch>

            <!-- Associate to the robot description parameter, the urdf file that model the robot-->
            <param name="robot_description" command = "$(find xacro)/xacro --inorder $(find ur5_v1)/urdf/ur5_1.xacro" />

            <!-- Read the joint value-->
            <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
            
            <!-- Visualization in Rviz-->
            <node name="rviz" pkg="rviz" type="rviz" /> 

            <!-- Visualization of the use_gui for playing with joint-->
            <arg name="use_gui" default="true" />
            <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"  output="screen" unless="$(arg use_gui)" />
            <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"  output="screen" if="$(arg use_gui)"/>    
            
        </launch>

### 3) Configurar Visualización en RViz y Guardar Configuración
- Para ejecutar RViz, ejecutar en terminal esto:

    cd ~/catkin_ws_1    
	roslaunch ur5_v1 rviz_ur5.launch

- No aparecerá el robot, pero se arregla ajustando ciertas configuraciones:
    - Global OPtions → FixedFrame →base_link
    - Grid → Plane Cell Count → 20
    - Grid → Cell Size → 0.1
    - En Displays → Add → :
        - RobotModel
        - TF
        - MotionPlanning
    - Acomodar la visualización que nosotros querramos
- Crear la carpeta 'config' en ~/catkin_ws_1/src/ur5_v1. 
- Guardar la configuracion en esa carpeta con el nombre de "config.rviz"

### 4) Configurar Controladores del Robot
- Abrir la carpeta ~/catkin_ws_1/src/ur5_v1/config
- Crear el archivo "ur5_controllers.yaml"
    - Checar cual es el tipo de "HardwareInterface" en el archivo.xacro. 
    - En nuestro caso será "EffortController", pero esto quiere decir que necesitaríamos tunnear el PID. Pero, no lo haremos porque UR ya lo hizo. 
- Buscar el archivo: ~/catkin_ws_1/src/universal_robots/ur_gazebo/config/ur5_controller.yaml 
- Copiar contenido y pegarlo en el que creamos nosotros.

- [ ] Nota 1: JointTrajectoryController es porque vamos a usar el plugin de RVIZ y ese usa JointTrajectoryController
- [ ] Nota 2: Usa un publish_rate alto (125 Hz), lo que puede mejorar la suavidad en simulación. Se puede usar un publish_rate más bajo (50 Hz), suficiente para pruebas, pero menos suave. Esto se ve en esta linea:
    publish_rate: &loop_hz 125 

### 5) Crear Modelos SDF de Objetos
- Crear la carpeta 'worlds' en ~/catkin_ws_1/src/ur5_v1
- Crear carpeta 'sdf' en ~/catkin_ws_1/src/ur5_v1/worlds
- Crear los archivos sdf de cada objeto que queremos colocar en el mundo en la carpeta'sdf'
- Crearemos los siguientes archivos:
    - cube1.sdf
    - cube2.sdf
    - cube3.sdf
    - mesa.sdf
    - dropbox.sdf
    - dropbox2.sdf
- Pegaremos cada uno de los siguientes codigos:

- [ ] En el archivo "cube1.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
        <model name="cube1">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
            <visual name="visual">
                <geometry>
                <box>
                    <size>0.05 0.05 0.05</size>
                </box>
                </geometry>
                <material>
                <ambient>0 0.9 0.9 1</ambient>
                <diffuse>0.6 0.4 0.2 1</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                <box>
                    <size>0.05 0.05 0.05</size>
                </box>
                </geometry>
            </collision>
            </link>
        </model>
        </sdf>

- [ ] En el archivo "cube2.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
        <model name="cube1">
            <pose>0 0 0 0 0 0</pose>
            <link name="link">
            <visual name="visual">
                <geometry>
                <box>
                    <size>0.04 0.04 0.04</size>
                </box>
                </geometry>
                <material>
                <ambient>0.6 0 0.2 1</ambient>
                <diffuse>0.6 0.4 0.2 1</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                <box>
                    <size>0.04 0.04 0.04</size>
                </box>
                </geometry>
            </collision>
            </link>
        </model>
        </sdf>

- [ ] En el archivo "cube3.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
        <model name="cube3">
            <include>
            <uri>https://fuel.ignitionrobotics.org/1.0/09ubberboy90/models/box 5cm</uri>
            <name>cube1</name>
            <pose>0 0 0 0 0 0</pose><!--Pose esta enmedio del cubo -->
            </include>
        </model>
        </sdf>
	
- [ ] En el archivo "mesa.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
        <model name="mesa">
            <static>true</static>
            <pose>0 0.3 0 0 0 0</pose>
            <link name="tablero">
            <pose>0 0 0.98 0 0 0</pose>
            <visual name="visual">
                <geometry>
                <box>
                    <size>2.0 1.21 0.01</size>
                </box>
                </geometry>
                <material>
                <ambient>0.6 0.4 0.2 1</ambient>
                <diffuse>0.6 0.4 0.2 1</diffuse>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                <box>
                    <size>2.0 1.21 0.01</size>
                </box>
                </geometry>
            </collision>
            </link>
            <!-- Pata 1 -->
            <link name="pata1">
            <pose>-0.95 -0.55 0.49 0 0 0</pose>
            <visual name="visual">
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
                <material>
                <ambient>0.3 0.3 0.3 1</ambient>
                </material>
            </visual>
            <collision name="collision">
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
            </collision>
            </link>
            <link name='pata2'>
            <pose>0.95 -0.55 0.49 0 0 0</pose>
            <visual name='visual'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
                <material>
                <ambient>0.3 0.3 0.3 1</ambient>
                </material>
            </visual>
            <collision name='collision'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
            </collision>
            </link>
            <link name='pata3'>
            <pose>-0.95 0.55 0.49 0 0 0</pose>
            <visual name='visual'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
                <material>
                <ambient>0.3 0.3 0.3 1</ambient>
                </material>
            </visual>
            <collision name='collision'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
            </collision>
            </link>
            <link name='pata4'>
            <pose>0.95 0.55 0.49 0 0 0</pose>
            <visual name='visual'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
                <material>
                <ambient>0.3 0.3 0.3 1</ambient>
                </material>
            </visual>
            <collision name='collision'>
                <geometry>
                <box>
                    <size>0.07 0.03 0.98</size>
                </box>
                </geometry>
            </collision>
            </link>
        </model>
        </sdf>

- [ ] En el archivo "dropbox.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
            <model name='dropbox'>
            <static>true</static>
            <link name='link'>
                <pose>0 0 0 0 0 0</pose>
                <!-- Base -->
                <visual name='base'>
                <pose>0 0 0.005 0 0 0</pose> <!-- altura base / 2 -->
                <geometry>
                    <box>
                    <size>0.3 0.2 0.01</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0 0.3 0.6 1</ambient>
                </material>
                </visual>
                <collision name='base_collision'>
                <pose>0 0 0.005 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.3 0.2 0.01</size>
                    </box>
                </geometry>
                </collision>
                <!-- Pared frontal -->
                <visual name='wall1'>
                <pose>0 0.1 0.08 0 0 0</pose> <!-- base altura: 0.005 + pared mitad altura: 0.075 -->
                <geometry>
                    <box>
                    <size>0.3 0.005 0.15</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0 0.3 0.6 1</ambient>
                </material>
                </visual>
                <collision name='wall1_collision'>
                <pose>0 0.1 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.3 0.005 0.15</size>
                    </box>
                </geometry>
                </collision>
                <!-- Pared trasera -->
                <visual name='wall2'>
                <pose>0 -0.1 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.3 0.005 0.15</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0 0.3 0.6 1</ambient>
                </material>
                </visual>
                <collision name='wall2_collision'>
                <pose>0 -0.1 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.3 0.005 0.15</size>
                    </box>
                </geometry>
                </collision>
                <!-- Pared izquierda -->
                <visual name='wall3'>
                <pose>0.15 0 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.005 0.2 0.15</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0 0.3 0.6 1</ambient>
                </material>
                </visual>
                <collision name='wall3_collision'>
                <pose>0.15 0 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.005 0.2 0.15</size>
                    </box>
                </geometry>
                </collision>
                <!-- Pared derecha -->
                <visual name='wall4'>
                <pose>-0.15 0 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.005 0.2 0.15</size>
                    </box>
                </geometry>
                <material>
                    <ambient>0 0.3 0.6 1</ambient>
                </material>
                </visual>
                <collision name='wall4_collision'>
                <pose>-0.15 0 0.08 0 0 0</pose>
                <geometry>
                    <box>
                    <size>0.005 0.2 0.15</size>
                    </box>
                </geometry>
                </collision>
            </link>
            </model>
        </sdf>

- [ ] En el archivo "dropbox2.sdf", pegar esto:

        <?xml version="1.0" ?>
        <sdf version="1.6">
        <model name="dropbox2">
            <static>true</static>
            <link name="link">
            <pose>0 0 0 0 0 0</pose>
            <!-- Base -->
            <visual name="base">
                <pose>0 0 0.005 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.2 0.01</size>
                </box>
                </geometry>
                <material>
                <ambient>0.8 0 0 1</ambient>
                </material>
            </visual>
            <collision name="base_collision">
                <pose>0 0 0.005 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.2 0.01</size>
                </box>
                </geometry>
            </collision>
            <!-- Pared frontal -->
            <visual name="wall1">
                <pose>0 0.1 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.005 0.15</size>
                </box>
                </geometry>
                <material>
                <ambient>0.8 0 0 1</ambient>
                </material>
            </visual>
            <collision name="wall1_collision">
                <pose>0 0.1 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.005 0.15</size>
                </box>
                </geometry>
            </collision>
            <!-- Pared trasera -->
            <visual name="wall2">
                <pose>0 -0.1 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.005 0.15</size>
                </box>
                </geometry>
                <material>
                <ambient>0.8 0 0 1</ambient>
                </material>
            </visual>
            <collision name="wall2_collision">
                <pose>0 -0.1 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.3 0.005 0.15</size>
                </box>
                </geometry>
            </collision>
            <!-- Pared izquierda -->
            <visual name="wall3">
                <pose>0.15 0 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.005 0.2 0.15</size>
                </box>
                </geometry>
                <material>
                <ambient>0.8 0 0 1</ambient>
                </material>
            </visual>
            <collision name="wall3_collision">
                <pose>0.15 0 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.005 0.2 0.15</size>
                </box>
                </geometry>
            </collision>
            <!-- Pared derecha -->
            <visual name="wall4">
                <pose>-0.15 0 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.005 0.2 0.15</size>
                </box>
                </geometry>
                <material>
                <ambient>0.8 0 0 1</ambient>
                </material>
            </visual>
            <collision name="wall4_collision">
                <pose>-0.15 0 0.08 0 0 0</pose>
                <geometry>
                <box>
                    <size>0.005 0.2 0.15</size>
                </box>
                </geometry>
            </collision>
            </link>
        </model>
        </sdf>


### 6) Crear Script para Spawnear Objetos (sin Launch)
- Crear la carpeta 'scripts' en ~/catkin_ws_1/src/ur5_v1
- Crear ahí una nueva carpeta llamada 'config'
- En la carpeta 'config' recien creada crear un archivo llamado 'spawn_objects_no_launch.py'
- Darle permisos de ejecucion al archivo, hay 2 opciones:
    - En Archivos buscar el archivo python, darle en Propiedades -> Permisos -> Permitir ejecutar el archivo como un programa
    - En terminal: chmod +x spawn_objects_no_launch.py
- Pegar el codigo de abajo

        #!/usr/bin/env python3
        import rospy
        from gazebo_msgs.srv import SpawnModel
        from geometry_msgs.msg import Pose

        def spawn_sdf_model(model_name, model_path, x, y, z):
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
                'mesa': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/mesa.sdf',
                'dropbox': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/dropbox.sdf',
                'dropbox2': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/dropbox2.sdf',
                'cubo1': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube1.sdf',
                'cubo2': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube2.sdf',
                'cubo3': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube3.sdf',
            }

            # Ajusta las posiciones de cada modelo
            positions = {
                'mesa': (0, 0.3, 0),
                'dropbox': (-0.7, 0.3, 0.985),
                'dropbox2': (-0.7, 0.65, 0.985),
                'cubo1': (0.6, 0.8, 1.01),
                'cubo2': (0.9, 0.4, 1.01),
                'cubo3': (0.75, 0.35, 1.01),
            }

            for name, path in models.items():
                x, y, z = positions[name]
                spawn_sdf_model(name, path, x, y, z)

### 7) Crear Launches de Simulación y Planificación
- Hacer los siguientes 2 archivos launch en una nueva carpeta en ~/catkin_ws_1/src/ur5_v1/launch
	- ur5_gazebo_1.launch
		- → Gazebo con UR5 corriendo
		- → Publica /robot_description, /tf, /joint_states
		- → Carga controladores

                <?xml version="1.0"?>
                <launch>

                    <!-- Cargar el modelo UR5 -->
                    <param name="robot_description" command="$(find xacro)/xacro '$(find ur5_v1)/urdf/ur5_1.xacro'" /> 
                
                    <!--Spawn Robot in Gazebo-->
                    <!-- Set the position in empty world of the base link-->
                    <arg name="x" default="0" />
                    <arg name="y" default="0" />
                    <arg name="z" default="0.985" />

                    <!-- put world file as argument-->
                    <arg name="world_file" default = "$(find ur5_v1)/worlds/my_custom_world.world" />

                    <!-- Lanzar Gazebo con tu mundo -->
                    <include file="$(find gazebo_ros)/launch/empty_world.launch">
                        <arg name="paused" value="false"/>
                        <arg name="gui" value="true"/>
                        <arg name="use_sim_time" value="true"/>
                        <arg name="world_name" value="$(arg world_file)"/>
                    </include>

                    <!-- Publicar estados del robot -->
                    <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
                    
                    <!-- Spawn the robot using the package gazebo_ros-->
                    <node name="spawn_the_robot" pkg="gazebo_ros" type="spawn_model"  output="screen" args="-urdf -param robot_description -model ur5 -x $(arg x) -y $(arg y) -z $(arg z)" />  

                    <!-- Controladores -->
                    <!-- Cargar Controladores -->
                    <rosparam file="$(find ur5_v1)/config/ur5_controllers.yaml" command="load"/>
                
                    <!-- Cargar the node Controller manager -->
                    <node name="controller_spawner" pkg="controller_manager" type="spawner"
                    args="joint_state_controller 
                    eff_joint_traj_controller 
                    --timeout 60 " />

                </launch>

    - ur5_moveit_with_rviz_1.launch
		- → Lanzas ur5_moveit_with_rviz.launch → MoveIt + remapeo + RViz

                <launch>
                <arg name="sim" default="true" />
                <arg name="debug" default="false" />

                <!-- Remapea trajectory controller para Gazebo -->
                <remap if="$(arg sim)" from="/scaled_pos_joint_traj_controller/follow_joint_trajectory" to="/eff_joint_traj_controller/follow_joint_trajectory"/>

                <!-- Lanza MoveIt  con la config de Universal Robots-->
                <include file="$(find ur5_moveit_config)/launch/move_group.launch">
                    <arg name="debug" value="$(arg debug)" />
                </include>

                <!-- Lanza RViz con la configuración de RVIZ guardada en config -->
                <node name="rviz" pkg="rviz" type="rviz" output="screen"
                        args="-d $(find ur5_v1)/config/config.rviz" />
                </launch>

- Ejecutar 2 terminales y en cada uno cada una de las sig instrucciones:
    - roslaunch ur5_v1 ur5_gazebo_1.launch
    - roslaunch ur5_v1 ur5_moveit_with_rviz_1.launch
- Para poder probar que se carguen los objetos en el entorno del paso anterior ejecutamos esto en una nueva 3er Terminal:
	- rosrun ur5_v1 spawn_objects_no_launch.py
- Después con el de python, se mostraran los objetos en Gazebo.	Puedes borrarlos del entorno en gazebo para volver a ejecutar el archivo python y ver su nueva ubicacion o parametros que cambiaste. Si no llegas a borrar un objeto y vuelves a ejecutar el codigo, no volvera a aparecer nuevamente, simplemente lo omitirá y se pasará al siguiente objeto

- [ ] NOTA: La simulacion en gazebo debe de estar en 'play' antes de ejecutar la 2da terminal con RViz y MoveIt.

![gazebo_play](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/gazebo_play.png)

### 8) Spawnear Objetos Desde Launch
- Crear archivo python para spawnear los objetos directamente desde un launch:

    - En la carpeta 'config' dentro de scripts, osea en ~/catkin_ws_1/src/ur5_v1/scripts/config
    - Crear un archivo llamado 'spawn_objects.py'
    - Darle permisos de ejecucion al archivo, hay 2 opciones:
        - En Archivos buscar el archivo python, darle en Propiedades -> Permisos -> Permitir ejecutar el archivo como un programa
        - En terminal: chmod +x spawn_objects.py
    - Pegar el codigo de abajo

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
                    'mesa': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/mesa.sdf',
                    'dropbox': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/dropbox.sdf',
                    'dropbox2': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/dropbox2.sdf',
                    'cubo1': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube1.sdf',
                    'cubo2': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube2.sdf',
                    'cubo3': '/home/gazebo-ros/catkin_ws_1/src/ur5_v1/worlds/sdf/cube3.sdf',
                }

                # Ajusta las posiciones de cada modelo
                positions = {
                    'mesa': (0, 0.3, 0),
                    'dropbox': (-0.7, 0.3, 0.985),
                    'dropbox2': (-0.7, 0.65, 0.985),
                    'cubo1': (0.6, 0.8, 1.01),
                    'cubo2': (0.9, 0.4, 1.01),
                    'cubo3': (0.75, -0.05, 1.01),
                }

                for name, path in models.items():
                    x, y, z = positions[name]
                    spawn_sdf_model(spawn_model_prox, name, path, x, y, z)

- Crear un nuevo archivo launch para ejecutar el archivo python
	- En la carpeta launch crear nuevo archivo llamado "spawn_objects.launch"	
	- Copiar el siguiente codigo

            <?xml version="1.0"?>
            <launch>
                <!-- Lanzar el script spawn_objects.py -->
                <node name="spawn_objects" pkg="ur5_v1" type="spawn_objects.py" output="screen" />
            </launch>  

### 9) Añadir Delay Antes de Spawnear Objetos
- Crear nuevo archivo python para colocar los objetos 5 segundos despues de colocar el robot
	- En la carpeta 'config' dentro de 'scripts', osea en ~/catkin_ws_1/src/ur5_v1/scripts/config
	- Crear un archivo llamado 'delayed_spawn.py'
	- Darle permisos de ejecucion al archivo, hay 2 opciones:
		- En Archivos buscar el archivo python, darle en Propiedades -> Permisos -> Permitir ejecutar el archivo como un programa
		- En terminal: chmod +x delayed_spawn.py.py
	- Pegar el codigo de abajo  

            #!/usr/bin/env python3

            import rospy
            import subprocess
            import time

            def main():
                rospy.init_node('delayed_spawn')
                rospy.loginfo("Esperando 5 segundos antes de lanzar spawn_objects.launch...")
                time.sleep(5)
                rospy.loginfo("Lanzando spawn_objects.launch...")
                subprocess.call(['roslaunch', 'ur5_v1', 'spawn_objects.launch'])

            if __name__ == '__main__':
                main()

### 10) Establecer Pose Inicial del Robot con Python
- En esta ruta '~/catkin_ws_1/src/ur5_v1/scripts/config', crear nuevo archivo python llamado "ur5_set_initial_pose.py". 
- Darle permisos de ejecucion al archivo, hay 2 opciones:
	- En Archivos buscar el archivo python, darle en Propiedades -> Permisos -> Permitir ejecutar el archivo como un programa
	- En terminal: chmod +x 'ur5_set_initial_pose.py'
- Copiar y pegar el siguiente código.

        #!/usr/bin/env python3

        import rospy
        from controller_manager_msgs.srv import ListControllers
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

        def wait_for_controller():
            rospy.wait_for_service('/controller_manager/list_controllers')
            list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
            while not rospy.is_shutdown():
                controllers = list_controllers()
                for c in controllers.controller:
                    if c.name == 'eff_joint_traj_controller' and c.state == 'running':
                        rospy.loginfo('Controller is running.')
                        return
                rospy.loginfo('Waiting for eff_joint_traj_controller to be running...')
                rospy.sleep(1)

        def send_initial_pose():
            pub = rospy.Publisher('/eff_joint_traj_controller/command', JointTrajectory, queue_size=10)
            rospy.sleep(1)  # give publisher time to connect

            traj = JointTrajectory()
            traj.joint_names = [
                'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
                'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
            ]
            point = JointTrajectoryPoint()
            point.positions = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
            point.velocities = [0.0] * 6
            point.time_from_start = rospy.Duration(1.0)
            traj.points.append(point)

            rospy.loginfo('Publishing initial pose...')
            pub.publish(traj)
            rospy.loginfo('Initial pose published.')

        if __name__ == '__main__':
            rospy.init_node('set_initial_pose')
            wait_for_controller()
            send_initial_pose()

- Crear una copia del archivo 'ur5_gazebo_1.launch' y renombrarlo 'ur5_gazebo_2.launch'
- Añadir a ese launch "ur5_gazebo_2.launch" lo siguiente:

        <!-- Establecer postura inicial con script Python -->
        <node name="set_initial_pose" pkg="ur5_v1" type="ur5_set_initial_pose.py" output="screen"/>            

### 11) Lanzar Simulación y Spawneo de Objetos
- Crear nuevo archivo launch para poner el robot y luego los objetos
- En la carpeta launch crear un nuevo archivo llamado 'ur5_gazebo_add_objects_1.launch'
	
        <?xml version="1.0"?>
        <launch>
            <!-- Incluye el launch principal del robot -->
            <include file="$(find ur5_v1)/launch/ur5_gazebo_2.launch" />

            <!-- Ejecuta el script delay que luego lanza spawn_objects.launch -->
            <node name="delayed_spawn" pkg="ur5_v1" type="delayed_spawn.py" output="screen" />
        </launch>

### 12) Ver Posición y Orientación en RPY en RViz
- Para poder ver la orientacion en rpy del TCP en vez de quaterniones que te da rviz, y tambien para ver la posición del TCP en rviz en pantalla.
-Usamos dos scripts de python y los guardamos en una nueva carpeta llamada 'config' dentro de 'scripts'. Es decir, en esta ruta: ~/catkin_ws_1/src/ur5_v1/scripts/config
- Los llamamos rpy_marker_rad.py y rpy_marker_deg.py
	- Les damos permisos de ejecucion manualmente o con:  chmod +x rpy_marker_rad.py
- Les pegamos los códigos de abajo
- Para probarlo, ejecutamos en terminales diferentes cada uno de los siguientes comandos:
    - roslaunch ur5_v1 ur5_gazebo_add_objects_1.launch
    - roslaunch ur5_v1 ur5_moveit_with_rviz_1.launch
	- rosrun ur5_v1 rpy_marker_rad.py
	- rosrun ur5_v1 rpy_marker_deg.py
- En RViz: Add → Marker → MarkerTopic /rpy_marker_rad o /rpy_marker_deg
- Guardar el archivo: config.rviz .Nota: no crear un nuevo archivo.rviz, solo guardar el que ya teniamos

- [ ] NOTA: Tambien Para ver posicion y orientacion del robot:
	- [ ] En terminal se puede con esta línea:

		rosrun tf tf_echo base_link tool0

        - Te debe de salir esto:
            At time 1746069792.500
            - Translation: [0.817, 0.192, -0.005]
            - Rotation: in Quaternion [0.507, 0.493, 0.493, 0.507]
                        in RPY (radian) [1.571, -0.000, 1.544]
                        in RPY (degree) [89.992, -0.007, 88.444]

	- [ ] En gazebo se puede ver, en wrist_3_link -> pose

- [ ] Código en rpy_marker_rad.py:

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

- [ ] Código de rpy_marker_deg.py: 

        #!/usr/bin/env python3

        import rospy
        import tf
        from visualization_msgs.msg import Marker
        import math

        def publish_rpy():
            rospy.init_node('rpy_marker_deg', anonymous=True)
            marker_pub = rospy.Publisher('rpy_marker_deg', Marker, queue_size=10)
            listener = tf.TransformListener()
            rate = rospy.Rate(10)

            while not rospy.is_shutdown():
                try:
                    (trans, rot) = listener.lookupTransform('base_link', 'tool0', rospy.Time(0))
                    roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
                    
                    roll_deg = math.degrees(roll)
                    pitch_deg = math.degrees(pitch)
                    yaw_deg = math.degrees(yaw)
                    
                    text = (
                        f"x: {trans[0]:<7.3f} roll: {roll_deg:<6.2f}\n"
                        f"y: {trans[1]:<7.3f} pitch: {pitch_deg:<6.2f}\n"
                        f"z: {trans[2]:<7.3f} yaw: {yaw_deg:<6.2f}"
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
        	



![TCP_RViz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/tcp_rviz.png)

![TCP_RViz_1](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/tcp_rviz_1.png)

### 13) Ver Articulaciones q1-q6 en RViz
- Para poder ver cuanto giran las articulaciones q1-q6 en rviz. 
- Usamos dos scripts de python y los guardamos en una nueva carpeta llamada 'config' dentro de 'scripts. Es decir, en esta ruta: ~/catkin_ws_1/src/ur5_v1/scripts/config
- Los llamamos joint_state_marker_rad.py y joint_state_marker_deg.py
    - Les damos permisos de ejecucion manualmente o con:  chmod +x rpy_marker_rad.py
- Les pegamos los códigos de abajo
- Para probarlo, ejecutamos en terminales diferentes cada uno de los siguientes comandos:
    - roslaunch ur5_v1 ur5_gazebo_add_objects_1.launch
    - roslaunch ur5_v1 ur5_moveit_with_rviz_1.launch
    - rosrun ur5_v1 joint_state_marker_rad.py
    - rosrun ur5_v1 joint_state_marker_deg.py
- En RViz: Add → Marker → MarkerTopic /joint_state_marker_rad o /joint_state_marker_deg	
- Guardar el archivo: config.rviz . Nota: no crear un nuevo archivo.rviz, solo guardar el que ya teniamos

- [ ] Código de joint_state_marker_rad.py: 

        #!/usr/bin/env python3

        import rospy
        from sensor_msgs.msg import JointState
        from visualization_msgs.msg import Marker

        class JointStateMarker:
            def __init__(self):
                rospy.init_node('joint_state_marker_rad')
                self.pub = rospy.Publisher('/joint_state_marker_rad', Marker, queue_size=10)
                rospy.Subscriber('/joint_states', JointState, self.callback)

                self.base_frame = "base_link"

            def callback(self, msg):
                name_to_position = dict(zip(msg.name, msg.position))

                # Obtener los valores articulares en radianes
                q1 = name_to_position.get('shoulder_pan_joint', 0)
                q2 = name_to_position.get('shoulder_lift_joint', 0)
                q3 = name_to_position.get('elbow_joint', 0)
                q4 = name_to_position.get('wrist_1_joint', 0)
                q5 = name_to_position.get('wrist_2_joint', 0)
                q6 = name_to_position.get('wrist_3_joint', 0)

                # Gripper (fijo aparte)
                gripper = name_to_position.get('robotiq_85_left_knuckle_joint', 0)

                # Preparar texto alineado
                text = (
                    f"q1: {q1:<7.3f}  q4: {q4:<7.3f}\n"
                    f"q2: {q2:<7.3f}  q5: {q5:<7.3f}\n"
                    f"q3: {q3:<7.3f}  q6: {q6:<7.3f}\n"
                    f"Gripper: {gripper:.3f}\n"
                )

                # Crear el marcador
                marker = Marker()
                marker.header.frame_id = self.base_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "joint_text"
                marker.id = 0
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD

                marker.pose.position.x = 0.0
                marker.pose.position.y = -0.1
                marker.pose.position.z = 0.3
                marker.pose.orientation.w = 1.0

                marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                marker.text = text

                self.pub.publish(marker)

        if __name__ == '__main__':
            JointStateMarker()
            rospy.spin()


- [ ] Código de joint_state_marker_deg.py:

        #!/usr/bin/env python3

        import rospy
        from sensor_msgs.msg import JointState
        from visualization_msgs.msg import Marker
        import math

        class JointStateMarker:
            def __init__(self):
                rospy.init_node('joint_state_marker_deg')
                self.pub = rospy.Publisher('/joint_state_marker_deg', Marker, queue_size=10)
                rospy.Subscriber('/joint_states', JointState, self.callback)

                self.base_frame = "base_link"

            def callback(self, msg):
                name_to_position = dict(zip(msg.name, msg.position))

                # Convertir a grados los joints principales
                q1 = math.degrees(name_to_position.get('shoulder_pan_joint', 0))
                q2 = math.degrees(name_to_position.get('shoulder_lift_joint', 0))
                q3 = math.degrees(name_to_position.get('elbow_joint', 0))
                q4 = math.degrees(name_to_position.get('wrist_1_joint', 0))
                q5 = math.degrees(name_to_position.get('wrist_2_joint', 0))
                q6 = math.degrees(name_to_position.get('wrist_3_joint', 0))

                # Gripper (fijo aparte)
                gripper = math.degrees(name_to_position.get('robotiq_85_left_knuckle_joint', 0))

                # Preparar texto alineado
                text = (
                    f"q1: {q1:<7.1f}  q4: {q4:<7.1f}\n"
                    f"q2: {q2:<7.1f}  q5: {q5:<7.1f}\n"
                    f"q3: {q3:<7.1f}  q6: {q6:<7.1f}\n"
                    f"Gripper: {gripper:.1f}\n"
                )

                # Crear el marcador
                marker = Marker()
                marker.header.frame_id = self.base_frame
                marker.header.stamp = rospy.Time.now()
                marker.ns = "joint_text"
                marker.id = 0
                marker.type = Marker.TEXT_VIEW_FACING
                marker.action = Marker.ADD

                marker.pose.position.x = 0.0
                marker.pose.position.y = -0.1
                marker.pose.position.z = 0.3
                marker.pose.orientation.w = 1.0

                marker.scale.z = 0.05
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 1.0
                marker.color.a = 1.0

                marker.text = text

                self.pub.publish(marker)

        if __name__ == '__main__':
            JointStateMarker()
            rospy.spin()

![qs_rad_RViz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/qs_rad_RViz.png)

![qs_deg_RViz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/qs_deg_RViz.png)

### 14)  Incluir Todos los Scripts en el Launch de MoveIt + RViz
- Crear nuevo launch con la ejecucion de los 4 scripts pasados en el launch de MoveIt + Rviz
	- Crear un nuevo archivo llamado ur5_moveit_with_rviz_2.launch
	- Añadir el siguiente codigo en ese archivo

            <launch>
            <arg name="sim" default="true" />
            <arg name="debug" default="false" />

            <!-- Remapea trajectory controller para Gazebo -->
            <remap if="$(arg sim)" from="/scaled_pos_joint_traj_controller/follow_joint_trajectory" to="/eff_joint_traj_controller/follow_joint_trajectory"/>

            <!-- Lanza MoveIt  con la config de Universal Robots-->
            <include file="$(find ur5_moveit_config)/launch/move_group.launch">
                <arg name="debug" value="$(arg debug)" />
            </include>

            <!-- Lanza RViz con la configuración de RVIZ guardada en config -->
            <node name="rviz" pkg="rviz" type="rviz" output="screen"
                    args="-d $(find ur5_v1)/config/config.rviz" />

            <!--______________________________________________________________________-->
            <!-- Nodo RPY en radianes -->
            <node name="rpy_marker_rad" pkg="ur5_v1" type="rpy_marker_rad.py" output="screen">
                <param name="reference_frame" value="base_link"/>
                <param name="target_frame" value="tool0"/>
            </node>

            <!-- Nodo RPY en grados -->
            <node name="rpy_marker_deg" pkg="ur5_v1" type="rpy_marker_deg.py" output="screen">
                <param name="reference_frame" value="base_link"/>
                <param name="target_frame" value="tool0"/>
            </node>

            <!-- Nodo joint_state_marker_rad -->
            <node name="joint_state_marker_rad" pkg="ur5_v1" type="joint_state_marker_rad.py" output="screen" />

            <!-- Nodo joint_state_marker_deg -->
            <node name="joint_state_marker_deg" pkg="ur5_v1" type="joint_state_marker_deg.py" output="screen" />

            </launch>

### 15) Crear Xacro personalizado del UR5 con gripper integrado   
- Crear un nuevo archivo en la carpeta /ur5_v1/urdf llamado "ur5_1_gripper.xacro"	
- Copiar y pegar todo el contenido del archivo existente "ur5_1.xacro"
- Añadir despues de <robot name="ur5_robot"> y el comentario grande, o después de la línea 61 lo siguiente:
        
        <xacro:include filename="$(find ur5_v1)/urdf/eef.xacro"/>

- Reemplazar la línea 6 por la siguiente:
        
        <robot name="ur5_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">	

    
- Crear en la misma carpeta el archivo "eef.xacro"
- En ese archivo pegar el siguiente codigo:        

        <?xml version="1.0" ?>
        <!--End Efector xacro file-->
        <robot name="robotiq_85_gripper" xmlns:xacro="http://ros.org/wiki/xacro">

            
            <!-- Manda a llamar el xacro directo de robotiq_gripper package-->
            <xacro:include filename="$(find robotiq_gripper)/urdf/robotiq_85_gripper.urdf.xacro" />

            <!--This is where is gonna be the base link of the gripper in 
                relation to the tool0(which is the endefector of the robot)-->
            <xacro:robotiq_85_gripper prefix="" parent="tool0" >
                <origin xyz = "0 0 0" rpy = "0 -1.57 0" /> <!--Posicion inicial del gripper-->
            </xacro:robotiq_85_gripper>
        </robot>

### 16) Lanzar el UR5 con gripper en RViz
- Crear un launch para que aparezca el gripper como el End Effector en RViz
- Crear archivo llamado "rviz_ur5_gripper.launch"
- Copiar y pegar el codigo de abajo 

        <?xml version="1.0"?>
        <launch>

            <!-- Associate to the robot description parameter, the urdf file that model the robot-->
            <param name="robot_description" command="$(find xacro)/xacro '$(find ur5_v1)/urdf/ur5_1_gripper.xacro'" /> 

            <!-- Read the joint value-->
            <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
            
            <!-- Visualization in Rviz-->
            <node name="rviz" pkg="rviz" type="rviz" />
            <!--<node name="rviz" pkg="rviz" type="rviz" output="screen"
                args="-d $(find ur5_v1)/config/config.rviz" />
            -->

            <!-- Visualization of the use_gui for playing with joint-->
            <arg name="use_gui" default="true" />
            <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher"  output="screen" unless="$(arg use_gui)" />
            <node name="joint_state_publisher_gui" pkg="joint_state_publisher_gui" type="joint_state_publisher_gui"  output="screen" if="$(arg use_gui)"/>    

        </launch>
    
- Para ejecutarlo y mostrar el gripperEjecutar en terminal:

    cd ~/catkin_ws_1
	catkin_make	
    roslaunch ur5_v1 rviz_ur5_gripper.launch

	- En Gloal Options -> Fixed Frame -> base_link
	- Abajo dar click en Add -> RobotModel

![ur5_gripper_RViz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/ur5_gripper_RViz.png)

### 17) Generar paquete MoveIt para UR5 con gripper
- En esta ruta: /catkin_ws_7/src, crear esta carpeta: ur_gripper_moveit_config	
- Ejecutar en la terminal: roslaunch moveit_setup_assistant setup_assistant.launch
	
	- Create New MoveIt ConfigurationPackage 
	- Escoger la ruta -> /home/gazebo-ros/catkin_ws_1/src/ur5_v1/urdf/ur5_1_gripper.xacro
    - Darle click a Load Files
    - Ir a Self-Collisions -> Generate Collision Matrix
	- Ir a planning group -> 
		- Add Group -> Colocar la sig Configuracion
			- Group Name:	manipulator
			- Kinematic Solver: kdl_kinematics_plugin/KDLKinematicsPlugin
			- Group Default Planner: RRT
			- Add joints -> seleccionar de shoulder_pan_joint al wrist_3_joint (Son 6 en total)-> Save
			- Add link -> base_link, de shoulder_link a wrist_3_link, flange, tool0 (Son 9 en total)-> Save
			
		- Add Group -> Colocar la sig Configuracion
			- Group Name:	gripper
			- Kinematic Solver: none
			- Group Default Planner: none
			- Add joints -> seleccionar robotiq_85_left_knuckle_joint -> Save
			- Add link -> seleccionar todos los robotiq_85_ (Son: 1 base, 4 left y 4 right) -> Save
	
	- Ir a Robot Poses -> Add pose. Añadir las siguientes:
		- "home" - Planning Group: manipulator - Todas art en 0.
		- "up" - Planning Group: manipulator - shoulder_lift_joint=-1.57rad=90°, wrist1= -1.57rad. El resto en 0
		- "open" - Planning Group: gripper - left_knuckle: 0
		- "close" - Planning Group: gripper - left_knuckle: 0.8040
	
	- Ir a End Effectors -> Add ->
		End Effector Name -> robotiq_gripper
		End Effector Group -> gripper
		Parent Link -> tool0
	
	- Ir a  Passive Joints -> 
		- Añadir -> 
			- robotiq_85_left_finger_joint
			- robotiq_85_right_finger_joint
	
	- Ir a controllers 
		- Dejar vacio
		
	- Ir a Author Information y llenar los campos solicitados
	
	- Ir a Configuration Files -> ajustar ruta a /catkin_ws_1/src/ur_gripper_moveit_config -> Generate Packages
	
	- Exit MoveIt Assistant
	
-En la nueva carpeta, buscar el archivo con esta ruta /ur_gripper_moveit_config/config/ros_controllers.yaml. Añadir ahí el siguiente código:

        controller_list:
        - name: "/eff_joint_traj_controller"
        action_ns: follow_joint_trajectory
        type: FollowJointTrajectory
        joints:
            - shoulder_pan_joint
            - shoulder_lift_joint
            - elbow_joint
            - wrist_1_joint
            - wrist_2_joint
            - wrist_3_joint

        - name: "/gripper_controller"
        action_ns: follow_joint_trajectory
        type: FollowJointTrajectory
        joints:
            - robotiq_85_left_knuckle_joint

- Ahora en esta ruta /ur_gripper_moveit_config/launch, crear un  nuevo archivo llamado 		"ur5_robot_moveit_controller_manager.launch"
- Añadir ahi el siguiente código en el nuevo archivo:

        <launch>
            <arg name = "moveit_controller_manager" default "moveit_simple_controller_manager/MoveItSimpleControllerManager" />
            <param name = "moveit_controller_manager" value = "$(arg moveit_controller_manager)" />

            <!-- load ros_controllers to the param server -->
            <rosparam file ="$(find ur_gripper_moveit_config)/config/ros_controllers.yaml" />
        </launch>


- Revisar que esta ocurriendo con este package por lo tanto lanzamos el demo del package desde una terminal:

		roslaunch ur_gripper_moveit_config demo.launch        

![ur5_gripper_moveit_rviz](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/ur5_gripper_moveit_rviz.png)
    

### 18) Ajustar URDFs para visualización correcta del gripper
Estos pasos corrigen errores comunes relacionados con el uso del plugin y la configuración de los *joints* del gripper Robotiq 85. Realizar ajustes para que se visualice correctamente el gripper.

---
Reemplazar plugin incorrecto en archivos URDF. 
- Para ello abre y edita los siguientes archivos:

    - /home/gazebo-ros/catkin_ws/src/ur_gripper_moveit_config/config/gazebo_ur5_robot.urdf
    - /home/gazebo-ros/catkin_ws/src/robotiq_gripper/urdf/robotiq_85_gripper.transmission.xacro

- Busca y elimina todas las instancias del siguiente plugin incorrecto:

    libroboticsgroup_upatras_gazebo_mimic_joint_plugin

- Y reemplázalas por el plugin correcto:

    libroboticsgroup_gazebo_mimic_joint_plugin

- Abre y edita los siguientes archivos:

    - /home/gazebo-ros/catkin_ws/src/ur_gripper_moveit_config/config/gazebo_ur5_robot.urdf
    - /home/gazebo-ros/catkin_ws/src/robotiq_gripper/urdf/robotiq_85_gripper.urdf.xacro

- Busca las líneas que definen los siguientes joints:

        <joint name="robotiq_85_left_finger_joint" type="continuous">
        <joint name="${prefix}robotiq_85_left_finger_joint" type="continuous">
        <joint name="robotiq_85_right_finger_joint" type="continuous">
        <joint name="${prefix}robotiq_85_right_finger_joint" type="continuous">

- Y cámbialas a:

        <joint name="robotiq_85_left_finger_joint" type="fixed">
        <joint name="${prefix}robotiq_85_left_finger_joint" type="fixed">
        <joint name="robotiq_85_right_finger_joint" type="fixed">
        <joint name="${prefix}robotiq_85_right_finger_joint" type="fixed">

Esto evitará problemas con la simulación al corregir definiciones erróneas en los dedos del gripper. Finalmente, compilamos los cambios en la terminal:

    cd ~/catkin_ws_1
    catkin_make


### 19) Crear archivo de controladores con soporte para el gripper

Ahora crearás un nuevo archivo de configuración para controlar el gripper **Robotiq 85** junto con el UR5.

---

- Haz una copia del archivo actual de controladores:

    cd ~/catkin_ws/src/ur5_v1/config
    cp ur5_controllers.yaml ur5_gripper_controllers.yaml

- Abre ur5_gripper_controllers.yaml y añade al final del archivo el siguiente bloque de código:

    gripper_controller:
    type: position_controllers/JointTrajectoryController
    joints:
        - robotiq_85_left_knuckle_joint
    gains:
        robotiq_85_left_knuckle_joint: {p: 100,  d: 1, i: 1, i_clamp: 1}
    constraints:
        goal_time: 0.6
        stopped_velocity_tolerance: 0.05
        robotiq_85_left_knuckle_joint: {trajectory: 0.1, goal: 0.1}
    stop_trajectory_duration: 0.5
    state_publish_rate:  25
    action_monitor_rate: 10

Con esto ya tienes el controlador del gripper configurado.
Este bloque define una trayectoria controlada para el robotiq_85_left_knuckle_joint, permitiendo el control básico de apertura y cierre del gripper en simulación.    


### 20) Nuevo archivo Launch UR5 con gripper en Gazebo

Vamos a generar un nuevo `launch` que cargue el modelo del UR5 con el **gripper Robotiq 85** en Gazebo.

---

- Ubícate en la carpeta de `launch` de tu paquete y crea una copia del archivo existente:

    cd ~/catkin_ws/src/ur5_v1/launch
    cp ur5_gazebo_2.launch ur5_gripper_gazebo_1.launch

- Cambiar el archivo .xacro (línea 5)
    
    De:

        <param name="robot_description" command="$(find xacro)/xacro '$(find ur5_v1)/urdf/ur5_1.xacro'" />

    A:

        <param name="robot_description" command="$(find xacro)/xacro '$(find ur5_v1)/urdf/ur5_1_gripper.xacro'" />

-Cambiar el archivo de controladores (línea 32), poner esta línea: 

    <rosparam file="$(find ur5_v1)/config/ur5_controllers.yaml" command="load"/>

- Añadir el controlador del gripper a los args
    
    De:

        args="joint_state_controller 
        eff_joint_traj_controller 
        --timeout 60" />

    A:

        args="joint_state_controller 
        eff_joint_traj_controller 
        gripper_controller 
        --timeout 60" />


Aqui está el código final del archivo ur5_gripper_gazebo_1.launch

    <?xml version="1.0"?>
    <launch>

        <!-- Cargar el modelo UR5 -->
        <param name="robot_description" command="$(find xacro)/xacro '$(find ur5_v1)/urdf/ur5_1_gripper.xacro'" /> 
    
        <!--Spawn Robot in Gazebo-->
        <arg name="x" default="0" />
        <arg name="y" default="0" />
        <arg name="z" default="1.01" />

        <!-- put world file as argument-->
        <arg name="world_file" default = "$(find ur5_v1)/worlds/my_custom_world.world" />

        <!-- Lanzar Gazebo con tu mundo -->
        <include file="$(find gazebo_ros)/launch/empty_world.launch">
            <arg name="paused" value="false"/>
            <arg name="gui" value="true"/>
            <arg name="use_sim_time" value="true"/>
            <arg name="world_name" value="$(arg world_file)"/>
        </include>

        <!-- Publicar estados del robot -->
        <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher"/>
        
        <!-- Spawn del robot -->
        <node name="spawn_the_robot" pkg="gazebo_ros" type="spawn_model"  output="screen" args="-urdf -param robot_description -model ur5 -x $(arg x) -y $(arg y) -z $(arg z)" />  

        <!-- Controladores -->
        <rosparam file="$(find ur5_v1)/config/ur5_gripper_controllers.yaml" command="load"/>
    
        <node name="controller_spawner" pkg="controller_manager" type="spawner"
        args="joint_state_controller 
        eff_joint_traj_controller 
        gripper_controller
        --timeout 60" />
        
        <!-- Postura inicial -->
        <node name="set_initial_pose" pkg="ur5_v1" type="ur5_set_initial_pose.py" output="screen"/>

    </launch>    

### 21) Crear nuevo Launch para lanzar UR5 con gripper en MoveIt y RViz

Este archivo permitirá levantar **MoveIt** junto con **RViz** y tu configuración personalizada con el **gripper Robotiq 85**.

---

- Copiar archivo base

    cd ~/catkin_ws_1/src/ur5_v1/launch
    cp ur5_moveit_with_rviz_2.launch ur5_gripper_moveit_with_rviz_1.launch

- Modificar archivo .launch. Cambiar la carpeta del paquete de configuración de MoveIt

    De:

        <include file="$(find ur5_moveit_config)/launch/move_group.launch">
    
    A:

        <include file="$(find ur_gripper_moveit_config)/launch/move_group.launch">



- El código final del archivo ur5_gripper_moveit_with_rviz_1.launch es este: 

        <launch>
        <arg name="sim" default="true" />
        <arg name="debug" default="false" />

        <!-- Remapea trajectory controller para Gazebo -->
        <remap if="$(arg sim)" from="/scaled_pos_joint_traj_controller/follow_joint_trajectory" to="/eff_joint_traj_controller/follow_joint_trajectory"/>

        <!-- Lanza MoveIt con la configuración hecha y ubicada en ur_gripper_moveit_config -->
        <include file="$(find ur_gripper_moveit_config)/launch/move_group.launch">
            <arg name="debug" value="$(arg debug)" />
        </include>

        <!-- Lanza RViz con la configuración visual guardada -->
        <node name="rviz" pkg="rviz" type="rviz" output="screen"
                args="-d $(find ur5_v1)/config/config.rviz" />

        <!--______________________________________________________________________-->
        <!-- Nodo RPY en radianes -->
        <node name="rpy_marker_rad" pkg="ur5_v1" type="rpy_marker_rad.py" output="screen">
            <param name="reference_frame" value="base_link"/>
            <param name="target_frame" value="tool0"/>
        </node>

        <!-- Nodo RPY en grados -->
        <node name="rpy_marker_deg" pkg="ur5_v1" type="rpy_marker_deg.py" output="screen">
            <param name="reference_frame" value="base_link"/>
            <param name="target_frame" value="tool0"/>
        </node>

        <!-- Nodo joint_state_marker_rad -->
        <node name="joint_state_marker_rad" pkg="ur5_v1" type="joint_state_marker_rad.py" output="screen" />

        <!-- Nodo joint_state_marker_deg -->
        <node name="joint_state_marker_deg" pkg="ur5_v1" type="joint_state_marker_deg.py" output="screen" />

        </launch>

### 22) Corregir estados predefinidos del gripper en SRDF

Vamos a simplificar los estados `open` y `close` de la garra eliminando los *joints* que no se usan, y dejar solo el que controla directamente el movimiento útil: `robotiq_85_left_knuckle_joint`.

---

- Abrir el archivo: 

    /home/gazebo-ros/catkin_ws/src/ur_gripper_moveit_config/config/ur5_with_gripper.srdf

- Reemplazar el bloque actual:

        <group_state name="open" group="gripper">
            <joint name="robotiq_85_left_finger_joint" value="0"/>
            <joint name="robotiq_85_left_knuckle_joint" value="0"/>
            <joint name="robotiq_85_right_finger_joint" value="0"/>
        </group_state>
        <group_state name="close" group="gripper">
            <joint name="robotiq_85_left_finger_joint" value="0"/>
            <joint name="robotiq_85_left_knuckle_joint" value="0.803"/>
            <joint name="robotiq_85_right_finger_joint" value="0"/>
        </group_state>

- Reemplazar por:

        <group_state name="open" group="gripper">
            <joint name="robotiq_85_left_knuckle_joint" value="0"/>
        </group_state>
        <group_state name="close" group="gripper">
            <joint name="robotiq_85_left_knuckle_joint" value="0.803"/>
        </group_state>

### 23) Ajustar rotación inicial del gripper en eef.xacro

Para ajustar la orientación con la que el *gripper* se monta inicialmente al UR5, es necesario modificar los ángulos **RPY** (roll, pitch, yaw) directamente en el archivo `eef.xacro`.

---

- Archivo a editar: 

    ~/catkin_ws/src/ur5_v1/urdf/eef.xacro

- Busca la línea que se encarga del posicionamiento y orientación inicial del gripper:

        <origin xyz="0 0 0" rpy="0 -1.57 1.57" /> <!-- Posición inicial del gripper -->

- Solo modifica los valores de rpy (en radianes) para rotar el gripper como desees. Por ejemplo:
    - Rotación roll 90° → 1.57 0 0
    - Rotación pitch -90° → 0 -1.57 0

- Combina según tu necesidad con: <origin xyz="..." rpy="..." />

### 24) Crear launch file para spawn del UR5 con gripper y objetos en Gazebo

---


Unificar el lanzamiento del robot con gripper y los objetos de simulación en un solo archivo `launch`.

---

- Ruta del nuevo archivo:

    ~/catkin_ws/src/ur5_v1/launch/ur5_gripper_gazebo_add_objects_1.launch

- Pegar el siguiente código en el nuevo archivo:

        <?xml version="1.0"?>
        <launch>
            <!-- Incluye el launch principal del robot con gripper -->
            <include file="$(find ur5_v1)/launch/ur5_gripper_gazebo_1.launch" />

            <!-- Ejecuta el script delay que luego lanza spawn_objects.launch -->
            <node name="delayed_spawn" pkg="ur5_v1" type="delayed_spawn.py" output="screen" />
        </launch>

- Lanzar en una terminal Gazebo con el UR5 + gripper + objetos:

    roslaunch ur5_v1 ur5_gripper_gazebo_add_objects_1.launch

![gazebo_completo](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/gazebo_completo.png)    

- Lanzar en una terminal diferente RViz + MoveIt:

    roslaunch ur5_v1 ur5_gripper_moveit_with_rviz_1.launch

![rviz_completo](https://github.com/ricardoRamoM/tutorial_UR5_pick_and_place_Gazebo_and_Real_ROS/blob/master/media/images/rviz_completo.png) 

---

A partir de ahora, usaremos el archivo ur5_gripper_gazebo_add_objects_1.launch como el launcher principal para levantar el robot y los objetos en Gazebo.

Después, abrimos otra terminal y ejecutamos ur5_gripper_moveit_with_rviz_1.launch para usar MoveIt con RViz.

### 25) Crear script en Python para mover el UR5


### 32. ▶️ Ejecución del Script de Pick & Place

En terminal:

    roslaunch ur5_moveit_config demo.launch

Luego, en otra terminal:

    rosrun <tu_paquete> ur5_pick_and_place.py


## 🤖 VI-Conexión con el Robot Físico UR5

    Configuración de red y comunicación con el UR5

    Lanzar el robot real con MoveIt

    Adaptar y ejecutar el mismo script Python en el robot físico

## 🧩 VII-Estructura del Código y Explicación del Script

    Desglose del script Python

    Cómo se comunican los nodos, controlan los movimientos y se integran con MoveIt























## ✅ VIII-Conclusión

Como se pudo observar la implementación de la simulacion de un Pick and Place a traves de ROS con el entorno de simulacion Gazebo fue exitosa con la previa configuracion de todo el workspace para su debido funcionamiento, con esto se pudo denotar el fuerte uso y aplicaciones que tiene el Robotics Operating System el cual nos ayuda a la comunicación y descripción de elementos de robotica que pueden tener un uso simulado como con su implementación física. Asi con esto  El UR5 es un brazo robotico con 6GDL el cual nos ayudo a comprender mas sobre como funciona ROS a la hora de marcar trayectorias y la resolucion de ciertas posiciones para nuestro espacio de trabajo.

Futuras versiones del trabajo implementaran mejoras en la resolucion de la cinematica inversa del entorno que tenemos, asi con esto la adaptación de MoveIt con los parametros de nuestro workspace y el añadimiento del gripper en la implementación fisica.

## 🔜 IX-Trabajo futuro

- [ ] Implementacion fisica del gripper Robotiq 2F-85
- [ ] Resolucion de la cinematica Inversa de acuerdo a las limitaciones del propio WorkSpace
- [ ] Pick And Place mejorado para una mejor y más rápida trayectoria con movimientos mas adecuados y suaves para su rapida solución.

## ⚠️ X-Advertencia

Como se indica en la licencia MIT, este software/hardware se proporciona sin ningún tipo de garantía. Por lo tanto, ningún colaborador es responsable de cualquier daño a tus componentes, materiales, PC, etc...
## 📚 XI-Recursos Adicionales

## 👥 XII-Autores del proyecto

- [ ] Ricardo Ramos Morales
- [ ] David León Céspedes

## 📬 XIII-Contacto

¿Tienes dudas o sugerencias?

    📧 Correo electrónico: ricardo.ramosms@udlap.mx
    📧 Correo electrónico: david.leoncs@udlap.mx

