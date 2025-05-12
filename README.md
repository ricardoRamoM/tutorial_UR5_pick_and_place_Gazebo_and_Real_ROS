# {Tutorial: Simulación y Ejecución de un Pick and Place con el Brazo UR5 y Gripper Robotiq usando ROS Noetic, Gazebo y MoveIt}

Este tutorial te guía paso a paso para simular y ejecutar una tarea de pick and place utilizando el brazo robótico UR5 y el gripper Robotiq 2F-85, integrando herramientas como Gazebo, MoveIt, RViz y Python en ROS Noetic sobre Ubuntu 20.04. Comenzarás configurando un entorno de simulación funcional y terminarás controlando el robot físico desde una computadora remota, aplicando los mismos scripts desarrollados en el entorno virtual. Ideal para quienes buscan unir teoría, simulación y práctica real en robótica colaborativa.

---
## 📑 Índice

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
   - [12. Ver Orientación en RPY en RViz](#12-ver-orientación-en-rpy-en-rviz)  
   - [13. Ver Articulaciones q1-q6 en RViz](#13-ver-articulaciones-q1-q6-en-rviz)  
   - [14. Incluir Todos los Scripts en el Launch de MoveIt + RViz](#14-incluir-todos-los-scripts-en-el-launch-de-moveit--rviz)  
6. [🔌 VI - Conexión con el Robot Físico UR5](#-vii--conexión-con-el-robot-físico-ur5)  
   - [1. Configuración de red y comunicación con el UR5](#1-configuración-de-red-y-comunicación-con-el-ur5)  
   - [2. Lanzar el robot real con MoveIt](#2-lanzar-el-robot-real-con-moveit)  
   - [3. Adaptar y ejecutar el script en el robot físico](#3-adaptar-y-ejecutar-el-script-en-el-robot-físico)  

7. [🧩 VII - Estructura del Código y Explicación del Script](#-viii--estructura-del-código-y-explicación-del-script)  
   - [1. Desglose del script Python](#1-desglose-del-script-python)  
   - [2. Comunicación de nodos y control de movimientos](#2-comunicación-de-nodos-y-control-de-movimientos)  

8. [✅ VIII - Conclusión](#-ix--conclusión)  
9. [🚀 IX - Mejoras Futuras](#-x--mejoras-futuras)  
10. [⚠️ X - Advertencia](#-xi--advertencia)  
11. [📚 XI - Recursos Adicionales](#-xii--recursos-adicionales)  
12. [👥 XII - Autores del Proyecto](#-xiii--autores-del-proyecto)  
13. [📬 XIII - Contacto](#-xiv--contacto)  

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

## 🛠️ IV-Configuración del entorno 
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
- { } Nota 2: Usa un publish_rate alto (125 Hz), lo que puede mejorar la suavidad en simulación. Se puede usar un publish_rate más bajo (50 Hz), suficiente para pruebas, pero menos suave. Esto se ve en esta linea:
    publish_rate: &loop_hz 125 

### 5) Crear Modelos SDF de Objetos


### 6) Crear Script para Spawnear Objetos (sin Launch)


### 7) Crear Launches de Simulación y Planificación


### 8) Spawnear Objetos Desde Launch


### 9) Añadir Delay Antes de Spawnear Objetos


### 10) Establecer Pose Inicial del Robot con Python


### 11) Lanzar Simulación y Spawneo de Objetos


### 12) Ver Orientación en RPY en RViz


### 13) Ver Articulaciones q1-q6 en RViz


### 14)  Incluir Todos los Scripts en el Launch de MoveIt + RViz





## 🤖 VI-Conexión con el Robot Físico UR5

    Configuración de red y comunicación con el UR5

    Lanzar el robot real con MoveIt

    Adaptar y ejecutar el mismo script Python en el robot físico

## 🧩 VII-Estructura del Código y Explicación del Script

    Desglose del script Python

    Cómo se comunican los nodos, controlan los movimientos y se integran con MoveIt























## ✅ VIII-Conclusión

Resumen de lo que se logró construir, aprendizajes obtenidos y posibles mejoras o versiones futuras del proyecto.

Como se pudo observar la implementación de la simulacion de un Pick and Place a traves de ROS con el entorno de simulacion Gazebo fue exitosa con la previa configuracion de todo el workspace para su debido funcionamiento, con esto se pudo denotar el fuerte uso y aplicaciones que tiene el Robotics Operating System el cual nos ayuda a la comunicación y descripción de elementos de robotica que pueden tener un uso simulado como con su implementación física. Asi con esto  El UR5 es un brazo robotico con 6GDL el cual nos ayudo a comprender mas sobre como funciona ROS a la hora de marcar trayectorias y la resolucion de ciertas posiciones para nuestro espacio de trabajo.

Futuras versiones del trabajo implementaran mejoras en la resolucion de la cinematica inversa del entorno que tenemos, asi con esto la adaptación de MoveIt con los parametros de nuestro workspace y el añadimiento del gripper en la implementación fisica.

## 🔜 IX-Mejoras futuras

- [ ] Implementacion fisica del gripper Robotiq 2F-85
- [ ] Resolucion de la cinematica Inversa de acuerdo a las limitaciones del propio WorkSpace
- [ ] Pick And Place mejorado para una mejor y mas rapida trayectoria con movimientos mas adecuados y suaves para su rapida solución.

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

