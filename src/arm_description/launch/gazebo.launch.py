#!/usr/bin/env python3
"""
gazebo.launch.py
================
Lanza el manipulador en Gazebo Harmonic (ROS 2 Jazzy).

Argumentos:
  modo:=gazebo_torque    (default) control de PAR: effort_controller +
                         control_torque.py (PID por articulación + G(q))
  modo:=gazebo_posicion  control cinemático original (JointTrajectoryController),
                         compatible con teach_repeat.py y teach_pendant_gui.py
  interfaz:=true|false   abre interfaz_trayectoria.py (solo en modo torque)

Secuencia:
  spawn del robot → joint_state_broadcaster → controladores → nodo de control
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument,
                            IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

PACKAGE = 'arm_description'


def _configurar(context):
    modo = LaunchConfiguration('modo').perform(context)
    interfaz = LaunchConfiguration('interfaz').perform(context).lower() in ('true', '1')
    if modo not in ('gazebo_torque', 'gazebo_posicion'):
        raise RuntimeError(f'modo:={modo} no válido (gazebo_torque | gazebo_posicion)')

    pkg_share = get_package_share_directory(PACKAGE)
    urdf_file = os.path.join(pkg_share, 'urdf', 'arm_description.xacro')
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' modo:=', modo]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'arm_serial', '-topic', 'robot_description'],
        output='screen',
    )

    # Reloj de simulación → ROS (todos los nodos usan use_sim_time)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    def spawner(*controladores):
        return Node(
            package='controller_manager',
            executable='spawner',
            arguments=[*controladores, '--controller-manager', '/controller_manager'],
            output='screen',
        )

    jsb = spawner('joint_state_broadcaster')
    if modo == 'gazebo_torque':
        controladores = spawner('effort_controller', 'gripper_controller')
        despues = [Node(
            package=PACKAGE,
            executable='control_torque.py',
            parameters=[{'use_sim_time': True}],
            output='screen',
        )]
        if interfaz:
            despues.append(Node(package=PACKAGE, executable='interfaz_trayectoria.py',
                                output='screen'))
    else:
        controladores = spawner('arm_controller', 'gripper_controller')
        despues = []

    acciones = [
        robot_state_publisher,
        gazebo,
        bridge,
        spawn_entity,
        RegisterEventHandler(OnProcessExit(target_action=spawn_entity, on_exit=[jsb])),
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[controladores])),
    ]
    if despues:
        acciones.append(RegisterEventHandler(
            OnProcessExit(target_action=controladores, on_exit=despues)))
    return acciones


def generate_launch_description():
    # Gazebo necesita la carpeta share/ para resolver package://arm_description/meshes
    workspace_share = os.path.dirname(get_package_share_directory(PACKAGE))
    return LaunchDescription([
        DeclareLaunchArgument('modo', default_value='gazebo_torque',
                              description='gazebo_torque | gazebo_posicion'),
        DeclareLaunchArgument('interfaz', default_value='true',
                              description='abrir interfaz_trayectoria.py (modo torque)'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', workspace_share),
        OpaqueFunction(function=_configurar),
    ])
