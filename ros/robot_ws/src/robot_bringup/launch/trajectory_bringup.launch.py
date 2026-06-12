#!/usr/bin/env python3

"""
Launch principal del proyecto.

Este archivo abre todo lo necesario para visualizar y mover el robot en RViz:

  1. Carga el modelo URDF del robot.
  2. Publica la descripción del robot con robot_state_publisher.
  3. Abre RViz con una configuración guardada.
  4. Ejecuta el nodo de cinemática que recibe puntos y mueve las juntas.
"""

import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def leer_archivo_texto(ruta_archivo: str) -> str:
  """
  Lee un archivo de texto y regresa su contenido.

  En este caso se usa para cargar el URDF completo como string, porque
  robot_state_publisher recibe el modelo mediante el parámetro
  robot_description.
  """
  with open(ruta_archivo, "r") as archivo:
    return archivo.read()


def crear_nodo_rviz(ruta_configuracion: str) -> Node:
  """
  Crea el nodo de RViz.

  El argumento -d indica qué archivo de configuración .rviz se debe abrir.
  """
  return Node(
    package="rviz2",
    executable="rviz2",
    name="rviz_visualizador",
    arguments=["-d", ruta_configuracion],
    output="screen"
  )


def crear_nodo_descripcion_robot(descripcion_urdf: str) -> Node:
  """
  Crea el nodo robot_state_publisher.

  Este nodo toma el URDF y calcula las transformaciones entre links.
  Gracias a esto, RViz sabe cómo dibujar el robot cuando cambian las juntas.
  """
  return Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    name="publicador_estado_robot",
    parameters=[
      {"robot_description": descripcion_urdf}
    ],
    output="screen"
  )


def crear_nodo_cinematica() -> Node:
  """
  Crea el nodo de cinemática del proyecto.

  Este nodo escucha:
    /clicked_point
    /goals_twist

  Y publica:
    /joint_states

  Por eso es el encargado de que el robot se mueva cuando seleccionamos
  puntos en RViz o mandamos coordenadas desde terminal.
  """
  return Node(
    package="robot_kinematics",
    executable="trajectory_publisher",
    name="control_trayectoria_rrr",
    output="screen"
  )


def generate_launch_description():
  """
  Función obligatoria para archivos launch de ROS 2.

  ROS llama esta función para saber qué nodos debe ejecutar.
  """
  paquete_descripcion = get_package_share_directory("robot_description")

  ruta_urdf = os.path.join(
    paquete_descripcion,
    "urdf",
    "robot_rrr.urdf"
  )

  ruta_rviz = os.path.join(
    paquete_descripcion,
    "rviz",
    "rviz.conf.rviz"
  )

  descripcion_robot = leer_archivo_texto(ruta_urdf)

  nodo_rviz = crear_nodo_rviz(ruta_rviz)
  nodo_robot_state = crear_nodo_descripcion_robot(descripcion_robot)
  nodo_cinematica = crear_nodo_cinematica()

  return LaunchDescription([
    nodo_robot_state,
    nodo_cinematica,
    nodo_rviz
  ])