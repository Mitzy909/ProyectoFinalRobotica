#!/usr/bin/env python3

"""
Publicador de posición final para el robot RRR.

Este nodo recibe un punto objetivo y calcula una trayectoria completa usando
la clase Robot. A diferencia del publicador de trayectoria principal, este
archivo también puede usarse como una versión sencilla para mover el robot
publicando directamente en /joint_states.

Entradas:
  /clicked_point -> punto seleccionado desde RViz
  /goals_twist   -> coordenadas mandadas desde terminal

Salida:
  /joint_states  -> posiciones articulares del robot
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState

from robot_kinematics.kinematics import Robot


class PublicadorPosicionFinal(Node):
  """
  Nodo que transforma objetivos cartesianos en posiciones articulares.

  El robot tiene tres juntas:
    base_joint -> giro de base
    joint1     -> primer eslabón
    joint2     -> segundo eslabón
  """

  def __init__(self):
    super().__init__("publicador_posicion_final_rrr")

    # Objeto que contiene la cinemática directa, inversa y generación de trayectoria.
    self.robot = Robot()

    # Nombres exactos de las juntas del URDF.
    self.nombres_juntas = [
      "base_joint",
      "joint1",
      "joint2"
    ]

    # Postura inicial del robot en radianes.
    # Orden: [base_joint, joint1, joint2].
    self.q_actual = [0.0, 0.0, 0.0]

    # Bandera para evitar que se acepte un nuevo objetivo mientras ya se mueve.
    self.movimiento_activo = False

    # Índice de la muestra actual dentro de la trayectoria calculada.
    self.muestra_actual = 0

    # Timer que se activa cuando existe una trayectoria por publicar.
    self.timer_movimiento = None

    # Mensaje JointState reutilizable.
    self.mensaje_juntas = JointState()
    self.mensaje_juntas.name = self.nombres_juntas

    # Publicador de estados articulares. RViz lo usa para actualizar el modelo.
    self.publicador_juntas = self.create_publisher(
      JointState,
      "/joint_states",
      10
    )

    # Suscriptor para puntos seleccionados con Publish Point en RViz.
    self.subscriptor_click = self.create_subscription(
      PointStamped,
      "/clicked_point",
      self.procesar_punto_rviz,
      10
    )

    # Suscriptor para objetivos enviados manualmente desde terminal.
    self.subscriptor_terminal = self.create_subscription(
      Twist,
      "/goals_twist",
      self.procesar_punto_terminal,
      10
    )

    # Timer permanente que mantiene publicada la última postura del robot.
    # Sin esto, RViz puede tardar en mostrar la configuración inicial.
    self.timer_estado = self.create_timer(
      0.1,
      self.publicar_postura_actual
    )

    self.get_logger().info(
      "Publicador de posición final listo. Usa Publish Point en RViz "
      "o publica un objetivo en /goals_twist."
    )

  def publicar_postura_actual(self):
    """
    Publica la postura actual mientras no hay movimiento activo.
    """
    if self.movimiento_activo:
      return

    self._enviar_joint_state(self.q_actual)

  def _enviar_joint_state(self, q):
    """
    Envía un mensaje JointState a ROS.

    q debe contener tres valores:
      [base_joint, joint1, joint2]
    """
    self.mensaje_juntas.header.stamp = self.get_clock().now().to_msg()
    self.mensaje_juntas.position = list(q)

    self.publicador_juntas.publish(self.mensaje_juntas)

  def procesar_punto_rviz(self, msg: PointStamped):
    """
    Recibe el punto publicado desde RViz.

    RViz manda el punto como PointStamped:
      msg.point.x
      msg.point.y
      msg.point.z
    """
    if self.movimiento_activo:
      self.get_logger().warn(
        "Se ignoró el punto porque el robot aún está en movimiento."
      )
      return

    x = float(msg.point.x)
    y = float(msg.point.y)
    z = float(msg.point.z)

    self.get_logger().info(
      "Objetivo desde RViz: x={:.3f}, y={:.3f}, z={:.3f}".format(
        x, y, z
      )
    )

    self.calcular_y_arrancar_trayectoria(x, y, z)

  def procesar_punto_terminal(self, msg: Twist):
    """
    Recibe coordenadas desde terminal usando /goals_twist.

    En este proyecto solo se usa la parte linear:
      linear.x -> coordenada x
      linear.y -> coordenada y
      linear.z -> coordenada z
    """
    if self.movimiento_activo:
      self.get_logger().warn(
        "Se ignoró el objetivo porque ya hay una trayectoria activa."
      )
      return

    x = float(msg.linear.x)
    y = float(msg.linear.y)
    z = float(msg.linear.z)

    self.get_logger().info(
      "Objetivo desde terminal: x={:.3f}, y={:.3f}, z={:.3f}".format(
        x, y, z
      )
    )

    self.calcular_y_arrancar_trayectoria(x, y, z)

  def calcular_y_arrancar_trayectoria(self, x: float, y: float, z: float):
    """
    Calcula la trayectoria articular hacia un punto cartesiano.

    La clase Robot se encarga de:
      1. Resolver la cinemática inversa.
      2. Generar una trayectoria suave.
      3. Guardar las matrices th_m, th_dot_m y th_dot_dot_m.
    """
    punto_objetivo = (float(x), float(y), float(z))

    try:
      self.robot.def_tray(
        t_f=2.0,
        frec=30.0,
        th_i=tuple(self.q_actual),
        xi_f=punto_objetivo
      )

    except ValueError as error:
      self.get_logger().warn(str(error))
      self.get_logger().warn(
        "No se pudo calcular una trayectoria válida. "
        "Selecciona un punto dentro del alcance del robot."
      )
      return

    self.get_logger().info(
      "Efector final calculado: x={:.3f}, y={:.3f}, z={:.3f}".format(
        float(self.robot.xi_m[0, -1]),
        float(self.robot.xi_m[1, -1]),
        float(self.robot.xi_m[2, -1])
      )
    )

    self.get_logger().info(
      "Juntas finales: base_joint={:.3f}, joint1={:.3f}, joint2={:.3f}".format(
        float(self.robot.th_m[0, -1]),
        float(self.robot.th_m[1, -1]),
        float(self.robot.th_m[2, -1])
      )
    )

    # Activa el modo trayectoria.
    self.movimiento_activo = True
    self.muestra_actual = 0

    # Si ya existía un timer anterior, se elimina para evitar duplicados.
    if self.timer_movimiento is not None:
      self.timer_movimiento.destroy()
      self.timer_movimiento = None

    # Crea un timer que publicará la trayectoria muestra por muestra.
    self.timer_movimiento = self.create_timer(
      self.robot.dt,
      self.publicar_muestra_trayectoria
    )

  def publicar_muestra_trayectoria(self):
    """
    Publica la siguiente muestra de la trayectoria.

    Cada llamada avanza el robot un paso en RViz.
    """
    if self.muestra_actual >= self.robot.muestras:
      self.detener_trayectoria()
      return

    self.q_actual = [
      float(self.robot.th_m[0, self.muestra_actual]),
      float(self.robot.th_m[1, self.muestra_actual]),
      float(self.robot.th_m[2, self.muestra_actual])
    ]

    self._enviar_joint_state(self.q_actual)

    self.muestra_actual += 1

    if self.muestra_actual >= self.robot.muestras:
      self.detener_trayectoria()

  def detener_trayectoria(self):
    """
    Termina el movimiento y destruye el timer de trayectoria.
    """
    self.movimiento_activo = False

    if self.timer_movimiento is not None:
      self.timer_movimiento.destroy()
      self.timer_movimiento = None

    self.get_logger().info("Movimiento terminado.")


def main():
  rclpy.init()

  nodo = PublicadorPosicionFinal()

  try:
    rclpy.spin(nodo)
  except KeyboardInterrupt:
    pass
  finally:
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()