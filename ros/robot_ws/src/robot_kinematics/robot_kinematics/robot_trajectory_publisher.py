#!/usr/bin/env python3

"""
Nodo encargado de mover el robot en RViz.

Este nodo recibe objetivos de dos formas:
  1. Desde RViz usando la herramienta Publish Point.
  2. Desde terminal usando el tópico /goals_twist.

Después calcula la trayectoria con la clase Robot y publica las posiciones
de las juntas en /joint_states para que robot_state_publisher actualice
el modelo visual en RViz.
"""

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped, Twist
from sensor_msgs.msg import JointState

from robot_kinematics.kinematics import Robot


class PublicadorTrayectoria(Node):
  """
  Nodo principal de trayectoria.

  Suscribe:
    /clicked_point -> puntos seleccionados en RViz.
    /goals_twist   -> coordenadas mandadas desde terminal.

  Publica:
    /joint_states  -> posición actual de base_joint, joint1 y joint2.
  """

  def __init__(self):
    super().__init__("publicador_trayectoria_rrr")

    # Modelo cinemático del robot. Aquí están fk, ik y generación de trayectoria.
    self.robot = Robot()

    # Nombres exactos de las juntas definidos en el URDF.
    # Si estos nombres no coinciden con el URDF, RViz no mueve el robot.
    self.joint_names = ["base_joint", "joint1", "joint2"]

    # Estado articular inicial del robot.
    # Orden: [base_joint, joint1, joint2].
    self.q_actual = [0.0, 0.0, 0.0]

    # Variables de control interno para saber si el robot está moviéndose.
    self.en_movimiento = False
    self.indice_trayectoria = 0
    self.timer_trayectoria = None

    # Mensaje reutilizable para publicar las juntas.
    self.msg_juntas = JointState()
    self.msg_juntas.name = self.joint_names

    # Publicador que alimenta a robot_state_publisher.
    self.pub_juntas = self.create_publisher(
      JointState,
      "/joint_states",
      10
    )

    # Suscriptor para coordenadas enviadas desde terminal.
    self.sub_objetivo = self.create_subscription(
      Twist,
      "/goals_twist",
      self.recibir_objetivo_terminal,
      10
    )

    # Suscriptor para puntos seleccionados en RViz.
    self.sub_click = self.create_subscription(
      PointStamped,
      "/clicked_point",
      self.recibir_objetivo_rviz,
      10
    )

    # También escuchamos /joint_states para conservar la última postura.
    # Esto ayuda si el robot fue movido por otra interfaz.
    self.sub_estado = self.create_subscription(
      JointState,
      "/joint_states",
      self.actualizar_estado_actual,
      10
    )

    # Publica periódicamente la postura actual cuando el robot está quieto.
    # Así RViz conserva visible la última configuración.
    self.timer_estado = self.create_timer(
      0.1,
      self.publicar_estado_actual
    )

    self.get_logger().info(
      "Nodo de trayectoria listo. Usa Publish Point en RViz "
      "o publica coordenadas en /goals_twist."
    )

  def publicar_estado_actual(self):
    """
    Publica la postura actual del robot cuando no hay trayectoria activa.

    Esto funciona como un pequeño 'latido' para mantener actualizado RViz.
    """
    if self.en_movimiento:
      return

    self._publicar_juntas(self.q_actual)

  def _publicar_juntas(self, posiciones):
    """
    Publica un JointState.

    posiciones debe tener tres valores:
      [base_joint, joint1, joint2]
    """
    self.msg_juntas.header.stamp = self.get_clock().now().to_msg()
    self.msg_juntas.position = list(posiciones)
    self.pub_juntas.publish(self.msg_juntas)

  def actualizar_estado_actual(self, msg: JointState):
    """
    Lee /joint_states y actualiza q_actual.

    Se buscan las juntas por nombre, no por posición en la lista. Esto evita
    errores si algún mensaje trae juntas en otro orden.
    """
    if self.en_movimiento:
      return

    if not msg.name or not msg.position:
      return

    q_nueva = list(self.q_actual)

    for i, nombre_junta in enumerate(self.joint_names):
      if nombre_junta in msg.name:
        indice = msg.name.index(nombre_junta)

        if indice < len(msg.position):
          q_nueva[i] = float(msg.position[indice])

    self.q_actual = q_nueva

  def recibir_objetivo_rviz(self, msg: PointStamped):
    """
    Callback para puntos seleccionados con Publish Point en RViz.
    """
    if self.en_movimiento:
      self.get_logger().warn("El robot ya está ejecutando una trayectoria.")
      return

    x = float(msg.point.x)
    y = float(msg.point.y)
    z = float(msg.point.z)

    self.get_logger().info(
      "Punto seleccionado en RViz: x={:.3f}, y={:.3f}, z={:.3f}".format(
        x, y, z
      )
    )

    self.iniciar_movimiento(x, y, z)

  def recibir_objetivo_terminal(self, msg: Twist):
    """
    Callback para objetivos enviados desde terminal.

    Solo se usa la parte linear del mensaje:
      linear.x -> coordenada X
      linear.y -> coordenada Y
      linear.z -> coordenada Z
    """
    if self.en_movimiento:
      self.get_logger().warn("El robot ya está ejecutando una trayectoria.")
      return

    x = float(msg.linear.x)
    y = float(msg.linear.y)
    z = float(msg.linear.z)

    self.get_logger().info(
      "Objetivo recibido por terminal: x={:.3f}, y={:.3f}, z={:.3f}".format(
        x, y, z
      )
    )

    self.iniciar_movimiento(x, y, z)

  def iniciar_movimiento(self, x: float, y: float, z: float):
    """
    Calcula y arranca una trayectoria hacia el punto deseado.

    Primero se llama a Robot.def_tray(), que calcula:
      - posiciones articulares
      - velocidades articulares
      - aceleraciones articulares
      - trayectoria cartesiana del efector final
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
        "No fue posible generar la trayectoria. "
        "Prueba con un punto dentro del alcance del robot."
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
      "Postura final: base_joint={:.3f}, joint1={:.3f}, joint2={:.3f}".format(
        float(self.robot.th_m[0, -1]),
        float(self.robot.th_m[1, -1]),
        float(self.robot.th_m[2, -1])
      )
    )

    self.en_movimiento = True
    self.indice_trayectoria = 0

    if self.timer_trayectoria is not None:
      self.timer_trayectoria.destroy()
      self.timer_trayectoria = None

    # Este timer publica una muestra de la trayectoria cada dt segundos.
    self.timer_trayectoria = self.create_timer(
      self.robot.dt,
      self.publicar_siguiente_muestra
    )

  def publicar_siguiente_muestra(self):
    """
    Publica una muestra de la trayectoria articular.

    Cada vez que se ejecuta esta función, el robot avanza un paso en RViz.
    """
    if self.indice_trayectoria >= self.robot.muestras:
      self.finalizar_movimiento()
      return

    self.q_actual = [
      float(self.robot.th_m[0, self.indice_trayectoria]),
      float(self.robot.th_m[1, self.indice_trayectoria]),
      float(self.robot.th_m[2, self.indice_trayectoria])
    ]

    self._publicar_juntas(self.q_actual)

    self.indice_trayectoria += 1

    if self.indice_trayectoria >= self.robot.muestras:
      self.finalizar_movimiento()

  def finalizar_movimiento(self):
    """
    Detiene el timer de trayectoria y muestra las gráficas del movimiento.
    """
    self.en_movimiento = False

    if self.timer_trayectoria is not None:
      self.timer_trayectoria.destroy()
      self.timer_trayectoria = None

    self.get_logger().info("Trayectoria terminada. Generando gráficas...")

    try:
      self.robot.mostrar_graficas()
    except Exception as error:
      self.get_logger().warn(
        "No se pudieron mostrar las gráficas: {}".format(error)
      )

    self.get_logger().info("Listo. Puedes enviar otro punto.")


def main():
  rclpy.init()
  nodo = PublicadorTrayectoria()

  try:
    rclpy.spin(nodo)
  except KeyboardInterrupt:
    pass
  finally:
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
  main()