#!/usr/bin/env python3

import math
from typing import Optional, Sequence, Tuple

import numpy as np


class Robot:
  """
  Modelo cinemático del robot RRR usado en RViz.

  El robot tiene tres juntas revolutas:
    q0 -> base_joint : gira la base alrededor del eje Z.
    q1 -> joint1     : mueve el primer eslabón en el plano vertical.
    q2 -> joint2     : mueve el segundo eslabón en el plano vertical.

  La base orienta el brazo hacia el punto deseado en XY.
  Después, joint1 y joint2 acomodan la altura Z y el alcance radial.
  """

  def __init__(
      self,
      longitudes: Tuple[float, float] = (0.60, 0.60),
      altura_base: float = 0.75
  ):
    # Longitudes físicas de los dos eslabones principales.
    self.L1 = float(longitudes[0])
    self.L2 = float(longitudes[1])

    # Altura aproximada desde el origen hasta el eje del hombro.
    self.H0 = float(altura_base)

    # Límites articulares. En el URDF las tres juntas permiten una vuelta completa.
    self.limites = {
      "base_joint": (-math.pi, math.pi),
      "joint1": (-math.pi, math.pi),
      "joint2": (-math.pi, math.pi),
    }

    # Variables que se llenan cuando se genera una trayectoria.
    self.dt = 0.0
    self.muestras = 0

    self.t_m = None

    self.xi_m = None
    self.xi_dot_m = None
    self.xi_dot_dot_m = None

    self.th_m = None
    self.th_dot_m = None
    self.th_dot_dot_m = None

  def _limites_como_lista(self):
    return [
      self.limites["base_joint"],
      self.limites["joint1"],
      self.limites["joint2"],
    ]

  @staticmethod
  def _normalizar_respecto_a(actual: float, referencia: float) -> float:
    """
    Regresa un ángulo equivalente, pero cercano a la posición actual.

    Esto evita que la base dé una vuelta larga cuando puede llegar al mismo
    punto con una rotación más corta.
    """
    while actual - referencia > math.pi:
      actual -= 2.0 * math.pi

    while actual - referencia < -math.pi:
      actual += 2.0 * math.pi

    return actual

  def en_limites(self, q: Sequence[float], tolerancia: float = 1e-8) -> bool:
    """
    Revisa que las tres juntas estén dentro de los límites permitidos.
    """
    for valor, (minimo, maximo) in zip(q, self._limites_como_lista()):
      if valor < minimo - tolerancia or valor > maximo + tolerancia:
        return False

    return True

  def fk(self, q: Sequence[float]) -> np.ndarray:
    """
    Cinemática directa.

    Recibe las juntas:
      q = [base_joint, joint1, joint2]

    Devuelve la posición cartesiana del efector final:
      [x, y, z]

    La idea es:
      1. joint1 y joint2 calculan el alcance radial r y la altura z.
      2. base_joint gira ese alcance radial sobre el plano XY.
    """
    q0, q1, q2 = [float(valor) for valor in q]

    # Proyección horizontal del brazo.
    radio = (
      self.L1 * math.cos(q1)
      + self.L2 * math.cos(q1 + q2)
    )

    # Altura del efector final.
    altura = (
      self.H0
      - self.L1 * math.sin(q1)
      - self.L2 * math.sin(q1 + q2)
    )

    # La base reparte el radio entre X y Y.
    x = radio * math.cos(q0)
    y = radio * math.sin(q0)

    return np.array([x, y, altura], dtype=float)

  def ik(
      self,
      x: float,
      y: float,
      z: float,
      seed: Optional[Sequence[float]] = None
  ) -> np.ndarray:
    """
    Cinemática inversa analítica.

    Recibe un punto objetivo [x, y, z] y calcula las juntas necesarias.

    seed es la posición actual del robot. Sirve para elegir la solución que
    implique el movimiento más pequeño.
    """
    if seed is None:
      seed = (0.0, 0.0, 0.0)

    x = float(x)
    y = float(y)
    z = float(z)

    # La base apunta hacia el objetivo en el plano XY.
    q_base = math.atan2(y, x)
    q_base = self._normalizar_respecto_a(q_base, float(seed[0]))

    # Distancia horizontal desde la base hasta el punto.
    radio_objetivo = math.hypot(x, y)

    # Diferencia vertical medida desde el eje del hombro.
    altura_objetivo = self.H0 - z

    # Ley de cosenos para el triángulo formado por L1, L2 y el punto objetivo.
    cos_codo = (
      radio_objetivo**2
      + altura_objetivo**2
      - self.L1**2
      - self.L2**2
    ) / (2.0 * self.L1 * self.L2)

    # Protección numérica: evita errores por redondeos muy pequeños.
    cos_codo = max(-1.0, min(1.0, cos_codo))

    soluciones_validas = []

    # Existen dos configuraciones típicas: codo arriba y codo abajo.
    for signo in (1.0, -1.0):
      q_codo = math.atan2(
        signo * math.sqrt(max(0.0, 1.0 - cos_codo**2)),
        cos_codo
      )

      q_hombro = math.atan2(altura_objetivo, radio_objetivo) - math.atan2(
        self.L2 * math.sin(q_codo),
        self.L1 + self.L2 * math.cos(q_codo)
      )

      q = np.array([q_base, q_hombro, q_codo], dtype=float)

      if self.en_limites(q):
        distancia_articular = np.linalg.norm(q - np.array(seed, dtype=float))
        soluciones_validas.append((distancia_articular, q))

    if not soluciones_validas:
      raise ValueError(
        "No se encontró una configuración válida para el punto solicitado."
      )

    # Se elige la solución que cambia menos respecto a la postura actual.
    soluciones_validas.sort(key=lambda dato: dato[0])
    return soluciones_validas[0][1]

  @staticmethod
  def _perfil_quintico(s: float, tiempo_total: float):
    """
    Perfil suave de quinto grado.

    lam         -> posición normalizada
    lam_dot     -> velocidad normalizada
    lam_dot_dot -> aceleración normalizada

    Este perfil empieza y termina con velocidad y aceleración cero.
    """
    lam = 10.0 * s**3 - 15.0 * s**4 + 6.0 * s**5
    lam_dot = (30.0 * s**2 - 60.0 * s**3 + 30.0 * s**4) / tiempo_total
    lam_dot_dot = (
      60.0 * s
      - 180.0 * s**2
      + 120.0 * s**3
    ) / (tiempo_total**2)

    return lam, lam_dot, lam_dot_dot

  def def_tray(
      self,
      t_f: float = 2.0,
      frec: float = 15.0,
      th_i: Sequence[float] = (0.0, 0.0, 0.0),
      xi_f: Sequence[float] = (0.60, 0.0, 0.67)
  ):
    """
    Genera la trayectoria desde una postura inicial hasta un punto cartesiano.

    th_i:
      posición inicial de las juntas [base_joint, joint1, joint2]

    xi_f:
      punto final deseado del efector [x, y, z]
    """
    th_i = np.array(th_i, dtype=float)
    xi_f = np.array(xi_f, dtype=float)

    if th_i.size != 3:
      raise ValueError("th_i debe contener tres valores articulares.")

    if xi_f.size != 3:
      raise ValueError("xi_f debe contener tres coordenadas: x, y, z.")

    # Primero se calcula a qué postura articular debe llegar el robot.
    th_f = self.ik(
      x=xi_f[0],
      y=xi_f[1],
      z=xi_f[2],
      seed=th_i
    )

    self.dt = 1.0 / float(frec)
    self.muestras = int(round(float(t_f) * float(frec))) + 1

    self.t_m = np.zeros((1, self.muestras), dtype=float)

    self.xi_m = np.zeros((3, self.muestras), dtype=float)
    self.xi_dot_m = np.zeros((3, self.muestras), dtype=float)
    self.xi_dot_dot_m = np.zeros((3, self.muestras), dtype=float)

    self.th_m = np.zeros((3, self.muestras), dtype=float)
    self.th_dot_m = np.zeros((3, self.muestras), dtype=float)
    self.th_dot_dot_m = np.zeros((3, self.muestras), dtype=float)

    desplazamiento = th_f - th_i

    for k in range(self.muestras):
      tiempo = self.dt * k
      self.t_m[0, k] = tiempo

      s = tiempo / float(t_f)
      s = max(0.0, min(1.0, s))

      lam, lam_dot, lam_dot_dot = self._perfil_quintico(s, float(t_f))

      # Interpolación articular suave.
      q = th_i + desplazamiento * lam

      self.th_m[:, k] = q
      self.th_dot_m[:, k] = desplazamiento * lam_dot
      self.th_dot_dot_m[:, k] = desplazamiento * lam_dot_dot

      # También se guarda la trayectoria cartesiana resultante.
      self.xi_m[:, k] = self.fk(q)

    # Velocidad y aceleración cartesianas calculadas numéricamente.
    if self.muestras > 1:
      self.xi_dot_m[:, 1:] = np.diff(self.xi_m, axis=1) / self.dt
      self.xi_dot_m[:, 0] = self.xi_dot_m[:, 1]

      self.xi_dot_dot_m[:, 1:] = np.diff(self.xi_dot_m, axis=1) / self.dt
      self.xi_dot_dot_m[:, 0] = self.xi_dot_dot_m[:, 1]

    print("Objetivo cartesiano [x, y, z]:", xi_f)
    print("Punto final obtenido [x, y, z]:", self.xi_m[:, -1])
    print("Juntas finales [base_joint, joint1, joint2]:", self.th_m[:, -1])

  def imp_tray(self, show: bool = True):
    """
    Grafica posición, velocidad y aceleración del efector final.
    """
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(nrows=3, ncols=3, figsize=(12, 8))
    fig.suptitle("Movimiento cartesiano del efector final")

    ejes = ["X", "Y", "Z"]
    magnitudes = [
      ("Posición", self.xi_m),
      ("Velocidad", self.xi_dot_m),
      ("Aceleración", self.xi_dot_dot_m),
    ]

    for fila, (nombre_mag, datos) in enumerate(magnitudes):
      for columna, eje in enumerate(ejes):
        axs[fila, columna].set_title(f"{nombre_mag} en {eje}")
        axs[fila, columna].plot(self.t_m.T, datos[columna, :].T)
        axs[fila, columna].grid(True)

    axs[2, 0].set_xlabel("Tiempo [s]")
    axs[2, 1].set_xlabel("Tiempo [s]")
    axs[2, 2].set_xlabel("Tiempo [s]")

    plt.tight_layout()

    if show:
      plt.show()

    return fig

  def imp_junt(self, show: bool = True):
    """
    Grafica posición, velocidad y aceleración de las juntas.
    """
    import matplotlib.pyplot as plt

    fig, axs = plt.subplots(nrows=3, ncols=3, figsize=(12, 8))
    fig.suptitle("Movimiento en espacio articular")

    juntas = ["base_joint", "joint1", "joint2"]
    magnitudes = [
      ("Posición", self.th_m),
      ("Velocidad", self.th_dot_m),
      ("Aceleración", self.th_dot_dot_m),
    ]

    for fila, (nombre_mag, datos) in enumerate(magnitudes):
      for columna, junta in enumerate(juntas):
        axs[fila, columna].set_title(f"{nombre_mag} de {junta}")
        axs[fila, columna].plot(self.t_m.T, datos[columna, :].T)
        axs[fila, columna].grid(True)

    axs[2, 0].set_xlabel("Tiempo [s]")
    axs[2, 1].set_xlabel("Tiempo [s]")
    axs[2, 2].set_xlabel("Tiempo [s]")

    plt.tight_layout()

    if show:
      plt.show()

    return fig

  def mostrar_graficas(self):
    """
    Abre juntas las gráficas cartesianas y articulares.
    """
    import matplotlib.pyplot as plt

    self.imp_tray(show=False)
    self.imp_junt(show=False)

    plt.show()


def main():
  robot = Robot()

  postura_inicial = (0.0, 0.0, 0.0)
  punto_prueba = (0.70, 0.20, 0.40)

  print("Cinemática directa en postura inicial:")
  print(robot.fk(postura_inicial))

  robot.def_tray(
    th_i=postura_inicial,
    xi_f=punto_prueba
  )

  robot.mostrar_graficas()


if __name__ == "__main__":
  main()