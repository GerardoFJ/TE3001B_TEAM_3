"""
master_control.py
-----------------
Lazo de control del robot Maestro (xArm Lite 6).

Ciclo de control a CONTROL_HZ:
  1. Leer posición articular del Maestro (operador lo mueve libremente)
  2. Enviar q_master al Esclavo via UDP
  3. Recibir F_ext estimado del Esclavo
  4. Aplicar feedback háptico en el Maestro (impedancia en posición)

Feedback háptico:
  Como el xArm Python SDK no permite inyectar torques articulares
  directamente en modo usuario (modo torque requiere acceso de bajo nivel),
  se implementa un feedback por IMPEDANCIA EN POSICIÓN:

      q_haptic = q_master - Kh · J^T · β · F_ext · dt²

  Cuando hay fuerza de contacto, el maestro recibe una corrección
  de posición que lo empuja en sentido contrario a la fuerza:
  el operador "siente" resistencia al continuar el movimiento.

  Ver: Pregunta C2 de la rúbrica — justificación de la alternativa.

Ejecutar en la PC del maestro:
    python3 master_control.py
"""

import sys
import time
import signal
import numpy as np

from xarm_setup import (
    setup_robot, move_to_home, safe_shutdown,
    get_joint_angles_rad, get_joint_torques,
    MODE_SERVO,
)
from jacobian_utils import jacobian_dh, condition_number
from network_comm import MasterSender, SlaveReceiver

# ── Configuración ──────────────────────────────────────────────────────────
ARM_MASTER_IP = '192.168.1.ZZZ'  # <── IP del xArm Maestro
SLAVE_IP      = '192.168.1.WWW'  # <── IP de la PC Esclava
TX_PORT       = 5005             # envía a esclavo
RX_PORT       = 5006             # recibe del esclavo
CONTROL_HZ    = 100              # Hz

# ── Parámetros de feedback háptico ────────────────────────────────────────
# β: ganancia de reflexión de fuerza [adimensional]
# τ_haptic = J^T · β · F_est  →  traducido a corrección de posición
BETA = 0.3          # ajustar experimentalmente (ver Pregunta C2)
KH   = 0.05         # ganancia de posición háptica [rad/N]

# Límite de saturación de fuerza reflejada [N]
FORCE_CLAMP_N = 20.0

# Umbral de singularidad: si kappa(J) > este valor, no aplicar háptica
SINGULARITY_THRESHOLD = 50.0

# ── Estado global ─────────────────────────────────────────────────────────
_running = True

def _signal_handler(sig, frame):
    global _running
    print("\n[master] Señal de interrupción recibida. Deteniendo...")
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def _apply_haptic_correction(
    arm,
    q_curr: np.ndarray,
    F_ext: np.ndarray,
    beta: float,
    kh: float,
) -> None:
    """
    Aplica corrección de posición proporcional a la fuerza reflejada.

    Estrategia (alternativa a control de torque directo):
        δq = -kh · J^T · (β · F_ext)
        q_cmd = q_curr + δq

    Args:
        arm: XArmAPI del maestro
        q_curr: ángulos actuales [rad]
        F_ext: fuerza estimada en esclavo [N, Nm], shape (6,)
        beta: ganancia de reflexión
        kh: ganancia de corrección de posición [rad/N]
    """
    # Verificar singularidad
    J = jacobian_dh(q_curr)
    kappa = condition_number(J)
    if kappa > SINGULARITY_THRESHOLD:
        return   # cerca de singularidad → no aplicar háptica

    # Limitar fuerza máxima reflejada
    F_clamped = F_ext.copy()
    f_norm = np.linalg.norm(F_clamped[:3])
    if f_norm > FORCE_CLAMP_N:
        F_clamped[:3] *= FORCE_CLAMP_N / f_norm

    # Corrección en espacio articular: δq = -kh · J^T · β · F
    delta_q = -kh * J.T @ (beta * F_clamped)

    q_cmd = q_curr + delta_q

    arm.set_servo_angle_j(
        angles=q_cmd.tolist(),
        speed=30.0,    # velocidad baja para corrección suave
        is_radian=True,
    )


def main():
    global _running

    print("=" * 60)
    print("  xArm Teleops — Control Maestro")
    print(f"  Robot: {ARM_MASTER_IP}  |  {CONTROL_HZ} Hz")
    print("=" * 60)

    # ── Inicializar robot ─────────────────────────────────────────────────
    arm = setup_robot(ARM_MASTER_IP, mode=MODE_SERVO)
    move_to_home(arm)

    # ── Sockets UDP ───────────────────────────────────────────────────────
    tx = MasterSender(remote_ip=SLAVE_IP, remote_port=TX_PORT)
    rx = SlaveReceiver(port=RX_PORT, timeout=0.01)

    # ── Variables de estado ───────────────────────────────────────────────
    dt          = 1.0 / CONTROL_HZ
    F_ext       = np.zeros(6)
    cycle_count = 0
    t_last_fb   = 0.0     # timestamp del último feedback recibido
    haptic_active = False

    # Métricas en tiempo real
    latencies = []   # RTT estimado por timestamp del paquete esclavo

    print("[master] Iniciando lazo de control. Ctrl+C para detener.\n")
    print("         Mueve el Maestro — el Esclavo replicará el movimiento.")
    print("         Cuando el Esclavo haga contacto, sentirás resistencia.\n")

    t_next = time.perf_counter()

    while _running:
        t_start = time.perf_counter()

        # 1. Leer posición articular del Maestro
        q_master = get_joint_angles_rad(arm)

        # 2. Enviar referencia al Esclavo
        tx.send(q_master)

        # 3. Recibir fuerza del Esclavo (no bloqueante, timeout=10 ms)
        pkt = rx.recv()
        if pkt is not None:
            t_slave, F_ext, tau_slave = pkt
            # Estimar latencia (mitad del RTT)
            latency_ms = (time.time() - t_slave) * 1000.0
            latencies.append(latency_ms)
            t_last_fb = time.perf_counter()

        # 4. Feedback háptico si hay fuerza significativa
        f_magnitude = float(np.linalg.norm(F_ext[:3]))
        if f_magnitude > 1.0:   # umbral mínimo de 1 N
            _apply_haptic_correction(arm, q_master, F_ext, BETA, KH)
            haptic_active = True
        else:
            haptic_active = False

        cycle_count += 1

        # Log periódico cada 5 segundos
        if cycle_count % (5 * CONTROL_HZ) == 0:
            lat_arr = np.array(latencies[-500:]) if latencies else np.array([0.0])
            print(
                f"[master] Ciclo {cycle_count:6d} | "
                f"F_ext={f_magnitude:.2f} N | "
                f"Háptica={'ON ' if haptic_active else 'OFF'} | "
                f"Latencia={lat_arr.mean():.1f}±{lat_arr.std():.1f} ms"
            )

        # ── Control de timing ─────────────────────────────────────────────
        t_elapsed = time.perf_counter() - t_start
        t_sleep = dt - t_elapsed
        if t_sleep > 0:
            time.sleep(t_sleep)

    # ── Resumen final ─────────────────────────────────────────────────────
    tx.close()
    rx.close()
    safe_shutdown(arm)

    if latencies:
        lat = np.array(latencies)
        print(f"\n[master] Latencia promedio: {lat.mean():.2f} ms ± {lat.std():.2f} ms")
        print(f"[master] P99: {np.percentile(lat, 99):.2f} ms")
    print(f"[master] Ciclos totales: {cycle_count}")


if __name__ == '__main__':
    main()
