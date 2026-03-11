"""
slave_control.py
----------------
Lazo de control del robot Esclavo (xArm Lite 6).

Ciclo de control a CONTROL_HZ:
  1. Recibir referencia articular del Maestro via UDP
  2. Enviar comando al xArm Esclavo (servo mode)
  3. Leer torques articulares medidos
  4. Estimar fuerza externa con ForceEstimator
  5. Enviar F_ext al Maestro via UDP

IP del Maestro: MASTER_IP (PC donde corre master_control.py)
IP del Esclavo: ARM_SLAVE_IP (xArm Lite 6 esclavo)

Ejecutar en la PC del esclavo:
    python3 slave_control.py
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
from force_estimator import ForceEstimator
from network_comm import MasterReceiver, SlaveSender

# ── Configuración ──────────────────────────────────────────────────────────
ARM_SLAVE_IP = '192.168.1.XXX'   # <── IP del xArm Esclavo
MASTER_IP    = '192.168.1.YYY'   # <── IP de la PC Maestro
RX_PORT      = 5005              # puerto en que recibe del maestro
TX_PORT      = 5006              # puerto al que envía al maestro
CONTROL_HZ   = 100               # Hz

# Escalado de posición esclavo = ALPHA * posición maestro
# 1.0 = mapeo 1:1, < 1.0 = movimientos más pequeños en esclavo
ALPHA = 1.0

# Ganancia de amortiguamiento para suavizar movimiento
KD_SLAVE = 0.5   # fracción de la velocidad articular que se amortigua

# Velocidad máxima para servo commands [rad/s]
MAX_SERVO_SPEED = np.radians(60.0)   # 60 °/s

# ── Estado global ─────────────────────────────────────────────────────────
_running = True

def _signal_handler(sig, frame):
    global _running
    print("\n[slave] Señal de interrupción recibida. Deteniendo...")
    _running = False

signal.signal(signal.SIGINT, _signal_handler)
signal.signal(signal.SIGTERM, _signal_handler)


def _clamp_angles(q_ref: np.ndarray, q_curr: np.ndarray, max_delta: float) -> np.ndarray:
    """Limita el cambio de ángulo por ciclo para evitar movimientos bruscos."""
    delta = np.clip(q_ref - q_curr, -max_delta, max_delta)
    return q_curr + delta


def main():
    global _running

    print("=" * 60)
    print("  xArm Teleops — Control Esclavo")
    print(f"  Robot: {ARM_SLAVE_IP}  |  {CONTROL_HZ} Hz")
    print("=" * 60)

    # ── Inicializar robot ────────────────────────────────────────────────
    arm = setup_robot(ARM_SLAVE_IP, mode=MODE_SERVO)
    move_to_home(arm)

    # ── Inicializar estimador de fuerza ───────────────────────────────────
    estimator = ForceEstimator(control_hz=CONTROL_HZ, cutoff_hz=5.0)
    estimator.reset_filter()

    # ── Calibración de ruido en reposo ────────────────────────────────────
    print("[slave] Calibrando nivel de ruido (2 s en reposo)...")
    q_home = get_joint_angles_rad(arm)
    n_cal = int(2.0 * CONTROL_HZ)
    tau_cal_samples = np.array([
        get_joint_torques(arm) for _ in range(n_cal)
    ])
    estimator.calibrate_noise(q_home, tau_cal_samples)
    threshold = estimator.suggest_threshold(safety_factor=3.0)
    estimator.force_threshold = threshold
    print(f"[slave] Umbral de contacto: {threshold:.2f} N")

    # ── Sockets UDP ───────────────────────────────────────────────────────
    rx = MasterReceiver(port=RX_PORT, timeout=0.01)
    tx = SlaveSender(remote_ip=MASTER_IP, remote_port=TX_PORT)

    # ── Variables de estado ───────────────────────────────────────────────
    q_curr = get_joint_angles_rad(arm)
    q_ref  = q_curr.copy()
    dt     = 1.0 / CONTROL_HZ
    max_delta_per_cycle = MAX_SERVO_SPEED * dt   # rad por ciclo

    missed_packets = 0
    cycle_count    = 0

    print("[slave] Iniciando lazo de control. Ctrl+C para detener.\n")

    # ── Lazo principal ────────────────────────────────────────────────────
    t_next = time.perf_counter()

    while _running:
        t_start = time.perf_counter()

        # 1. Recibir referencia del Maestro
        pkt = rx.recv()
        if pkt is None:
            missed_packets += 1
            if missed_packets % 100 == 0:
                print(f"[slave] Advertencia: {missed_packets} paquetes perdidos")
        else:
            t_master, q_master = pkt
            # Aplicar escalado + límite de cambio por ciclo
            q_ref_raw = ALPHA * q_master
            q_ref = _clamp_angles(q_ref_raw, q_curr, max_delta_per_cycle)

        # 2. Enviar comando al xArm en modo servo
        code = arm.set_servo_angle_j(
            angles=q_ref.tolist(),
            speed=np.degrees(MAX_SERVO_SPEED),
            is_radian=True,
        )
        if code not in (0, None):
            print(f"[slave] Código de error en servo command: {code}")
            if code in (1, 2, 3):   # errores críticos
                break

        # 3. Leer torques y ángulos actuales
        q_curr = get_joint_angles_rad(arm)
        tau_meas = get_joint_torques(arm)

        # 4. Estimar fuerza externa
        F_ext = estimator.update(q_curr, tau_meas)

        # Detección de contacto
        if estimator.is_contact(F_ext):
            pass   # log o acción extra aquí

        # 5. Enviar fuerza al Maestro
        tx.send(F_ext, tau_meas)

        cycle_count += 1

        # ── Control de timing (lazo fijo a CONTROL_HZ) ─────────────────
        t_elapsed = time.perf_counter() - t_start
        t_sleep = dt - t_elapsed
        if t_sleep > 0:
            time.sleep(t_sleep)

    # ── Shutdown ──────────────────────────────────────────────────────────
    rx.close()
    tx.close()
    safe_shutdown(arm)
    print(f"\n[slave] Lazo terminado. Ciclos: {cycle_count}, "
          f"Paquetes perdidos: {missed_packets}")


if __name__ == '__main__':
    main()
