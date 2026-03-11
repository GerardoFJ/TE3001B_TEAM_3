"""
xarm_setup.py
-------------
Inicialización, configuración de seguridad y shutdown del xArm Lite 6.

Uso:
    arm = setup_robot('192.168.1.10', mode=1)
    move_to_home(arm)
    ...
    safe_shutdown(arm)
"""

import time
import numpy as np
from xarm.wrapper import XArmAPI

# ── Modos de operación (set_mode) ─────────────────────────────────────────
MODE_POSITION  = 0   # Control de posición (trayectorias suaves)
MODE_SERVO     = 1   # Servo mode (tiempo real, set_servo_angle_j)
MODE_TEACH     = 2   # Modo enseñanza (gravity compensation)
MODE_VEL_JOINT = 4   # Velocidad articular
MODE_VEL_CART  = 5   # Velocidad cartesiana
MODE_POSE_CART = 6   # Posición cartesiana tiempo real

# ── Posición home [grados] ────────────────────────────────────────────────
# Configuración segura neutral del xArm Lite 6
HOME_ANGLES_DEG = [0.0, 0.0, 0.0, 90.0, 0.0, 0.0]

# ── Límites de seguridad ──────────────────────────────────────────────────
DEFAULT_MAX_SPEED_DEG_S = 60.0   # °/s — conservador para teleoperación
DEFAULT_MAX_ACC_DEG_S2  = 200.0  # °/s²
DEFAULT_MAX_TCP_SPEED   = 100.0  # mm/s
DEFAULT_MAX_TCP_ACC     = 500.0  # mm/s²

# Códigos de error del xArm que requieren reset
_ERROR_CODES_NEED_RESET = {1, 2, 3, 11, 12, 13, 14, 15, 16}


def setup_robot(
    ip: str,
    mode: int = MODE_SERVO,
    max_joint_speed: float = DEFAULT_MAX_SPEED_DEG_S,
    max_joint_acc: float = DEFAULT_MAX_ACC_DEG_S2,
    max_tcp_speed: float = DEFAULT_MAX_TCP_SPEED,
    max_tcp_acc: float = DEFAULT_MAX_TCP_ACC,
    gripper: bool = False,
) -> XArmAPI:
    """
    Conecta y configura el xArm Lite 6 para teleoperación.

    Args:
        ip: dirección IP del robot (ej. '192.168.1.10')
        mode: modo de operación (default: MODE_SERVO = 1)
        max_joint_speed: velocidad articular máxima [°/s]
        max_joint_acc: aceleración articular máxima [°/s²]
        max_tcp_speed: velocidad TCP máxima [mm/s]
        max_tcp_acc: aceleración TCP máxima [mm/s²]
        gripper: inicializar gripper si True

    Returns:
        arm: instancia configurada de XArmAPI lista para operar
    """
    print(f"[xarm_setup] Conectando a {ip}...")
    arm = XArmAPI(ip, is_radian=True)

    # Esperar conexión estable
    time.sleep(0.5)
    if not arm.connected:
        raise ConnectionError(f"No se pudo conectar al xArm en {ip}")

    print(f"[xarm_setup] Conectado. Modelo: {arm.version}")

    # Limpiar errores previos
    _clear_errors(arm)

    # Habilitar motores
    arm.motion_enable(enable=True)
    time.sleep(0.2)

    # Configurar límites de seguridad
    arm.set_joint_maxacc([np.radians(max_joint_acc)] * 6)
    arm.set_tcp_maxacc(max_tcp_acc)
    arm.set_tcp_maxspeed(max_tcp_speed)

    # Establecer modo y estado
    arm.set_mode(mode)
    arm.set_state(0)   # estado sport (activo)
    time.sleep(0.3)

    if gripper:
        arm.set_gripper_enable(True)
        arm.set_gripper_mode(0)

    print(f"[xarm_setup] Robot listo. Modo={mode}, vel_max={max_joint_speed}°/s")
    return arm


def move_to_home(
    arm: XArmAPI,
    speed_deg_s: float = 30.0,
    wait: bool = True,
) -> None:
    """
    Mueve el robot a la posición home segura.
    Cambia temporalmente a modo posición para el movimiento.
    """
    print("[xarm_setup] Moviendo a posición home...")
    current_mode = arm.mode

    arm.set_mode(MODE_POSITION)
    arm.set_state(0)
    time.sleep(0.1)

    code = arm.set_servo_angle(
        angle=HOME_ANGLES_DEG,
        speed=speed_deg_s,
        is_radian=False,
        wait=wait,
    )
    if code != 0:
        print(f"[xarm_setup] Advertencia: set_servo_angle retornó código {code}")

    # Restaurar modo original
    arm.set_mode(current_mode)
    arm.set_state(0)
    time.sleep(0.2)
    print("[xarm_setup] Home alcanzado.")


def safe_shutdown(arm: XArmAPI) -> None:
    """Detiene el robot de forma segura y cierra la conexión."""
    print("[xarm_setup] Apagando robot de forma segura...")
    try:
        arm.set_state(4)    # stop
        time.sleep(0.3)
        arm.motion_enable(enable=False)
        arm.disconnect()
    except Exception as e:
        print(f"[xarm_setup] Error durante shutdown: {e}")
    print("[xarm_setup] Desconectado.")


def check_robot_state(arm: XArmAPI) -> bool:
    """
    Verifica que el robot esté en estado operacional.
    Retorna True si está OK, False si hay error.
    """
    error_code = arm.error_code
    warn_code  = arm.warn_code
    state      = arm.state

    if error_code != 0:
        print(f"[xarm_setup] ERROR código {error_code} — estado {state}")
        return False
    if warn_code != 0:
        print(f"[xarm_setup] Advertencia código {warn_code}")
    return True


def _clear_errors(arm: XArmAPI) -> None:
    """Limpia errores y warnings del robot."""
    if arm.error_code != 0:
        print(f"[xarm_setup] Limpiando error código {arm.error_code}...")
        arm.clean_error()
        arm.clean_warn()
        time.sleep(0.2)


def get_joint_angles_rad(arm: XArmAPI) -> np.ndarray:
    """Lee ángulos articulares actuales en radianes."""
    _, angles = arm.get_servo_angle(is_radian=True)
    return np.array(angles[:6], dtype=float)


def get_joint_torques(arm: XArmAPI) -> np.ndarray:
    """
    Lee torques articulares estimados [Nm].
    Nota: el xArm Lite 6 no tiene sensores de torque directos;
    los torques se estiman internamente a partir de la corriente del motor.
    """
    _, states = arm.get_joint_states(is_radian=True)
    # states = [positions, velocities, torques]
    if states is None or len(states) < 3:
        return np.zeros(6)
    return np.array(states[2][:6], dtype=float)
