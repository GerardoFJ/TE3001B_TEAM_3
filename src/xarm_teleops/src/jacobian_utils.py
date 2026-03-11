"""
jacobian_utils.py
-----------------
Jacobiano geométrico para xArm Lite 6.

Dos estrategias disponibles:
  1. Numérica (recomendada): usa arm.get_forward_kinematics() del SDK → exacta
     para el robot real pero requiere 6 llamadas extra por ciclo.
  2. Analítica DH: sin llamadas de red, más rápida, útil si el SDK no
     está disponible (simulación, pruebas unitarias).

Uso típico:
    J = jacobian_numeric(arm, q_rad)          # 6×6
    J = jacobian_dh(q_rad)                    # 6×6
    kappa = condition_number(J)               # número de condición
"""

import numpy as np


# ── Parámetros DH para xArm Lite 6 (DH estándar) ──────────────────────────
# Fuente: UFACTORY xArm Lite 6 User Manual
# Columnas: [a (m), d (m), alpha (rad), offset_theta (rad)]
# Los ángulos articulares se suman al offset_theta en cada fila.
_DH = np.array([
    #   a       d        alpha      offset
    [0.0000, 0.2433,  np.pi/2,   0.0      ],   # joint 1
    [0.2000, 0.0000,  0.0,      -np.pi/2  ],   # joint 2
    [0.0870, 0.0000,  np.pi/2,   0.0      ],   # joint 3
    [0.0000, 0.2276, -np.pi/2,   0.0      ],   # joint 4
    [0.0000, 0.0000,  np.pi/2,   0.0      ],   # joint 5
    [0.0000, 0.0615,  0.0,       0.0      ],   # joint 6
], dtype=float)

# Eje de rotación local de cada articulación (todos giran en z en DH estándar)
_Z = np.array([0.0, 0.0, 1.0])


def _dh_transform(a: float, d: float, alpha: float, theta: float) -> np.ndarray:
    """Matriz de transformación homogénea DH estándar 4×4."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,  -st*ca,  st*sa,  a*ct],
        [st,   ct*ca, -ct*sa,  a*st],
        [0.0,  sa,     ca,     d   ],
        [0.0,  0.0,    0.0,    1.0 ],
    ])


def _fk_dh(q_rad: np.ndarray):
    """
    Cinemática directa via DH.
    Retorna (T_0n, T_list) donde T_list[i] = T_{0,i+1}.
    """
    T = np.eye(4)
    frames = []
    for i, (a, d, alpha, offset) in enumerate(_DH):
        T = T @ _dh_transform(a, d, alpha, q_rad[i] + offset)
        frames.append(T.copy())
    return frames[-1], frames


def jacobian_dh(q_rad: np.ndarray) -> np.ndarray:
    """
    Jacobiano geométrico 6×6 calculado analíticamente con los parámetros DH.

    Columna i del Jacobiano:
        J_v[:, i] = z_{i-1} × (p_n - p_{i-1})
        J_w[:, i] = z_{i-1}

    Args:
        q_rad: vector de ángulos articulares [rad], shape (6,)

    Returns:
        J: ndarray shape (6, 6)
    """
    q_rad = np.asarray(q_rad, dtype=float).flatten()
    assert len(q_rad) == 6, "Se esperan 6 ángulos articulares"

    _, frames = _fk_dh(q_rad)
    p_n = frames[-1][:3, 3]   # posición del efector final

    J = np.zeros((6, 6))
    T_prev = np.eye(4)
    for i in range(6):
        z_i = T_prev[:3, 2]           # eje z del frame anterior
        p_i = T_prev[:3, 3]           # origen del frame anterior
        J[:3, i] = np.cross(z_i, p_n - p_i)   # parte traslacional
        J[3:, i] = z_i                          # parte rotacional
        T_prev = frames[i]

    return J


def jacobian_numeric(arm, q_rad: np.ndarray, delta: float = 1e-4) -> np.ndarray:
    """
    Jacobiano numérico 6×6 usando arm.get_forward_kinematics() del SDK.

    Requiere 6 llamadas adicionales al SDK por ciclo (~0.5–2 ms extra).
    Garantiza exactitud frente al modelo real del robot.

    Args:
        arm: instancia de XArmAPI
        q_rad: ángulos articulares [rad], shape (6,)
        delta: perturbación en radianes para diferencias finitas

    Returns:
        J: ndarray shape (6, 6)  [posición (mm→m), orientación (rad)]
    """
    q = np.asarray(q_rad, dtype=float).flatten()
    _, pose0 = arm.get_forward_kinematics(q.tolist(), input_is_radian=True)
    pose0 = np.array(pose0, dtype=float)
    pose0[:3] /= 1000.0   # mm → m

    J = np.zeros((6, 6))
    for i in range(6):
        q_p = q.copy()
        q_p[i] += delta
        _, pose_p = arm.get_forward_kinematics(q_p.tolist(), input_is_radian=True)
        pose_p = np.array(pose_p, dtype=float)
        pose_p[:3] /= 1000.0
        J[:, i] = (pose_p - pose0) / delta

    return J


def condition_number(J: np.ndarray) -> float:
    """
    Número de condición de J (criterio de singularidad).
    kappa → ∞ cerca de singularidad.
    Regla práctica: kappa > 50 → cuidado, kappa > 200 → zona singular.
    """
    return np.linalg.cond(J)


def pseudoinverse_jacobian(J: np.ndarray, damping: float = 0.01) -> np.ndarray:
    """
    Pseudo-inversa amortiguada (Damped Least Squares) de Jᵀ.
    Evita amplificación de ruido cerca de singularidades.

        (Jᵀ)⁺ = J · (JᵀJ + λ²I)⁻¹

    Args:
        J: Jacobiano 6×6
        damping: factor de amortiguamiento λ (aumentar cerca de singularidades)

    Returns:
        Jt_pinv: pseudo-inversa de Jᵀ, shape (6, 6)
    """
    Jt = J.T
    n = Jt.shape[1]
    return J @ np.linalg.inv(Jt @ J + damping**2 * np.eye(n))
