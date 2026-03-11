"""
force_estimator.py
------------------
Estimación de fuerza externa en el efector final a partir de
torques articulares medidos (sin sensor F/T externo).

Ecuación central:
    F̂_ext = (Jᵀ)⁺ · Δτ
    Δτ = τ_medido - τ_modelo
    τ_modelo = g(q)   [solo compensación gravitacional, simplificado]

Referencia: Siciliano et al., "Robotics: Modelling, Planning and Control", 2009.
"""

import numpy as np
from jacobian_utils import jacobian_dh, pseudoinverse_jacobian, condition_number


# ── Parámetros de masa del xArm Lite 6 (aproximados, kg) ──────────────────
# Fuente: UFACTORY datasheet. Ajustar si se conocen los valores exactos.
_LINK_MASS = np.array([1.2, 1.1, 0.9, 0.8, 0.3, 0.2])   # kg por link

# Centro de masa de cada link en su frame local [x, y, z] en metros (aprox.)
_LINK_COM = np.array([
    [0.0,    0.0,    0.12  ],
    [0.10,   0.0,    0.0   ],
    [0.04,   0.0,    0.0   ],
    [0.0,    0.0,    0.11  ],
    [0.0,    0.0,    0.0   ],
    [0.0,    0.0,    0.03  ],
])

_G = np.array([0.0, 0.0, -9.81])   # gravedad en frame base [m/s²]

# ── Parámetros del filtro pasa-bajas ──────────────────────────────────────
# α = exp(-2π·fc·Ts)  →  fc = frecuencia de corte [Hz], Ts = período [s]
_DEFAULT_CUTOFF_HZ = 5.0   # Hz — suficiente para detectar contactos ~1-2 Hz


class ForceEstimator:
    """
    Estimador de fuerza externa para xArm Lite 6.

    Uso:
        est = ForceEstimator(control_hz=100)
        F_est = est.update(q_rad, tau_measured)
        in_contact = est.is_contact(F_est)
    """

    def __init__(
        self,
        control_hz: float = 100.0,
        cutoff_hz: float = _DEFAULT_CUTOFF_HZ,
        force_threshold_n: float = 2.0,
        damping_dls: float = 0.01,
    ):
        """
        Args:
            control_hz: frecuencia del lazo de control [Hz]
            cutoff_hz: frecuencia de corte del filtro pasa-bajas [Hz]
            force_threshold_n: umbral de fuerza para detección de contacto [N]
            damping_dls: factor λ para pseudo-inversa amortiguada
        """
        self.dt = 1.0 / control_hz
        self.force_threshold = force_threshold_n
        self.damping = damping_dls

        # Coeficiente del filtro pasa-bajas de primer orden
        self._alpha = np.exp(-2.0 * np.pi * cutoff_hz * self.dt)
        self._f_filtered = np.zeros(6)   # estado del filtro (6 componentes)

        # Estadísticas de ruido en reposo (se estiman en calibración)
        self._noise_std = np.zeros(6)

    # ── Interfaz principal ────────────────────────────────────────────────

    def update(self, q_rad: np.ndarray, tau_measured: np.ndarray) -> np.ndarray:
        """
        Estima la fuerza/torque externo en el efector final.

        Args:
            q_rad: ángulos articulares [rad], shape (6,)
            tau_measured: torques articulares medidos [Nm], shape (6,)

        Returns:
            F_est: [Fx, Fy, Fz, Mx, My, Mz] en el frame base [N, Nm], shape (6,)
        """
        q = np.asarray(q_rad, dtype=float).flatten()
        tau = np.asarray(tau_measured, dtype=float).flatten()

        # 1. Compensación gravitacional
        tau_gravity = self._gravity_torque(q)
        delta_tau = tau - tau_gravity

        # 2. Jacobiano y pseudo-inversa amortiguada
        J = jacobian_dh(q)
        Jt_pinv = pseudoinverse_jacobian(J, damping=self.damping)

        # 3. Estimación de fuerza
        F_raw = Jt_pinv @ delta_tau

        # 4. Filtro pasa-bajas de primer orden
        self._f_filtered = (
            self._alpha * self._f_filtered + (1.0 - self._alpha) * F_raw
        )

        return self._f_filtered.copy()

    def is_contact(self, F_est: np.ndarray) -> bool:
        """Retorna True si la magnitud de fuerza supera el umbral."""
        return float(np.linalg.norm(F_est[:3])) > self.force_threshold

    def reset_filter(self) -> None:
        """Reinicia el estado del filtro (útil al inicio de cada experimento)."""
        self._f_filtered = np.zeros(6)

    def calibrate_noise(
        self, q_rad: np.ndarray, tau_samples: np.ndarray
    ) -> None:
        """
        Calibra el nivel de ruido en reposo.
        Llama con el robot inmóvil durante ~2 segundos.

        Args:
            q_rad: configuración de reposo [rad]
            tau_samples: matriz de muestras de torque (N_muestras × 6)
        """
        estimates = np.array([
            self.update(q_rad, tau_samples[i]) for i in range(len(tau_samples))
        ])
        self._noise_std = estimates.std(axis=0)
        snr_db = 20.0 * np.log10(self.force_threshold / (self._noise_std[:3].mean() + 1e-9))
        print(f"[ForceEstimator] Ruido en reposo (std): {self._noise_std[:3]}")
        print(f"[ForceEstimator] SNR estimado: {snr_db:.1f} dB")

    def suggest_threshold(self, safety_factor: float = 3.0) -> float:
        """Sugiere un umbral de contacto = safety_factor × std del ruido."""
        if self._noise_std.any():
            return float(safety_factor * self._noise_std[:3].mean())
        return self.force_threshold

    # ── Internos ─────────────────────────────────────────────────────────

    def _gravity_torque(self, q_rad: np.ndarray) -> np.ndarray:
        """
        Calcula τ_gravity = Σ_i  m_i · gᵀ · J_v_com_i(q)

        Implementación simplificada: solo compensación estática de gravedad.
        Para mayor precisión usar el modelo URDF con pinocchio/KDL.
        """
        from jacobian_utils import _fk_dh, _dh_transform, _DH

        _, frames = _fk_dh(q_rad)
        tau_g = np.zeros(6)

        for link_idx in range(6):
            # Posición del COM del link en frame base
            T_0i = frames[link_idx]
            p_com_local = np.append(_LINK_COM[link_idx], 1.0)
            p_com_world = (T_0i @ p_com_local)[:3]

            # Contribución de este link a cada articulación anterior
            T_prev = np.eye(4)
            for joint_idx in range(link_idx + 1):
                z_j = T_prev[:3, 2]
                p_j = T_prev[:3, 3]
                # τ_j += m_i · (g × (p_com - p_j)) · z_j
                tau_g[joint_idx] += _LINK_MASS[link_idx] * np.dot(
                    np.cross(_G, p_com_world - p_j), z_j
                )
                T_prev = frames[joint_idx]

        return tau_g
