"""
Virtual force/torque sensor for xArm Lite 6.

Estimates external wrench at the end effector from motor torque residuals:
    F_ext = (J^T)+ * (tau_measured - tau_gravity)

Uses damped least-squares (DLS) pseudo-inverse to avoid noise amplification
near singularities, followed by a first-order low-pass filter.

Adapted from xarm_teleops/src/force_estimator.py + jacobian_utils.py,
merged into a self-contained module using kinematics from this package.
"""
import numpy as np

from xarm_bilateral_teleop.kinematics import (
    geometric_jacobian,
    gravity_torques,
)


class ForceEstimator:
    """
    Sensorless external force estimator for xArm Lite 6.

    Usage:
        est = ForceEstimator(control_hz=100)
        F_est = est.update(q_rad, tau_measured)
        in_contact = est.is_contact(F_est)
    """

    def __init__(
        self,
        control_hz: float = 100.0,
        cutoff_hz: float = 5.0,
        force_threshold_n: float = 2.0,
        damping_dls: float = 0.1,
    ):
        self.dt = 1.0 / control_hz
        self.force_threshold = force_threshold_n
        self.damping = damping_dls

        # First-order LPF coefficient: alpha = exp(-2*pi*fc*Ts)
        self._alpha = np.exp(-2.0 * np.pi * cutoff_hz * self.dt)
        self._f_filtered = np.zeros(6)

        # Calibration: bias (mean residual at rest) and noise std
        self._bias = np.zeros(6)
        self._noise_std = np.zeros(6)

    def update(self, q_rad: np.ndarray, tau_measured: np.ndarray) -> np.ndarray:
        """
        Estimate external wrench [Fx, Fy, Fz, Mx, My, Mz] in base frame.

        Args:
            q_rad: joint angles [rad], shape (6,)
            tau_measured: joint torques [Nm], shape (6,)

        Returns:
            F_est: estimated wrench, shape (6,)
        """
        q = np.asarray(q_rad, dtype=float).ravel()
        tau = np.asarray(tau_measured, dtype=float).ravel()

        # 1. Gravity compensation
        tau_g = gravity_torques(q)
        delta_tau = tau - tau_g

        # 2. Jacobian and DLS pseudo-inverse of J^T
        J = geometric_jacobian(q)
        Jt = J.T
        JtJ = Jt @ J
        try:
            Jt_pinv = J @ np.linalg.inv(JtJ + self.damping**2 * np.eye(6))
            F_raw = Jt_pinv @ delta_tau - self._bias
        except np.linalg.LinAlgError:
            F_raw = np.zeros(6)

        # 3. Guard against NaN (numerical issues near singularity)
        if not np.isfinite(F_raw).all():
            F_raw = np.zeros(6)

        # 4. Low-pass filter
        self._f_filtered = (
            self._alpha * self._f_filtered + (1.0 - self._alpha) * F_raw
        )

        return self._f_filtered.copy()

    def is_contact(self, F_est: np.ndarray) -> bool:
        """True if linear force magnitude exceeds threshold."""
        return float(np.linalg.norm(F_est[:3])) > self.force_threshold

    def reset_filter(self) -> None:
        self._f_filtered = np.zeros(6)

    def calibrate_noise(
        self, q_rad: np.ndarray, tau_samples: np.ndarray
    ) -> None:
        """
        Calibrate bias and noise floor at rest.
        Call with robot stationary for ~2 seconds.

        The bias captures the mean residual (e.g. gravity compensation
        mismatch in fake hardware) and is subtracted in subsequent updates.

        Args:
            q_rad: rest configuration [rad]
            tau_samples: torque samples (N_samples x 6)
        """
        # Temporarily zero out the bias so we measure the raw residual
        self._bias = np.zeros(6)
        self.reset_filter()
        estimates = np.array([
            self.update(q_rad, tau_samples[i]) for i in range(len(tau_samples))
        ])
        # Use the last half of samples (after LPF has converged) for bias
        n_half = max(1, len(estimates) // 2)
        self._bias = estimates[n_half:].mean(axis=0)
        self._noise_std = estimates[n_half:].std(axis=0)
        self.reset_filter()

    def suggest_threshold(self, safety_factor: float = 3.0) -> float:
        """Suggest contact threshold = safety_factor * noise std."""
        mean_std = float(self._noise_std[:3].mean())
        if np.isfinite(mean_std) and mean_std > 0.0:
            return safety_factor * mean_std
        return self.force_threshold
