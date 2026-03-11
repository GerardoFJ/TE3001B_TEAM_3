#!/usr/bin/env python3
"""
xArm Lite 6 Kinematics and Simplified Dynamics.

Forward kinematics and geometric Jacobian using the joint origins
from lite6_default_kinematics.yaml (URDF convention).

Joint transform convention (URDF):
    T_parent_child(q_i) = Trans(x,y,z) * RPY(roll,pitch,yaw) * Rz(q_i)

All joint axes are Z in each joint's local frame (axis xyz="0 0 1" in URDF).

Frames: link_base -> link1 -> ... -> link6 -> link_eef
link_eef is fixed at link6 with zero offset.

Joint origins (from lite6_default_kinematics.yaml):
  joint1: xyz=(0, 0, 0.2435),      rpy=(0, 0, 0)
  joint2: xyz=(0, 0, 0),           rpy=(pi/2, -pi/2, pi)
  joint3: xyz=(0.2002, 0, 0),      rpy=(-pi, 0, pi/2)
  joint4: xyz=(0.087, -0.22761, 0),rpy=(pi/2, 0, 0)
  joint5: xyz=(0, 0, 0),           rpy=(pi/2, 0, 0)
  joint6: xyz=(0, 0.0625, 0),      rpy=(-pi/2, 0, 0)
"""
import numpy as np

# ---------------------------------------------------------------------------
# Joint origins from lite6_default_kinematics.yaml
# Format: ([x, y, z], [roll, pitch, yaw])  -- all in metres / radians
# ---------------------------------------------------------------------------
_PI = np.pi
_JOINT_ORIGINS = [
    ([0.0,    0.0,       0.2435],  [0.0,     0.0,    0.0]),     # joint1
    ([0.0,    0.0,       0.0],     [_PI/2,  -_PI/2,  _PI]),     # joint2
    ([0.2002, 0.0,       0.0],     [-_PI,    0.0,    _PI/2]),    # joint3
    ([0.087,  -0.22761,  0.0],     [_PI/2,   0.0,    0.0]),      # joint4
    ([0.0,    0.0,       0.0],     [_PI/2,   0.0,    0.0]),      # joint5
    ([0.0,    0.0625,    0.0],     [-_PI/2,  0.0,    0.0]),      # joint6
]

# ---------------------------------------------------------------------------
# Link inertial parameters from xarm6_type9_HT_BR2.yaml
# COM positions are in each link's local frame [m]
# ---------------------------------------------------------------------------
_LINK_MASS = np.array([1.411, 1.34, 0.953, 1.284, 0.804, 0.13])  # kg

_LINK_COM = [
    np.array([-0.00036,  0.04195,  -0.0025]),   # link1 COM in link1 frame
    np.array([ 0.179,    0.0,       0.0584]),    # link2 COM in link2 frame
    np.array([ 0.072,   -0.0357,   -0.001]),     # link3 COM in link3 frame
    np.array([-0.002,   -0.0285,   -0.0813]),    # link4 COM in link4 frame
    np.array([ 0.0,      0.01,      0.0019]),    # link5 COM in link5 frame
    np.array([ 0.0,     -0.00194,  -0.0102]),    # link6 COM in link6 frame
]

# Gravity vector in link_base frame (pointing down)
_G_VEC = np.array([0.0, 0.0, -9.81])

# Diagonal joint inertia approximation (kg·m²) -- tuned rough estimates
_M_DIAG = np.array([0.50, 0.50, 0.20, 0.15, 0.05, 0.02])


# ---------------------------------------------------------------------------
# Internal helpers
# ---------------------------------------------------------------------------

def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  →  3×3."""
    cr, sr = np.cos(roll),  np.sin(roll)
    cp, sp = np.cos(pitch), np.sin(pitch)
    cy, sy = np.cos(yaw),   np.sin(yaw)
    Rx = np.array([[1,  0,   0 ],
                   [0,  cr, -sr],
                   [0,  sr,  cr]])
    Ry = np.array([[ cp, 0, sp],
                   [  0, 1,  0],
                   [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0],
                   [sy,  cy, 0],
                   [ 0,   0, 1]])
    return Rz @ Ry @ Rx


def _make_T(xyz, rpy) -> np.ndarray:
    """Build 4×4 homogeneous transform from xyz list and rpy list."""
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3,  3] = xyz
    return T


def _rz4(theta: float) -> np.ndarray:
    """4×4 rotation matrix about Z by theta."""
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[0, 0] =  c;  T[0, 1] = -s
    T[1, 0] =  s;  T[1, 1] =  c
    return T


# Pre-compute fixed joint-origin transforms once
_T_FIXED = [_make_T(xyz, rpy) for xyz, rpy in _JOINT_ORIGINS]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """
    4×4 homogeneous transform from link_base to link_eef.

    Args:
        q: (6,) joint angles [rad]

    Returns:
        T: (4, 4) transform matrix
    """
    T = np.eye(4)
    for i in range(6):
        T = T @ _T_FIXED[i] @ _rz4(q[i])
    return T


def get_eef_position(q: np.ndarray) -> np.ndarray:
    """
    End-effector position in link_base frame.

    Args:
        q: (6,) joint angles [rad]

    Returns:
        p: (3,) position [m]
    """
    return forward_kinematics(q)[:3, 3]


def position_jacobian(q: np.ndarray) -> np.ndarray:
    """
    3×6 geometric position Jacobian.

    Column i:  J[:, i] = z_i × (p_e - p_i)
    where z_i is the Z-axis of joint i's frame (before applying q_i),
    and   p_i is the origin of that same frame.

    Args:
        q: (6,) joint angles [rad]

    Returns:
        J: (3, 6) Jacobian
    """
    # Accumulate transforms, snapping the joint axis frame (fixed part only)
    T_acc = np.eye(4)
    joint_frames = []
    for i in range(6):
        T_acc = T_acc @ _T_FIXED[i]         # fixed origin → joint axis frame
        joint_frames.append(T_acc.copy())    # save before applying rotation
        T_acc = T_acc @ _rz4(q[i])          # apply joint rotation

    pe = T_acc[:3, 3]   # EEF position after all joints

    J = np.zeros((3, 6))
    for i in range(6):
        zi = joint_frames[i][:3, 2]   # Z-axis of joint i (col 2 of rotation)
        pi = joint_frames[i][:3, 3]   # origin of joint i frame
        J[:, i] = np.cross(zi, pe - pi)

    return J


def _get_link_frames_and_joint_frames(q: np.ndarray):
    """
    Internal helper: returns (link_frames, joint_frames) as lists of 4×4 arrays.

    link_frames[i]  = transform after applying q[i]  (child link frame)
    joint_frames[i] = transform before applying q[i] (joint axis frame)
    """
    T = np.eye(4)
    link_frames   = []
    joint_frames  = []
    for i in range(6):
        T = T @ _T_FIXED[i]
        joint_frames.append(T.copy())
        T = T @ _rz4(q[i])
        link_frames.append(T.copy())
    return link_frames, joint_frames


def gravity_torques(q: np.ndarray) -> np.ndarray:
    """
    Gravity torque vector G(q) using the potential-energy Jacobian.

    G[i] = Σ_{j=i}^{5}  m_j * g_vec · (z_i × (p_com_j - p_i))
    Args:
        q: (6,) joint angles [rad]

    Returns:
        G: (6,) gravity torques [N·m]
    """
    link_frames, joint_frames = _get_link_frames_and_joint_frames(q)

    # COM positions in link_base frame
    p_com = []
    for j in range(6):
        R_j = link_frames[j][:3, :3]
        t_j = link_frames[j][:3,  3]
        p_com.append(t_j + R_j @ _LINK_COM[j])

    G = np.zeros(6)
    for i in range(6):
        zi = joint_frames[i][:3, 2]
        pi = joint_frames[i][:3, 3]
        for j in range(i, 6):
            J_col = np.cross(zi, p_com[j] - pi)
            G[i] += _LINK_MASS[j] * float(np.dot(_G_VEC, J_col))

    return G


def inertia_matrix(q: np.ndarray) -> np.ndarray:
    """
    Diagonal approximation of the joint-space mass/inertia matrix.

    A full rigid-body model requires the CRBA algorithm; this provides
    a diagonal estimate suitable for CTC feedforward on low-speed motions.

    Args:
        q: (6,) joint angles [rad]  (unused in diagonal approx but kept for API)

    Returns:
        M: (6, 6) diagonal inertia matrix [kg·m²]
    """
    return np.diag(_M_DIAG)


def friction_torques(qd: np.ndarray,
                     b_visc: np.ndarray | None = None,
                     fc: np.ndarray | None = None) -> np.ndarray:
    """
    Viscous + Coulomb friction model.

    F[i] = b_visc[i]*qd[i] + fc[i]*tanh(qd[i]/eps)

    Args:
        qd:     (6,) joint velocities [rad/s]
        b_visc: (6,) viscous coefficients [N·m·s/rad]  (default provided)
        fc:     (6,) Coulomb amplitudes   [N·m]        (default provided)

    Returns:
        F: (6,) friction torques [N·m]
    """
    if b_visc is None:
        b_visc = np.array([1.0, 1.0, 1.0, 0.8, 0.5, 0.3])
    if fc is None:
        fc = np.array([0.5, 0.5, 0.3, 0.3, 0.2, 0.1])
    return b_visc * qd + fc * np.tanh(qd / 0.01)
