"""
xArm Lite 6 kinematics and simplified dynamics.

Forward kinematics and geometric Jacobian using joint origins from
lite6_default_kinematics.yaml (URDF convention).

Joint transform: T_parent_child(q_i) = Trans(x,y,z) * RPY(r,p,y) * Rz(q_i)

Adapted from xarm_perturbations/xarm_kinematics.py with the addition of
a full 6x6 geometric Jacobian (linear + angular) for force estimation.
"""
import numpy as np

_PI = np.pi

# Joint origins from lite6_default_kinematics.yaml
# Format: ([x, y, z], [roll, pitch, yaw])
_JOINT_ORIGINS = [
    ([0.0,    0.0,       0.2435],  [0.0,     0.0,    0.0]),
    ([0.0,    0.0,       0.0],     [_PI/2,  -_PI/2,  _PI]),
    ([0.2002, 0.0,       0.0],     [-_PI,    0.0,    _PI/2]),
    ([0.087,  -0.22761,  0.0],     [_PI/2,   0.0,    0.0]),
    ([0.0,    0.0,       0.0],     [_PI/2,   0.0,    0.0]),
    ([0.0,    0.0625,    0.0],     [-_PI/2,  0.0,    0.0]),
]

# Link inertial parameters from xarm6_type9_HT_BR2.yaml
_LINK_MASS = np.array([1.411, 1.34, 0.953, 1.284, 0.804, 0.13])

_LINK_COM = [
    np.array([-0.00036,  0.04195,  -0.0025]),
    np.array([ 0.179,    0.0,       0.0584]),
    np.array([ 0.072,   -0.0357,   -0.001]),
    np.array([-0.002,   -0.0285,   -0.0813]),
    np.array([ 0.0,      0.01,      0.0019]),
    np.array([ 0.0,     -0.00194,  -0.0102]),
]

_G_VEC = np.array([0.0, 0.0, -9.81])


def _rpy_to_rot(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF RPY: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
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
    T = np.eye(4)
    T[:3, :3] = _rpy_to_rot(*rpy)
    T[:3,  3] = xyz
    return T


def _rz4(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[0, 0] =  c;  T[0, 1] = -s
    T[1, 0] =  s;  T[1, 1] =  c
    return T


_T_FIXED = [_make_T(xyz, rpy) for xyz, rpy in _JOINT_ORIGINS]


# ---------------------------------------------------------------------------
# Public API
# ---------------------------------------------------------------------------

def forward_kinematics(q: np.ndarray) -> np.ndarray:
    """4x4 homogeneous transform from link_base to link_eef."""
    T = np.eye(4)
    for i in range(6):
        T = T @ _T_FIXED[i] @ _rz4(q[i])
    return T


def get_eef_position(q: np.ndarray) -> np.ndarray:
    """End-effector position in link_base frame (3,)."""
    return forward_kinematics(q)[:3, 3]


def _get_frames(q: np.ndarray):
    """Returns (link_frames, joint_frames) as lists of 4x4 arrays."""
    T = np.eye(4)
    link_frames = []
    joint_frames = []
    for i in range(6):
        T = T @ _T_FIXED[i]
        joint_frames.append(T.copy())
        T = T @ _rz4(q[i])
        link_frames.append(T.copy())
    return link_frames, joint_frames


def position_jacobian(q: np.ndarray) -> np.ndarray:
    """3x6 geometric position Jacobian: J_v[:, i] = z_i x (p_e - p_i)."""
    T_acc = np.eye(4)
    joint_frames = []
    for i in range(6):
        T_acc = T_acc @ _T_FIXED[i]
        joint_frames.append(T_acc.copy())
        T_acc = T_acc @ _rz4(q[i])

    pe = T_acc[:3, 3]
    J = np.zeros((3, 6))
    for i in range(6):
        zi = joint_frames[i][:3, 2]
        pi = joint_frames[i][:3, 3]
        J[:, i] = np.cross(zi, pe - pi)
    return J


def geometric_jacobian(q: np.ndarray) -> np.ndarray:
    """
    Full 6x6 geometric Jacobian.

    Top 3 rows: linear velocity  J_v[:, i] = z_i x (p_e - p_i)
    Bottom 3 rows: angular velocity  J_w[:, i] = z_i
    """
    T_acc = np.eye(4)
    joint_frames = []
    for i in range(6):
        T_acc = T_acc @ _T_FIXED[i]
        joint_frames.append(T_acc.copy())
        T_acc = T_acc @ _rz4(q[i])

    pe = T_acc[:3, 3]
    J = np.zeros((6, 6))
    for i in range(6):
        zi = joint_frames[i][:3, 2]
        pi = joint_frames[i][:3, 3]
        J[:3, i] = np.cross(zi, pe - pi)
        J[3:, i] = zi
    return J


def gravity_torques(q: np.ndarray) -> np.ndarray:
    """Gravity torque vector G(q) using potential-energy Jacobian."""
    link_frames, joint_frames = _get_frames(q)

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


def condition_number(J: np.ndarray) -> float:
    """Condition number of J (singularity detector)."""
    return float(np.linalg.cond(J))
