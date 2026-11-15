"""Independent NumPy Craig-DH Panda kinematics, SI units and world [v; omega].

DH dimensions: Franka's Craig-convention kinematic table, cross-checked against
vendored Menagerie panda_nohand.xml. See docs/modeling_notes.md for references.
No MuJoCo or SciPy operations are used in this module.
"""
from dataclasses import dataclass
import numpy as np

# Columns: a_(i-1) [m], alpha_(i-1) [rad], d_i [m], theta offset [rad].
DH_PARAMETERS = np.array([
    [0., 0., .333, 0.],
    [0., -np.pi/2, 0., 0.],
    [0., np.pi/2, .316, 0.],
    [.0825, np.pi/2, 0., 0.],
    [-.0825, -np.pi/2, .384, 0.],
    [0., np.pi/2, 0., 0.],
    [.088, np.pi/2, 0., 0.],
])
DH_PARAMETERS.setflags(write=False)


def vector7(q, name='q'):
    q = np.asarray(q, dtype=float)
    if q.shape != (7,) or not np.all(np.isfinite(q)):
        raise ValueError(f'{name} must be a finite (7,) vector')
    return q


def quaternion_matrix(quat):
    """Rotation matrix of a (w,x,y,z) quaternion, normalized internally."""
    q = np.asarray(quat, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-15:
        raise ValueError('Expected finite nonzero wxyz quaternion')
    w, x, y, z = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def transform(position=(0., 0., 0.), rotation=None):
    T = np.eye(4)
    T[:3, 3] = position
    if rotation is not None:
        T[:3, :3] = rotation
    return T


def modified_dh(a, alpha, d, theta):
    """Craig A_i = Rx(alpha) Tx(a) Rz(theta) Tz(d)."""
    ct, st, ca, sa = np.cos(theta), np.sin(theta), np.cos(alpha), np.sin(alpha)
    return np.array([[ct, -st, 0., a],
                     [st*ca, ct*ca, -sa, -d*sa],
                     [st*sa, ct*sa, ca, d*ca],
                     [0., 0., 0., 1.]])


@dataclass(frozen=True)
class KinematicState:
    links: np.ndarray   # [base, link1, ..., link7], shape (8,4,4).
    origins: np.ndarray # Joint origins in world, shape (7,3).
    axes: np.ndarray    # Joint +z axes in world, shape (7,3).
    flange: np.ndarray  # Model attachment frame (includes fixed z rotation).
    tcp: np.ndarray


def point_jacobian(state, point, n_joints=7):
    """Geometric world Jacobian of a point on a body after n_joints joints."""
    J = np.zeros((6, 7))
    J[:3, :n_joints] = np.cross(state.axes[:n_joints], np.asarray(point)-state.origins[:n_joints]).T
    J[3:, :n_joints] = state.axes[:n_joints].T
    return J


def point_jacobian_dot(state, point, dq, n_joints=7):
    """Analytic time derivative for a point fixed to the indicated body.

    z_dot_j = omega_before_j x z_j and o_dot_j follows the upstream joints;
    no finite difference or MuJoCo Jacobian is used.
    """
    dq = vector7(dq, 'dq')
    p = np.asarray(point)
    J = point_jacobian(state, p, n_joints)
    pdot = J[:3] @ dq
    Jdot = np.zeros((6, 7))
    for j in range(n_joints):
        axes, origins = state.axes[:j], state.origins[:j]
        omega = dq[:j] @ axes
        odot = dq[:j] @ np.cross(axes, state.origins[j]-origins)
        zdot = np.cross(omega, state.axes[j])
        Jdot[:3, j] = np.cross(zdot, p-state.origins[j]) + np.cross(state.axes[j], pdot-odot)
        Jdot[3:, j] = zdot
    return Jdot


class PandaKinematics:
    """Panda FK at model flange / TCP and analytic point/TCP Jacobians.

    The default attachment quaternion is normalized exactly as in Menagerie,
    including its authored decimal precision. flange_to_tcp defaults to Tz(.06).
    """
    def __init__(self, peg_length=.06, flange_to_tcp=None, link7_to_flange=None):
        if peg_length <= 0:
            raise ValueError('peg_length must be positive')
        self.link7_to_flange = (transform([0, 0, .107], quaternion_matrix([.3826834, 0, 0, .9238795]))
                                if link7_to_flange is None else np.array(link7_to_flange, dtype=float, copy=True))
        self.flange_to_tcp = (transform([0, 0, peg_length]) if flange_to_tcp is None
                              else np.array(flange_to_tcp, dtype=float, copy=True))
        for T in (self.link7_to_flange, self.flange_to_tcp):
            if T.shape != (4, 4) or not np.all(np.isfinite(T)):
                raise ValueError('Fixed transforms must be finite (4,4) matrices')

    def evaluate(self, q):
        q = vector7(q)
        links = [np.eye(4)]
        for theta, (a, alpha, d, offset) in zip(q, DH_PARAMETERS):
            links.append(links[-1] @ modified_dh(a, alpha, d, theta+offset))
        links = np.asarray(links)
        # The rotation Rz(theta) leaves its own axis and origin unchanged, so
        # the post-joint frame has the same joint origin/z as the pre-joint one.
        origins, axes = links[1:, :3, 3].copy(), links[1:, :3, 2].copy()
        flange = links[-1] @ self.link7_to_flange
        return KinematicState(links, origins, axes, flange, flange@self.flange_to_tcp)

    def fk_flange(self, q):
        """4x4 world pose of the model attachment/flange frame."""
        return self.evaluate(q).flange

    def fk_tcp(self, q):
        """4x4 world pose of the peg end-face TCP."""
        return self.evaluate(q).tcp

    def jacobian(self, q):
        state = self.evaluate(q)
        return point_jacobian(state, state.tcp[:3, 3])

    def jacobian_dot(self, q, dq):
        state = self.evaluate(q)
        return point_jacobian_dot(state, state.tcp[:3, 3], dq)


def fk_flange(q):
    return PandaKinematics().fk_flange(q)


def fk_tcp(q):
    return PandaKinematics().fk_tcp(q)


def jacobian(q):
    return PandaKinematics().jacobian(q)
