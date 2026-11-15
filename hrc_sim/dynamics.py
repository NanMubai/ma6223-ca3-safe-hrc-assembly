"""Independent rigid-body dynamics using NumPy DH body/COM Jacobians.

Only from_mujoco_model reads model constants, once, through a duck-typed model
interface. All subsequent FK, Jacobians, M, g, h and task terms use only NumPy.
M_total includes armature; rigid_mass_matrix excludes it to prevent double-counting.
"""
from dataclasses import dataclass, field
import numpy as np
from .kinematics import (PandaKinematics, point_jacobian, point_jacobian_dot,
                         quaternion_matrix, transform, vector7)


@dataclass(frozen=True)
class BodyInertia:
    """COM and COM inertia in body axes, plus fixed DH-link-to-body transform."""
    name: str
    link_index: int
    mass: float
    com: np.ndarray
    inertia: np.ndarray
    link_to_body: np.ndarray = field(default_factory=lambda: np.eye(4))


class PandaDynamics:
    def __init__(self, bodies, kinematics=None, armature=None, damping=None,
                 gravity=(0., 0., -9.81), task_damping=.01):
        self.task_damping = task_damping
        self.bodies = tuple(bodies)
        self.kinematics = kinematics or PandaKinematics()
        self.armature = vector7(np.full(7, .1) if armature is None else armature, 'armature').copy()
        self.damping = vector7(np.ones(7) if damping is None else damping, 'damping').copy()
        self.gravity = np.asarray(gravity, dtype=float).copy()
        if self.gravity.shape != (3,) or not np.all(np.isfinite(self.gravity)):
            raise ValueError('gravity must be finite (3,)')
        if np.any(self.armature < 0) or np.any(self.damping < 0):
            raise ValueError('armature and damping must be nonnegative')
        for body in self.bodies:
            if not 0 <= body.link_index <= 7 or body.mass < 0:
                raise ValueError('Invalid body link index or mass')
            if np.shape(body.com) != (3,) or np.shape(body.inertia) != (3, 3):
                raise ValueError('Body COM/inertia must have shapes (3,) and (3,3)')

    @classmethod
    def from_mujoco_model(cls, model, kinematics=None):
        """Extract constant masses, COMs and inertias ONCE; never call mj_jac.

        MuJoCo stores diagonal principal moments and an inertial-frame rotation;
        reconstruct the full COM tensor in the native body frame before use.
        The independent hand tree is deliberately excluded from the arm model.
        """
        attachment = model.body('attachment')
        tcp = model.site('tcp')
        kin = kinematics or PandaKinematics(
            link7_to_flange=transform(attachment.pos, quaternion_matrix(attachment.quat)),
            flange_to_tcp=transform(tcp.pos, quaternion_matrix(tcp.quat)))
        names = [f'link{i}' for i in range(1, 8)] + ['tool']
        bodies = []
        for name in names:
            b = model.body(name)
            Ri = quaternion_matrix(b.iquat)
            I = Ri @ np.diag(b.inertia) @ Ri.T
            index = 7 if name == 'tool' else int(name[4:])
            fixed = np.eye(4)
            if name == 'tool':
                fixed = kin.link7_to_flange @ transform(b.pos, quaternion_matrix(b.quat))
            bodies.append(BodyInertia(name, index, float(b.mass[0]), b.ipos.copy(), I, fixed))
        joint_ids = np.array([model.joint(f'joint{i}').id for i in range(1, 8)])
        dofs = model.jnt_dofadr[joint_ids]
        return cls(bodies, kin, model.dof_armature[dofs], model.dof_damping[dofs], model.opt.gravity)

    def _body_terms(self, state, body, dq=None):
        T = state.links[body.link_index] @ body.link_to_body
        p = T[:3, :3] @ body.com + T[:3, 3]
        I = T[:3, :3] @ body.inertia @ T[:3, :3].T
        J = point_jacobian(state, p, body.link_index)
        Jdot = None if dq is None else point_jacobian_dot(state, p, dq, body.link_index)
        return p, I, J, Jdot

    def mass_matrix(self, q, include_armature=True):
        state = self.kinematics.evaluate(q)
        M = np.diag(self.armature) if include_armature else np.zeros((7, 7))
        for body in self.bodies:
            _, I, J, _ = self._body_terms(state, body)
            M += body.mass*J[:3].T@J[:3] + J[3:].T@I@J[3:]
        return M

    def rigid_mass_matrix(self, q):
        return self.mass_matrix(q, include_armature=False)

    def gravity_torque(self, q):
        state = self.kinematics.evaluate(q)
        g = np.zeros(7)
        for body in self.bodies:
            _, _, J, _ = self._body_terms(state, body)
            g -= body.mass * J[:3].T @ self.gravity
        return g

    def coriolis_forces(self, q, dq):
        """h=C(q,dq)dq from COM bias accelerations and gyroscopic moments.

        This Newton-Euler/Jacobian sum equals the Christoffel-symbol vector,
        without numerical mass-matrix differentiation or a MuJoCo call.
        """
        dq = vector7(dq, 'dq')
        state = self.kinematics.evaluate(q)
        h = np.zeros(7)
        for body in self.bodies:
            _, I, J, Jdot = self._body_terms(state, body, dq)
            omega = J[3:]@dq
            h += J[:3].T @ (body.mass*(Jdot[:3]@dq))
            h += J[3:].T @ (I@(Jdot[3:]@dq) + np.cross(omega, I@omega))
        return h

    def bias_forces(self, q, dq):
        return self.coriolis_forces(q, dq) + self.gravity_torque(q)

    def inverse_dynamics(self, q, dq, ddq, include_damping=True):
        dq, ddq = vector7(dq, 'dq'), vector7(ddq, 'ddq')
        tau = self.mass_matrix(q)@ddq + self.bias_forces(q, dq)
        return tau + self.damping*dq if include_damping else tau

    def task_space_dynamics(self, q, dq=None, damping=None, include_joint_damping=False):
        """Independent world task terms, with explicit N_vel and N_tau projectors.

        mu excludes passive joint damping by default; explicitly compensate D*dq
        at the actuator, or select include_joint_damping=True for mu += mu_D.
        Damped inverse uses A + damping**2 I and sacrifices exact decoupling.
        """
        dq = np.zeros(7) if dq is None else vector7(dq, 'dq')
        M, J = self.mass_matrix(q), self.kinematics.jacobian(q)
        Jdot = self.kinematics.jacobian_dot(q, dq)
        h, g = self.coriolis_forces(q, dq), self.gravity_torque(q)
        if include_joint_damping:
            h += self.damping*dq
        return task_space_dynamics(M, J, h, g, Jdot@dq, damping=self.task_damping if damping is None else damping)


def task_space_dynamics(M, J, h=None, g=None, jdot_dq=None, damping=.01):
    """Return Lambda, mu, p, Jbar, N_vel, N_tau and Jdot_dq.

    M is TOTAL arm inertia, including armature. jdot_dq is [a_bias; alpha_bias].
    damping=0 requires full row rank; damping>0 returns regularized task terms.
    """
    if not np.isfinite(damping) or damping < 0:
        raise ValueError('damping must be finite and nonnegative')
    M, J = np.asarray(M), np.asarray(J)
    n, m = M.shape[0], J.shape[0]
    h = np.zeros(n) if h is None else np.asarray(h)
    g = np.zeros(n) if g is None else np.asarray(g)
    jdot_dq = np.zeros(m) if jdot_dq is None else np.asarray(jdot_dq)
    MinvJT = np.linalg.solve(M, J.T)
    A = J@MinvJT
    Lambda = np.linalg.solve(A + damping**2*np.eye(m), np.eye(m))
    Jbar = MinvJT@Lambda
    mu = Lambda@(J@np.linalg.solve(M, h)-jdot_dq)
    p = Lambda@J@np.linalg.solve(M, g)
    return dict(Lambda=Lambda, mu=mu, p=p, Jbar=Jbar, N_vel=np.eye(n)-Jbar@J,
                N_tau=np.eye(n)-J.T@Jbar.T, Jdot_dq=jdot_dq.copy())
