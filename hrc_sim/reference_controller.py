"""Minimal constant Cartesian impedance example, not part E's controller."""
import numpy as np
from .config import ReferenceControllerConfig


class ReferenceController:
    def __init__(self, env, config=None, q_posture=None):
        self.env = env
        self.config = config or ReferenceControllerConfig()
        self.q_posture = np.array(env.config.q_home if q_posture is None else q_posture, dtype=float)

    def compute(self, position, rotation=None):
        """Return seven commanded torques WITHOUT gravity (env adds it)."""
        p, R, _ = self.env.get_ee_pose()
        Rd = np.diag([-1., 1., -1.]) if rotation is None else np.asarray(rotation)
        er = 0.5 * sum((np.cross(R[:, i], Rd[:, i]) for i in range(3)), start=np.zeros(3))
        e = np.r_[np.asarray(position)-p, er]
        J = self.env.jacobian()
        c = self.config
        wrench = c.stiffness*e - c.damping*self.env.get_ee_velocity()
        q, dq = self.env.get_joint_state()
        N_tau = self.env.task_space_dynamics()['N_tau']
        return J.T@wrench + N_tau@(c.posture_stiffness*(self.q_posture-q)-c.posture_damping*dq)

    __call__ = compute
