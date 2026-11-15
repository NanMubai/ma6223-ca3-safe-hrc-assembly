"""Seven-axis Panda API with a separate dynamic, mocap-driven hand.

Public joint arrays always use joint1..joint7 order. Full MuJoCo state additionally
contains the hand free joint. Wrenches are environment-on-robot at the TCP, [f;t].
"""
from copy import deepcopy
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from .config import SimConfig

ASSETS = Path(__file__).resolve().parents[1] / 'assets'


def _s(a):
    return ' '.join(str(float(x)) for x in a)


def _quat(a):
    q = np.asarray(a, dtype=float)
    if q.shape != (4,) or not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-12:
        raise ValueError('Quaternion must be finite (w,x,y,z), with nonzero norm')
    return q / np.linalg.norm(q)


def _model_xml(c):
    """Apply configuration before compilation, including correct box inertias."""
    root = ET.parse(ASSETS / 'scene_box_in_box.xml').getroot()
    inc = root.find('include')
    panda = ET.parse(ASSETS / inc.get('file')).getroot()
    root.remove(inc)
    for element in reversed(list(panda)):
        root.insert(0, element)
    root.find('compiler').set('meshdir', str(ASSETS / 'franka_emika_panda/assets'))
    root.find('option').attrib.update(timestep=str(c.dt), gravity=_s(c.gravity))
    for geom in root.findall('.//geom'):
        # Defaults are included too, so inherited collision meshes get these values.
        geom.attrib.update(friction=_s(c.friction), solref=_s(c.solref), solimp=_s(c.solimp))
    def named(tag, name):
        return root.find(f'.//{tag}[@name="{name}"]')
    for i in range(7):
        named('joint', f'joint{i+1}').set('range', _s([c.q_min[i], c.q_max[i]]))
        named('motor', f'actuator{i+1}').set('ctrlrange', _s([-c.tau_max[i], c.tau_max[i]]))
    z = c.peg_half_sizes[2]
    named('geom', 'peg').attrib.update(size=_s(c.peg_half_sizes), pos=_s([0, 0, z]), mass=str(c.peg_mass))
    named('site', 'tcp').set('pos', _s([0, 0, 2*z]))
    named('body', 'fixture').attrib.update(pos=_s(c.hole_pos), quat=_s(_quat(c.hole_quat)))
    hx, hy = c.hole_half_sizes
    t, depth, bevel = c.wall_thickness, c.hole_depth, c.chamfer
    fixture = named('body', 'fixture')
    for geom in list(fixture.findall('geom')):
        if geom.get('name', '').startswith(('chamfer_', 'rim_')):
            fixture.remove(geom)
    for axis, sign, name in [(0, 1, 'hole_xplus'), (0, -1, 'hole_xminus'), (1, 1, 'hole_yplus'), (1, -1, 'hole_yminus')]:
        pos = [0., 0., -(depth+bevel)/2]
        pos[axis] = sign * ((hx if axis == 0 else hy) + t/2)
        size = [t/2, hy+t, (depth-bevel)/2] if axis == 0 else [hx, t/2, (depth-bevel)/2]
        named('geom', name).attrib.update(pos=_s(pos), size=_s(size))
        if bevel:
            h = hx if axis == 0 else hy
            # Inner face joins (h,-bevel) to (h+bevel,0): a true 45-degree
            # planar lead-in, built from rotated boxes, not a staircase.
            pos = [0., 0., -.75*bevel]
            pos[axis] = sign*(h+.75*bevel)
            angle = sign*np.pi/4 if axis == 0 else -sign*np.pi/4
            quat = [np.cos(angle/2), 0., np.sin(angle/2), 0.] if axis == 0 else [np.cos(angle/2), np.sin(angle/2), 0., 0.]
            size = [bevel/(2*np.sqrt(2)), hy+t, bevel/np.sqrt(2)] if axis == 0 else [hx+t, bevel/(2*np.sqrt(2)), bevel/np.sqrt(2)]
            attrs = dict(type='box', rgba='0.30 0.55 0.70 1', friction=_s(c.friction), solref=_s(c.solref), solimp=_s(c.solimp))
            ET.SubElement(fixture, 'geom', name='chamfer_'+name, pos=_s(pos), quat=_s(quat), size=_s(size), **attrs)
            pos[axis], pos[2] = sign*(h+(t+bevel)/2), -bevel/2
            size = [(t-bevel)/2, hy+t, bevel/2] if axis == 0 else [hx+bevel, (t-bevel)/2, bevel/2]
            ET.SubElement(fixture, 'geom', name='rim_'+name, pos=_s(pos), size=_s(size), **attrs)
    named('geom', 'hole_floor').attrib.update(pos=_s([0, 0, -depth-t/2]), size=_s([hx+t, hy+t, t/2]))
    named('site', 'hole_bottom').set('pos', _s([0, 0, -depth]))
    # Table surface sits under the fixture floor; it does not obstruct the hole.
    table = np.asarray(c.hole_pos, dtype=float) - [0, 0, depth+t]
    named('body', 'table').set('pos', _s(table))
    named('geom', 'table_top').attrib.update(pos=_s([0, 0, -c.table_half_sizes[2]]), size=_s(c.table_half_sizes))
    named('body', 'hand_target').attrib.update(pos=_s(c.hand.parked_pos), quat=_s(_quat(c.hand.parked_quat)))
    named('body', 'human_hand').attrib.update(pos=_s(c.hand.parked_pos), quat=_s(_quat(c.hand.parked_quat)), gravcomp=str(float(c.hand.gravity_compensation)))
    named('geom', 'hand').attrib.update(size=_s([c.hand.radius, c.hand.cylinder_half_length]), mass=str(c.hand.mass), solref=_s(c.hand.contact_solref), solimp=_s(c.hand.contact_solimp), solmix=str(c.hand.solmix))
    named('weld', 'hand_weld').attrib.update(solref=_s([max(c.hand.kinematic_solref[0], 2*c.dt), c.hand.kinematic_solref[1]]), solimp=_s(c.hand.weld_solimp))
    named('key', 'home').set('qpos', _s([*c.q_home, *c.hand.parked_pos, *_quat(c.hand.parked_quat)]))
    return ET.tostring(root, encoding='unicode')


class HRCSim:
    def __init__(self, config=None):
        self.config = deepcopy(config) if config is not None else SimConfig()
        self.model = mujoco.MjModel.from_xml_string(_model_xml(self.config))
        self.data = mujoco.MjData(self.model)
        self._gravity_data = mujoco.MjData(self.model)
        self.joint_ids = np.array([self.model.joint(f'joint{i}').id for i in range(1, 8)])
        self.qpos_indices = self.model.jnt_qposadr[self.joint_ids].copy()
        self.dof_indices = self.model.jnt_dofadr[self.joint_ids].copy()
        # Actuators must target exactly the seven independent scalar arm joints.
        assert self.model.nu == 7 and len(set(self.dof_indices)) == 7
        assert np.array_equal(self.model.actuator_trnid[:, 0], self.joint_ids)
        assert np.all(self.model.jnt_type[self.joint_ids] == mujoco.mjtJoint.mjJNT_HINGE)
        self.tcp_id = self.model.site('tcp').id
        self.ft_id = self.model.site('ft_site').id
        self.tool_id = self.model.body('tool').id
        self.hand_id = self.model.body('human_hand').id
        self.hand_geom_id = self.model.geom('hand').id
        self.hand_mocap_id = self.model.body('hand_target').mocapid[0]
        self.hand_weld_id = self.model.equality('hand_weld').id
        hj = self.model.joint('hand_free').id
        self._hand_qadr = int(self.model.jnt_qposadr[hj])
        self._hand_dadr = int(self.model.jnt_dofadr[hj])
        base = self.model.body('link0').id
        self.robot_body_ids = {base}
        for b in range(1, self.model.nbody):
            if int(self.model.body_parentid[b]) in self.robot_body_ids:
                self.robot_body_ids.add(b)
        self._hole_geom_ids = set(np.flatnonzero(self.model.geom_bodyid == self.model.body('fixture').id))
        self._force_adr = self.model.sensor('wrist_force').adr[0]
        self._torque_adr = self.model.sensor('wrist_torque').adr[0]
        self._renderer = None
        self._render_size = None
        self._base_seed = self.config.seed
        self._rng = np.random.default_rng(self._base_seed)
        self._ft_noise = np.zeros(6)
        self._tau_noise = np.zeros(7)
        self._last_tau_applied = np.zeros(7)
        self.hand_enabled = False
        self.hand_mode = 'kinematic'
        self.reset()

    def reset(self, q0=None, seed=None):
        """Reset robot, controls, applied forces, time, hand and seeded sensor RNG."""
        q = self.config.q_home if q0 is None else np.asarray(q0, dtype=float)
        if q.shape != (7,) or not np.all(np.isfinite(q)):
            raise ValueError('q0 must be a finite (7,) array')
        if np.any(q < self.config.q_min) or np.any(q > self.config.q_max):
            raise ValueError('q0 outside configured joint limits')
        if seed is not None:
            if not isinstance(seed, (int, np.integer)) or seed < 0:
                raise ValueError('seed must be a nonnegative integer')
            self._base_seed = int(seed)
        self._rng = np.random.default_rng(self._base_seed)
        mujoco.mj_resetDataKeyframe(self.model, self.data, self.model.key('home').id)
        self.data.qpos[self.qpos_indices] = q
        self.set_hand(self.config.hand.parked_pos, self.config.hand.parked_quat, enabled=False)
        self.data.ctrl[:] = np.clip(self.gravity_torque(), -self.config.tau_max, self.config.tau_max) if self.config.gravity_compensation else 0
        self._last_tau_applied = self.data.ctrl.copy()
        self._sample_noise()
        mujoco.mj_forward(self.model, self.data)
        return self._obs()

    def step(self, tau, n_substeps=1):
        """Apply clipped torque with optional gravity and passive-damping compensation.

        tau is held over n_substeps; gravity is recomputed at each physics step.
        Joint velocity limits are metadata for controllers, not velocity clamps.
        """
        tau = np.asarray(tau, dtype=float)
        if tau.shape != (7,) or not np.all(np.isfinite(tau)):
            raise ValueError('tau must be a finite (7,) array')
        if not isinstance(n_substeps, (int, np.integer)) or n_substeps < 1:
            raise ValueError('n_substeps must be a positive integer')
        for _ in range(n_substeps):
            g = self.gravity_torque() if self.config.gravity_compensation else 0
            damping = self.joint_damping*self.data.qvel[self.dof_indices] if self.config.damping_compensation else 0
            self.data.ctrl[:] = np.clip(tau + g + damping, -self.config.tau_max, self.config.tau_max)
            self._last_tau_applied = self.data.ctrl.copy()
            mujoco.mj_step(self.model, self.data)
            self._sample_noise()
        # mj_step leaves derived fields at the pre-integration configuration.
        # Synchronize pose/J/contact/sensor fields with the returned q and time.
        mujoco.mj_forward(self.model, self.data)
        return self._obs()

    def _obs(self):
        p, R, quat = self.get_ee_pose()
        q, dq = self.get_joint_state()
        return dict(t=float(self.data.time), q=q, dq=dq, ee_pos=p, ee_quat=quat,
                    ee_R=R, ee_vel=self.get_ee_velocity(), ft_wrench=self.get_ext_wrench(),
                    contact_wrench=self.get_contact_wrench(), tau_ext=self.get_ext_joint_torque(measured=True))

    def get_joint_state(self):
        return self.data.qpos[self.qpos_indices].copy(), self.data.qvel[self.dof_indices].copy()

    def get_ee_pose(self):
        p = self.data.site_xpos[self.tcp_id].copy()
        R = self.data.site_xmat[self.tcp_id].reshape(3, 3).copy()
        quat = np.empty(4)
        mujoco.mju_mat2Quat(quat, R.ravel())
        return p, R, quat

    def jacobian(self):
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacSite(self.model, self.data, jp, jr, self.tcp_id)
        return np.vstack((jp[:, self.dof_indices], jr[:, self.dof_indices]))

    def jacobian_dot(self):
        jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
        mujoco.mj_jacDot(self.model, self.data, jp, jr, self.data.site_xpos[self.tcp_id], self.tool_id)
        return np.vstack((jp[:, self.dof_indices], jr[:, self.dof_indices]))

    def get_ee_velocity(self):
        return self.jacobian() @ self.data.qvel[self.dof_indices]

    def mass_matrix(self):
        """Exact 7x7 arm block, including joint armature and attached tool.

        The hand is a separate tree: its mass block is uncoupled except through
        contact constraints, which are external forces, not mass-matrix terms.
        """
        M = np.empty((self.model.nv, self.model.nv))
        mujoco.mj_fullM(self.model, self.data, M)
        return M[np.ix_(self.dof_indices, self.dof_indices)].copy()

    def bias_forces(self):
        """Coriolis/centrifugal plus gravity; excludes passive damping/contact."""
        return self.data.qfrc_bias[self.dof_indices].copy()

    def gravity_torque(self):
        """Gravity at current q, independent of velocity and applied forces."""
        d = self._gravity_data
        d.qpos[:] = self.data.qpos
        d.qvel[:] = 0
        mujoco.mj_forward(self.model, d)
        return d.qfrc_bias[self.dof_indices].copy()

    @property
    def joint_damping(self):
        """Seven passive viscous coefficients, N m s/rad (copy)."""
        return self.model.dof_damping[self.dof_indices].copy()

    @property
    def last_tau_applied(self):
        """Actual clipped motor command in the last physics step, N m (copy)."""
        return self._last_tau_applied.copy()

    def limits_violated(self, margin=0.0):
        if not np.isfinite(margin) or margin < 0:
            raise ValueError('margin must be finite and nonnegative')
        q, dq = self.get_joint_state()
        c = self.config
        return dict(position=(q < c.q_min+margin) | (q > c.q_max-margin),
                    velocity=np.abs(dq) > c.dq_max,
                    torque=np.abs(self.last_tau_applied) >= c.tau_max-1e-9)

    def task_space_dynamics(self, damping=None, include_joint_damping=False):
        """Continuous damped task inverse, explicit velocity/torque projectors.

        damping=None uses config.task_damping (default .01); damping=0 requests
        exact identities at full rank. mu excludes passive damping unless asked.
        """
        from .dynamics import task_space_dynamics
        J, M = self.jacobian(), self.mass_matrix()
        g = self.gravity_torque()
        h = self.bias_forces()-g
        dq = self.data.qvel[self.dof_indices]
        if include_joint_damping:
            h += self.joint_damping*dq
        value = self.config.task_damping if damping is None else damping
        return task_space_dynamics(M, J, h, g, self.jacobian_dot()@dq, damping=value)

    def _sample_noise(self):
        # Standard samples are cached: runtime fault amplitudes can be changed
        # immediately, but extra reads never advance either independent stream.
        self._ft_noise = self._rng.standard_normal(6)
        self._tau_noise = self._rng.standard_normal(7)

    def get_ext_wrench(self, frame='world'):
        """Wrist F/T estimate of environment-on-tool wrench at TCP, [f; torque].

        Subtracts full rigid tool gravity AND inertia using simulated qacc.
        Link collisions upstream of this wrist are deliberately invisible.
        Bias/noise default to flange sensor axes/origin; world mode uses TCP.
        Noise is cached once per physics step; reads never advance the RNG.
        """
        if frame not in ('world', 'tcp'):
            raise ValueError("frame must be 'world' or 'tcp'")
        m, d, b = self.model, self.data, self.tool_id
        Rf = d.site_xmat[self.ft_id].reshape(3, 3)
        sensor_f = Rf @ d.sensordata[self._force_adr:self._force_adr+3]
        sensor_t = Rf @ d.sensordata[self._torque_adr:self._torque_adr+3]
        # Newton-Euler tool inertia about its COM, shifted to the flange.
        jp, jr = np.zeros((3, m.nv)), np.zeros((3, m.nv))
        jdp, jdr = np.zeros_like(jp), np.zeros_like(jr)
        com = d.xipos[b]
        mujoco.mj_jac(m, d, jp, jr, com, b)
        mujoco.mj_jacDot(m, d, jdp, jdr, com, b)
        acc = jp @ d.qacc + jdp @ d.qvel
        alpha = jr @ d.qacc + jdr @ d.qvel
        omega = jr @ d.qvel
        Ri = d.ximat[b].reshape(3, 3)
        I = Ri @ np.diag(m.body_inertia[b]) @ Ri.T
        inertial_f = m.body_mass[b] * (acc - m.opt.gravity)
        inertial_t = I @ alpha + np.cross(omega, I @ omega)
        inertial_t += np.cross(com - d.site_xpos[self.ft_id], inertial_f)
        f = inertial_f - sensor_f
        t = inertial_t - sensor_t + np.cross(d.site_xpos[self.ft_id] - d.site_xpos[self.tcp_id], f)
        wrench = np.r_[f, t]
        fault = np.asarray(self.config.sensor_bias) + np.asarray(self.config.noise_std)*self._ft_noise
        if self.config.sensor_bias_frame == 'sensor':
            ff = Rf@fault[:3]
            fault = np.r_[ff, Rf@fault[3:]+np.cross(d.site_xpos[self.ft_id]-d.site_xpos[self.tcp_id], ff)]
        elif self.config.sensor_bias_frame != 'world':
            raise ValueError('sensor_bias_frame must be sensor or world')
        wrench += fault
        if frame == 'tcp':
            Rt = d.site_xmat[self.tcp_id].reshape(3, 3).T
            wrench = np.r_[Rt @ wrench[:3], Rt @ wrench[3:]]
        return wrench

    def _contacts(self, group=None, normals_only=False):
        group = 'all' if group is None else group
        if group not in ('all', 'peg_hole', 'robot_hand'):
            raise ValueError("group must be 'all', 'peg_hole' or 'robot_hand'")
        peg_id = self.model.geom('peg').id
        for i in range(self.data.ncon):
            c = self.data.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if g1 < 0 or g2 < 0 or c.efc_address < 0:
                continue
            b1, b2 = int(self.model.geom_bodyid[g1]), int(self.model.geom_bodyid[g2])
            if group == 'peg_hole' and not ((g1 == peg_id and g2 in self._hole_geom_ids) or (g2 == peg_id and g1 in self._hole_geom_ids)):
                continue
            if group == 'robot_hand' and not ((b1 == self.hand_id and b2 in self.robot_body_ids) or (b2 == self.hand_id and b1 in self.robot_body_ids)):
                continue
            if b1 not in self.robot_body_ids and b2 not in self.robot_body_ids:
                continue
            cf = np.zeros(6)
            mujoco.mj_contactForce(self.model, self.data, i, cf)
            if normals_only:
                yield i, float(abs(cf[0]))
                continue
            Rc = c.frame.reshape(3, 3).T  # Contact axes are ROWS in MuJoCo.
            force, torque = Rc @ cf[:3], Rc @ cf[3:]
            # mj_contactForce is the force on geom2; geom1 gets its negative.
            for b, sign in ((b1, -1), (b2, 1)):
                if b in self.robot_body_ids:
                    yield b, c.pos.copy(), sign*force, sign*torque, float(abs(cf[0]))

    def get_contact_wrench(self, group=None):
        """Sum robot contact wrenches in WORLD at TCP; excludes xfrc_applied."""
        w = np.zeros(6)
        tcp = self.data.site_xpos[self.tcp_id]
        for _, pos, f, t, _ in self._contacts(group):
            w[:3] += f
            w[3:] += t + np.cross(pos-tcp, f)
        return w

    def get_max_normal_force(self, group=None):
        """Largest individual contact normal force, not the sum over a patch."""
        return max((normal for *_, normal in self._contacts(group)), default=0.0)

    def get_contact_normal_sum(self, group=None):
        """Sum contact normal magnitudes, N; count each contact once."""
        return sum(normal for _, normal in self._contacts(group, normals_only=True))

    def get_ext_joint_torque(self, measured=False):
        """Sum J_contact.T f_contact over ALL robot contacts; measured adds cached faults.

        Includes link collisions upstream of wrist; excludes applied perturbation
        forces, damping, gravity and mocap equality reactions on the hand.
        """
        tau = np.zeros(7)
        for body, point, f, t, _ in self._contacts():
            jp, jr = np.zeros((3, self.model.nv)), np.zeros((3, self.model.nv))
            mujoco.mj_jac(self.model, self.data, jp, jr, point, body)
            tau += jp[:, self.dof_indices].T @ f + jr[:, self.dof_indices].T @ t
        if measured:
            tau += np.asarray(self.config.tau_ext_bias) + np.asarray(self.config.tau_ext_noise_std)*self._tau_noise
        return tau

    def set_hand(self, pos, quat=None, enabled=True, mode='kinematic', allow_penetration=False):
        """Move mocap target. On enable, initialize hand at target; then weld follows.

        Kinematic is a stiff dynamic weld (quasi-static worst case); compliant
        is a position spring; release_hand gives free transient impact. Disable parks both free body
        and target and removes collisions. It does not change simulation time.
        """
        if mode not in ('kinematic', 'compliant'):
            raise ValueError("mode must be 'kinematic' or 'compliant'")
        p = np.asarray(pos, dtype=float)
        if p.shape != (3,) or not np.all(np.isfinite(p)):
            raise ValueError('pos must be a finite (3,) array')
        q = _quat(quat if quat is not None else [1, 0, 0, 0])
        if not enabled:
            p = np.asarray(self.config.hand.parked_pos)
            q = _quat(self.config.hand.parked_quat)
        fresh = not self.hand_enabled
        if fresh or not enabled:
            self.data.qpos[self._hand_qadr:self._hand_qadr+7] = np.r_[p, q]
            self.data.qvel[self._hand_dadr:self._hand_dadr+6] = 0
            self.data.xfrc_applied[self.hand_id] = 0
        self.data.mocap_pos[self.hand_mocap_id] = p
        self.data.mocap_quat[self.hand_mocap_id] = q
        self.model.geom_contype[self.hand_geom_id] = 1 if enabled else 0
        self.model.geom_conaffinity[self.hand_geom_id] = 1 if enabled else 0
        # Broad-phase collision filtering uses these cached body aggregates too.
        self.model.body_contype[self.hand_id] = 1 if enabled else 0
        self.model.body_conaffinity[self.hand_id] = 1 if enabled else 0
        sr = self.config.hand.kinematic_solref if mode == 'kinematic' else self.config.hand.compliant_solref
        self.model.eq_solref[self.hand_weld_id] = [max(sr[0], 2*self.config.dt), sr[1]]
        self.data.eq_active[self.hand_weld_id] = True
        self.hand_enabled, self.hand_mode = bool(enabled), mode
        mujoco.mj_forward(self.model, self.data)
        if fresh and enabled and not allow_penetration:
            deepest = max((-float(c.dist) for c in self.data.contact
                           if self.hand_geom_id in (c.geom1, c.geom2)), default=0.)
            if deepest > self.config.hand.placement_max_penetration:
                self.set_hand(self.config.hand.parked_pos, enabled=False)
                raise ValueError(f'Hand placement penetrates {deepest*1000:.3f} mm; approach from outside or explicitly allow_penetration=True')

    def release_hand(self, velocity):
        """Release enabled hand into free flight with world linear velocity, m/s.

        Deactivates the weld and zeros angular velocity. Contact compliance is
        unchanged; default hand gravcomp keeps the stated incident speed fixed.
        """
        v = np.asarray(velocity, dtype=float)
        if v.shape != (3,) or not np.all(np.isfinite(v)):
            raise ValueError('velocity must be finite (3,)')
        if not self.hand_enabled:
            raise ValueError('Enable/place the hand before release_hand')
        self.data.eq_active[self.hand_weld_id] = False
        self.data.qvel[self._hand_dadr:self._hand_dadr+6] = np.r_[v, np.zeros(3)]
        self.hand_mode = 'free'
        mujoco.mj_forward(self.model, self.data)

    def render(self, width=960, height=720, camera=None):
        """Return uint8 RGB (height,width,3). Set MUJOCO_GL=egl before importing."""
        if not all(isinstance(v, (int, np.integer)) and v > 0 for v in (width, height)):
            raise ValueError('width and height must be positive integers')
        if self._renderer is None or self._render_size != (width, height):
            if self._renderer is not None:
                self._renderer.close()
            self.model.vis.global_.offwidth = max(width, self.model.vis.global_.offwidth)
            self.model.vis.global_.offheight = max(height, self.model.vis.global_.offheight)
            self._renderer = mujoco.Renderer(self.model, height=height, width=width)
            self._render_size = (width, height)
        self._renderer.update_scene(self.data, camera=camera or self.config.camera)
        return self._renderer.render().copy()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
            self._renderer = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()


class VideoRecorder:
    """Small streaming imageio recorder; call capture at the chosen video rate."""
    def __init__(self, path, fps=30, width=960, height=720, camera=None):
        import imageio.v2 as imageio
        self.path = Path(path)
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.writer = imageio.get_writer(self.path, fps=fps)
        self.width, self.height, self.camera = width, height, camera

    def capture(self, env):
        self.writer.append_data(env.render(self.width, self.height, self.camera))

    def close(self):
        self.writer.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()
