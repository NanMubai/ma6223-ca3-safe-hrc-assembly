"""Interface and physical-convention regressions for the seven-axis arm API."""
import xml.etree.ElementTree as ET
from pathlib import Path

import mujoco
import numpy as np
import pytest
from numpy.testing import assert_allclose

from hrc_sim import HRCSim, SimConfig
from scripts.demo_insertion import run_insertion


@pytest.fixture
def env():
    with HRCSim() as sim:
        yield sim


def test_model_loads_and_torque_actuators(env):
    # Approved deviation: independent dynamic hand needs 7 qpos / 6 extra dofs.
    assert (env.model.nq, env.model.nv, env.model.nu) == (14, 13, 7)
    assert len(env.joint_ids) == 7
    assert len(set(env.qpos_indices)) == len(set(env.dof_indices)) == 7
    assert_allclose(env.model.actuator_ctrlrange, np.c_[-env.config.tau_max, env.config.tau_max])
    assert_allclose(env.model.actuator_gainprm[:, 0], 1)
    assert_allclose(env.model.actuator_biasprm, 0)
    assert_allclose(env.model.actuator_gear[:, 0], 1)
    assert np.all(env.model.actuator_biastype == mujoco.mjtBias.mjBIAS_NONE)
    assert_allclose(env.model.dof_armature[env.dof_indices], .1)
    assert_allclose(env.model.dof_damping[env.dof_indices], 1)
    xml = ET.parse(Path(__file__).resolve().parents[1]/'assets/panda_torque.xml')
    assert len(xml.findall('./actuator/motor')) == 7
    assert_allclose(env.get_ee_pose()[0], np.asarray(env.config.hole_pos)+[0, 0, .15], atol=1e-8)
    assert_allclose(env.get_ee_pose()[1][:, 2], [0, 0, -1], atol=1e-7)
    assert env.model.opt.integrator == mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    assert env.model.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC


def test_zero_torque_gravity_compensation_stays_still_two_seconds(env):
    p, _, _ = env.get_ee_pose()
    q, _ = env.get_joint_state()
    env.step(np.zeros(7), n_substeps=2000)
    assert np.linalg.norm(env.get_ee_pose()[0]-p) < .001
    assert_allclose(env.get_joint_state()[0], q, atol=1e-9)
    assert env.data.time == pytest.approx(2)
    assert_allclose(env.data.body('human_hand').xpos, env.config.hand.parked_pos, atol=1e-8)


def test_gravity_compensation_can_be_disabled():
    with HRCSim(SimConfig(gravity_compensation=False)) as sim:
        p = sim.get_ee_pose()[0]
        sim.step(np.zeros(7), n_substeps=100)
        assert np.linalg.norm(sim.get_ee_pose()[0]-p) > .001
        assert_allclose(sim.data.ctrl, 0)


@pytest.mark.parametrize('sign', [-1, 1])
def test_torque_saturation(env, sign):
    env.step(sign*np.full(7, 1e5))
    assert_allclose(env.data.ctrl, sign*env.config.tau_max)
    assert_allclose(env.data.actuator_force, sign*env.config.tau_max)


def test_gravity_is_added_before_saturation(env):
    g = env.gravity_torque()
    tau = np.array([10, -80, 3, 80, 4, 11, 10.])
    env.step(tau)
    assert_allclose(env.data.ctrl, np.clip(tau+g, -env.config.tau_max, env.config.tau_max))


def test_ft_free_and_known_applied_force_sign_and_tcp_moment(env):
    assert_allclose(env.get_ext_wrench(), 0, atol=1e-10)
    force = np.array([5., -3., 2.])
    torque = np.array([.3, -.4, .2])
    env.data.xfrc_applied[env.tool_id] = np.r_[force, torque]
    mujoco.mj_forward(env.model, env.data)
    expected = np.r_[force, torque+np.cross(env.data.xipos[env.tool_id]-env.get_ee_pose()[0], force)]
    assert_allclose(env.get_ext_wrench(), expected, atol=1e-9)
    R = env.get_ee_pose()[1]
    assert_allclose(env.get_ext_wrench('tcp'), np.r_[R.T@expected[:3], R.T@expected[3:]], atol=1e-9)
    # Inertial compensation also works away from rest, not just at a static pose.
    env.data.qvel[env.dof_indices] = [.1, -.2, .3, -.1, .05, .15, -.2]
    mujoco.mj_forward(env.model, env.data)
    expected = np.r_[force, torque+np.cross(env.data.xipos[env.tool_id]-env.get_ee_pose()[0], force)]
    assert_allclose(env.get_ext_wrench(), expected, atol=1e-9)
    env.data.xfrc_applied[:] = 0
    mujoco.mj_forward(env.model, env.data)
    assert_allclose(env.get_ext_wrench(), 0, atol=1e-9)


def approach_hand_to_link(env, mode='kinematic'):
    """Find an exterior surface, enable 1 mm clear, then approach at 50 mm/s."""
    center = env.data.body('link3').xipos.copy()
    geoms = [g for g in range(env.model.ngeom) if int(env.model.geom_bodyid[g]) in env.robot_body_ids
             and env.model.geom_contype[g]]
    def distance(radius):
        env.data.qpos[env._hand_qadr:env._hand_qadr+3] = center+[0, -radius, 0]
        mujoco.mj_forward(env.model, env.data)
        return min(mujoco.mj_geomDistance(env.model, env.data, env.hand_geom_id, g, 1., None) for g in geoms)
    lo, hi = 0., .5
    for _ in range(35):
        mid = (lo+hi)/2
        if distance(mid) > 0:
            hi = mid
        else:
            lo = mid
    p = center+[0, -(hi+.001), 0]
    env.reset()
    env.set_hand(p, mode=mode)
    assert env.get_max_normal_force('robot_hand') == 0
    for _ in range(80):
        p += [0, .00005, 0]
        env.set_hand(p, mode=mode)
        env.step(np.zeros(7))
    return p


def test_link_hand_contact_visible_in_joint_torque_but_not_wrist(env):
    approach_hand_to_link(env, mode='compliant')
    assert env.get_max_normal_force('robot_hand') > .1
    assert np.linalg.norm(env.get_ext_joint_torque()) > .01
    assert_allclose(env.get_contact_wrench('peg_hole'), 0, atol=1e-12)
    assert_allclose(env.get_ext_wrench(), 0, atol=1e-10)
    assert_allclose(env.get_ext_joint_torque(), env.data.qfrc_constraint[env.dof_indices], atol=1e-10)


def test_hand_enable_disable_and_modes(env):
    p = approach_hand_to_link(env)
    assert env.hand_enabled and env.model.geom_contype[env.hand_geom_id] == 1
    assert_allclose(env.model.eq_solref[env.hand_weld_id], env.config.hand.kinematic_solref)
    env.set_hand(p, mode='compliant')
    assert_allclose(env.model.eq_solref[env.hand_weld_id], env.config.hand.compliant_solref)
    env.release_hand([0, .1, 0])
    assert not env.data.eq_active[env.hand_weld_id] and env.hand_mode == 'free'
    assert_allclose(env.data.qvel[env._hand_dadr:env._hand_dadr+3], [0, .1, 0])
    env.set_hand(p, enabled=False)
    assert not env.hand_enabled and env.data.eq_active[env.hand_weld_id]
    assert env.model.geom_contype[env.hand_geom_id] == env.model.body_contype[env.hand_id] == 0
    assert_allclose(env.data.body('human_hand').xpos, env.config.hand.parked_pos)
    assert_allclose(env.get_ext_joint_torque(), 0)
    env.reset()
    assert env.hand_mode == 'kinematic'


def test_penetrating_hand_placement_refused_and_rolled_back(env):
    p = env.data.body('link3').xipos.copy()+[.04, 0, 0]
    with pytest.raises(ValueError, match='penetrates'):
        env.set_hand(p)
    assert not env.hand_enabled
    assert_allclose(env.data.body('human_hand').xpos, env.config.hand.parked_pos)
    env.set_hand(p, allow_penetration=True)  # Explicit opt-in, no integration.
    assert env.hand_enabled


def test_observation_keys_shapes_and_copies(env):
    obs = env.reset(seed=5)
    shapes = dict(q=(7,), dq=(7,), ee_pos=(3,), ee_quat=(4,), ee_R=(3, 3), ee_vel=(6,), ft_wrench=(6,), contact_wrench=(6,), tau_ext=(7,))
    assert set(obs) == {'t', *shapes}
    assert isinstance(obs['t'], float)
    for key, shape in shapes.items():
        assert obs[key].shape == shape
        assert np.all(np.isfinite(obs[key]))
    obs['q'][:] = 100
    assert np.max(abs(env.get_joint_state()[0])) < 4
    assert_allclose(obs['ee_R']@obs['ee_R'].T, np.eye(3), atol=1e-12)
    assert np.linalg.norm(obs['ee_quat']) == pytest.approx(1)


def test_reset_determinism_including_noise():
    cfg = SimConfig(noise_std=np.full(6, .02), sensor_bias=np.arange(6)*.1, sensor_bias_frame='world')
    with HRCSim(cfg) as sim:
        a0 = sim.reset(seed=42)
        a1 = sim.step(np.arange(7)*.1, n_substeps=10)
        sim.set_hand([.4, .2, .4], mode='compliant')
        b0 = sim.reset(seed=42)
        b1 = sim.step(np.arange(7)*.1, n_substeps=10)
        for a, b in [(a0, b0), (a1, b1)]:
            for key in a:
                assert_allclose(a[key], b[key], atol=1e-12)
        sim.reset(seed=1)
        sim.config.noise_std[:] = 0
        assert_allclose(sim.get_ext_wrench(), cfg.sensor_bias, atol=1e-10)


def test_model_terms_and_jacobian_finite_difference(env):
    M = env.mass_matrix()
    assert M.shape == (7, 7)
    assert_allclose(M, M.T, atol=1e-12)
    assert np.min(np.linalg.eigvalsh(M)) > 0
    fullM = np.empty((env.model.nv, env.model.nv))
    mujoco.mj_fullM(env.model, env.data, fullM)
    assert_allclose(M, fullM[np.ix_(env.dof_indices, env.dof_indices)])
    hand_dofs = np.arange(env._hand_dadr, env._hand_dadr+6)
    assert_allclose(fullM[np.ix_(env.dof_indices, hand_dofs)], 0)
    assert_allclose(env.bias_forces(), env.gravity_torque(), atol=1e-12)
    J = env.jacobian()
    p = env.get_ee_pose()[0]
    for j, adr in enumerate(env.qpos_indices):
        env.data.qpos[adr] += 1e-7
        mujoco.mj_forward(env.model, env.data)
        assert_allclose((env.get_ee_pose()[0]-p)/1e-7, J[:3, j], atol=1e-7)
        env.data.qpos[adr] -= 1e-7
    mujoco.mj_forward(env.model, env.data)
    terms = env.task_space_dynamics(damping=0)
    assert_allclose(J@terms['Jbar'], np.eye(6), atol=1e-9)
    assert_allclose(J@terms['N_vel'], 0, atol=1e-10)
    assert_allclose(J@np.linalg.solve(M, terms['N_tau']), 0, atol=1e-10)
    assert 'N' not in terms
    assert_allclose(terms['Jdot_dq'], env.jacobian_dot()@env.get_joint_state()[1], atol=1e-12)
    assert_allclose(terms['Lambda']@J@np.linalg.solve(M, env.gravity_torque()), terms['p'], atol=1e-9)
    assert_allclose(terms['mu'], 0, atol=1e-10)


def test_task_dynamics_finite_at_singularity(env):
    # Exact geometric singularity (rank 5), within the Panda joint ranges.
    q = np.array([0, 0, 0, -.467002423653011, 0, .467002423653011, 0])
    env.reset(q)
    for value in env.task_space_dynamics().values():
        assert np.all(np.isfinite(value))


def test_custom_geometry_config():
    c = SimConfig(peg_half_sizes=(.01, .01, .04), peg_mass=.3, clearance=.001,
                  hole_depth=.05, hole_pos=(.52, .02, .29))
    with HRCSim(c) as sim:
        assert_allclose(sim.model.geom('peg').size, c.peg_half_sizes)
        assert sim.model.body_mass[sim.tool_id] == pytest.approx(.3)
        assert_allclose(sim.data.site('hole_top').xpos, c.hole_pos)
        assert_allclose(sim.data.site('hole_bottom').xpos, np.asarray(c.hole_pos)-[0, 0, c.hole_depth])
        wall = sim.model.geom('hole_xplus')
        assert wall.pos[0]-wall.size[0] == pytest.approx(c.hole_half_sizes[0])


def test_reference_controller_insertion_succeeds(env):
    result = run_insertion(env)
    assert result['success'], result
    assert .030 <= result['depth'] <= env.config.hole_depth+.001
    assert result['lateral_error'] < env.config.clearance
    assert 20 < result['peak_force'] < 22
    # At the final bottom contact the tool sensor agrees with ground truth.
    assert_allclose(env.get_ext_wrench(), env.get_contact_wrench('peg_hole'), atol=1e-10)


def test_invalid_inputs(env):
    with pytest.raises(ValueError):
        env.reset(seed=-1)
    with pytest.raises(ValueError):
        env.reset(seed=1.5)
    with pytest.raises(ValueError):
        env.step([0]*6)
    with pytest.raises(ValueError):
        env.step(np.full(7, np.nan))
    with pytest.raises(ValueError):
        env.step(np.zeros(7), n_substeps=0)
    with pytest.raises(ValueError):
        env.get_ext_wrench('flange')
    with pytest.raises(ValueError):
        env.get_contact_wrench('unknown')
    with pytest.raises(ValueError):
        env.set_hand([0, 0, 0], quat=[0, 0, 0, 0])
    with pytest.raises(ValueError):
        SimConfig(dt=.003)  # Fixed 4 ms contact constant is now too short.


def test_damping_compensation_and_limit_status():
    with HRCSim(SimConfig(damping_compensation=True)) as sim:
        dq = np.linspace(-.2, .2, 7)
        sim.data.qvel[sim.dof_indices] = dq
        mujoco.mj_forward(sim.model, sim.data)
        expected = sim.gravity_torque()+sim.joint_damping*dq
        sim.step(np.zeros(7))
        assert_allclose(sim.last_tau_applied, expected, atol=1e-12)
        assert_allclose(sim.joint_damping, 1)
        assert not any(np.any(v) for v in sim.limits_violated().values())
        sim.step(np.full(7, 1e4))
        assert np.all(sim.limits_violated()['torque'])
        sim.data.qvel[sim.dof_indices[0]] = sim.config.dq_max[0]+.1
        sim.data.qpos[sim.qpos_indices[1]] = sim.config.q_max[1]+.001
        assert sim.limits_violated()['velocity'][0]
        assert sim.limits_violated()['position'][1]


def test_cached_sensor_noise_seed_frames_and_joint_faults():
    cfg = SimConfig(noise_std=.02, tau_ext_noise_std=.01, tau_ext_bias=np.arange(7)*.1)
    with HRCSim(cfg) as sim:
        first = sim.reset()
        assert_allclose(sim.get_ext_wrench(), first['ft_wrench'])
        assert_allclose(sim.get_ext_joint_torque(measured=True), first['tau_ext'])
        assert_allclose(sim.get_ext_joint_torque(), 0)
        a = sim.step(np.zeros(7), n_substeps=5)
        sim.reset()
        for _ in range(4):
            sim.step(np.zeros(7))
            sim.get_ext_wrench(); sim.get_ext_wrench()
            sim.get_ext_joint_torque(measured=True)
        b = sim.step(np.zeros(7))
        assert_allclose(a['ft_wrench'], b['ft_wrench'], atol=1e-12)
        assert_allclose(a['tau_ext'], b['tau_ext'], atol=1e-12)
        assert_allclose(first['ft_wrench'], sim.reset()['ft_wrench'])
        sim.config.noise_std[:] = 0
        sim.config.sensor_bias[:] = [1, 2, 3, .1, .2, .3]
        Rf = sim.data.site('ft_site').xmat.reshape(3, 3)
        f = Rf@sim.config.sensor_bias[:3]
        expected = np.r_[f, Rf@sim.config.sensor_bias[3:]+np.cross(sim.data.site('ft_site').xpos-sim.data.site('tcp').xpos, f)]
        assert_allclose(sim.get_ext_wrench(), expected, atol=1e-12)
        sim.config.sensor_bias_frame = 'world'
        assert_allclose(sim.get_ext_wrench(), sim.config.sensor_bias, atol=1e-12)


def test_accelerated_free_motion_wrench_with_real_steps(env):
    peak_acc = 0.
    for i in range(200):
        env.step(2*np.sin(i*.03+np.arange(7)))
        peak_acc = max(peak_acc, np.max(abs(env.data.qacc[env.dof_indices])))
        assert_allclose(env.get_contact_wrench(), 0, atol=1e-12)
        assert_allclose(env.get_ext_wrench(), 0, atol=1e-10)
    assert peak_acc > 1.


def test_continuous_damping_and_passive_mu(env):
    env.data.qvel[env.dof_indices] = np.linspace(-.1, .1, 7)
    mujoco.mj_forward(env.model, env.data)
    exact = env.task_space_dynamics(damping=0)
    regularized = env.task_space_dynamics()
    full_mu = env.task_space_dynamics(damping=0, include_joint_damping=True)
    J, M = env.jacobian(), env.mass_matrix()
    assert_allclose(np.linalg.inv(regularized['Lambda'])-np.linalg.inv(exact['Lambda']), .01**2*np.eye(6), atol=1e-12)
    assert_allclose(full_mu['mu']-exact['mu'], exact['Lambda']@J@np.linalg.solve(M, env.joint_damping*env.get_joint_state()[1]), atol=1e-12)
    assert_allclose(exact['N_tau'], exact['N_vel'].T, atol=1e-12)
    assert 'N' not in exact
