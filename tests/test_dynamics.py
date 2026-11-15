"""Independent textbook body-inertia reconstruction and physical verification."""
import ast
import inspect
import hrc_sim.kinematics as kin_module
import hrc_sim.dynamics as dyn_module
import numpy as np
import mujoco
import pytest
from numpy.testing import assert_allclose
from hrc_sim import HRCSim
from hrc_sim.dynamics import PandaDynamics


def test_mass_matrix_relative_accuracy_symmetry_and_spd(modeling_random):
    assert modeling_random['metrics']['mass_relative']['max'] < 1e-10
    assert modeling_random['metrics']['mass_symmetry']['max'] < 1e-12
    assert modeling_random['min_mass_eigenvalue'] > 0


@pytest.mark.parametrize('name,tolerance', [('gravity', 1e-10), ('coriolis', 1e-10), ('forward_eom', 1e-10), ('inverse_eom', 1e-10)])
def test_joint_dynamics_over_1000_states(modeling_random, name, tolerance):
    metric = modeling_random['metrics'][name]
    assert metric['max'] < tolerance, metric


@pytest.mark.parametrize('name,tolerance', [('task_acceleration', 1e-10), ('task_acceleration_fd', 2e-3),
    ('task_with_damping', 1e-10), ('null_acceleration', 1e-10), ('null_acceleration_fd', 1e-4),
    ('null_algebra', 1e-10), ('env_task_terms', 1e-10),
    ('regularized_acceleration', 1.01e-4), ('regularized_identity', 1e-10)])
def test_operational_space_forward_simulation(modeling_task, name, tolerance):
    metric = modeling_task['metrics'][name]
    assert metric['max'] < tolerance, metric


def test_damped_inverse_at_actual_legal_singularity(modeling_singularity):
    s = modeling_singularity
    assert s['rank'] == 5
    assert s['jacobian_singular_values'][-1] < 1e-12
    assert s['all_finite'] and s['env_all_finite']
    assert s['lambda_spectral_norm'] <= (1+1e-10)*s['lambda_bound']
    # A damped null projector has a measurable, nonzero task leakage.
    assert 0 < s['damped_null_defect'] < 1e-3


@pytest.mark.parametrize('case', ['rim', 'inside_bottom'])
def test_loaded_contact_penetration_and_force_balance(modeling_contacts, case):
    c = modeling_contacts[case]
    assert c['max_transient_penetration_m'] < 1e-4, c
    assert c['max_penetration_m'] < 1e-4, c
    assert c['max_vertical_force_error_N'] < .05, c  # 1% of the known 5 N load.
    assert c['max_steady_tcp_speed'] < 1e-4, c


def test_armature_tool_and_passive_damping_conventions():
    with HRCSim() as env:
        dyn = PandaDynamics.from_mujoco_model(env.model)
        q = env.config.q_home
        dq = np.array([.1, -.2, .3, -.1, .1, .2, -.1])
        assert {b.name for b in dyn.bodies} == {*(f'link{i}' for i in range(1, 8)), 'tool'}
        assert_allclose(dyn.mass_matrix(q)-dyn.rigid_mass_matrix(q), np.diag(np.full(7, .1)), atol=1e-15)
        assert_allclose(dyn.damping, np.ones(7))
        terms = dyn.task_space_dynamics(q, dq, damping=0)
        damped = dyn.task_space_dynamics(q, dq, damping=0, include_joint_damping=True)
        J, M = dyn.kinematics.jacobian(q), dyn.mass_matrix(q)
        assert_allclose(damped['mu']-terms['mu'], terms['Lambda']@J@np.linalg.solve(M, dq), atol=1e-12)
        assert_allclose(terms['N_tau'], (np.eye(7)-terms['Jbar']@J).T, atol=1e-12)
        assert_allclose(terms['N_tau']@terms['N_tau'], terms['N_tau'], atol=1e-12)


def test_source_independence_and_copied_constants(monkeypatch):
    for module in (kin_module, dyn_module):
        tree = ast.parse(inspect.getsource(module))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                assert all(not alias.name.startswith('mujoco') for alias in node.names)
            if isinstance(node, ast.ImportFrom):
                assert not (node.module or '').startswith('mujoco')
            if isinstance(node, ast.Call):
                name = getattr(node.func, 'id', getattr(node.func, 'attr', ''))
                assert not name.startswith('mj_')
    with HRCSim() as env:
        dyn = PandaDynamics.from_mujoco_model(env.model)
        q, dq = env.config.q_home, np.arange(7)*.02
        refM, refg, refh = dyn.mass_matrix(q), dyn.gravity_torque(q), dyn.coriolis_forces(q, dq)
        def forbidden(*args, **kwargs):
            raise AssertionError('Independent NumPy model called MuJoCo')
        for name in ('mj_jac', 'mj_jacSite', 'mj_jacDot', 'mj_fullM', 'mj_rne', 'mj_forward'):
            monkeypatch.setattr(mujoco, name, forbidden)
        # Even changes to the extraction source cannot affect copied constants.
        env.model.body_mass[:] = 0
        assert_allclose(dyn.mass_matrix(q), refM)
        assert_allclose(dyn.gravity_torque(q), refg)
        assert_allclose(dyn.coriolis_forces(q, dq), refh)
        assert all(np.all(np.isfinite(v)) for v in dyn.task_space_dynamics(q, dq, damping=0).values())
