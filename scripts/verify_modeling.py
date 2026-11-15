#!/usr/bin/env python
"""Independent NumPy model verification and reproducible report artifacts.

Run from the repository root: MUJOCO_GL=egl python scripts/verify_modeling.py
All error statistics are per-state maxima (position uses the Euclidean norm).
"""
import argparse
import json
import os
from pathlib import Path
import sys

if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

if __name__ == '__main__':
    os.environ.setdefault('MUJOCO_GL', 'egl')

import mujoco
import numpy as np
from hrc_sim import HRCSim
from hrc_sim.kinematics import PandaKinematics
from hrc_sim.dynamics import PandaDynamics
from scripts.verify_scenarios import verify_hand_impacts, verify_misalignment, print_scenarios

ROOT = Path(__file__).resolve().parents[1]
SEED = 6223
N_SAMPLES = 1000


def _stat(values, tolerance, unit, reference):
    a = np.asarray(values, dtype=float)
    return dict(max=float(np.max(a)), mean=float(np.mean(a)), n_samples=int(a.size),
                tolerance=float(tolerance), unit=unit, reference=reference,
                passed=bool(np.all(np.isfinite(a)) and np.max(a) < tolerance))


def _set_state(env, q, dq, tau=None):
    env.data.qpos[env.qpos_indices] = q
    env.data.qvel[env.dof_indices] = dq
    env.data.qfrc_applied[:] = 0
    env.data.xfrc_applied[:] = 0
    env.data.ctrl[:] = np.zeros(7) if tau is None else tau
    mujoco.mj_forward(env.model, env.data)


def _unconstrained(env):
    # Joint limits can activate inside their safety margin. Disable them, along
    # with contacts, for isolated smooth equation-of-motion comparisons only.
    env.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_CONTACT) | int(mujoco.mjtDisableBit.mjDSBL_LIMIT)


def orientation_distance(R, reference):
    """SO(3) geodesic angle using atan2, accurate also at roundoff-scale errors."""
    delta = R @ reference.T
    skew = .5*np.array([delta[2, 1]-delta[1, 2], delta[0, 2]-delta[2, 0], delta[1, 0]-delta[0, 1]])
    return float(np.arctan2(np.linalg.norm(skew), np.clip((np.trace(delta)-1)/2, -1, 1)))


def finite_difference_jacobian(kin, q, eps=1e-6):
    J = np.empty((6, 7))
    R = kin.fk_tcp(q)[:3, :3]
    for j in range(7):
        step = np.eye(7)[j]*eps
        Tp, Tm = kin.fk_tcp(q+step), kin.fk_tcp(q-step)
        J[:3, j] = (Tp[:3, 3]-Tm[:3, 3])/(2*eps)
        omega = ((Tp[:3, :3]-Tm[:3, :3])/(2*eps)) @ R.T
        J[3:, j] = .5*np.array([omega[2, 1]-omega[1, 2], omega[0, 2]-omega[2, 0], omega[1, 0]-omega[0, 1]])
    return J


def verify_random_samples(n_samples=N_SAMPLES, seed=SEED):
    """Uniform full-joint-range FK/J/M/g/h and forward/inverse EOM checks."""
    values = {name: [] for name in ('fk_position', 'fk_orientation', 'flange_position', 'flange_orientation',
              'jacobian_mujoco', 'jacobian_finite_difference', 'jacobian_dot', 'mass_relative',
              'mass_symmetry', 'gravity', 'coriolis', 'forward_eom', 'inverse_eom')}
    min_eigenvalue = np.inf
    rng = np.random.default_rng(seed)
    with HRCSim() as env:
        _unconstrained(env)
        kin = PandaKinematics()
        dyn = PandaDynamics.from_mujoco_model(env.model)
        for _ in range(n_samples):
            q = rng.uniform(env.config.q_min, env.config.q_max)
            dq = rng.uniform(-.3, .3, 7)
            tau = rng.uniform(-.25, .25, 7)*env.config.tau_max
            _set_state(env, q, dq, tau)
            assert np.max(abs(env.data.qfrc_constraint[env.dof_indices])) < 1e-12
            T, Tf = kin.fk_tcp(q), kin.fk_flange(q)
            p, R, _ = env.get_ee_pose()
            values['fk_position'].append(np.linalg.norm(T[:3, 3]-p))
            values['fk_orientation'].append(orientation_distance(T[:3, :3], R))
            values['flange_position'].append(np.linalg.norm(Tf[:3, 3]-env.data.site('attachment_site').xpos))
            values['flange_orientation'].append(orientation_distance(Tf[:3, :3], env.data.site('attachment_site').xmat.reshape(3, 3)))
            J, Jdot = kin.jacobian(q), kin.jacobian_dot(q, dq)
            values['jacobian_mujoco'].append(np.max(abs(J-env.jacobian())))
            values['jacobian_finite_difference'].append(np.max(abs(J-finite_difference_jacobian(kin, q))))
            values['jacobian_dot'].append(np.max(abs(Jdot-env.jacobian_dot())))
            M, g, h = dyn.mass_matrix(q), dyn.gravity_torque(q), dyn.coriolis_forces(q, dq)
            Mref = env.mass_matrix()
            values['mass_relative'].append(np.linalg.norm(M-Mref, 'fro')/np.linalg.norm(Mref, 'fro'))
            values['mass_symmetry'].append(np.max(abs(M-M.T)))
            min_eigenvalue = min(min_eigenvalue, np.linalg.eigvalsh(M)[0])
            values['gravity'].append(np.max(abs(g-env.gravity_torque())))
            values['coriolis'].append(np.max(abs(h-(env.bias_forces()-env.gravity_torque()))))
            # env.mass_matrix and dyn.mass_matrix BOTH already include armature.
            qacc = env.data.qacc[env.dof_indices]
            values['forward_eom'].append(np.max(abs(M@qacc+h+g+dyn.damping*dq-env.data.qfrc_actuator[env.dof_indices])))
            ddq = rng.uniform(-2., 2., 7)
            env.data.qacc[:] = 0
            env.data.qacc[env.dof_indices] = ddq
            mujoco.mj_inverse(env.model, env.data)
            predicted = M@ddq+h+g+dyn.damping*dq
            values['inverse_eom'].append(np.max(abs(predicted-env.data.qfrc_inverse[env.dof_indices])))
    specs = dict(
        fk_position=(1e-10, 'm', 'MuJoCo TCP position'), fk_orientation=(1e-10, 'rad', 'MuJoCo TCP orientation'),
        flange_position=(1e-10, 'm', 'MuJoCo attachment position'), flange_orientation=(1e-10, 'rad', 'MuJoCo attachment orientation'),
        jacobian_mujoco=(1e-10, 'mixed SI', 'mj_jacSite'), jacobian_finite_difference=(1e-8, 'mixed SI', 'central FK difference, epsilon=1e-6 rad'),
        jacobian_dot=(1e-10, 'mixed SI/s', 'mj_jacDot'), mass_relative=(1e-10, 'relative Frobenius', 'HRCSim mass matrix including armature'),
        mass_symmetry=(1e-12, 'kg m^2', 'transpose'), gravity=(1e-10, 'N m', 'HRCSim gravity_torque'),
        coriolis=(1e-10, 'N m', 'MuJoCo bias minus gravity'), forward_eom=(1e-10, 'N m', 'mj_forward'), inverse_eom=(1e-10, 'N m', 'mj_inverse'))
    metrics = {name: _stat(values[name], *spec) for name, spec in specs.items()}
    return dict(seed=seed, n_samples=n_samples, velocity_range_rad_s=[-.3, .3],
                inverse_acceleration_range_rad_s2=[-2., 2.], finite_difference_step_rad=1e-6,
                min_mass_eigenvalue=float(min_eigenvalue), positive_definite=bool(min_eigenvalue > 0), metrics=metrics)


def verify_task_space(n_samples=N_SAMPLES, seed=SEED+1):
    """Forward dynamics AND a 1 microsecond integration check at random states."""
    names = ('task_acceleration', 'task_acceleration_fd', 'task_with_damping',
             'null_acceleration', 'null_acceleration_fd', 'null_algebra', 'env_task_terms',
             'regularized_acceleration', 'regularized_identity')
    errors = {name: [] for name in names}
    rng = np.random.default_rng(seed)
    rejected = 0
    dt = 1e-6
    with HRCSim() as env:
        _unconstrained(env)
        env.model.opt.timestep = dt
        dyn = PandaDynamics.from_mujoco_model(env.model)
        kin = dyn.kinematics
        while len(errors['task_acceleration']) < n_samples:
            q = rng.uniform(env.config.q_min, env.config.q_max)
            M, J = dyn.mass_matrix(q), kin.jacobian(q)
            A = J@np.linalg.solve(M, J.T)
            if np.linalg.cond(A) > 1e5:
                rejected += 1
                continue  # Exact inversion is tested only at well-conditioned states.
            dq = rng.uniform(-.1, .1, 7)
            F = rng.uniform(-1., 1., 6)*[1., 1., 1., .2, .2, .2]
            terms = dyn.task_space_dynamics(q, dq, damping=0)
            g, N = dyn.gravity_torque(q), terms['N_tau']
            tau = J.T@F + dyn.damping*dq  # Cancel passive damping, keep h and g.
            assert np.all(abs(tau) < env.config.tau_max)
            _set_state(env, q, dq, tau)
            Jref, Jdref = env.jacobian(), env.jacobian_dot()
            a = Jref@env.data.qacc[env.dof_indices]+Jdref@dq
            predicted = np.linalg.solve(terms['Lambda'], F-terms['mu']-terms['p'])
            errors['task_acceleration'].append(np.max(abs(a-predicted)))
            regularized = dyn.task_space_dynamics(q, dq)
            pred_d = np.linalg.solve(regularized['Lambda'], F-regularized['mu']-regularized['p'])
            errors['regularized_acceleration'].append(np.max(abs(pred_d-a)))
            errors['regularized_identity'].append(np.max(abs(pred_d-a-dyn.task_damping**2*F)))
            envterms = env.task_space_dynamics(damping=0)
            errors['env_task_terms'].append(max(np.linalg.norm(terms[k]-envterms[k])/max(1., np.linalg.norm(envterms[k]))
                                            for k in ('Lambda', 'mu', 'p', 'Jbar')))
            v0 = Jref@dq
            mujoco.mj_step(env.model, env.data)
            mujoco.mj_forward(env.model, env.data)
            a_fd = (env.get_ee_velocity()-v0)/dt
            errors['task_acceleration_fd'].append(np.max(abs(a_fd-predicted)))
            # Retain actual passive damping: mu_D = Lambda J M^-1 D dq.
            _set_state(env, q, dq, J.T@F)
            mud = terms['mu'] + terms['Lambda']@J@np.linalg.solve(M, dyn.damping*dq)
            actual = env.jacobian()@env.data.qacc[env.dof_indices]+env.jacobian_dot()@dq
            expected = np.linalg.solve(terms['Lambda'], F-mud-terms['p'])
            errors['task_with_damping'].append(np.max(abs(actual-expected)))
            # Null motion starts at rest; cancel gravity. A velocity-state test
            # would additionally contain Jdot*dq even under pure null torque.
            tau0 = rng.uniform(-.2, .2, 7)
            taun = g+N@tau0
            assert np.all(abs(taun) < env.config.tau_max)
            _set_state(env, q, np.zeros(7), taun)
            errors['null_algebra'].append(np.max(abs(J@np.linalg.solve(M, N))))
            errors['null_acceleration'].append(np.max(abs(env.jacobian()@env.data.qacc[env.dof_indices])))
            mujoco.mj_step(env.model, env.data)
            mujoco.mj_forward(env.model, env.data)
            errors['null_acceleration_fd'].append(np.max(abs(env.get_ee_velocity()/dt)))
    specs = dict(regularized_acceleration=(1.01e-4, 'm/s^2 or rad/s^2', 'default .01 damping; expected lambda^2 F defect'),
                 regularized_identity=(1e-10, 'mixed SI', 'regularized defect minus lambda^2 F'),
                 task_acceleration=(1e-10, 'm/s^2 or rad/s^2', 'mj_forward + analytic Jdot dq'),
                 task_acceleration_fd=(2e-3, 'm/s^2 or rad/s^2', 'forward-integrated TCP velocity difference, dt=1e-6 s'),
                 task_with_damping=(1e-10, 'm/s^2 or rad/s^2', 'mj_forward with passive damping included in mu'),
                 null_acceleration=(1e-10, 'm/s^2 or rad/s^2', 'mj_forward, rest + gravity compensation'),
                 null_acceleration_fd=(1e-4, 'm/s^2 or rad/s^2', 'forward-integrated TCP velocity difference, dt=1e-6 s'),
                 null_algebra=(1e-10, 'mixed SI', 'J M^-1 N'),
                 env_task_terms=(1e-10, 'relative norm', 'HRCSim Lambda/mu/p/Jbar'))
    return dict(seed=seed, n_samples=n_samples, rejected_ill_conditioned=rejected,
                condition_limit=1e5, integration_dt=dt,
                velocity_range_rad_s=[-.1, .1], null_tau0_range_Nm=[-.2, .2],
                metrics={name: _stat(errors[name], *spec) for name, spec in specs.items()})


def verify_singularity():
    """A legal exact geometric singularity; damping bounds the inverse gain."""
    q = np.array([0., 0., 0., -.467002423653011, 0., .467002423653011, 0.])
    with HRCSim() as env:
        _unconstrained(env)
        dyn = PandaDynamics.from_mujoco_model(env.model)
        J, M = dyn.kinematics.jacobian(q), dyn.mass_matrix(q)
        A = J@np.linalg.solve(M, J.T)
        damping = dyn.task_damping
        terms = dyn.task_space_dynamics(q, damping=damping)
        s = np.linalg.svd(J, compute_uv=False)
        defect = np.linalg.norm(J@np.linalg.solve(M, terms['N_tau']), 2)
        Lnorm = np.linalg.norm(terms['Lambda'], 2)
        finite = all(np.all(np.isfinite(x)) for x in terms.values())
        env.reset(q)
        envterms = env.task_space_dynamics()
    return dict(q=q.tolist(), jacobian_singular_values=s.tolist(), rank=int(np.linalg.matrix_rank(J)),
                damping=damping, smallest_task_inverse_eigenvalue=float(np.linalg.eigvalsh(A)[0]),
                lambda_spectral_norm=float(Lnorm), lambda_bound=float(1/damping**2),
                damped_null_defect=float(defect), all_finite=bool(finite),
                env_all_finite=all(np.all(np.isfinite(x)) for x in envterms.values()),
                passed=bool(finite and np.linalg.matrix_rank(J)<6 and Lnorm <= (1+1e-8)/damping**2))


def _vertical_ik(kin, target, q0):
    """Independent 2D Newton IK for the vertical, y=0 contact experiments."""
    q = np.array(q0, copy=True)
    for _ in range(40):
        p = kin.fk_tcp(q)[:3, 3]
        e = np.asarray(target)[[0, 2]]-p[[0, 2]]
        if np.linalg.norm(e) < 1e-12:
            return q
        J = kin.jacobian(q)[[0, 2]]
        reduced = np.column_stack((J[:, 1]+J[:, 5], J[:, 3]-J[:, 5]))
        change = np.linalg.solve(reduced, e)
        q[[1, 3]] += change
        q[5] = q[1]-q[3]
    raise RuntimeError('Vertical IK did not converge')


def verify_contacts(load=5., duration=3., steady_duration=.5):
    """Load a rim-supported peg and an inserted peg on the cavity bottom.

    Gravity is compensated; a known constant 5 N downward Cartesian load is
    applied by the arm, with lateral/orientation stiffness and velocity damping.
    Contact distance is reported throughout and over the final steady window.
    """
    cases = {}
    for name, offset, z_offset in [('rim', .030, 0.), ('inside_bottom', 0., -.040)]:
        with HRCSim() as env:
            kin = PandaKinematics()
            target = np.asarray(env.config.hole_pos)+[offset, 0, z_offset]
            q = _vertical_ik(kin, target+[0, 0, .002], env.config.q_home)
            env.reset(q)
            Rd = np.diag([-1., 1., -1.])
            normal_forces, vertical_forces, penetrations, speeds = [], [], [], []
            max_transient_penetration = 0.
            for i in range(round(duration/env.config.dt)):
                p, R, _ = env.get_ee_pose()
                v = env.get_ee_velocity()
                er = .5*sum((np.cross(R[:, j], Rd[:, j]) for j in range(3)), start=np.zeros(3))
                wrench = np.r_[1200*(target[:2]-p[:2])-85*v[:2], -load-65*v[2], 80*er-12*v[3:]]
                env.step(env.jacobian().T@wrench)
                penetration = 0.
                for c in env.data.contact:
                    pair = {int(c.geom1), int(c.geom2)}
                    if env.model.geom('peg').id in pair and pair.intersection(env._hole_geom_ids):
                        penetration = max(penetration, -float(c.dist))
                max_transient_penetration = max(max_transient_penetration, penetration)
                if i >= round((duration-steady_duration)/env.config.dt):
                    normal_forces.append(env.get_max_normal_force('peg_hole'))
                    vertical_forces.append(env.get_contact_wrench('peg_hole')[2])
                    penetrations.append(penetration)
                    speeds.append(np.linalg.norm(env.get_ee_velocity()))
            force_error = np.abs(np.asarray(vertical_forces)-load)
            cases[name] = dict(applied_downward_load_N=load, n_steps=round(duration/env.config.dt),
                steady_samples=len(penetrations), duration_s=duration, steady_duration_s=steady_duration,
                max_penetration_m=float(max(penetrations)), mean_penetration_m=float(np.mean(penetrations)),
                max_transient_penetration_m=float(max_transient_penetration), penetration_tolerance_m=1e-4,
                mean_vertical_force_N=float(np.mean(vertical_forces)), max_vertical_force_error_N=float(np.max(force_error)),
                mean_vertical_force_error_N=float(np.mean(force_error)), force_tolerance_N=.05,
                max_individual_normal_force_N=float(max(normal_forces)), max_steady_tcp_speed=float(max(speeds)),
                passed=bool(max_transient_penetration < 1e-4 and max(force_error) < .05))
    return cases


def render_scene(path):
    """1600x1200 light-background rendering, exact projected frame overlays."""
    os.environ.setdefault('MUJOCO_GL', 'egl')
    # Set EGL before importing MuJoCo in main; this function also works when
    # the caller has already selected its own working rendering backend.
    import tempfile
    os.environ.setdefault('MPLCONFIGDIR', str(Path(tempfile.gettempdir())/'hrc-modeling-matplotlib'))
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    import matplotlib.patheffects as pe
    with HRCSim() as env:
        floor = env.model.geom('floor').id
        ground = int(env.model.geom_matid[floor])
        env.model.mat_texid[ground] = -1
        env.model.mat_rgba[ground] = [.97, .98, .99, 1.]
        env.model.mat_emission[ground] = .7  # White backdrop without overexposing the arm.
        env.model.geom_size[floor, :2] = [200, 200]
        env.model.vis.headlight.ambient[:] = [.25, .25, .25]
        env.model.vis.headlight.diffuse[:] = [.55, .55, .55]
        env.model.light_diffuse[:] = .65
        env.model.mat_specular[:] = .25
        env.model.mat_reflectance[:] = 0
        w, h = 1600, 1200
        camera_id = env.model.camera(env.config.camera).id
        eye, target = np.array([1.15, -1.25, 1.15]), np.array([.32, 0., .42])
        back = (eye-target)/np.linalg.norm(eye-target)
        right = np.cross([0., 0., 1.], back); right /= np.linalg.norm(right)
        up = np.cross(back, right)
        env.model.cam_pos[camera_id] = eye
        mujoco.mju_mat2Quat(env.model.cam_quat[camera_id], np.column_stack((right, up, back)).ravel())
        mujoco.mj_forward(env.model, env.data)
        env.render(w, h)
        for flag in (mujoco.mjtRndFlag.mjRND_SHADOW, mujoco.mjtRndFlag.mjRND_REFLECTION):
            env._renderer.scene.flags[flag] = False
        rgb = env._renderer.render().copy()
        cam = env.model.camera(env.config.camera).id
        cp = env.data.cam_xpos[cam]
        Rc = env.data.cam_xmat[cam].reshape(3, 3)
        focal = h/(2*np.tan(np.deg2rad(env.model.cam_fovy[cam])/2))
        def project(p):
            local = Rc.T@(np.asarray(p)-cp)
            return np.array([w/2+focal*local[0]/(-local[2]), h/2-focal*local[1]/(-local[2])])
        dpi = w/3.5  # 3.5-inch single-column figure with readable 7-7.5 pt labels.
        fig = plt.figure(figsize=(3.5, 2.625), dpi=dpi)
        ax = fig.add_axes([0, 0, 1, 1])
        ax.imshow(rgb)
        ax.set(xlim=(0, w), ylim=(h, 0))
        ax.axis('off')
        colors = ['#d53332', '#238347', '#2464ba']
        p, Rt, _ = env.get_ee_pose()
        for label, origin, rotation, length in [('World W', np.zeros(3), np.eye(3), .13), ('TCP T', p, Rt, .085)]:
            start = project(origin)
            for i, (axis_label, color) in enumerate(zip('xyz', colors)):
                end = project(origin+length*rotation[:, i])
                arrow = ax.annotate('', xy=end, xytext=start, arrowprops=dict(arrowstyle='-|>', color=color, lw=1.0, mutation_scale=7))
                arrow.arrow_patch.set_path_effects([pe.Stroke(linewidth=1.8, foreground='white'), pe.Normal()])
                label_shift = [-20, 35] if label.startswith('TCP') and i == 0 else [5, -5]
                ax.text(*(end+np.array(label_shift)), axis_label, color=color, fontsize=7, fontweight='bold',
                        path_effects=[pe.withStroke(linewidth=1.2, foreground='white')])
            offset = [-180, 100] if label.startswith('World') else [90, 95]
            ax.text(*(start+offset), label, fontsize=7.5, fontweight='bold', color='#263341',
                    bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=2.5))
        for label, point, delta in [('peg', env.data.xipos[env.tool_id], [210, -140]),
                                    ('hole fixture', env.data.site('hole_top').xpos, [240, 100])]:
            anchor = project(point)
            ax.annotate(label, xy=anchor, xytext=anchor+delta, color='#263341', fontsize=7.5,
                        arrowprops=dict(arrowstyle='-', color='#536173', lw=.6),
                        bbox=dict(facecolor='white', edgecolor='none', alpha=.9, pad=1.))
        fig.savefig(path, dpi=dpi, facecolor='white')
        plt.close(fig)
        from PIL import Image
        pixels = np.asarray(Image.open(path).convert('RGB'))
        yy, xx = np.where(np.min(pixels, axis=2) < 240)
        x0, x1, y0, y1 = xx.min()-40, xx.max()+40, yy.min()-40, yy.max()+40
        crop_h = max(y1-y0, (x1-x0)*3/4)
        crop_w = crop_h*4/3
        cx, cy = (x0+x1)/2, (y0+y1)/2
        canvas = Image.new('RGB', (w+400, h+400), 'white')
        canvas.paste(Image.fromarray(pixels), (200, 200))
        image = canvas.crop((round(cx-crop_w/2)+200, round(cy-crop_h/2)+200, round(cx+crop_w/2)+200, round(cy+crop_h/2)+200))
        image.resize((1600, 1200), Image.Resampling.LANCZOS).save(path, dpi=(457.142857, 457.142857), compress_level=9)


def summary_rows(report):
    r, t = report['random']['metrics'], report['task']['metrics']
    rows = []
    for label, key in [('FK position (m)', 'fk_position'), ('FK orientation (rad)', 'fk_orientation'),
                       ('Jacobian / MuJoCo', 'jacobian_mujoco'), ('Jacobian / central FD', 'jacobian_finite_difference'),
                       ('Mass (relative)', 'mass_relative'), ('Gravity (N m)', 'gravity'), ('Coriolis (N m)', 'coriolis'),
                       ('Forward EOM (N m)', 'forward_eom'), ('Inverse EOM (N m)', 'inverse_eom')]:
        v = r[key]
        rows.append((label, v['max'], v['mean'], v['tolerance'], v['n_samples'], v['passed']))
    for label, key in [('Task acceleration', 'task_acceleration'), ('Default damped acceleration', 'regularized_acceleration'), ('Task acceleration / step FD', 'task_acceleration_fd'),
                       ('Task with passive damping', 'task_with_damping'), ('Null acceleration', 'null_acceleration'),
                       ('Null acceleration / step FD', 'null_acceleration_fd')]:
        v = t[key]
        rows.append((label, v['max'], v['mean'], v['tolerance'], v['n_samples'], v['passed']))
    for case, v in report['contacts'].items():
        rows.append((f'{case} penetration (m)', v['max_transient_penetration_m'], v['mean_penetration_m'], v['penetration_tolerance_m'], v['n_steps'], v['passed']))
        rows.append((f'{case} steady load error (N)', v['max_vertical_force_error_N'], v['mean_vertical_force_error_N'], v['force_tolerance_N'], v['steady_samples'], v['passed']))
    return rows


def update_notes_tables(report):
    """Keep the requested derivation's numeric tables sourced from this JSON."""
    path = ROOT/'docs/modeling_notes.md'
    if not path.exists():
        return
    lines = ['### Recorded versions and sampling', '',
             '| Quantity | Value |', '| --- | --- |',
             f"| MuJoCo / NumPy | {report['mujoco_version']} / {report['numpy_version']} |",
             f"| Random seed / samples | {report['random']['seed']} / {report['random']['n_samples']} |",
             f"| Task seed / accepted samples | {report['task']['seed']} / {report['task']['n_samples']} |",
             f"| Task rejected ill-conditioned states / condition threshold | {report['task']['rejected_ill_conditioned']} / {report['task']['condition_limit']:.9e} |",
             f"| Task integration step, s | {report['task']['integration_dt']:.9e} |",
             f"| Minimum total-mass eigenvalue, kg m^2 | {report['random']['min_mass_eigenvalue']:.9e} |",
             f"| Mass positive definite / all checks passed | {report['random']['positive_definite']} / {report['passed']} |", '']
    for title, section in [('Random-state model errors', 'random'), ('Operational-space errors', 'task')]:
        lines += [f'### {title}', '', '| Metric | Max | Mean | N | Tolerance | Unit |', '| --- | ---: | ---: | ---: | ---: | --- |']
        for key, value in report[section]['metrics'].items():
            lines.append(f"| `{key}` | {value['max']:.9e} | {value['mean']:.9e} | {value['n_samples']} | {value['tolerance']:.9e} | {value['unit']} |")
        lines.append('')
    lines += ['### Contact diagnostics', '', '| Quantity | Rim | Inside bottom |', '| --- | ---: | ---: |']
    for key, value in report['contacts']['rim'].items():
        def fmt(v):
            return f'{v:.9e}' if isinstance(v, float) else str(v)
        lines.append(f"| `{key}` | {fmt(value)} | {fmt(report['contacts']['inside_bottom'][key])} |")
    lines += ['', '### Singularity diagnostics', '', '| Quantity | Value |', '| --- | --- |']
    for key, value in report['singularity'].items():
        if isinstance(value, list):
            shown = ', '.join(f'{v:.9e}' for v in value)
        elif isinstance(value, float):
            shown = f'{value:.9e}'
        else:
            shown = str(value)
        lines.append(f'| `{key}` | {shown} |')
    lines += ['', '### Free-hand calibration', '',
              '| Speed (m/s) | Peak (N) | Elastic target (N) | Relative error | Tolerance |',
              '| ---: | ---: | ---: | ---: | ---: |']
    for row in report['hand_impacts']['rows']:
        lines.append(f"| {row['speed_m_s']:.2f} | {row['peak_N']:.9e} | {row['reference_N']:.9e} | {row['relative_error']:.9e} | {row['tolerance']:.2f} |")
    lines += ['', '### Misalignment capture', '', '| Case | Depth (mm) | Peak resultant (N) | Outcome |', '| --- | ---: | ---: | --- |']
    for row in report['misalignment']['rows']:
        lines.append(f"| {row['case']} | {row['depth']*1000:.6f} | {row['peak_force']:.6f} | {row['outcome']} |")
    lines.append('')
    start, end = '<!-- BEGIN GENERATED VERIFICATION -->', '<!-- END GENERATED VERIFICATION -->'
    original = path.read_text()
    before, rest = original.split(start, 1)
    _, after = rest.split(end, 1)
    path.write_text(before+start+'\n\n'+'\n'.join(lines)+'\n'+end+after)


def write_artifacts(report, out, scene=False):
    out.mkdir(parents=True, exist_ok=True)
    (out/'modeling_verification.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    r = report['random']['metrics']
    def sci(value):
        mantissa, exponent = f'{value:.2e}'.split('e')
        return f'${mantissa}\\times10^{{{int(exponent)}}}$'
    latex_rows = [
        ('TCP pos. [m]', 'MuJoCo', sci(r['fk_position']['max'])),
        ('TCP rot. [rad]', 'MuJoCo', sci(r['fk_orientation']['max'])),
        ('$J$ [m/rad; 1]', 'MuJoCo / FD', sci(r['jacobian_mujoco']['max'])+' / '+sci(r['jacobian_finite_difference']['max'])),
        ('$\\mathcal{M}$ [1]', 'MuJoCo (rel.)', sci(r['mass_relative']['max'])),
        ('$g$ [N m]', 'MuJoCo', sci(r['gravity']['max'])),
        ('EOM [N m]', 'Fwd. / inv.', sci(r['forward_eom']['max'])+' / '+sci(r['inverse_eom']['max'])),
    ]
    tex = ['% Generated: six data rows; scoped footnotesize for 252 pt IEEE columns.',
           r'{\footnotesize', r'\setlength{\tabcolsep}{2pt}', r'\begin{tabular}{@{}lll@{}}',
           r'\hline', r'Quantity & Reference & Max error \\', r'\hline']
    tex += [' & '.join(row)+r' \\' for row in latex_rows]
    tex += [r'\hline', r'\end{tabular}', '}', '']
    (out/'modeling_verification_table.tex').write_text('\n'.join(tex))
    update_notes_tables(report)
    if scene:
        render_scene(out/'scene.png')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', type=int, default=N_SAMPLES)
    parser.add_argument('--seed', type=int, default=SEED)
    parser.add_argument('--render-figure', action='store_true',
                        help='Re-render report/figures/scene.png (committed artefact; rendering is not byte-reproducible)')
    args = parser.parse_args()
    if args.samples < 1000:
        parser.error('Verification requires at least 1000 samples')
    report = dict(mujoco_version=mujoco.__version__, numpy_version=np.__version__,
                  random=verify_random_samples(args.samples, args.seed),
                  task=verify_task_space(args.samples, args.seed+1),
                  singularity=verify_singularity(), contacts=verify_contacts(),
                  hand_impacts=verify_hand_impacts(), misalignment=verify_misalignment())
    rows = summary_rows(report)
    report['passed'] = bool(all(v['passed'] for section in ('random', 'task') for v in report[section]['metrics'].values())
        and report['random']['positive_definite'] and report['singularity']['passed']
        and report['singularity']['env_all_finite'] and all(v['passed'] for v in report['contacts'].values())
        and report['hand_impacts']['passed'] and report['misalignment']['passed'])
    print(f"Verification: seed={args.seed}, N={args.samples} random + {args.samples} task states")
    print(f"{'Quantity':37s} {'Max error':>12s} {'Mean error':>12s} {'Tolerance':>12s} {'N':>6s} {'Pass':>5s}")
    for label, maximum, mean, tolerance, n, passed in rows:
        print(f'{label:37s} {maximum:12.3e} {mean:12.3e} {tolerance:12.3e} {n:6d} {str(passed):>5s}')
    s = report['singularity']
    print(f"Minimum mass eigenvalue: {report['random']['min_mass_eigenvalue']:.6e} kg m^2")
    print(f"Singularity: rank={s['rank']}, sigma_min={s['jacobian_singular_values'][-1]:.3e}, "
          f"damping={s['damping']:.3e}, ||Lambda_d||={s['lambda_spectral_norm']:.3e}, "
          f"null defect={s['damped_null_defect']:.3e}, passed={s['passed']}")
    print_scenarios(report['hand_impacts'], report['misalignment'])
    out = ROOT/'report/figures'
    write_artifacts(report, out, scene=args.render_figure)
    print(f'Artifacts: {out}')
    print(f"Overall passed: {report['passed']}")
    return 0 if report['passed'] else 1


if __name__ == '__main__':
    # This precedes lazy MuJoCo GL imports when the standalone script runs.
    os.environ.setdefault('MUJOCO_GL', 'egl')
    raise SystemExit(main())
