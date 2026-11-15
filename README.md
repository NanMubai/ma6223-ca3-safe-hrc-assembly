# Safe human–robot collaborative box-in-box assembly

Franka Panda simulation and modeling for learning from demonstration and variable impedance control. The robot inserts a 30×30×60 mm square peg into a 40 mm deep square cavity. The throat has **0.5 mm clearance per side**; a configurable **3 mm, 45° chamfer** provides a lead-in. The arm has seven torque motors. This README is the interface contract for parts A, B, D and E.

## Install, test and run

From the repository root:

```bash
conda env create -f environment.yml
conda activate hrc
pip install -e .
pytest -q
MUJOCO_GL=egl python scripts/demo_insertion.py --offscreen
MUJOCO_GL=egl python scripts/demo_insertion.py --offscreen --video  # optional
MUJOCO_GL=egl python scripts/verify_modeling.py
python scripts/verify_scenarios.py  # hand calibration and misalignment table only
```

The `hrc` environment pins MuJoCo 4.0.1. Set `MUJOCO_GL=egl` before importing MuJoCo for rendering without a display. Tests require no renderer. The demo runs **nominal and 1.5 mm lateral-offset** insertions, prints depth and raw peak forces for each, and saves `outputs/demo_insertion.png` and `outputs/demo_insertion_offset.png`; `--video` also saves `outputs/demo_insertion.mp4`. Exit status is nonzero if either case fails to reach 30 mm depth. `inserted` means ≥39 mm, `partial` means 3–39 mm and `jammed` means <3 mm; the separate `success` flag follows the course requirement of depth ≥30 mm.

## Repository layout

| Path | Purpose |
| --- | --- |
| `assets/franka_emika_panda/` | Unmodified Menagerie Panda at `370c439`, with upstream license |
| `assets/panda_torque.xml` | Original kinematics/inertials, seven torque motors, instrumented tool/peg |
| `assets/scene_box_in_box.xml` | Table, chamfered fixture, hole sites, dynamic hand, mocap weld, home keyframe |
| `hrc_sim/config.py` | `SimConfig` (`HRCConfig` alias), `HandConfig`, `ReferenceControllerConfig` |
| `hrc_sim/env.py` | Simulation, model terms, sensors, contact truth, hand control, rendering, video |
| `hrc_sim/kinematics.py` | Independent NumPy Craig DH, flange/TCP FK, analytic J and Jdot |
| `hrc_sim/dynamics.py` | Independent NumPy per-body inertia/gravity/Coriolis reconstruction and task terms |
| `hrc_sim/reference_controller.py` | Constant-gain Cartesian impedance example; part E supplies variable impedance |
| `scripts/demo_insertion.py` | Nominal and offset insertion; reusable `run_insertion` |
| `scripts/verify_modeling.py`, `scripts/verify_scenarios.py` | Random model verification, impact calibration, capture-range checks, report artifacts |
| `tests/` | Environment, kinematics, dynamics and review regression tests; 1000-state fixed-seed checks |
| `docs/modeling_notes.md` | Full derivation, parameters, automatically synchronized verification tables and limitations |
| `report/figures/` | Generated verification JSON/LaTeX table and labelled scene figure |
| `outputs/` | Generated demo PNG/video files, ignored by git |

The scene XML loads directly. `HRCSim` expands the include and applies geometry/contact configuration **before compilation**, preserving consistent inertias when dimensions/masses change. Asset paths are relative to this editable source tree; retain `assets/` beside `hrc_sim/`.

## Frames, units and conventions

World origin is the robot base; **z is up**. Use SI units: m, rad, kg, s, N and N·m. Joint arrays are always joint1…joint7. The flange is the `attachment` frame at `ft_site`. Peg length extends along flange **+z**, pointing down at home. The **TCP** (`tcp`) is the centre of the free end face, 60 mm from the flange by default. `hole_top` is the entrance centre at `(0.5,0,0.28)` m; `hole_bottom` is 40 mm below it. Home TCP is 150 mm above the entrance. Depth is `(hole_top-tcp) dot hole_axis`, positive into the opening.

Quaternions are **(w,x,y,z)**. `ee_R` maps TCP vectors into world. Twists/Jacobians use **[linear; angular]**, in world axes at the TCP: `J` is 6×7 and `ee_vel=J@dq`. Wrenches use **[force;torque]** and mean **environment exerting load on the robot**, referenced at TCP. `frame='tcp'` rotates the axes while keeping that reference point.

The full model has **nq=14, nv=13, nu=7**: seven arm hinges plus an independently rooted dynamic hand free joint. This approved change makes physical compliant/free impacts possible. All public arm quantities exclude the hand; indices come from named joint position/velocity addresses. `mass_matrix()` is the exact arm block including the tool and 0.1 kg·m² armature per joint; the hand is uncoupled in the smooth mass matrix and interacts with the arm through contact constraints.

Panda/FER limits follow the official [joint-limit YAML](https://github.com/frankarobotics/franka_ros/blob/develop/franka_description/robots/panda/joint_limits.yaml) and [live FER table](https://frankarobotics.github.io/docs/robot_specifications.html#limits-for-franka-emika-robot-fer), rather than FR3's different joint limits.

## Frozen simulation interface

```python
import numpy as np
from hrc_sim import HRCSim, SimConfig

with HRCSim(SimConfig()) as sim:
    obs = sim.reset(seed=42)
    terms = sim.task_space_dynamics()
    tau = terms['N_tau'] @ np.zeros(7)  # Explicit torque projector.
    obs = sim.step(tau)                # Gravity added by the environment.
    assert not any(np.any(v) for v in sim.limits_violated().values())
```

Returned arrays are copies. Configuration is copied into `sim.config`; edit its fault arrays there for runtime injection. `model`/`data` are public MuJoCo objects for experiments. After directly changing state/applied forces call `mujoco.mj_forward(sim.model,sim.data)` before reading derived fields; this does not resample cached measurement noise.

| API | Contract |
| --- | --- |
| `HRCSim(config=None)` | Compile configured scene and reset |
| `reset(q0=None, seed=None)` | Observation; optional `(7,)` rad arm pose; resets state/time/controls/applied forces/hand and noise stream; fault configuration persists |
| `step(tau, n_substeps=1)` | `(7,)` N·m command held for positive integer physics steps; returns synchronized observation |
| `get_joint_state()` | `(q,dq)`, each `(7,)`, rad and rad/s |
| `get_ee_pose()` | `(p,R,quat)`: `(3,)` m, `(3,3)`, `(4,)` wxyz |
| `get_ee_velocity()` | `(6,)` world `[v;omega]`, m/s and rad/s |
| `jacobian()`, `jacobian_dot()` | `(6,7)` TCP world J and its time derivative |
| `mass_matrix()` | `(7,7)` total arm inertia including tool and armature; **do not add armature again** |
| `bias_forces()`, `gravity_torque()` | `(7,)` N·m; bias is Coriolis/centrifugal plus gravity, excluding damping/contact; gravity uses a zero-velocity scratch state |
| `joint_damping` | Read-only copy `(7,)` passive viscous coefficients, N·m·s/rad |
| `last_tau_applied` | Read-only copy `(7,)` actual clipped motor command from last step/reset, N·m |
| `limits_violated(margin=0.0)` | Dict `position`, `velocity`, `torque`, each bool `(7,)`; position margin in rad; torque indicates a command at a saturation boundary |
| `task_space_dynamics(damping=None, include_joint_damping=False)` | Dict described below; None uses configured continuous damping, 0 requests exact full-rank identities |
| `get_ext_wrench(frame='world')` | Faulty compensated wrist `(6,)` wrench at TCP, N/N·m; frame `'world'` or `'tcp'` |
| `get_ext_joint_torque(measured=False)` | `(7,)` N·m contact truth by default; `True` adds cached configured joint-sensor faults |
| `get_contact_wrench(group=None)` | `(6,)` contact truth, world at TCP; `'peg_hole'`, `'robot_hand'`, `'all'` or None=all robot contacts |
| `get_max_normal_force(group=None)` | Largest individual contact normal magnitude, N; zero for no matching contacts |
| `get_contact_normal_sum(group=None)` | Sum of individual normal magnitudes, N, counting each contact once |
| `set_hand(pos, quat=None, enabled=True, mode='kinematic', allow_penetration=False)` | World `(3,)` m, optional wxyz `(4,)`; place on initial enable, subsequently move target; modes `'kinematic'`/`'compliant'`; disable parks body/target and removes collisions |
| `release_hand(velocity)` | Enabled hand enters `'free'` mode: weld off, world linear velocity `(3,)` m/s, angular velocity zero |
| `render(width=960,height=720,camera=None)` | RGB uint8 `(height,width,3)`; default fixed camera `'insertion'` |
| `close()` | Release renderer; also called by context-manager exit |
| `VideoRecorder(path,fps=30,width=960,height=720,camera=None)` | Streaming imageio context manager: `.capture(sim)` at selected frame cadence; `.close()` when done |

The observation keys remain fixed:

| Key | Shape / meaning |
| --- | --- |
| `t` | Scalar simulation time, s |
| `q`, `dq` | `(7,)` arm positions/velocities |
| `ee_pos`, `ee_R`, `ee_quat` | `(3,)`, `(3,3)`, `(4,)`, TCP world pose |
| `ee_vel` | `(6,)`, TCP world twist |
| `ft_wrench` | `(6,)`, compensated wrist estimate **with faults**, world at TCP |
| `contact_wrench` | `(6,)`, all robot contact truth, world at TCP |
| `tau_ext` | `(7,)`, **measured** joint contact torque, including configured faults; use `get_ext_joint_torque()` for truth |

## Torque and operational-space terms

```text
tau_applied = clip(tau_command
                   + gravity_torque(q)    # if gravity_compensation=True, default
                   + D*dq,               # if damping_compensation=True, default False
                   -tau_max, +tau_max)
```

Both compensations are recomputed each substep before clipping. Motors enforce ±87 N·m on joints 1–4 and ±12 N·m on joints 5–7. Velocity limits are monitored rather than clamped; position limits are compliant MuJoCo constraints and can overshoot under load. `limits_violated` exposes these events. No torque slew-rate limit is imposed.

Passive joint damping is 1 N·m·s/rad per joint. At home it contributes roughly **5–20 N·s/m of apparent Cartesian damping**, depending on direction; this is pose dependent and forms a floor under the commanded Cartesian damping unless explicitly compensated. With `include_joint_damping=True`, mu includes `D*dq`; this is independent of the actuator's `damping_compensation` flag. If actuator damping compensation is enabled, use mu excluding D in the corresponding compensated equation.

**Both** environment and independent dynamics return exactly these task-space keys:

| Key | Shape / definition |
| --- | --- |
| `Lambda` | `(6,6)`, `(J M^-1 J.T + damping² I)^-1` |
| `Jbar` | `(7,6)`, `M^-1 J.T Lambda` |
| `N_vel` | `(7,7)`, `I-Jbar J`, velocity projector |
| `N_tau` | `(7,7)`, `I-J.T Jbar.T = N_vel.T`, torque projector |
| `Jdot_dq` | `(6,)`, `jacobian_dot() @ dq` |
| `mu` | `(6,)`, `Lambda (J M^-1 h-Jdot_dq)`; h excludes gravity and optionally includes passive damping |
| `p` | `(6,)`, `Lambda J M^-1 gravity_torque()` |

There is **no bare `N` key**. The default `task_damping=0.01` is added continuously at every configuration, without a condition-number switch. At full rank `damping=0` gives `J M^-1 N_tau=0`; regularization sacrifices exact inversion/null decoupling to bound gain near singularities. The reference controller uses the environment's `N_tau` for posture torques. `task_condition_limit` and the old example-controller `null_damping` field were removed.

The independent NumPy interface provides `PandaKinematics.fk_flange(q)`, `.fk_tcp(q)`, `.jacobian(q)`, `.jacobian_dot(q,dq)` and `PandaDynamics.from_mujoco_model(model)` for one-time constant extraction. Dynamics methods are `.mass_matrix(q)`, `.rigid_mass_matrix(q)`, `.gravity_torque(q)`, `.coriolis_forces(q,dq)`, `.bias_forces(q,dq)`, `.inverse_dynamics(q,dq,ddq,include_damping=True)` and `.task_space_dynamics(q,dq=None,damping=None,include_joint_damping=False)`. Its `task_damping` default is also 0.01. The functional `dynamics.task_space_dynamics(M,J,h=None,g=None,jdot_dq=None,damping=0.01)` accepts total inertia and caller-supplied bias terms. All three task dicts share the explicit keys above.

## Sensors and force metrics

Wrist compensation removes the tool's complete gravity and inertia using ideal simulator acceleration and known mass/COM/inertia. A freely accelerating tool reads approximately zero; a positive world-x perturbation on the tool reads positive world-x. Torque is shifted from flange to TCP. Upstream link contacts are invisible to wrist F/T and remain visible in contact-based joint torque.

`get_ext_joint_torque` sums contact-point Jacobian-transpose wrenches over every robot-side contact, including upstream links. Contact truth excludes gravity, damping, limit reactions, hand weld reactions and `xfrc_applied`; wrist F/T includes applied tool perturbations. MuJoCo contact axes are matrix rows; positive force acts on geom2. Both sides of a self contact contribute to joint reactions; spatial equal/opposite resultants cancel. See [MuJoCo sensor](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-force) and [contact conventions](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#contacts).

`SimConfig.sensor_bias`/`noise_std` are six-vectors. **Default `sensor_bias_frame='sensor'`** defines faults in flange sensor axes at the flange point: force is rotated into world and its moment arm to TCP is added. `'world'` preserves the earlier convention of faults already expressed at TCP in world axes. Noise uses the same frame as bias; optional `frame='tcp'` rotates the final wrench. `tau_ext_bias`/`tau_ext_noise_std` are seven-vectors in joint axes. Scalar noise amplitudes are broadcast at configuration construction.

Standard Gaussian samples for both sensors are drawn **once at reset and once per physics substep**, then cached. Repeated reads, observation calls and plotting never advance the streams. Fault amplitudes/bias may be edited on `sim.config` and take effect immediately using the current cached sample. The base seed defaults to `SimConfig.seed=0`; explicit `reset(seed=...)` replaces the stored base seed and subsequent unseeded resets repeat it. Runtime fault settings persist across reset; set their arrays to zero to clear them.

Three force metrics are deliberately separate: `norm(get_contact_wrench(group)[:3])` is the **resultant**; `get_max_normal_force` is the **largest single normal**; `get_contact_normal_sum` is the **sum of normal magnitudes**. They can differ substantially and the largest normal can exceed the resultant. Demo and scenario peaks are **raw, single-physics-step, unfiltered** maxima, including touchdown transients; they are not steady forces. Part B must state its chosen metric and any filtering cutoff before evaluating requirements.

## Fixture capture range and hand modes

`SimConfig` exposes dt/gravity, peg half-sizes/mass, clearance, chamfer, depth/pose, wall/table dimensions, friction/solref/solimp, home pose, joint limits, compensation flags, sensor faults/seed/frame, task damping, camera and nested hand parameters. Geometry is compiled at construction. Changing hole pose or peg length may require a suitable `q_home`; no automatic IK is run on construction.

Default rigid contacts use `dt=0.001`, `implicitfast`, an elliptic cone, `condim=3`, friction `(0.3,0.005,0.0001)`, `solref=(0.004,1)` and `solimp=(0.95,0.99,0.001)`. The critically damped response spans four steps; the smooth 1 mm impedance transition is not a geometric clearance. Adjacent/near-adjacent link and wrist/tool collisions are excluded. The chamfer consists of four inclined box faces plus outer rim caps; the straight 31 mm square throat starts 3 mm below the 37 mm square mouth. `chamfer=0` restores the sharp four-wall rim.

With the unchanged constant-gain example and a slow four-second descent, tested lateral offsets **0, 0.5, 1 and 2 mm all reach 40 mm depth** with the chamfer. The 1 mm sharp-rim case jams near 0.09 mm. Tested 1° tilt and 1°/2° yaw insert; 2° tilt remains partially wedged around 35 mm. These results establish the tested x-offset capture cases, not arbitrary combined angular/lateral capture. The generated table in `docs/modeling_notes.md` reports every depth and peak, including 100 N-scale contact transients in offset cases. Demonstrations for D/E should retain this capture envelope or validate their own trajectories.

Hand modes have distinct meanings:

- **`kinematic`**: prescribed-motion clamp through a stiff weld `(0.002,1)`; a continuing target can keep loading/pushing the robot. It is a dynamic approximation to a position source.
- **`compliant`**: softer weld `(0.04,1)`, a target-position spring (roughly 16–37 N/mm over the reviewed loads), still a position source. It is not a free ISO transient model.
- **`free`**, entered by `release_hand(velocity)`: weld disabled; the 0.6 kg effective mass hits at its specified incident velocity. Default hand gravity compensation preserves its free-flight incident speed. This is the calibrated elastic transient benchmark.

**Warning:** only the initial enable is guarded against penetration. A large target jump of an already-enabled hand is not guarded and can produce kilonewton forces; move the target in small increments or use `release_hand()`.

`HandConfig.contact_solref=(-125000,-1)`, `contact_solimp=(0.95,0.95,0.001)` and `solmix=1000` are applied **after** global geometry contact parameters. These direct-format solver coefficients are not literal N/m material constants. They calibrate a free 0.6 kg hand onto the fixed table to the `v sqrt(m k)` reference with `k=75000 N/m = 75 N/mm`: peaks are approximately **21.30, 53.24 and 106.49 N** at **0.1, 0.25 and 0.5 m/s**, all within **0.4%** of the reference; regression tolerance is **±2%**, tighter than the requested ±15%. Contact mixing and direct-format scaling are described in [MuJoCo's solver documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters). This is a calibrated effective-mass/spring study model; mass remains **TBD by part A**, and impacts against a moving robot have a different reduced mass than this fixed-table benchmark.

The capsule radius is 40 mm and its total length 180 mm. It starts parked at `(2,0,1)` with collisions disabled. On fresh enable, placements deeper than `HandConfig.placement_max_penetration=0.001 m` are rejected with `ValueError` and the hand is returned to its disabled parked state. Start outside and approach in small increments; `allow_penetration=True` is an explicit experiment override. Subsequent target changes move the physical hand through its weld without teleporting it. Disabling restores the parked body/target and weld; reset also returns mode to `kinematic`.

## Notes for downstream parts

| Part | Note |
| --- | --- |
| A | Use `release_hand(velocity)` for transient cases; the calibration is a peak-force match on a fixed surface, impacts on the moving arm give lower peaks (45.6 N on the peg, 29.1 N on link5 at 0.25 m/s). Default `sensor_bias_frame` is `'sensor'`. Observation `tau_ext` is the measured value (equal to truth when no fault is configured). |
| B | `limits_violated()` and the three force metrics are available; choose and state the metric used for each requirement. |
| D | Capture range with the chamfer is about ±3 mm lateral; 2° tilt still wedges at 35 mm. |
| E | Use `N_tau`; default task terms are regularised with λ = 0.01 (pass `damping=0` for exact identities away from singularities); with the environment's gravity compensation the controller equation has no `p` term. |
