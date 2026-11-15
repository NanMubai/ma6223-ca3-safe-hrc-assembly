# Panda kinematics, dynamics and numerical verification

This is the full derivation and verification record for the simulation/modeling part of **Safe Human–Robot Collaborative Assembly via Learning from Demonstration and Variable Impedance Control**. The independent implementation is in `hrc_sim/kinematics.py` and `hrc_sim/dynamics.py`; it does not use MuJoCo FK, Jacobians, mass or inverse-dynamics algorithms. A factory reads only constant physical parameters from the compiled model once. Verification uses MuJoCo as a separate numerical reference.

Run from the editable repository root:

```bash
conda activate hrc
MUJOCO_GL=egl python scripts/verify_modeling.py
python -m pytest -q
```

The script writes `report/figures/modeling_verification.json`, a six-data-row `modeling_verification_table.tex` without a float/caption, and `scene.png`. The PNG is 1600 × 1200 pixels, 4:3, intended for a 3.5-inch report column. Its labelled peg/fixture and world/TCP frame arrows are projected from the actual camera matrices onto a cropped, closer rendering; labels/axes are an annotation layer, not physical robot parts. The table uses scoped `\footnotesize`, math-mode scientific notation and explicit units; a direct IEEEtran measurement gives **212.97845 pt**, within its **252 pt** column. Shadow/reflection rendering effects are disabled for byte-reproducible PNG output; the camera angle and crop retain legible TCP axes and peg/fixture labels. The script also refreshes the generated numerical section below from its JSON data, so rerunning with a new seed/sample count keeps this record synchronized.

## 1. Frames, units and physical state

The world/base frame `{W}={0}` has origin at the Panda base and z pointing up. Joint coordinates are `q=(q1,...,q7)` in rad, velocities in rad/s and torques in N·m. Positions, masses, times and forces use m, kg, s and N. Quaternions are scalar-first `(w,x,y,z)`. A transform `^A T_B` maps coordinates from frame B into A; its rotation columns are B's axes expressed in A.

Frame `{i}` is the native post-joint body frame of `linki`, i=1,...,7. `{F}` is the model's `attachment`/flange frame, at the `ft_site`. `{T}` is the `tcp` site at the centre of the square peg's free end face. The peg extends from the flange along local **+z**. At home local +z points vertically down. The TCP is 60 mm from the flange by default, not at the peg COM (30 mm).

All Jacobians and twists use world axes and order `[linear; angular]`: `V=[v;omega]=J dq`. Wrenches use `[force;torque]`, expressed in world axes unless explicitly requested in TCP axes, and are referenced at the TCP. External wrench means **environment-on-robot**. Positive insertion depth is `(p_hole_top-p_TCP) dot z_hole`. The default entrance is `(0.5,0,0.28)` m and the bottom is 40 mm below it.

The complete model has `nq=14, nv=13, nu=7`: seven arm hinges plus a free hand with seven position/six velocity coordinates. This was an approved deviation from the original `nq=7` requirement, necessary to give the hand physical inertia and a compliant weld. Every public arm quantity is seven-dimensional, with indices found from named joints' `jnt_qposadr`/`jnt_dofadr`; it does not rely on blindly taking the first seven coordinates. The hand is in a separate world-rooted kinematic tree. Its unconstrained mass matrix has zero cross block with the arm; contacts couple the trees through external constraint forces. Therefore the 7×7 arm block of the full inertia matrix is exact.

## 2. Modified (Craig) DH chain and FK

The table is derived directly from the supplied [Menagerie Panda kinematic tree](../assets/franka_emika_panda/panda_nohand.xml), vendored at revision `370c439`, and its link dimensions agree with [Franka's Craig-convention table](https://frankarobotics.github.io/docs/robot_specifications.html#denavit-hartenberg-parameters). The current manufacturer page labels that table FR3; here the **Panda XML itself** is the model authority, and the numerical link dimensions coincide. Panda joint limits are the separate FER/Panda values on that page, not FR3's different joint ranges.

Craig modified DH is used in this exact multiplication order:

\[
 A_i = {}^{i-1}T_i
     = R_x(\alpha_{i-1})T_x(a_{i-1})R_z(q_i)T_z(d_i).
\]

| i | a_(i−1), m | alpha_(i−1), rad | d_i, m | theta_i |
| --- | ---: | ---: | ---: | --- |
| 1 | 0 | 0 | 0.333 | q1 |
| 2 | 0 | −π/2 | 0 | q2 |
| 3 | 0 | +π/2 | 0.316 | q3 |
| 4 | 0.0825 | +π/2 | 0 | q4 |
| 5 | −0.0825 | −π/2 | 0.384 | q5 |
| 6 | 0 | +π/2 | 0 | q6 |
| 7 | 0.088 | +π/2 | 0 | q7 |
| Fixed DH flange | 0 | 0 | 0.107 | 0 |

Explicitly, writing c=cos(q_i), s=sin(q_i), ca=cos(alpha), sa=sin(alpha),

\[
 A_i=\begin{bmatrix}
 c&-s&0&a\\
 s\,ca&c\,ca&-sa&-d\,sa\\
 s\,sa&c\,sa&ca&d\,ca\\
 0&0&0&1
 \end{bmatrix}.
\]

The distinction between the fixed **DH flange** and the model's **attachment frame** matters: Menagerie rotates the latter about z by nominally 135°. We retain the exact normalized authored quaternion `(0.3826834,0,0,0.9238795)`, rather than silently replacing its rounded values with a different constant. Let its rotation be `R_attach`. Then

\[
 {}^0T_7=\prod_{i=1}^7 A_i,\qquad
 {}^7T_F=T_z(0.107)\,R_{attach},\qquad
 {}^FT_T=T_z(0.060),
\]
\[
 {}^0T_F={}^0T_7{}^7T_F,\qquad
 {}^0T_T={}^0T_F{}^FT_T.
\]

For a configured peg half-length `hz`, the last translation is `2 hz`. `PandaKinematics()` uses the default fixed transforms independently of MuJoCo. `PandaDynamics.from_mujoco_model(model)` may read the configured constant attachment/site transforms once, in addition to masses/inertias, so nondefault peg dimensions remain consistent. Joint FK still comes from the DH table.

## 3. Analytic geometric Jacobians and their derivatives

Let `o_j` be any point on joint j's axis and `z_j` its unit axis, both expressed in world. The post-joint DH origin and third rotation column can be used: rotation about z does not change its axis, and translation along z does not change the cross product below. For a world point p fixed to a body downstream of n joints:

\[
 J_{v,j}(p)=z_j\times(p-o_j),\qquad J_{\omega,j}=z_j,
 \quad j\le n;
 \qquad J_j=0\quad j>n.
\]

Stacking the linear and angular blocks gives `J(p)=[Jv;Jomega]`, 6×7. The TCP uses all seven columns; link-i COM uses only columns 1,...,i. The tool also uses all seven. This same formula is used for every body's COM in the independent dynamics, not `mj_jac`.

Differentiating analytically,

\[
 \omega_{<j}=\sum_{k<j}z_k\dot q_k,\quad
 \dot z_j=\omega_{<j}\times z_j,\quad
 \dot o_j=\sum_{k<j}[z_k\times(o_j-o_k)]\dot q_k,
\]
\[
 \dot p=J_v(p)\dot q,\quad
 \dot J_{v,j}=\dot z_j\times(p-o_j)+z_j\times(\dot p-\dot o_j),
 \qquad \dot J_{\omega,j}=\dot z_j.
\]

This provides independent `Jdot` and COM acceleration terms. Finite differences are used only for verification: central differences of FK with epsilon=1e−6 rad, with angular columns extracted from `Rdot R.T` in world axes. Orientation error against MuJoCo is the SO(3) geodesic angle computed with `atan2` of skew magnitude and trace; unlike a bare `acos`, it remains accurate near zero error.

## 4. Rigid-body parameters and joint-space dynamics

For each moving Panda body and the separate peg/tool, the constant data are mass `m_i`, local COM `c_i`, and COM inertia tensor `I_i` in native body axes. The fixed base has a zero Jacobian and need not enter the moving inertia sum; the attachment has zero mass. The tool is included as an eighth moving mass of 0.2 kg by default.

MuJoCo stores diagonal principal moments and a local inertial-frame quaternion. The one-time extraction reconstructs the full tensor:

\[
 I_i=R_{B_iI_i}\operatorname{diag}(I_{principal,i})R_{B_iI_i}^{T}.
\]

Its eigenvectors are not discarded. At q, independent DH FK gives `^0 T_Bi`, `p_ci=R_i c_i+p_i`, and the COM Jacobians. The world tensor is `I_i^W=R_i I_i R_i.T`. The kinetic energy is

\[
 T=\tfrac12\sum_i\left[m_i\|J_{v,i}\dot q\|^2
 +(J_{\omega,i}\dot q)^T I_i^W(J_{\omega,i}\dot q)\right]
 +\tfrac12\dot q^T I_a\dot q.
\]

Hence the **rigid-body** inertia and the **total** inertia are

\[
 M_{rb}(q)=\sum_i\left[m_iJ_{v,i}^TJ_{v,i}
 +J_{\omega,i}^TR_iI_iR_i^TJ_{\omega,i}\right],\qquad
 \mathcal M(q)=M_{rb}(q)+I_a.
\]

The armature is `Ia=diag(0.1,...,0.1)` kg·m², retained from Menagerie. It is constant reflected joint inertia, so it contributes to acceleration but not gravity or Coriolis terms. **Both `HRCSim.mass_matrix()` and `PandaDynamics.mass_matrix()` return total inertia `mathcal M` including armature. Do not add it a second time.** `rigid_mass_matrix()` returns `M_rb` separately.

For constant world gravity `g_vec=(0,0,−9.81)` m/s², potential energy is `U=−sum m_i g_vec.T p_ci`, and

\[
 g(q)=\nabla_q U=-\sum_i m_iJ_{v,i}^Tg_{vec}.
\]

To obtain `h=C(q,dq)dq` independently without differentiating numerical reference mass matrices, use the bias accelerations `a_bias,i=Jdot_vi dq`, `alpha_bias,i=Jdot_omega_i dq` and `omega_i=Jomega_i dq`. Newton–Euler virtual work gives

\[
 h(q,\dot q)=\sum_i\left[
 J_{v,i}^T m_i a_{bias,i}
 +J_{\omega,i}^T\{I_i^W\alpha_{bias,i}
 +\omega_i\times(I_i^W\omega_i)\}\right].
\]

This is the usual Christoffel/Coriolis vector, including centrifugal and gyroscopic effects. A particular nonunique matrix C is unnecessary. `bias_forces()` returns `h+g`, excluding damping. MuJoCo's arm damping is `D=diag(1,...,1)` N·m·s/rad; there is no additional modeled dry joint friction or motor dynamics. These meanings follow the [MuJoCo joint armature/damping definitions](https://mujoco.readthedocs.io/en/stable/XMLreference.html#body-joint).

The smooth joint equation, including tool and reflected inertia, is therefore

\[
 \boxed{(M_{rb}(q)+I_a)\ddot q+C(q,\dot q)\dot q+g(q)+D\dot q
       =\tau+\tau_{ext}.}
\]

For a pure TCP wrench, `tau_ext=J.T F_ext`; upstream link contacts require the full contact Jacobian sum instead. `PandaDynamics.inverse_dynamics()` implements the left-hand side. Contact and joint-limit reactions are omitted only in the isolated smooth-dynamics verification, where those constraints are explicitly disabled. The forward residual uses the **actual actuator torque**, and the inverse residual uses `mj_inverse`'s required smooth torque including passive damping.

Seven ideal torque motors have unit gear and no servo bias. The torque limit is 87 N·m on joints 1–4 and 12 N·m on joints 5–7. In normal `HRCSim.step`,

\[
 \tau=\operatorname{clip}(\tau_{cmd}+g(q),-\tau_{max},+\tau_{max}).
\]

Gravity compensation is on by default and can be disabled. Optional `SimConfig.damping_compensation=True` also adds `D dq` before clipping; its default is False. `joint_damping`, `last_tau_applied` and `limits_violated(margin)` expose passive coefficients, actual saturated commands and position/velocity/torque status. The passive Cartesian damping floor is roughly 5–20 N·s/m at home (pose/direction dependent), so low commanded damping is not the full impedance unless D is compensated. Thus a zero command at rest holds home, while controllers should omit gravity from commands when this option is enabled. Saturation occurs **after** adding gravity; at saturation the achieved wrench can differ from the requested one. Velocity limits are metadata, and position limits are simulator constraints. No torque slew-rate limit is imposed.

## 5. Operational-space dynamics and null-space torques

Write `a=dV/dt=J ddq+Jdot dq`; its angular component is world angular acceleration, not a second derivative of Euler angles or quaternion entries. At a full-row-rank TCP Jacobian,

\[
 A=J\mathcal M^{-1}J^T,\quad
 \Lambda=A^{-1},\quad
 \bar J=\mathcal M^{-1}J^T\Lambda,\quad
 \boxed{N_\tau=I-J^T\bar J^T.}
\]

Both task-space APIs now return **`N_tau`** for this torque projector and **`N_vel=I−Jbar J`** for velocities; no bare `N` key remains. The reference controller obtains `N_tau` from the environment, using dynamically consistent posture torques. `J M_total^-1 N_tau=0`, whereas a generic Euclidean Jacobian projector does not have this property.

For actuator torques `tau=J.T F+N_tau tau0`, define

\[
 \mu_{rb}=\Lambda[J\mathcal M^{-1}h-\dot J\dot q],\qquad
 \mu_D=\Lambda J\mathcal M^{-1}D\dot q,\qquad
 p=\Lambda J\mathcal M^{-1}g.
\]

The equation **as physically simulated with passive damping** is

\[
 \boxed{\Lambda a+\mu+p=F+F_{ext},\qquad \mu=\mu_{rb}+\mu_D.}
\]

Both methods' option `include_joint_damping=True` returns this full mu. Their default returns `mu_rb`, the standard rigid-body task term. With explicit actuator compensation `tau=J.T F+D dq+N_tau tau0`, the same equation uses `mu=mu_rb`. The tests verify **both conventions**, rather than silently ignoring passive damping. `p` is the operational gravity term regardless of whether the environment later adds gravity to a command.

For arbitrary external joint torques, the task-projected external force is `F_ext=Lambda J M_total^-1 tau_ext=Jbar.T tau_ext`. It equals an actual TCP wrench when `tau_ext=J.T wrench` and rank is full. Multiple upstream link contacts need not have an equivalent physical wrench acting solely at the TCP.

A null-space torque contributes no **instantaneous task acceleration increment** in the full-rank case. Zero total acceleration under a null torque additionally requires cancellation of gravity and bias/kinematic terms. The null test starts at rest, uses `tau=g+N_tau tau0`, and verifies zero TCP acceleration. At nonzero dq, cancelling joint bias alone leaves the geometric `Jdot dq` term.

A continuous damping term is added at **every** configuration, with default lambda=0.01; there is no condition-number switch. Both methods accept `damping=None` (configured default), `damping=0` (exact full-rank identities), and the shared `include_joint_damping` option. The damped inverse is

\[
 \Lambda_\lambda=(A+\lambda^2I_6)^{-1},\quad
 \bar J_\lambda=\mathcal M^{-1}J^T\Lambda_\lambda,\quad
 N_{\tau,\lambda}=I-J^T\bar J_\lambda^T.
\]

For positive-semidefinite A, its maximum spectral gain is bounded by `1/lambda²` up to floating-point error. Regularization keeps values finite but sacrifices exact task inversion, null decoupling and projector idempotence. A rank-deficient direction is physically uncontrollable; a finite damped value cannot create authority in it. Translational and rotational rows use their SI units directly; no dimensionless characteristic-length rescaling is introduced.

The chosen singular check is `q=(0,0,0,−0.467002423653011,0,0.467002423653011,0)` rad, inside joint limits, with rank 5. The original environment test used `(0,0,0,−0.0698,0,0.0698,0)`, which actually has rank 6; that test input was corrected. The review revision updates `env.py` to use explicit projector names, continuous damping, public `jacobian_dot()` and the shared `Jdot_dq`/passive-damping options.

## 6. Wrist F/T compensation and contact torque truth

Let pF, pT and pc be the flange, TCP and tool COM positions; all quantities in this section are in world axes. Raw flange force/torque sensor values are rotated from the flange site axes into world. Denote the resulting transmitted wrench by `s=[sf;st]`. For the rigid tool,

\[
 f_{dyn}=m_t(a_c-g_{vec}),\quad
 t_{dyn,F}=I_t^W\alpha+\omega\times(I_t^W\omega)
             +(p_c-p_F)\times f_{dyn}.
\]

The engine's sensor convention is checked numerically by applying a known `xfrc_applied` force/torque on the tool. In this convention the environmental load is

\[
 f_{ext}=f_{dyn}-s_f,\quad
 t_{ext,T}=t_{dyn,F}-s_t+(p_F-p_T)\times f_{ext}.
\]

The final term references torque at the TCP, not the flange. `a_c=Jvc ddq+Jdot_vc dq`, `alpha=Jomega ddq+Jdot_omega dq`, and the full rigid tool inertia is included. Free static **and moving** tool readings therefore approach zero; a positive world-x applied force reads positive world-x. `frame='tcp'` rotates force and torque by `R_TCP.T`, without moving the reference point again. Sensor bias and independent Gaussian noise default to flange sensor axes/origin (`sensor_bias_frame="sensor"`); their force is rotated into world and its moment arm to TCP is added. `sensor_bias_frame="world"` retains faults already defined at TCP in world axes. These faults precede the optional TCP-axis rotation. Noise is sampled once at reset and once per physics substep, then cached; additional reads never advance the streams. `SimConfig.seed=0` supplies a deterministic base seed, replaced by an explicit reset seed. Joint measurements have independent `tau_ext_bias`/`tau_ext_noise_std`: `get_ext_joint_torque(measured=False)` returns truth, True returns a cached faulty measurement, and observation `tau_ext` uses True. Runtime fault settings persist across reset until explicitly cleared. Compensation assumes ideal known tool parameters and simulator acceleration; it is not a noisy hardware acceleration observer.

Wrist F/T measures loads on the tool side of the flange; it cannot observe upstream link contacts. The independent definition of ground-truth generalized contact torque is

\[
 \tau_{ext}=\sum_{c,b\in robot}
 \left[J_v(b,p_c)^Tf_{c\to b}+J_\omega(b,p_c)^Tt_{c\to b}\right].
\]

Each contact wrench is obtained from `mj_contactForce`. Its frame stores axes as **rows** and must be transposed to rotate into world. The returned positive wrench acts on geom2, with its negative on geom1. Both robot sides are included for self contacts. Equal/opposite internal contact forces cancel in the total spatial resultant, but can still produce joint reactions through different body Jacobians. `get_contact_wrench` sums the robot-side spatial wrench about TCP; `get_ext_joint_torque` retains each contact's own body Jacobian. Groups are `peg_hole`, `robot_hand`, and `all`; `get_max_normal_force` returns the largest **individual** normal force, not a patch sum.

Contact truth excludes applied perturbation forces, gravity, viscous damping and the hand-target equality reaction. The wrist measurement includes perturbations on the tool, so these two observables intentionally differ under `xfrc_applied`. See [MuJoCo force sensors](https://mujoco.readthedocs.io/en/stable/XMLreference.html#sensor-force) and [contact frames](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#contacts) for engine conventions; tests enforce the interpreted physical signs.

## 7. Contact and hand models

The square peg is 30×30×60 mm, 0.2 kg. Four box walls define a 31×31 mm straight throat, giving 0.5 mm clearance per side; a default 3 mm, 45-degree lead-in expands its mouth to 37×37 mm, using tilted primitive boxes and outer rim caps. The straight throat begins 3 mm below the entrance. `SimConfig.chamfer=0` disables the lead-in. The cavity is 40 mm deep with a bottom plate. Default contact parameters are `dt=0.001 s`, `integrator=implicitfast`, `cone=elliptic`, `condim=3`, `friction=(0.3,0.005,0.0001)`, `solref=(0.004,1)` and `solimp=(0.95,0.99,0.001)`. With condim=3 only normal and sliding friction are active; the stored torsional/rolling coefficients are unused for these contacts.

The 4 ms, critically damped contact time constant spans four timesteps and exceeds the two-timestep stability floor. The 1 mm impedance transition is smooth; it is not a prescribed penetration depth or clearance. Sliding friction 0.3 avoids excessive insertion stiction. Adjacent/near-adjacent arm links and wrist/tool neighbors are excluded from collision; other self contacts remain possible. These are compliant numerical contacts, not a perfectly rigid complementarity model, as described by [MuJoCo's constraint formulation](https://mujoco.readthedocs.io/en/stable/computation/#constraint-model).

The hand is a free capsule with radius 40 mm, cylindrical half-length 50 mm (180 mm total length), and effective mass **0.6 kg, TBD by part A**. This is the project's ISO/TS 15066 effective-mass assumption, not an independently validated anatomical mass or compliance. A mocap target drives it through a weld. The `kinematic` mode uses a stiff dynamic weld, `solref=(0.002,1)`, for quasi-static clamping, while `compliant` uses `solref=(0.04,1)` as a position spring, not a free ISO transient. The target trajectory and weld compliance, not mocap teleportation of an already-enabled body, determine the physical impact response. Weld time constants are bounded below by `2 dt`. The body is gravity compensated, parked at `(2,0,1)` m and collision-disabled by default; reset parks both its free state and target. A moving `kinematic` target is a prescribed-motion clamp, while `compliant` is a position spring (reviewed weld stiffness roughly 16–37 N/mm); continuing either target can keep supplying work. The public `release_hand(velocity)` disables the weld, sets world linear velocity and zeros angular velocity, entering **free** mode for the elastic transient benchmark. Fresh hand placement deeper than 1 mm is rejected before integration and rolled back to the disabled parked state unless explicitly overridden with `allow_penetration=True`; contact tests now approach from a clear surface.

Hand-specific `contact_solref=(-125000,-1)`, `contact_solimp=(0.95,0.95,0.001)` and `solmix=1000` are applied after global contact parameters. MuJoCo's negative direct format sets constraint-reference acceleration coefficients, not literal material stiffness in N/m; a non-positive direct pair uses the elementwise minimum when mixed, while high solmix makes hand impedance dominate. See the [primary solver/mixing documentation](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters). These values were calibrated at the production 1 ms timestep against a free 0.6 kg capsule striking the fixed table vertically, with its tip initially 2 mm clear and its centre at `(0.65,0.12,table_z+0.092)` m. The hand's gravity compensation preserves incident speed. For the project reference k=75 N/mm, energy balance `m v²/2=k delta²/2` gives `F_peak=v sqrt(m k)`. Speeds 0.1/0.25/0.5 m/s give peaks about 21.30/53.24/106.49 N, all within 0.4% of the elastic reference; regression tolerance is ±2% (requested target ±15%). This fixed-table benchmark has reduced mass 0.6 kg; impacts against the moving arm require their actual reduced mass and are not assumed to have the same peak. The contact is intentionally nearly elastic and is a study calibration, not validated human tissue.

The capture experiment holds the specified lateral/orientation error throughout a 2 s approach to 20 mm above the entrance, 4 s quintic descent and 1.5 s hold, with the same constant reference gains. It reports raw resultant contact peaks. Lateral 0/0.5/1/2 mm cases reach 40 mm with the chamfer; the 1 mm sharp control jams at about 0.09 mm. Tilt 1 degree and yaw 1/2 degrees insert; tilt 2 degrees wedges near 35 mm. Outcomes are `inserted` for depth ≥39 mm, `partial` for 3–39 mm, and `jammed` below 3 mm. The separate course success flag uses depth ≥30 mm. These are tested x offsets, not guarantees for combined errors or arbitrary controller settings. The nominal and 1.5 mm-offset demo exercise bottom and wall/chamfer contact respectively.

The contact sanity experiments initialize the vertical peg 2 mm above either the right rim (x offset 30 mm) or the cavity bottom. A known 5 N downward Cartesian load, lateral stiffness 1200 N/m, orientation stiffness 80 N·m/rad, and velocity gains 85 N·s/m (lateral), 65 N·s/m (vertical), and 12 N·m·s/rad (angular) are applied with environmental gravity compensation. Each runs 3 s at the normal 1 ms step; the final 0.5 s is the steady window. Contact penetration is checked over **the entire transient**, not only after settling. Steady upward reaction is compared with the 5 N load using a 0.05 N (1%) tolerance.

## 8. Verification protocol, tolerances and numerical results

The reproducible random suite samples 1000 configurations uniformly over **all seven configured joint ranges**, seed 6223, with velocities ±0.3 rad/s. It checks FK, SO(3) orientation, analytic Jacobians against `mj_jacSite`, central-difference FK, analytic `Jdot`, total inertia including armature, gravity, Coriolis and forward/inverse equations of motion. Mass error is a relative Frobenius norm; other matrix/vector comparisons are maximum absolute entries per state, and position uses Euclidean error. Positive definiteness is checked by the minimum symmetric eigenvalue at every state. Contacts and joint-limit constraints are disabled only in these smooth checks so random self intersections or limit margins cannot contaminate the reference equations.

The task suite uses a further 1000 random states, seed 6224, with velocities ±0.1 rad/s and random task wrench components ±1 N and ±0.2 N·m. It rejects states with `cond(J M_total^-1 J.T)>1e5` for **exact** inverse checks; the actual singularity is tested separately with damping. The reference includes MuJoCo forward acceleration, plus an actual one-step forward integration and a TCP velocity difference. Null torque tests use a random seven-vector within ±0.2 N·m, start at rest and compensate gravity; all task commands are explicitly asserted below torque limits.

The continuous-time task acceleration tolerance is 1e−10 m/s² or rad/s². The finite-step test uses a dedicated **1 microsecond step** and tolerances 2e−3 for arbitrary task acceleration, 1e−4 for the null case. This does not alter the production 1 ms contact model. `implicitfast` advances velocity using implicit damping, and the velocity difference also samples a changed Jacobian; their discrepancy from the initial instantaneous acceleration is **first order in the step**, unlike the roundoff-scale algebraic residual. The observed full-sample maxima below are substantially smaller than these absolute tolerances. Exact identities use `damping=0`; the default continuous lambda=0.01 is separately checked with an acceleration tolerance 1.01e−4. Its expected regularization defect is exactly `lambda² F`, verified independently to a 1e−10 residual. The near-singular directions are excluded from exact inverse tests because their inverses amplify floating-point error; damping bounds the gain but intentionally permits measurable null leakage. See [MuJoCo integration](https://mujoco.readthedocs.io/en/stable/computation/#numerical-integration).

The generated tables contain every error metric, sample count and tolerance from the JSON, together with all contact and singularity diagnostics (rounded for display; the JSON retains full precision). `N/A` is used only for diagnostics that are not error thresholds.

<!-- BEGIN GENERATED VERIFICATION -->

### Recorded versions and sampling

| Quantity | Value |
| --- | --- |
| MuJoCo / NumPy | 4.0.1 / 2.4.6 |
| Random seed / samples | 6223 / 1000 |
| Task seed / accepted samples | 6224 / 1000 |
| Task rejected ill-conditioned states / condition threshold | 29 / 1.000000000e+05 |
| Task integration step, s | 1.000000000e-06 |
| Minimum total-mass eigenvalue, kg m^2 | 1.035920965e-01 |
| Mass positive definite / all checks passed | True / True |

### Random-state model errors

| Metric | Max | Mean | N | Tolerance | Unit |
| --- | ---: | ---: | ---: | ---: | --- |
| `fk_position` | 1.347218015e-15 | 3.794073388e-16 | 1000 | 1.000000000e-10 | m |
| `fk_orientation` | 1.032538457e-15 | 3.676507256e-16 | 1000 | 1.000000000e-10 | rad |
| `flange_position` | 1.128119899e-15 | 3.518418422e-16 | 1000 | 1.000000000e-10 | m |
| `flange_orientation` | 1.032538457e-15 | 3.676507256e-16 | 1000 | 1.000000000e-10 | rad |
| `jacobian_mujoco` | 4.107825191e-15 | 1.441268545e-15 | 1000 | 1.000000000e-10 | mixed SI |
| `jacobian_finite_difference` | 3.401682269e-10 | 1.415576877e-10 | 1000 | 1.000000000e-08 | mixed SI |
| `jacobian_dot` | 1.887379142e-15 | 5.699061015e-16 | 1000 | 1.000000000e-10 | mixed SI/s |
| `mass_relative` | 3.035245858e-15 | 1.089909411e-15 | 1000 | 1.000000000e-10 | relative Frobenius |
| `mass_symmetry` | 4.440892099e-16 | 8.536791066e-17 | 1000 | 1.000000000e-12 | kg m^2 |
| `gravity` | 6.394884622e-14 | 1.823563522e-14 | 1000 | 1.000000000e-10 | N m |
| `coriolis` | 2.460878723e-14 | 4.122090493e-15 | 1000 | 1.000000000e-10 | N m |
| `forward_eom` | 2.629008122e-13 | 4.427258560e-14 | 1000 | 1.000000000e-10 | N m |
| `inverse_eom` | 6.394884622e-14 | 1.774508318e-14 | 1000 | 1.000000000e-10 | N m |

### Operational-space errors

| Metric | Max | Mean | N | Tolerance | Unit |
| --- | ---: | ---: | ---: | ---: | --- |
| `regularized_acceleration` | 9.997111859e-05 | 7.508442422e-05 | 1000 | 1.010000000e-04 | m/s^2 or rad/s^2 |
| `regularized_identity` | 3.868856769e-12 | 9.864526710e-14 | 1000 | 1.000000000e-10 | mixed SI |
| `task_acceleration` | 2.930988785e-12 | 1.071459943e-13 | 1000 | 1.000000000e-10 | m/s^2 or rad/s^2 |
| `task_acceleration_fd` | 1.054872471e-04 | 3.502100215e-05 | 1000 | 2.000000000e-03 | m/s^2 or rad/s^2 |
| `task_with_damping` | 2.916777930e-12 | 1.074153093e-13 | 1000 | 1.000000000e-10 | m/s^2 or rad/s^2 |
| `null_acceleration` | 2.822198947e-13 | 2.512508842e-14 | 1000 | 1.000000000e-10 | m/s^2 or rad/s^2 |
| `null_acceleration_fd` | 5.138710334e-06 | 9.668722836e-07 | 1000 | 1.000000000e-04 | m/s^2 or rad/s^2 |
| `null_algebra` | 1.640216585e-12 | 4.705484260e-14 | 1000 | 1.000000000e-10 | mixed SI |
| `env_task_terms` | 7.314697216e-13 | 2.604492347e-14 | 1000 | 1.000000000e-10 | relative norm |

### Contact diagnostics

| Quantity | Rim | Inside bottom |
| --- | ---: | ---: |
| `applied_downward_load_N` | 5.000000000e+00 | 5.000000000e+00 |
| `n_steps` | 3000 | 3000 |
| `steady_samples` | 500 | 500 |
| `duration_s` | 3.000000000e+00 | 3.000000000e+00 |
| `steady_duration_s` | 5.000000000e-01 | 5.000000000e-01 |
| `max_penetration_m` | 1.224599018e-07 | 1.440503938e-07 |
| `mean_penetration_m` | 1.224597472e-07 | 1.440503307e-07 |
| `max_transient_penetration_m` | 3.523861266e-05 | 3.056481616e-05 |
| `penetration_tolerance_m` | 1.000000000e-04 | 1.000000000e-04 |
| `mean_vertical_force_N` | 5.000000112e+00 | 4.999999904e+00 |
| `max_vertical_force_error_N` | 1.241319323e-07 | 1.073637437e-07 |
| `mean_vertical_force_error_N` | 1.118146845e-07 | 9.559784817e-08 |
| `force_tolerance_N` | 5.000000000e-02 | 5.000000000e-02 |
| `max_individual_normal_force_N` | 1.062683679e+00 | 1.250041874e+00 |
| `max_steady_tcp_speed` | 2.224852051e-07 | 1.556871510e-06 |
| `passed` | True | True |

### Singularity diagnostics

| Quantity | Value |
| --- | --- |
| `q` | 0.000000000e+00, 0.000000000e+00, 0.000000000e+00, -4.670024237e-01, 0.000000000e+00, 4.670024237e-01, 0.000000000e+00 |
| `jacobian_singular_values` | 1.990895262e+00, 1.795345986e+00, 4.953866584e-01, 3.981758907e-01, 2.166276590e-01, 1.236616849e-16 |
| `rank` | 5 |
| `damping` | 1.000000000e-02 |
| `smallest_task_inverse_eigenvalue` | 2.403727038e-16 |
| `lambda_spectral_norm` | 1.000000000e+04 |
| `lambda_bound` | 1.000000000e+04 |
| `damped_null_defect` | 4.633869198e-04 |
| `all_finite` | True |
| `env_all_finite` | True |
| `passed` | True |

### Free-hand calibration

| Speed (m/s) | Peak (N) | Elastic target (N) | Relative error | Tolerance |
| ---: | ---: | ---: | ---: | ---: |
| 0.10 | 2.129610140e+01 | 2.121320344e+01 | 3.907847424e-03 | 0.02 |
| 0.25 | 5.324264519e+01 | 5.303300859e+01 | 3.952945683e-03 | 0.02 |
| 0.50 | 1.064853837e+02 | 1.060660172e+02 | 3.953825251e-03 | 0.02 |

### Misalignment capture

| Case | Depth (mm) | Peak resultant (N) | Outcome |
| --- | ---: | ---: | --- |
| lateral_0mm | 40.000023 | 21.005939 | inserted |
| lateral_0.5mm | 40.000023 | 22.736289 | inserted |
| lateral_1mm | 40.000023 | 116.425743 | inserted |
| lateral_2mm | 40.000017 | 108.962567 | inserted |
| tilt_1deg | 39.740936 | 22.677033 | inserted |
| tilt_2deg | 34.687379 | 48.001482 | partial |
| yaw_1deg | 40.000023 | 20.996740 | inserted |
| yaw_2deg | 39.523742 | 71.129495 | inserted |
| sharp_lateral_1mm | 0.086782 | 90.651618 | jammed |

<!-- END GENERATED VERIFICATION -->

## 9. Model limitations and scope

- Matching MuJoCo confirms consistency with this model, not accuracy of a particular physical Panda; Menagerie inertias, dimensions and nominal joint limits are accepted as supplied, without physical identification or structural flexibility.
- The peg and fixture are perfect rigid square boxes with known alignment and fixed dimensions. Surface roughness, chamfers, manufacturing tolerances, fixture compliance and thermal variation are absent. Nominal insertion success is not a robustness proof.
- Contact forces depend on contact discretization, integration, penetration and solver parameters. The verified 5 N resting cases do not validate high-speed impacts or maximum human forces; the peak individual normal and summed spatial force are different measurements.
- The human capsule and its effective mass/weld compliance are study assumptions. Neither hand anatomy nor human injury thresholds are established by this numerical verification.
- Motors are ideal torque actuators with clipping and reflected armature. Hardware servo delays, motor electrical dynamics, torque slew limits, nonlinear friction, calibration errors and real FCI filtering are not simulated. Passive linear damping is present and must be included or compensated when comparing accelerations.
- Wrist compensation uses ideal simulator acceleration, tool mass/COM/inertia and known gravity. Configurable bias/noise model sensor faults, but not every mechanical/electronic failure or filtering effect. Upstream contacts are visible only in generalized contact truth, not wrist F/T.
- Exact task-space inverse dynamics and null decoupling require full row rank, correct parameters and unsaturated torques. Damping is a numerical regularization, not exact inversion or exact null projection. World angular velocity is used directly; no global three-parameter orientation chart is assumed.
- The physical hand adds state coordinates but not a smooth mass coupling to the arm. Arm-only model terms exclude equality constraints and external-contact dynamics; those appear through generalized contact forces. Random smooth tests deliberately disable constraints and do not assert collision-free reachability across the whole joint box.

All requested review revisions are implemented, except m9 as directed by the coordinator (the protected parameter-audit file is handled elsewhere). The review found no incorrect physical interface numbers; the revisions make projector names, measurement semantics, hand modes, capture range and saturation/damping explicit. The 2-degree tilt case is recorded as partial insertion rather than hidden as nominal success; arbitrary misalignment capture is not a promised criterion.
