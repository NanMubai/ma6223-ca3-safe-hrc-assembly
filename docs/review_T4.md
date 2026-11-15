# T4 — independent review of part C (simulation + modeling)

Date: 2026-11-15. Reviewer: independent worker (read-only on the repo; this file is the only write).

**Counts:** BLOCKER 0 · MAJOR 4 · MINOR 10 · NOTE 9

**Verdict in one line:** the numerics of the interface are correct and reproducible; the method signatures are safe to freeze for B and D now, for E after a one-line rename of the null-projector key (M1), and for A only after three small additive hand hooks (M2, M3).

## How this was checked

All reruns were done in a scratch copy of the repo (`rsync` without `.git`), because `scripts/verify_modeling.py` and `scripts/demo_insertion.py` write into `report/figures/`, `docs/modeling_notes.md` and `outputs/`. Checksums of those files in the real repo were identical before and after the review.

| Rerun | Result |
| --- | --- |
| `python -m pytest -q` | `41 passed in 9.22s` |
| `MUJOCO_GL=egl python scripts/verify_modeling.py` | `Overall passed: True`; JSON, `.tex` and `modeling_notes.md` regenerate byte-identical; `scene.png` does not (see m10) |
| `MUJOCO_GL=egl python scripts/demo_insertion.py --offscreen` | depth 40.000 mm, success, peak 26.490 N, peak individual normal 16.556 N; identical on a second run and on a fresh instance |

Five probe scripts (`p1.py` … `p5.py`) were written in the session temp dir; the numbers quoted below are their printed output. They are not in the repo; the snippets given are enough to reproduce each number.

---

## MAJOR

### M1 — The key `N` means opposite things in the two task-space APIs (trap for part E)

- `HRCSim.task_space_dynamics()['N']` is `I − J̄J` (`hrc_sim/env.py:215`): it projects **velocities**.
- `PandaDynamics.task_space_dynamics()['N']` and `dynamics.task_space_dynamics()['N']` are `I − JᵀJ̄ᵀ` (`hrc_sim/dynamics.py:156`): they project **torques**.
- `ReferenceController` uses a third one, the Euclidean `I − Jᵀ(JJᵀ)⁻¹J` (`hrc_sim/reference_controller.py:22`), which is not dynamically consistent.

Probe at home (`p1.py`):

```text
env N == dyn N ?  False     env N == dyn N.T ?  True     max|N − N.T| = 0.129
J M⁻¹ env.N   (env N misused as a torque projector) = 0.180
J M⁻¹ env.N.T                                        = 1.6e-15
TCP acceleration from a 1.5 N·m posture torque (tau = g + P·tau0):
  P = env N (misused)       0.0034 m/s²   0.0524 rad/s²
  P = env N.T (correct)     0.0000        0.0000
  P = dyn N                 0.0000        0.0000
  P = reference controller  0.0004        0.0063
```

Is it documented? Yes: `README.md:111` ("`N` projects velocities; use `N.T` to project torques"), the docstring at `env.py:201`, and `docs/modeling_notes.md:172`. So it is unambiguous for a careful reader. It is still a trap: E writes `tau = Jᵀ F + N tau0`, the same package returns the transposed matrix under the same key depending on which module is called, the misuse is silent (shapes match, the error is a small task disturbance that looks like tuning noise), and the only shipped example uses neither.

**Suggested fix (one convention):** keep Khatib's definition `N = I − J̄J` and never return a bare `N`. Both dicts return `N_vel = I − J̄J` and `N_tau = N_vel.T = I − JᵀJ̄ᵀ`; drop the `N` key (or keep it for one release as an alias of `N_vel` with a `DeprecationWarning`). Change `ReferenceController` to `tau = Jᵀ F + N_tau @ tau0` using `env.task_space_dynamics()` so the example shows the intended usage. Add a test asserting `J M⁻¹ N_tau ≈ 0` for the env dict (the existing env test at `tests/test_env.py` only checks `J @ N ≈ 0`, which passes for the velocity projector and says nothing about torques).

### M2 — Hand contact forces come from global solver parameters, not ISO/TS 15066; there is no per-hand contact parameter and no way to command a free transient impact

What exists: hand mass 0.6 kg, and two weld stiffnesses. What `param_check.md` flagged (no 75 N/mm) is correct, and the consequence is larger than "a missing number".

Measured (`p4.py`):

| Quantity | Measured | ISO/TS 15066 hand |
| --- | --- | --- |
| Hand capsule pressed on the fixed table, weld off, 10–300 N | 1381–1488 N/mm | 75 N/mm |
| `kinematic` weld static stiffness (force on hand vs displacement) | 2762–2775 N/mm | — |
| `compliant` weld static stiffness | 16 N/mm at 10 N, 37.5 N/mm at 50–140 N | — |
| Weld-driven hand, target moving at 0.25 m/s and continuing, hits peg side | peak 308 N (kinematic), 384 N (compliant); TCP pushed 200 mm | — |
| Same, hits link5 | peak 656 N / 585 N; TCP pushed 79–101 mm | — |
| Free-flying hand (weld deactivated by hand), 0.25 m/s, hits peg side | 58.0 N, pulse half-width 2 ms | two-body `v·sqrt(μk)` = 48.1 N with μ = 0.495 kg |
| Same on link5 | 39.8 N | — |
| Free-flying on peg at 0.1 / 0.5 m/s | 23.2 / 116.0 N | 21.2 / 106.1 N (upper bound, μ = 0.6 kg) |
| Free-flying on peg with hand geom `solref = (−75000, −100)` | 28.5 N | 48.1 N |

Reading of these numbers:

1. The contact is about 19× stiffer than the ISO hand, and the peak of a transient is set by the `solref` time constant (4 ms, critically damped), which is why the pulse is 2 ms wide. The free-flight peak lands within about 20% of the ISO two-body value only by coincidence of `solref = 0.004`; change `SimConfig.solref` for insertion reasons and A's peak forces move with it.
2. `_model_xml` overwrites friction/`solref`/`solimp` on **every** geom (`env.py:41`), so the hand cannot have its own contact parameters through config. A can only poke `sim.model.geom_solref[sim.hand_geom_id]` after construction.
3. A negative (direct) `solref` of −75000 does not give the ISO force either (28.5 N), so "just set 75 N/mm" is not a one-liner; it needs a calibration test.
4. There is no API for a free transient impact. `set_hand` zeroes the hand velocity on enable and has no velocity argument; I had to set `data.eq_active[hand_weld_id] = 0` and write `data.qvel[sim._hand_dadr:…]` (a private index). With the documented method (moving the target in small increments) the hand is a position source: if the target keeps moving, both modes bulldoze the arm and report 300–650 N. That is a legitimate quasi-static worst case, but it is not the ISO transient case, and the README does not say so.

**Minimal hooks A needs (all additive):**

- `HandConfig.contact_solref`, `contact_solimp` (and `solmix`, set high so the hand value dominates the pair); apply them to the `hand` geom after the global overwrite at `env.py:41`.
- `set_hand(..., velocity=None)` or `release_hand(velocity)` that deactivates the weld and sets the free-joint linear velocity, so a 0.6 kg free hand hits at a stated speed.
- A calibration test: free hand at 0.25 m/s onto the fixed table (μ = 0.6 kg) must give `v·sqrt(m·k)` = 53.0 N within a stated tolerance; tune `contact_solref` until it does. Record the calibrated values in `docs/modeling_notes.md`.
- One README paragraph: `kinematic` mode with a moving target is a prescribed-motion clamp (force unbounded by the hand), `compliant` is a 16–37 N/mm position spring, neither is the ISO tissue spring.

### M3 — `set_hand(enabled=True)` teleports the hand with no penetration guard; overlapping placement gives kilonewton forces and throws the arm

On first enable the hand free body is written straight to the target (`env.py:322`). If that pose overlaps the robot the solver resolves centimetres of penetration in one step.

Probe (`p4.py` (g), the exact placement used by `tests/test_env.py:95` and `:108`):

```text
enable inside link3: instant normal force 31989 N, tau_ext = [851.5, −7696.7, −219.7, 204.3, 0, 0, 0]
after 0.3 s of zero command: peak normal 88084 N, max |dq| = 49.54 rad/s, joints displaced up to 2.57 rad
```

The test comment says not to use this placement as an impact estimate, but nothing in the README or the code stops A from doing exactly this when "injecting a collision", and A's headline metric is peak contact force.

**Suggested fix:** in `set_hand`, after the `mj_forward` on a fresh enable, compute the deepest hand contact (`min(c.dist)` over contacts with `hand_geom_id`); raise `ValueError` (or warn and refuse) if it is deeper than about 1 mm, with an `allow_penetration=False` argument for the tests. Replace the two tests' placement with a surface-touching pose plus a few steps of approach.

### M4 — No lead-in chamfer: the capture range equals the 0.5 mm clearance, any larger error jams on the rim, and the nominal demo never touches the hole walls

Probe (`p3.py`; reference controller, approach to 20 mm above the hole with the stated error, 4 s quintic descent, 1.5 s hold):

| Case | Final depth | Peak ΣF | Peak single normal | Final force | Max penetration | Outcome |
| --- | --- | --- | --- | --- | --- | --- |
| nominal | 40.00 mm | 19.7 N | 11.7 N | 0.8 N | 0.004 mm | inserted |
| lateral 0.3 mm | 40.00 | 19.7 | 11.7 | 0.8 | 0.004 | inserted |
| lateral 0.5 mm | 40.00 | 23.0 | 22.1 | 0.8 | 0.005 | inserted |
| lateral 1.0 mm | 0.09 | 92.9 | 25.9 | 32.7 | 0.037 | **jam on rim** |
| lateral 2.0 mm | 0.08 | 82.5 | 42.3 | 32.7 | 0.016 | **jam on rim** |
| lateral 1 mm, lateral K = 200 N/m | 0.09 | 84.2 | 23.2 | 32.7 | 0.017 | **jam** (compliance does not help) |
| tilt 1° | 39.74 | 22.6 | 19.9 | 1.0 | 0.004 | inserted |
| tilt 2° | 32.85 | 54.1 | 87.0 | 6.6 | 0.003 | **wedged** at 33 mm, 2 mN force chatter |
| tilt 2°, rotational K = 5 N·m/rad | 39.63 | 94.1 | 136.1 | 1.2 | 0.006 | inserted |
| yaw 1° about peg axis | 40.00 | 19.7 | 11.2 | 0.8 | 0.004 | inserted |
| yaw 2° | 0.18 | 108.5 | 94.2 | 32.7 | 0.033 | **jam** (30(cos 2° + sin 2°) = 31.03 mm > 31 mm) |
| 1 mm + 1° | −0.17 | 92.5 | 47.5 | 32.9 | 0.037 | **jam** |
| 2 mm + 2° | −0.43 | 83.7 | 42.9 | 33.1 | 0.016 | **jam** |
| lateral 2 mm, 0.5 s descent | 40.00 | 375.6 | 205.7 | 2.0 | 0.128 | inserted after rim strike |
| tilt 2° + 60 N extra push | 0.57 | 1265.8 | 1212.4 | 92.5 | 0.300 | jam, no tunnelling |

Nothing exploded, produced NaN or tunnelled (worst penetration 0.34 mm at about 1 kN), and the wrist estimate matched contact truth to 1e-12 N throughout. The contact model is stable. The problem is the task geometry:

- Sharp square peg on a sharp flat rim has no mechanism that turns lateral compliance into alignment, so lowering stiffness cannot rescue a 1 mm error (row 6). Part E's variable impedance has nothing to exploit in translation; only rotational compliance helps (tilt rows).
- D's demonstrations and GMR reproduction must stay inside ±0.5 mm and under 2° yaw or they fail at the rim with a steady 32.7 N.
- In the shipped demo the first peg–fixture contact is the bottom, at t = 5.921 s; the walls are never touched (`p2.py`). The demo and `test_reference_controller_insertion_succeeds` therefore do not exercise contact-rich insertion at all.

This is a scene property, not an interface defect, and the README does say the demo is "not robustness to arbitrary alignment errors". It still decides whether the project's claim can be shown.

**Suggested fix:** add a configurable chamfer (`SimConfig.chamfer`, e.g. 2–3 mm at 45°, four tilted thin boxes on the rim or a bevelled peg tip built from a small box stack) and a misalignment regression test (1 mm lateral must insert with the chamfer). If the team prefers no chamfer, state the ±0.5 mm / <2° capture range in the README and report as a hard requirement for D and E, and consider `clearance = 1–2 mm` (2 mm clearance inserts cleanly with 1 mm error, last probe row: 19.7 N).

---

## MINOR

### m1 — Passive joint damping (1 N·m·s/rad per joint) is neither compensated nor in `mu`; the env has no option or accessor for it

Docs and code agree with each other (`README.md:107`, `modeling_notes.md` §4–5, `dynamics.py` `include_joint_damping`), so this is consistent. But the env's `task_space_dynamics()` has no `include_joint_damping`, and the equation `Λa + μ + p = F` with env terms only holds if the controller adds `D dq`:

```text
residual of Λa + μ + p − F over 50 random states, env terms only:
  tau = Jᵀ F + D dq : 7.5e-12        tau = Jᵀ F : up to 55.9
equivalent task-space damping at home, diag(J̄ᵀ D J̄) = [19.6, 5.3, 11.0] N·s/m, [1.49, 2.31, 0.99] N·m·s/rad
constant 5 N world-y Cartesian force, no controller damping: apparent damping 24.0 N·s/m
```

So E's low-damping phases have a floor of roughly 5–20 N·s/m that is not in the commanded impedance. The README calls the torque path "FCI convention"; from memory of the libfranka documentation the real FCI command excludes gravity **and friction**, which I did not re-fetch. **Fix:** add `SimConfig.damping_compensation=False` (adds `D dq` before the clip) or `task_space_dynamics(include_joint_damping=False)` on the env, expose `sim.joint_damping`, and state the 5–20 N·s/m floor in the README.

### m2 — `_jacobian_dot()` is private although E needs `J̇ q̇`

`env.py:167`. It is numerically right (finite difference of `J` along `q + dq·h`: 1.4e-9). **Fix:** make it public as `jacobian_dot()` and add `Jdot_dq` to the task-space dict.

### m3 — The env's singularity handling is a hard switch with a near-useless damping value, and differs from `dynamics.py`

`env.py:206`: damping `1e-4` squared is added only when `cond(A) > 1e8`.

```text
env ‖Λ‖₂ at the exact singularity: 1.0e8
1e-3 rad away: cond(A) = 1.5e8 → ‖Λ‖₂ = 8.8e6       (dynamics.py takes an explicit damping argument instead)
```

Any controller multiplying by `Λ` near a singularity saturates all torques, and the terms jump when the threshold is crossed. **Fix:** always add `task_damping² I` (continuous), raise the default to about `1e-2`, and use the same signature in both modules.

### m4 — Velocity limits are not enforced and no limit/saturation status is exposed

Documented (`README.md:107`, `env.py:129`). Free sinusoidal torque probe reached `max|dq| = 9.94 rad/s` (4.6× the 2.175 limit). Torque clipping is correct but silent: after `step([0,200,0,0,0,0,0])`, `ctrl[1] = 87` and the caller is not told. A joint driven at 60 N·m overshoots its position limit by 4.7 mrad; the limit reaction (−60.07 N·m in `qfrc_constraint`) is not part of `tau_ext`, which is as documented.

**Fix (cheap, worth adding):**

```python
def limits_violated(self, margin=0.0):
    q, dq = self.get_joint_state()
    c = self.config
    return dict(position=(q < c.q_min + margin) | (q > c.q_max - margin),
                velocity=np.abs(dq) > c.dq_max,
                torque=np.abs(self.data.ctrl) >= c.tau_max - 1e-9)
```

B can assert it is all-false in the end-to-end run. Also store `self.last_tau_applied`.

### m5 — Sensor fault injection: reproducibility and realism gaps

- Noise is drawn on every `get_ext_wrench()` call (`env.py:248`), so one extra call by user code shifts the whole noise stream (confirmed).
- `reset()` without a seed creates an unseeded generator (`env.py:117`); two unseeded resets give different noise (confirmed). Only matters when `noise_std > 0`.
- Bias is constant in **world** axes at the TCP (`README.md:125`). A real sensor offset is constant in the sensor frame; with the world definition the "fault" rotates relative to the tool as the wrist turns.
- `tau_ext` is exact contact truth with no noise, bias or fault path, and it excludes `xfrc_applied`. A's "collision detection from external joint torque" would be detecting on a perfect signal.
- Runtime edits to `sim.config.sensor_bias` survive `reset()`.

Statistics are right: 20000 samples gave mean bias and std within 0.5% of the configured values; the TCP-frame option rotates bias correctly.

**Fix:** sample noise once per physics step and cache it; seed from a stored base seed when `seed is None`; add `sensor_bias_frame='world'|'sensor'`; add `tau_ext_bias` / `tau_ext_noise_std` (or a documented `get_ext_joint_torque(measured=True)`).

### m6 — Reported peak forces are single-step, solver-time-constant transients

The demo's "peak 26.490 N" occurs in the first step of bottom contact (t = 5.921 s, 2 contact points, Fxy = 7.6 N); the steady value is 0.81 N. A rim touchdown at about 29 mm/s gives 83–93 N. `get_max_normal_force` returns the largest single contact point and can exceed the resultant (87.0 N vs 54.1 N when wedged). **Fix:** have B define the requirement metric explicitly (resultant vs single normal; raw vs low-pass filtered at a stated cutoff), and add a `get_contact_normal_sum(group)` helper.

### m7 — Test quality

- Plain `pytest -q` fails at collection: `ModuleNotFoundError: No module named 'scripts'` (`tests/conftest.py:3`, `tests/test_env.py:11`). Only `python -m pytest` from the repo root works. **Fix:** add `pythonpath = ["."]` under `[tool.pytest.ini_options]`.
- Assertions that cannot fail: `n_samples >= 1000` (`test_kinematics.py:11,18`, `test_dynamics.py:11,20,29`) check a constant passed by the fixture.
- `test_extracted_model_calculation_never_calls_mujoco` (`test_dynamics.py:69`) patches attributes on the `mujoco` module; it would not catch `from mujoco import mj_jac`. It passes today because `dynamics.py` does not import MuJoCo at all; assert that directly on the module source or namespace.
- Tolerances are 1e-6 against observed 1e-15: a regression of nine orders of magnitude would pass.
- `0 < peak_force < 100` (`test_env.py:206`) is loose around 26.5 N.
- Missing: wrist estimate ≈ 0 during real accelerated motion (the test only sets `qvel`), any misalignment case, torque-projector use of the env's `N`, hand placement that is not a deep penetration.

### m8 — README is stale after T3

The layout table (`README.md:21–33`) lists only `tests/test_env.py` and omits `hrc_sim/kinematics.py`, `hrc_sim/dynamics.py`, `scripts/verify_modeling.py`, `tests/test_kinematics.py`, `tests/test_dynamics.py` and `docs/`. No README statement about frames, signs or shapes was found to be wrong.

### m9 — `param_check.md` wrongly says the live Franka docs are FR3-only; its Panda numbers are nevertheless trustworthy

I re-fetched every source it names:

| Source | HTTP | Finding |
| --- | --- | --- |
| `frankaemika.github.io/docs/control_parameters.html` | 404 | as stated |
| Wayback snapshot 2023-02-07 of that page | 200 | contains "Limits for Panda"; all seven rows match the quotes exactly |
| `frankarobotics/franka_ros` `robots/panda/joint_limits.yaml` (the README's link) | 200 | all seven joints match `config.py` exactly |
| `frankarobotics.github.io/docs/robot_specifications.html` (live) | 200 | **contains a table "Limits for Franka Emika Robot (FER)"** with q, dq, ddq, dddq, τ, τ̇ rows identical to the archived Panda table |
| `inertial.yaml` | 200 | header and link masses as quoted |

So `param_check.md:15`, `:56`, `:172` and `:247` ("FR3-only", "no Panda table") are wrong: the agent searched for the word "Panda", and the live page calls the same robot FER. `docs/modeling_notes.md:27` is right on this point. Only the DH table on the live page is labelled FR3. **Fix:** correct those four lines and cite the live FER table as the primary source, with the archive and the YAML as corroboration.

### m10 — Report artefacts need small changes for a single IEEE column

- `modeling_verification_table.tex`: six data rows (meets ≤6). Typeset in `IEEEtran` conference class it is **277.7 pt wide against a 252.0 pt column** at normal size (overfull by 25.7 pt); at `\footnotesize` it is 229.4 pt and fits. The Jacobian row has no units; "e-15" style is not typeset as math. **Fix:** emit `\footnotesize`, use `$1.35\times10^{-15}$` or `siunitx`, shorten "MuJoCo (relative)" to "MuJoCo (rel.)".
- `scene.png` (viewed): 1600×1200, clean white background, world and TCP triads present with legible "World W" and "TCP T" labels. Content occupies only 36% of the canvas (bounding box x 277–1082, y 18–880); the right third and bottom quarter are empty, so at 3.5 in the robot is small. The TCP y arrow is foreshortened to a stub and the TCP x label sits on the wrist housing. Peg, fixture and hole are not labelled. **Fix:** crop to the bounding box plus margin (or move the camera closer), nudge the camera azimuth so TCP y is visible, add "peg" and "hole" labels.
- Rerunning the script changes `scene.png` (2.7% of pixels differ, max channel difference 93), so the figure is not byte-reproducible while the numeric artefacts are.

---

## NOTE (checked and found correct, or informational)

**n1 — Independence is real (item 1).** `kinematics.py` and `dynamics.py` contain no `import mujoco` and no `mj_*` call; the only MuJoCo contact is `from_mujoco_model`, which reads `body_mass`, `ipos`, `iquat`, `inertia`, attachment/TCP transforms, armature, damping and gravity once. `PandaKinematics()` used for the FK check does not even read those: the DH table is hard-coded. Errors near 1e-15 are what two double-precision evaluations of the same closed-form chain give. Checks that involve no MuJoCo at all: `h(q,dq)` against Christoffel symbols from finite differences of the NumPy `M(q)`: 2.1e-10; `g(q)` against finite differences of the NumPy potential energy: 8.2e-9. Caveat: inertial constants are shared, so this verifies the algorithms, not the parameters; a wrong inertia in the XML would pass. The report should say "independent implementation", not "independent model".

**n2 — Torque path (item 2).** `ctrl = clip(tau + g(q), ±tau_max)` at `env.py:138`, gravity recomputed each substep from scratch data with zero velocity; it includes the tool (1 kg peg changes g by [0, −3.92, 0, 3.74, 0, 0.69, 0] N·m and still holds with 0.0 m drift over 2 s). Saturation applies after adding gravity. Armature is included in both mass matrices and not double counted.

**n3 — `get_ext_wrench` (item 3).** Sign, reference point and frame option are right. Free accelerated motion with TCP accelerations up to 114 m/s² and joint speeds up to 9.9 rad/s reads `max|wrench| = 4.1e-14`. During the full insertion `max|ft − contact truth| = 2.5e-14`; with the hand hitting the peg, 6.8e-14. A +7 N world-x force applied to the moving tool reads +7.000 N. Compensation uses the simulator's exact `qacc`; a real sensor would not have it, as the docs say.

**n4 — `get_ext_joint_torque` (item 4).** For peg contacts `max|tau_ext − Jᵀ w_contact| = 1.8e-15`. For a moving hand on link5 (513 contact steps) `max|tau_ext − qfrc_constraint| = 2.8e-14` while the wrist reads 3e-15. Sign is consistent with `τ_ext = Jᵀ F_ext`.

**n5 — Jacobian, twist, quaternion, μ, p (item 6).** `J dq` against a finite-difference pose twist `[v; ω]` in world at the TCP: 5.2e-10. `(w,x,y,z)` → `R` round trip: 3e-15. With `D dq` compensated, `Λa + μ + p − F` using env terms only: 7.5e-12. Home orientation is a 180° rotation, so `ee_quat = (0, 0, 1, 0)` with `w = 0`; no sign flip occurred in a 2 s random wiggle, but D should still enforce hemisphere continuity before fitting a GMM on quaternions.

**n6 — Parked hand (item 7).** Running the demo with the weld deactivated gives bitwise identical joint results and peak force; the hand stays at (2, 0, 1). Tracking a 0.25 m/s target: lag 0.75 mm (kinematic), 19.7 mm (compliant). A quasi-static clamp of the hand between peg and fixture settles at 63.95 N, equal to the commanded `K_z·error` = 64.0 N, and the wrist reads the same.

**n7 — Determinism (item 10).** Demo results are identical across repeated runs in one instance and across fresh instances; seeded resets reproduce noise.

**n8 — Performance.** `step()` runs at about 12.4 kHz (12× real time) against 184 kHz for raw `mj_step`: each step does an extra full `mj_forward` for gravity, a trailing `mj_forward`, and the observation. Fine for this project; if D needs many rollouts, compute gravity with `mj_rne` on the scratch data instead.

**n9 — Armature dominates rotational task inertia.** At home `Λ` about TCP z is 0.104 kg·m² with armature and 0.005 without; translational entries rise 20–25%. E should expect rotational dynamics set by the 0.1 kg·m² armature rather than by link inertia.

---

## Verdict: is the interface safe to freeze?

| Consumer | Freeze? | Condition |
| --- | --- | --- |
| **D** (demos, GMM/GMR) | Yes | Observation dict, pose, twist and quaternion conventions are correct and reproducible. D must keep demonstrations within ±0.5 mm / <2° yaw unless M4 is fixed. |
| **B** (end-to-end, requirements) | Yes | Add `limits_violated()` (m4) and fix the peak-force metric definition (m6) before writing requirement checks; both are additive. |
| **E** (variable impedance) | After M1 | Rename `N` to `N_vel` / `N_tau` in both modules first; it is the one change that alters an existing return value, so do it before E writes code. m1–m3 are additive and can follow. |
| **A** (hand collisions, faults) | Not yet | Needs the M2 hooks (per-hand contact parameters, free-flight release, ISO calibration test) and the M3 penetration guard; without them the peak forces A reports are determined by `solref = 0.004` and by placement accidents. m5 for the fault model. All are additive. |

No BLOCKER was found: nothing in the existing interface returns a wrong number. Every recommended change except M1 is additive, so the existing signatures can be frozen today provided M1 is applied first.
