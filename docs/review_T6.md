# T6 — review of report section IV-A and re-verification of the T4 fixes

Date: 2026-11-15. Read-only on the repo except this file; no git commit. All reruns and the LaTeX build were done in scratch copies; checksums of `README.md`, `hrc_sim/`, `assets/*.xml`, `scripts/`, `tests/`, `docs/`, `outputs/`, `report/figures/`, `report/sections/`, `report/main.tex` and `report/refs.bib` were identical before and after.

**Summary**

- Part (a), `report/sections/04a_system_modeling.tex`: 6 items WRONG, 2 MARGINAL, the rest OK. The most important is that eq. (3) carries a gravity term `p` that the stated torque path cancels.
- Part (b), T4 fixes: M1–M4 and m1–m8 FIXED; m10 PARTIAL (the figure is not byte-reproducible here and is included too small).
- The README interface can be frozen for A, B, D and E.

---

## Part (a) — Section IV-A

### Build and budget

Built with `latexmk -pdf` from a clean copy of `report/` (no stale aux files).

| Check | Result | Evidence |
| --- | --- | --- |
| Build | OK | `latexmk exit 0`, 2 pages |
| Undefined references / citations | OK | no `undefined` or `multiply defined` warning in `main.log` |
| Overfull boxes | OK | none |
| Underfull boxes in IV-A | **MARGINAL** | three `Underfull \hbox (badness 10000)` at source lines 23–25 (the `τ_max` sentence); the stretched word gaps are visible on the page |
| Words (`texcount`) | OK | 215 in text, 2 in headers, 16 in captions; `-sum` gives 291 because it counts each of the 54 inline formulas as one word. Budget 350 |
| Numbered equations | OK | 4 (budget 5) |
| Floats | OK | 1 figure, 1 table |
| Column space | OK, 26 pt spare | page 1 left column 139 pt + page 1 right column 500 pt + page 2 left column 176 pt = 814 pt; column height 672 pt, so **1.21 columns** (budget 1.25 = 840 pt). The file header says "0.6 pages" = 1.20 columns |
| Table width | OK | 212.98 pt in a 252 pt column |

### Checklist: equations, numbers, claims, citations

Line numbers are those of `04a_system_modeling.tex`.

| # | Line | Item | Verdict | Evidence |
| --- | --- | --- | --- | --- |
| 1 | 6 | "box-in-box assembly cell `\cite{kana2023boxinbox}`" | OK (wording) | key present; Kana et al., Sensors 2023, box-in-box insertion. The cell here is our own simplified design, so "inspired by" is more accurate than "we model the cell [4]" |
| 2 | 6 | MuJoCo, 1 ms step | OK | `config.py` `dt = 0.001`; `todorov2012mujoco` present |
| 3 | 6 | 7-DoF torque-controlled Panda from Menagerie | OK | `panda_torque.xml` has seven `motor` actuators; `menagerie2022` present with commit `370c439…` |
| 4 | 6 | `\cite{frankadocs}` | **WRONG** | `refs.bib` gives `https://frankaemika.github.io/docs/` with "Accessed: 2026-11-15"; that URL returns HTTP 404 today. `https://frankarobotics.github.io/docs/` returns 200 and is what `README.md` and `param_check.md` now cite |
| 5 | 6 | peg 30×30×60 mm, 0.2 kg | OK | `peg_half_sizes=(0.015,0.015,0.030)`, `peg_mass=0.2` |
| 6 | 6 | hole 40 mm deep, 0.5 mm per-side clearance, 3 mm 45° chamfer | OK | `hole_depth=0.040`, `clearance=0.0005`, `chamfer=0.003`; chamfer boxes rotated ±π/4 in `env.py` |
| 7 | 6 | world frame at base, z up; TCP at peg end-face centre | OK | README "Frames, units and conventions" |
| 8 | 6 | FK from the modified DH chain | OK | `kinematics.py` Craig DH; values match the FER table |
| 9 | 8 | eq. (1) `ẋ = [v; ω] = J(q) q̇`, world axes, 6×7 | OK | probe: `J dq` vs finite-difference pose twist 5.2e-10 (T4 n5, unchanged code) |
| 10 | 19 | `I_a = 0.1 I_7 kg·m²`, `D = 1.0 I_7 N·m·s/rad` | OK | `dof_armature = 0.1`, `joint_damping = [1,1,1,1,1,1,1]` |
| 11 | 19 | `τ_ext = Jᵀ F_ext` given as the definition of `τ_ext` | **WRONG** | true only for a wrench at the TCP. Line 36 then says link collisions give `τ_ext` via contact Jacobians, which contradicts it. `modeling_notes.md:151`: "For a pure TCP wrench, `tau_ext=J.T F_ext`; upstream link contacts require the full contact Jacobian sum instead"; `:190` gives the general `F_ext = J̄ᵀ τ_ext` |
| 12 | 21 | eq. (2) terms and signs | OK | matches `modeling_notes.md` boxed equation and `inverse_dynamics`; forward/inverse EOM residuals 2.63e-13 / 6.39e-14 N·m |
| 13 | 24 | `τ = clip(τ_cmd + g(q), ±τ_max)`, `τ_max = (87,87,87,87,12,12,12)` | OK | `env.py:167` with `damping_compensation=False` default; `tau_max` in `config.py` |
| 14 | 24–26 | eq. (3) `Λẍ + μ + p = F + F_ext` with **`τ_cmd`** `= Jᵀ F + N_τ τ_0` | **WRONG** | with the torque path of item 13 the environment adds `g(q)`, which cancels `g` in eq. (2), so `p` must not appear. Probe through the `step()` torque path, 100 random states, `damping=0`, `include_joint_damping=True`: residual of `Λa + μ + p − F` = **429**; residual of `Λa + μ − F` = **2.4e-12**. At home `p_z` = 47.1 N. `modeling_notes.md:174` writes the same law for the **actuator** torque `τ`, where `p` is correct; the report changed `τ` to `τ_cmd` |
| 15 | 26 | `μ = μ_rb + μ_D` | OK | passive damping is simulated and not compensated by default; `task_with_damping` 2.917e-12 |
| 16 | 31 | eq. (4) `Λ`, `J̄`, `N_τ = I − Jᵀ J̄ᵀ` | OK | `dynamics.py` `task_space_dynamics`; probe with `damping=0`: `J M⁻¹ N_tau` = 1.6e-15, `N_tau == N_vel.T` |
| 17 | 34 | `μ_rb`, `μ_D`, `p` formulas | OK | same function; `env_task_terms` 7.3e-13 |
| 18 | 34 | "continuous damping (λ = 0.01 at all q)" and `Λ_λ`, `J̄_λ`, `N_τ,λ` | OK | `task_damping = 1e-2`, added unconditionally; `‖Λ‖₂` = 1.0e4 at the singular pose (bound `1/λ²`) |
| 19 | 36 | F/T gives `F_ext` at {T} after tool gravity/inertia compensation | OK | free accelerated motion reads ≤4e-14; equals contact truth to 2.5e-14 during insertion. "At {T}" is ambiguous: the reference point is the TCP but the default axes are world |
| 20 | 36 | contact parameters condim 3, friction 0.3, solref (0.004,1), solimp (0.95,0.99,0.001) | OK | `config.py`; applies to all geoms except the hand |
| 21 | 36 | "0.6 kg hand body is calibrated to the ISO/TS 15066 stiffness of 75 N/mm" | **WRONG (imprecise)** | what is calibrated is the **peak force of a free impact** against `v·sqrt(m k)`: 21.30 / 53.24 / 106.49 N at 0.1 / 0.25 / 0.5 m/s, within 0.4% (JSON `hand_impacts`). The contact is not a 75 N/mm spring: static hand-on-table stiffness measured 201–320 N/mm for 10–140 N. README says the coefficients "are not literal N/m material constants" |
| 22 | 36 | `\cite{iso15066}`, `Section~\ref{sec:safety}` | OK | key present; reference resolves |
| 23 | 36 | independent NumPy FK, J, M, g match to machine precision over 1000 random configurations | OK | JSON `random`: n = 1000; 1.35e-15 m, 1.03e-15 rad, 4.11e-15, 3.04e-15 (relative), 6.39e-14 N·m |
| 24 | 36 | "(3)–(4) reproduce simulated TCP accelerations within 2.93×10⁻¹²" | **WRONG (number and scope)** | 2.931e-12 is JSON `task_acceleration`, the variant with `D q̇` cancelled at the actuator and `μ = μ_rb`. Eq. (3) as written (`μ = μ_rb + μ_D`) is `task_with_damping` = **2.92e-12**. Both hold only for λ = 0 on states with `cond(A) < 1e5` (29 states rejected). With the λ = 0.01 stated two sentences earlier the error is 1.0e-4 (`regularized_acceleration` 9.997e-5) |
| 25 | 40 | Table caption "max error over 1000 random configurations" | OK | N = 1000 for all six rows. Two small inaccuracies: the second Jacobian number is against finite differences, not MuJoCo, and the EOM row uses random states (q, q̇, q̈), not only configurations |
| 26 | 42 | table values | OK | match JSON to the printed digits: 1.35e-15, 1.03e-15, 4.11e-15 / 3.40e-10, 3.04e-15, 6.39e-14, 2.63e-13 / 6.39e-14 |
| 27 | 24, 29 | `\cite{khatib1987unified}` | OK | key present; the right source for eq. (3)–(4) |
| 28 | — | every symbol defined once and used consistently | **WRONG** | `F` (commanded task wrench) and `τ_0` (null-space torque) are never defined in words; `x` is never defined although `ẋ`, `ẍ` are used; three names for inertia (`M_rb`, `𝓜`, and `M` "in Table I"); identity written `I_7`, `I_6` and bare `I` in eq. (4); the sign convention of `F_ext` (environment on robot) is not stated |
| 29 | 14 | figure legibility at printed size | **MARGINAL** | included at `0.45\columnwidth` = 113 pt (40 mm) wide. From the PNG, label cap height is about 60 px of 1600, i.e. about 4.2 pt cap height (roughly a 6 pt font) in print; axis letters about 5–6 pt; the TCP x and y arrows are about 1.5 mm long and hard to tell apart. Readable on screen, below the usual 8 pt figure-text guidance. The "TCP T" label sits closer to the fixture than to the TCP triad |

### English and clarity

| Line | Text | Problem | Suggested wording |
| --- | --- | --- | --- |
| 6 | "A rigid … peg … on the flange with a wrist force/torque (F/T) sensor aligns with a fixed … hole" | "aligns with" is not the task; the sentence reads as if the peg carries the sensor | "A rigid … peg is mounted on the flange behind a wrist force/torque (F/T) sensor and is inserted into a fixed …" |
| 6 | "In SI units, the world frame {W} is at the base" | dangling modifier | "All quantities are in SI units. The world frame {W} is at the robot base …" |
| 6 | "(full derivations and parameters are in the project repository documentation)" | vague pointer | cite the repository once, or drop |
| 19 | "total inertia 𝓜(q) = M_rb(q) + I_a (M in Table I)" | parenthesis is cryptic | use `𝓜` in the table header and delete the parenthesis |
| 24 | "where motor torque adds gravity compensation to controller torque τ_cmd and saturates at …" | subject and verb do not fit; causes the three underfull lines | "where the motor torque is the controller torque τ_cmd plus gravity compensation, saturated at τ_max:" then the clip formula |
| 29 | "torque null-space projector N_τ for the redundant DoF" | "for the redundant DoF" is loose | "the dynamically consistent null-space torque projector N_τ" |
| 36 | "while link collisions give τ_ext via contact Jacobians" | reads as a measurement; it is simulator ground truth | "the simulator also provides the external joint torque τ_ext, summed over all robot contacts through their contact Jacobians" |

---

## Part (b) — T4 findings after the Codex fix

### Reruns

| Command | Result |
| --- | --- |
| `pytest -q` (plain, from repo root) | `52 passed in 35.41s`; collection works (`pythonpath = ["."]` in `pyproject.toml`) |
| `MUJOCO_GL=egl python scripts/verify_modeling.py` | `Overall passed: True`; all lines match `T4_fix_validation.md`; JSON, table and `modeling_notes.md` regenerate byte-identical; `scene.png` does not (see m10) |
| `MUJOCO_GL=egl python scripts/demo_insertion.py --offscreen` | nominal: 40.000 mm, 21.006 N; offset 1.5 mm: 40.000 mm, 117.875 N; both `inserted`; exit 0 |

### Per finding

| ID | Status | Evidence from my own probes |
| --- | --- | --- |
| **M1** `N` ambiguity | **FIXED** | keys are `Jbar, Jdot_dq, Lambda, N_tau, N_vel, mu, p`, no `N`, identical in `HRCSim` and `PandaDynamics`. `damping=0`: `J M⁻¹ N_tau` = 1.6e-15, `J N_vel` = 8.2e-16, TCP acceleration from a 1.5 N·m posture torque through `N_tau` = 1.4e-15 m/s². `J M⁻¹ N_vel` = 0.18, so the two are still distinct and now labelled. `ReferenceController` uses the env's `N_tau`. Tested at `tests/test_env.py:208` |
| **M2** hand contact / free impact | **FIXED** | `release_hand` + hand-only contact parameters. Free 0.6 kg hand onto the table: 21.63 / 54.08 / 108.04 N at 0.1 / 0.25 / 0.5 m/s against 21.21 / 53.03 / 106.07 N (+2.0% in my horizontal-capsule setup; the repo's own setup gives +0.4%), pulse half-width 6 ms (a 75 N/mm, 0.6 kg spring gives 5.9 ms). Changing the global `solref` to 0.01 leaves the peak at 54.08 N, so the hand is now decoupled from the insertion contact tuning. Onto the peg at 0.25 m/s: 45.6 N (two-body value 48.1 N); onto link5: 29.1 N. Caveat, documented in README: static stiffness is 201–320 N/mm, not 75 |
| **M3** penetration guard | **FIXED** | the T4 placement now raises `ValueError: Hand placement penetrates 83.121 mm…`; afterwards `hand_enabled=False`, `ncon=0`, hand at (2,0,1), and the arm does not move in 200 steps (`max|dq|` = 0). `allow_penetration=True` still gives 42 kN as an explicit opt-in. Residual, not part of M3: an already-enabled hand whose target is jumped 0.5 m into a link is dragged in by the kinematic weld and reaches 44 kN; README tells users to approach in small increments |
| **M4** chamfer / capture range | **FIXED** | with the default 3 mm chamfer: lateral 1, 2, 3 mm all reach 39.9–40.0 mm (peaks 107–116 N at chamfer touchdown); 4 mm jams at 0.09 mm, consistent with a 3 mm chamfer; 1 mm + 1° inserts; yaw 2° inserts (39.5 mm); tilt 2° stays partial at 34.7 mm (documented); `chamfer=0` with 1 mm still jams (control). Wrist estimate equals contact truth to 2.4e-13 in every case; worst penetration 0.035 mm |
| **m1** joint damping | **FIXED** | `joint_damping` property; `include_joint_damping=True` gives residual 1.3e-11 for `τ = Jᵀ F` (67 without it); `SimConfig.damping_compensation=True` adds `D q̇` before the clip; README states the 5–20 N·s/m floor |
| **m2** private `_jacobian_dot` | **FIXED** | public `jacobian_dot()`, `_jacobian_dot` removed, `Jdot_dq` key equals `jacobian_dot() @ dq` exactly |
| **m3** singularity switch | **FIXED** | `‖Λ‖₂` = 1.0e4 at the singular pose, 9990 at 1e-3 rad away, 9058 at 1e-2 rad: continuous and bounded. Side effect to be aware of: the **default** terms are now regularised everywhere (at home `ΛA − I` = 1.2e-3, `J M⁻¹ N_tau` = 3.2e-4, the same posture torque leaks 1.1e-3 m/s²); `damping=0` restores exact identities. README documents this |
| **m4** limits status | **FIXED** | `limits_violated()` all false at home; flags joints 1 and 7 after driving them to their stops; `last_tau_applied` returns the clipped command |
| **m5** sensor faults | **FIXED** | repeated reads identical; unseeded `reset()` reproduces the stream; sensor-frame bias verified (`Rf @ bias` plus moment arm to TCP); `get_ext_joint_torque(measured=True)` returns truth + bias; 4000-step mean/std match configuration |
| **m6** peak-force metric | **FIXED** | `get_contact_normal_sum` added; README defines resultant, largest normal and normal sum, says peaks are raw single-step values and that B must state metric and filter |
| **m7** tests | **FIXED** | plain `pytest` collects; the `n_samples >= 1000` assertions are gone; independence test parses the module AST for any `mujoco` import or `mj_*` call; tolerances tightened to 1e-10; demo peak bound is `20 < peak < 22`; new tests for real-step accelerated wrench, chamfer capture and sharp-rim jam, hand-specific contact parameters, penetration rollback; the hand-contact tests now approach from outside |
| **m8** README stale | **FIXED** | layout table lists `kinematics.py`, `dynamics.py`, both verify scripts, `tests/`, `docs/`; I read the whole interface section against `env.py` and found no wrong statement |
| **m10** report artefacts | **PARTIAL** | Table: fixed (212.98 pt, math-typeset numbers, units). Scene: cropped (content now 82% of the canvas), peg and fixture labelled. Not fixed: (1) the validation file claims the render is byte-identical on repeat; here two fresh renders are identical to each other (`575c34cb…`) but differ from the committed file (`a0d161e5…`) in 4.8% of pixels by at most 38 grey levels, i.e. visually identical but not byte-reproducible across sessions; (2) at the `0.45\columnwidth` used in IV-A the labels print at about 6 pt (item 29) |

m9 (the `param_check.md` "FR3-only" statement) was outside this task's list; I note that the file now cites the live FER table.

---

## Required corrections

Section IV-A, in priority order:

1. **Eq. (3), gravity term (item 14).** Either keep `τ_cmd` and write `Λẍ + μ = F + F_ext` (delete `p` and its definition, which also frees a line), or state the law for the actuator torque `τ = Jᵀ F + N_τ τ_0` as `modeling_notes.md` does and keep `p`. The first matches how part E will actually command the robot.
2. **`τ_ext` (item 11).** Replace "and `τ_ext = Jᵀ F_ext`" with "external joint torque `τ_ext` (`= Jᵀ F_ext` for a wrench `F_ext` applied by the environment at the TCP)". If `F_ext` in eq. (3) is meant generally, add `F_ext = J̄ᵀ τ_ext`.
3. **Acceleration claim (item 24).** Use 2.92×10⁻¹² and say "with λ = 0, over 1000 well-conditioned random states"; or keep 2.93×10⁻¹² and say passive damping was cancelled.
4. **Hand calibration sentence (item 21).** "a free 0.6 kg hand body whose contact is calibrated so that its impact peak force matches the ISO/TS 15066 hand model (75 N/mm) within 2%".
5. **`frankadocs` (item 4).** Change the URL in `refs.bib` to `https://frankarobotics.github.io/docs/` (author "Franka Robotics GmbH"), keeping the access date.
6. **Symbols (item 28).** Define `F` and `τ_0` where `τ_cmd` is introduced, define `x` or write the twist as `V`, use `𝓜` in the table and drop "(M in Table I)", write `I_7` in eq. (4), state that `F_ext` is the wrench exerted by the environment on the robot.
7. **Typesetting.** Rewrite the `τ_max` sentence (see English table) to remove the three badness-10000 lines.
8. **Figure (item 29, m10).** Render a version meant for the printed width (fonts set so labels are at least 7–8 pt at the included size, TCP triad arrows longer, "TCP T" label next to the triad) or include at about `0.6\columnwidth`; that costs about 28 pt and the section has 26 pt spare, so pair it with correction 1, which frees a line.
9. **Wording.** Apply the English table; soften "We model the box-in-box assembly cell [4]" to "inspired by [4]"; in the table caption say "max error over 1000 random states" and "against MuJoCo and finite differences".

Repository (optional, none blocks the freeze):

- Add one README sentence under hand modes: a large target jump of an enabled hand is not guarded and can produce kilonewton forces.
- Correct the "repeated render was byte-identical" line in `T4_fix_validation.md`, or treat `scene.png` as a committed artefact that is not regenerated.

---

## Can the README interface be frozen?

**Yes, for A, B, D and E.** Every T4 MAJOR and every interface-affecting MINOR is fixed and reproduced independently; the one PARTIAL item (m10) concerns a report figure, not the interface. Things each consumer should know when freezing:

| Consumer | Note |
| --- | --- |
| A | Use `release_hand(velocity)` for transient cases; the calibration is a peak-force match on a fixed surface, impacts on the moving arm give lower peaks (45.6 N on the peg, 29.1 N on link5 at 0.25 m/s). Default `sensor_bias_frame` is now `'sensor'`. Observation `tau_ext` is the measured value (equal to truth when no fault is configured) |
| B | `limits_violated()` and the three force metrics are available; choose and state the metric |
| D | Capture range with the chamfer is about ±3 mm lateral; 2° tilt still wedges at 35 mm |
| E | Use `N_tau`; default task terms are regularised with λ = 0.01 (pass `damping=0` for exact identities away from singularities); with the environment's gravity compensation the controller equation has no `p` term |
