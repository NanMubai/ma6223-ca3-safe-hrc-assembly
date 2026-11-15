# Parameter cross-check — Franka Panda / MuJoCo Menagerie / ISO-TS 15066

Date: **2026-11-15**. Scope: `assets/panda_torque.xml`,
`assets/franka_emika_panda/panda_nohand.xml`, `hrc_sim/config.py`, `README.md`.
Read-only check; the only file written is this one. No git commit.

Method: every external value below comes from a source fetched with `curl`
(see **Source registry**, with the verbatim quote); the repo values were read
from the files or inspected in memory with
`/home/eva/miniforge3/envs/hrc/bin/python` + MuJoCo 4.0.1. Nothing is filled
in from memory. Local file references are written as
`path:line` (no URL). The legacy Franka page
`https://frankaemika.github.io/docs/control_parameters.html` returns **HTTP
404**; the live docs page
`https://frankarobotics.github.io/docs/robot_specifications.html` (S2)
contains the **"Limits for Franka Emika Robot (FER)"** table, which is the
Panda. The limits below are therefore quoted from the live FER table (S2),
with the Internet Archive snapshot **2023-02-07** of the legacy page (S1,
explicitly headed "Limits for Panda") and the Panda `joint_limits.yaml` (S3)
as corroboration. The live page's DH table is separately labelled FR3 (S2),
so the DH rows use S1/S4 (Panda-labelled archive + URDF).

**Result in one line:** 38 items checked; every length, limit, mass, inertia and
hand-mass value that *is* present in the repo matches its primary source exactly. Four requested limit/stiffness
parameters are **not represented** (acceleration, jerk, torque-rate, ISO hand
spring constant), and the Menagerie `attachment` frame carries a 135° yaw that
is not in the URDF/DH flange frame (offset length unaffected).

---

## 1. Modified DH parameters vs XML link offsets

The XML stores each link's transform in the parent body frame; the `a, d,
alpha` shown for the repo are the values obtained after converting the body
translation/rotation (quaternion) to the Craig-convention DH form, and were
also checked against the URDF (`franka_arm.xacro`, S4). `link3`/`link5` use a
y-offset in the XML because their parent frame is rotated ±90° about x; after
that rotation the offset is along the local z axis, i.e. `d`, exactly as the
DH table requires.

| Item | Value in repo | Value in source | Source URL | Match | Note |
|---|---|---|---|---|---|
| DH Joint 1 (`a, d, α`) | `link1` pos `0 0 0.333`, quat identity → `0, 0.333, 0` | `Joint 1: a=0, d=0.333, α=0` | [S1](https://web.archive.org/web/20230207221030/https://frankaemika.github.io/docs/control_parameters.html) / [S4](https://raw.githubusercontent.com/frankarobotics/franka_ros/develop/franka_description/robots/common/franka_arm.xacro) | yes | URDF joint1 origin `xyz="0 0 0.333"` |
| DH Joint 2 (`a, d, α`) | `link2` pos `0 0 0`, quat `1 -1 0 0` (-90° x) → `0, 0, -π/2` | `Joint 2: a=0, d=0, α=-π/2` | S1 / S4 | yes | URDF joint2 `rpy="${-pi/2} 0 0"` |
| DH Joint 3 (`a, d, α`) | `link3` pos `0 -0.316 0`, quat `1 1 0 0` (+90° x) → `0, 0.316, +π/2` | `Joint 3: a=0, d=0.316, α=+π/2` | S1 / S4 | yes | -0.316 y in the rotated parent frame is +0.316 along z (same transform) |
| DH Joint 4 (`a, d, α`) | `link4` pos `0.0825 0 0`, quat +90° x → `0.0825, 0, +π/2` | `Joint 4: a=0.0825, d=0, α=+π/2` | S1 / S4 | yes | URDF joint4 `xyz="0.0825 0 0"` |
| DH Joint 5 (`a, d, α`) | `link5` pos `-0.0825 0.384 0`, quat -90° x → `-0.0825, 0.384, -π/2` | `Joint 5: a=-0.0825, d=0.384, α=-π/2` | S1 / S4 | yes | URDF joint5 same |
| DH Joint 6 (`a, d, α`) | `link6` pos `0 0 0`, quat +90° x → `0, 0, +π/2` | `Joint 6: a=0, d=0, α=+π/2` | S1 / S4 | yes | URDF joint6 `rpy="${pi/2} 0 0"` |
| DH Joint 7 (`a, d, α`) | `link7` pos `0.088 0 0`, quat +90° x → `0.088, 0, +π/2` | `Joint 7: a=0.088, d=0, α=+π/2` | S1 / S4 | yes | URDF joint7 same |
| Flange offset | `attachment` pos `0 0 0.107`, quat `0.3826834 0 0 0.9238795` (+135° z) | `Flange: a=0, d=0.107, α=0, θ=0` | S1 / S4 | yes for **d**; note orientation | `d=0.107` exactly; the extra 135° yaw is a Menagerie attachment-frame convention (URDF joint8 has `rpy="0 0 0"`), not part of the DH length table — see discrepancy 3 |

All eight link `body_pos`/`body_quat` transforms in `panda_torque.xml` (and the attachment transform) are
**bit-identical** to the vendored Menagerie `panda_nohand.xml` (script zero
difference), so the "kinematic tree copied unchanged" comment is correct.

---

## 2. Joint limits and actuator limits

Panda values are from the live **"Limits for Franka Emika Robot (FER)"**
table (S2); the archived page's **"Limits for Panda"** table (S1) and the
`joint_limits.yaml` (S3) corroborate every row. The repo values are from
`hrc_sim/config.py` (`q_min`, `q_max`, `dq_max`, `tau_max`) and the two XML
files. Order everywhere is `joint1 … joint7` (rad, rad/s, N·m).

Source links for this section: [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (live FER table; primary), [S1](https://web.archive.org/web/20230207221030/https://frankaemika.github.io/docs/control_parameters.html) (archive, "Limits for Panda"; corroboration), [S3](https://raw.githubusercontent.com/frankarobotics/franka_ros/develop/franka_description/robots/panda/joint_limits.yaml) (YAML; corroboration).

| Item | Value in repo | Value in source | Source URL | Match | Note |
|---|---|---|---|---|---|
| Position limits `q_min` / `q_max` | `q_min=[-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973]`; `q_max=[2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973]` | `q_min` row: `-2.8973 -1.7628 -2.8973 -3.0718 -2.8973 -0.0175 -2.8973`; `q_max` row: `2.8973 1.7628 2.8973 -0.0698 2.8973 3.7525 2.8973` rad | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 / S3 | yes | exact; live FER rows equal the archived "Limits for Panda" rows; YAML `lower`/`upper` agree; `q_home` lies inside and `reset` enforces it (`env.py:115`) |
| Joint `range` in XML | `panda_torque.xml`: joint2 `-1.7628 1.7628`, joint4 `-3.0718 -0.0698`, joint6 `-0.0175 3.7525`, others inherit `range="-2.8973 2.8973"`; `panda_nohand.xml`: same values | same as the `q_min`/`q_max` rows | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 / S3 | yes | both XMLs; `env._model_xml` rewrites them from config before compiling |
| Velocity limits `dq_max` | `[2.175, 2.175, 2.175, 2.175, 2.610, 2.610, 2.610]` rad/s | `qdot_max` row: `2.1750 2.1750 2.1750 2.1750 2.6100 2.6100 2.6100` rad/s | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 / S3 | yes | not encoded in XML and **not enforced**: README line 107 says "velocity limits are controller metadata"; no `dq` clamp in `env.step` |
| Torque limits `tau_max` | `[87, 87, 87, 87, 12, 12, 12]` N·m | `tau_j_max` row: `87 87 87 87 12 12 12` N·m | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 / S3 | yes | exact |
| `panda_torque.xml` motor `ctrlrange` | `[-87,87]` joints 1–4; `[-12,12]` joints 5–7 | same as `tau_j_max` | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 | yes | these are **torque** motors (`gear=1`, no `forcerange`), so `ctrlrange` = torque limit, not position |
| `panda_nohand.xml` actuator `ctrlrange` / `forcerange` | position `ctrlrange` default `-2.8973 2.8973`, joint2 `-1.7628 1.7628`, joint4 `-3.0718 -0.0698`, joint6 `-0.0175 3.7525`; torque `forcerange` default `-87 87`, joints 5–7 `-12 12` | `q_max/q_min` and `tau_j_max` rows | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 / S3 | yes | Menagerie uses **position** `ctrlrange` + **torque** `forcerange`; do not compare its `ctrlrange` to `tau_max`. `panda_torque.xml` replaces these with torque motors |
| Acceleration limits | **absent** from `config.py`, both XMLs, `env.py`, README | `qddot_max` row: `15 7.5 10 12.5 15 20 20` rad/s² | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 | **no — not represented** | MuJoCo has no acceleration-limit attribute; no trajectory/acceleration limiter is implemented. Consequence: simulated joint accelerations can exceed Panda/FCI limits |
| Jerk limits | **absent** (same files) | `qdddot_max` row: `7500 3750 5000 6250 7500 10000 10000` rad/s³ | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 | **no — not represented** | not in MuJoCo; no jerk limiting |
| Torque-rate limit | **absent**; README line 107 explicitly: "No torque-rate limit is imposed" | `tau_dot_j_max` row: `1000` N·m/s for all joints | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER; primary) / S1 | **no — not represented (documented)** | README is honest about it; if the report claims FCI-compatible trajectories, this is a gap |
| Panda-vs-FR3 mix-up check | all repo limits = Panda values; no FR3 values found (`grep` for FR3 q/vel numbers returned nothing) | FR3 examples (S2 FR3 table): `q6_min=0.4398` vs Panda `-0.0175`; `dq=[2.62,2.62,2.62,2.62,5.26,4.18,5.26]` vs Panda `[2.175,…,2.61]`; `qddot=10` for all vs Panda mixed | [S2](https://frankarobotics.github.io/docs/robot_specifications.html) (FER and FR3 tables) / S1 | **yes — no mix-up** | the live page contains **both** the FER (Panda) and FR3 limits tables; the repo matches FER/Panda (S2 FER, S1), not FR3 |

Note: the live page's DH table is labelled "Franka Research 3" (S2), while
its **limits** table is labelled "Limits for Franka Emika Robot (FER)" (the
Panda). The live FER and archived Panda limit rows are identical, and the
repo matches them; the FR3 limit rows differ (e.g. `q6_min`, `dq`, `qddot`).
For the DH rows the archive's explicitly Panda-labelled table (S1) and the
Panda URDF (S4) remain the primary sources; the live FR3-labelled DH table
happens to list the same `a, d, α` values.

---

## 3. Link masses and provenance

| Item | Value in repo | Value in source | Source URL | Match | Note |
|---|---|---|---|---|---|
| Link masses `link0…link7` | `[0.629769, 4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 0.735522]` kg | identical list in `inertial.yaml`: `0.629769, 4.970684, 0.646926, 3.228604, 3.587895, 1.225946, 1.666555, 7.35522e-01` | [S7](https://raw.githubusercontent.com/frankaemika/franka_ros/develop/franka_description/robots/common/inertial.yaml) / [S6](https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda/panda_nohand.xml) | yes | exact equality; `panda_torque.xml` and vendored `panda_nohand.xml` have bit-identical mass/COM/full-inertia for all 8 links |
| Link COM + full inertia | XML `pos`/`fullinertia` values (e.g. link1 `0.003875 0.002081 -0.04762`, `0.70337 0.70661 0.0091170 -0.00013900 0.0067720 0.019169`) | same numbers in `inertial.yaml` (origin + `xx yy zz xy xz yz`) | S7 / S6 | yes | script comparison: **exact** match for mass, COM and all six inertia elements of all 8 links |
| Provenance of the inertials | repo README: `assets/franka_emika_panda/` = "Unmodified MuJoCo Menagerie Panda at revision `370c439`"; `panda_torque.xml` comment: "Kinematic tree/inertials copied unchanged from Menagerie @370c439." | Menagerie README step 7: "Matched inertial parameters with [inertial.yaml](...)" ; `inertial.yaml` header: "This file does not contain official inertial properties of panda robot. The values are from the identification results published in: Identification of the Franka Robotics PandaRobot With Retrieval of Feasible Parameters Using Penalty-Based Optimization by: Claudio Gaz, Marco Cognetti, Alexander Oliva, Paolo Robuffo Giordano, Alessandro de Luca" | [S5](https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda/README.md) / S7 | yes | **not official Franka data**: they are Gaz et al. 2019 identification results, imported through `franka_ros`; the local vendored `panda_nohand.xml` is byte-identical (SHA-256 `f05c9674…af09`) to the Menagerie file at commit `370c439` |
| Total mass plausibility | links sum = **16.691901 kg**; adding Menagerie/`inertial.yaml` hand `0.73 kg` + two fingers `2×0.015 kg` = **17.451901 kg**; with the 0.2 kg peg = 17.651901 kg | Gaz et al. 2019: "it has a total weight of approximately 18 kg"; the identification constrained "`mrob,min = 16 kg` and `mrob,max = 20 kg`" | [S8](https://inria.hal.science/hal-02265293/file/IROS_19_Panda.pdf) | yes — plausible | 17.45 kg is ~3.0% below 18 kg; without hand/fingers 16.69 kg is ~7.3% below but inside the paper's 16–20 kg bound. The fetched text does not state whether "18 kg" includes hand/cabling, so both comparisons are given |
| Project tool/peg mass | `tool` body 0.2 kg (peg geom `mass="0.2"`), peg 0.03×0.03×0.06 m; arm+tool = 16.891901 kg | not a Panda parameter | local `panda_torque.xml:175`, `config.py:28` | n/a | the tool is a project addition; do not include it when comparing to the Panda 18 kg figure. The scene's `human_hand` (0.6 kg) and table/fixture are scene objects, not robot mass |

---

## 4. Hand model and contact parameters

| Item | Value in repo | Value in source | Source URL | Match | Note |
|---|---|---|---|---|---|
| Human hand mass | `HandConfig.mass = 0.6` kg (comment "ISO/TS 15066 effective hand mass: TBD by part A"); scene `human_hand` capsule `mass="0.6"`; env writes it to the geom | ISO/TS 15066 body-model table: "Hands and fingers … Effective Mass (kg) … 0.6" | [S9](https://arxiv.org/pdf/2311.13814) / [S10](https://arxiv.org/pdf/2203.02706) | yes (provisional) | 0.6 kg is the ISO **effective** mass, used here as the dynamic body mass; README marks it "TBD by part A" and "a study parameter, not a validated biological hand model" |
| Human hand spring constant | **no `k` parameter** in `config.py` or XML; only weld/contact `solref` (`0.002/0.04/0.004`) | ISO/TS 15066 table: "Hands and fingers … Effective Spring Constant K (N/mm) … 75"; paper text: "hand with mh = 0.6 kg, k = 75 N/mm" | S9 / S10 | **no — not represented** | 75 N/mm = 75 000 N/m is nowhere in the repo (`grep` for `75e3/75000/75 N/mm`). The hand is a rigid capsule; contact stiffness is MuJoCo's `solref/solimp`, not the ISO spring. See discrepancy 1 |
| Hand capsule geometry | radius `0.04` m, cylinder half-length `0.05` m; total length `2×(0.05+0.04)=0.18 m`; scene capsule `size="0.04 0.05"` | ISO table specifies only F_max, K, effective mass — **no hand dimensions** | S9 | n/a | dimensions are a study choice; README says 180 mm/40 mm and flags it; no source value to match |
| World contact `solref` | `solref=(0.004, 1.0)`, `dt=0.001` s; 4 ms = **4 dt** | MuJoCo: "Rigid contacts usually prefer lowering it to 0.004–0.01 … keeping `solref[0] >= 2 * timestep`"; "The timeconst parameter should be at least two times larger than the simulation time step … enforced internally" | [S11](https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters) | yes | 0.004 is in the recommended 0.004–0.01 range and ≥ 2dt; `config.__post_init__` rejects `< 2dt` |
| Kinematic hand weld `solref` | `(0.002, 1.0)` = **2 dt** (lower bound) | same rule: timeconst ≥ 2·timestep; MuJoCo clamps with `max(solref[0], 2*timestep)` | S11 / [S12](https://mujoco.readthedocs.io/en/stable/XMLreference.html) | yes | exactly at the bound; `env.set_hand` and `_model_xml` clamp to `max(sr[0], 2*dt)`. No margin |
| Compliant hand weld `solref` | `(0.04, 1.0)` = **40 dt** | same rule; 0.04 is a soft weld by choice (4× the rigid-contact upper recommendation, 40×dt) | S11 | yes | this is a **weld** (mocap equality), deliberately soft for transient impacts — not the ISO hand spring; see discrepancy 4 |
| `solimp` width / `condim` | `solimp=(0.95, 0.99, 0.001)`; width `0.001 m = 1 mm`; `condim=3` | MuJoCo: `solimp[2]` is the width of the impedance transition, "default 1 mm"; "`d_w` only applies at penetrations approaching width (`solimp[2]`, default 1 mm); scale width to the compliant layer or object size" | S11 | yes (rules) | 1 mm equals the total diametral clearance (2×0.5 mm per side), so at nominal fit the contact is still in the soft transition; intentional per README ("smooth 1 mm impedance transition"), but relevant to interpreting force peaks |

Contact-sanity conclusion: `solref`/`solimp` are numerically valid (4dt
contact, 2dt and 40dt welds, width 1 mm), and the config validates the world
`solref` against `2dt`; the hand welds are clamped by the env. The ISO hand
stiffness is the only hand parameter that is missing.

---

## 5. Units / conventions claimed in README vs code

All "local" values were read in `hrc_sim/env.py`, `hrc_sim/config.py`,
`assets/panda_torque.xml` and checked with short Python snippets on 2026-11-15
(no files changed).

| Item | Value in repo | Value in source | Source URL | Match | Note |
|---|---|---|---|---|---|
| Quaternion order | README line 42: "Quaternions are **(w, x, y, z)**"; `set_hand` accepts wxyz; `ee_quat` is MuJoCo `mju_mat2Quat` output | `_quat` docstring: "finite (w,x,y,z)"; `mju_mat2Quat` / `mocap_quat` are MuJoCo wxyz | local: `README.md:42`, `env.py:22-26`, `env.py:158-160`, `env.py:318` | yes | reconstructed `R` from `ee_quat` matched `ee_R` to `8.9e-16` at home; home quat `[0,0,1,0]` is a 180° y-rotation, consistent with the peg-down frame |
| Wrench order and sign | README line 44: "[force; torque] … environment exerting force on the robot"; line 45: positive world-x applied force reads positive | `wrench = np.r_[f, t]`; `f = inertial_f - sensor_f`; contact loop gives `+force` to geom2 and `-force` to geom1 | local: `README.md:44-45`, `env.py:245-252`, `env.py:274-278`; [S12](https://mujoco.readthedocs.io/en/stable/XMLreference.html) | yes | known `xfrc_applied = [+5,-3,+2] N, [+0.3,-0.4,+0.2] N·m` returned exactly `[5,-3,2,0.39,-0.25,0.2]` (torque shifted by the TCP moment arm), so order and sign are as documented |
| Jacobian row order / size | README line 43: "**[linear; angular]**, world axes; `J` is **6 × 7**, at the TCP" | `mj_jacSite` translational/rotational rows stacked `np.vstack((jp, jr))`, sliced to the seven arm dofs | local: `README.md:43`, `env.py:162-165` | yes | `J.shape == (6,7)`; finite-difference check `J·dq` vs FD: linear error `2.7e-12 m`, angular error `4.9e-12 rad` |
| Twist convention | README line 43: `ee_vel = J @ dq`, `(6,)` world `[v; ω]` | `get_ee_velocity` literally returns `jacobian() @ qvel[dof_indices]` | local: `README.md:43`, `env.py:172-173` | yes | same row order as the Jacobian |
| Units | README line 39: "SI: m, rad, N, N·m, s, kg"; XML `angle="radian"`, `timestep="0.001"`, gravity `0 0 -9.81` | config/XML use those units throughout; no conversion factors in env | local: `README.md:39`, `config.py:25-37`, `panda_torque.xml:3-4` | yes | joint limits rad, torque N·m, masses kg, lengths m |
| TCP / home geometry | README line 40: TCP is "the centre of the peg's free end face, 60 mm from the flange"; line 1: "home TCP is 150 mm above the entrance" | XML: `peg pos="0 0 0.03" size="0.015 0.015 0.03"`, `tcp pos="0 0 0.06"`; measured at home: TCP `[0.5,0,0.43]`, hole_top `[0.5,0,0.28]`, Δz `0.150` m | local: `README.md:1,40`, `panda_torque.xml:175-176` | yes | peg local +z = world -z at home (`R[:,2]=[0,0,-1]`); square peg axes align with hole axes (peg `R=diag(-1,1,-1)`) |
| API dimensions / mass matrix | README line 123: `nq=14, nv=13, nu=7`; `mass_matrix()` 7×7 arm block incl. tool and armature | compiled model `nq=14, nv=13, nu=7`; `mj_fullM` sliced to the 7 arm dofs | local: `README.md:123`, `env.py:175-182` | yes | hand free joint is the extra 7 qpos / 6 dofs; arm dofs are resolved by name |
| Torque / velocity / position enforcement | README line 107: torque limits enforced; velocity limits metadata; position limits MuJoCo constraints; no torque-rate limit | `env.step` clips `tau+g` to ±`tau_max`; no `dq_max` use; XML joint ranges; no rate limiter | local: `README.md:107`, `env.py:125-138`, `panda_torque.xml:8,110,123,140` | yes | exactly as documented, including the absence of a torque-rate limit (see §2) |

---

## Discrepancies, ranked by severity

1. **ISO/TS 15066 hand spring constant `k = 75 N/mm` is missing** (medium;
   safety-analysis impact). The hand mass 0.6 kg is correct, but there is no
   `k` in `HandConfig`, the XML or the env; the hand is a rigid capsule and
   the robot–hand contact stiffness is set by MuJoCo `solref/solimp`
   (`0.004 s`, width 1 mm), not by 75 000 N/m. Any Section V calculation that
   uses the ISO two-body/transient force model must add `k_H = 75 N/mm` (or
   clearly state that the simulation uses a different contact model). This is
   a modelling gap, not a wrong number.
2. **Acceleration, jerk and torque-rate limits are absent** (medium, if the
   report claims Panda/FCI-compatible motion). The Panda table gives
   `qddot = [15, 7.5, 10, 12.5, 15, 20, 20] rad/s²`,
   `qdddot = [7500, 3750, 5000, 6250, 7500, 10000, 10000] rad/s³` and
   `tau_dot = 1000 N·m/s`. `config.py`/XML contain only position, velocity and
   torque limits; `dq_max` is not enforced either (README says it is
   metadata), and README explicitly says no torque-rate limit is imposed.
   Nothing contradicts the sources — the parameters are simply not
   represented.
3. **`attachment` / `ft_site` has a 135° yaw offset relative to the URDF/DH
   flange frame** (low; kinematic/FCI-frame caveat). The flange distance
   `d=0.107 m` and the flange +z axis are correct, but Menagerie's
   `attachment` quat is `0.3826834 0 0 0.9238795` (+135° about z), while the
   URDF joint8 is `rpy="0 0 0"`. Any independent FK to `ft_site`/`tcp` must
   include this fixed yaw; the world/TCP wrench functions already do. Raw
   `ft_site` axes are therefore not the FCI flange axes.
4. **`compliant_solref = 0.04` is a weld parameter, not the ISO hand
   stiffness** (informational). It is a deliberately soft mocap weld for
   transient impacts (40×dt), not a 75 N/mm spring; the actual robot–hand
   contact uses `solref = 0.004 s`. Do not use the weld value as `k_H`.
5. **`solimp` width (1 mm) equals the total diametral clearance** (informational).
   With 0.5 mm per side, the contact is still in the impedance transition at
   nominal fit; this is intentional ("smooth 1 mm impedance transition") but
   means shallow contact forces are soft, so peak-force comparisons need care.
6. **Kinematic weld time constant is exactly at the 2·dt lower bound**
   (informational). MuJoCo/`env` clamp to `max(sr[0], 2*dt)`; valid, but with
   zero margin. `config.py` validates only `SimConfig.solref`, not the nested
   `HandConfig` solrefs (the env clamps them, so this is not a numerical
   violation).
7. **Source-access notes, not repo errors** (informational).
   `frankaemika.github.io` (including the control-parameters page) is 404
   today. The live `frankarobotics.github.io` page (S2) does contain the
   **"Limits for Franka Emika Robot (FER)"** table — the Panda — and those
   values were taken as primary; the 2023-02-07 archive (S1) and the Panda
   `joint_limits.yaml` (S3) corroborate them. Only the live page's DH table is
   labelled FR3; the Panda DH rows use S1/S4. The ISO/TS 15066 full text is
   paywalled and was **not** fetched; the 0.6 kg / 75 N/mm values are quoted
   from two public papers (S9, S10), with the table caption attributing them
   to ISO/TS 15066. A Panda datasheet was not fetched (the live Franka site
   redirects Panda to FR3); the 18 kg figure comes from the Gaz et al. paper
   (S8), which is also the stated source of `inertial.yaml`.

No numeric value that is present in the repo disagreed with its source; the only
frame-level difference is the 135° attachment yaw in discrepancy 3.

---

## Source registry (verbatim quotes from fetched text)

Formatting: HTML-extracted text has runs of whitespace collapsed; table cell
order and numeric values are unchanged. PDF text was extracted with
`pdftotext -layout`.

### S1 — Franka docs, archived 2023-02-07 (Panda DH + limits; corroboration for limits)
<https://web.archive.org/web/20230207221030/https://frankaemika.github.io/docs/control_parameters.html>
(fetched 2026-11-15; the live `frankaemika.github.io` URL returns 404). The snapshot was fetched in raw `id_` form to extract the table text; the URL above resolves to the same snapshot.

> "Limits for Panda"
>
> "Joint space limits are:"
>
> "q_max | 2.8973 | 1.7628 | 2.8973 | -0.0698 | 2.8973 | 3.7525 | 2.8973 | rad"
>
> "q_min | -2.8973 | -1.7628 | -2.8973 | -3.0718 | -2.8973 | -0.0175 | -2.8973 | rad"
>
> "qdot_max | 2.1750 | 2.1750 | 2.1750 | 2.1750 | 2.6100 | 2.6100 | 2.6100 | rad/s"
>
> "qddot_max | 15 | 7.5 | 10 | 12.5 | 15 | 20 | 20 | rad/s²"
>
> "qdddot_max | 7500 | 3750 | 5000 | 6250 | 7500 | 10000 | 10000 | rad/s³"
>
> "tau_j_max | 87 | 87 | 87 | 87 | 12 | 12 | 12 | Nm"
>
> "tau_dot_j_max | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | Nm/s"

> "The Denavit–Hartenberg parameters for the Panda's kinematic chain are derived following Craig's convention and are as follows:"
>
> "Joint | a (m) | d (m) | alpha (rad) | theta (rad)"
>
> "Joint 1 | 0 | 0.333 | 0 | theta_1"
>
> "Joint 2 | 0 | 0 | -pi/2 | theta_2"
>
> "Joint 3 | 0 | 0.316 | pi/2 | theta_3"
>
> "Joint 4 | 0.0825 | 0 | pi/2 | theta_4"
>
> "Joint 5 | -0.0825 | 0.384 | -pi/2 | theta_5"
>
> "Joint 6 | 0 | 0 | pi/2 | theta_6"
>
> "Joint 7 | 0.088 | 0 | pi/2 | theta_7"
>
> "Flange | 0 | 0.107 | 0 | 0"

(The source renders `alpha`/`theta` and the limit labels with MathJax symbols; labels above are normalized and the signs/values were checked against the page's MathJax text.)

### S2 — live Franka docs: FER (Panda) limits + FR3 DH/limits
<https://frankarobotics.github.io/docs/robot_specifications.html> (fetched
2026-11-15).

> "Limits for Franka Emika Robot (FER)"
>
> "Joint-space limits:"
>
> "q_max | 2.8973 | 1.7628 | 2.8973 | -0.0698 | 2.8973 | 3.7525 | 2.8973 | rad"
>
> "q_min | -2.8973 | -1.7628 | -2.8973 | -3.0718 | -2.8973 | -0.0175 | -2.8973 | rad"
>
> "qdot_max | 2.1750 | 2.1750 | 2.1750 | 2.1750 | 2.6100 | 2.6100 | 2.6100 | rad/s"
>
> "qddot_max | 15 | 7.5 | 10 | 12.5 | 15 | 20 | 20 | rad/s²"
>
> "qdddot_max | 7500 | 3750 | 5000 | 6250 | 7500 | 10000 | 10000 | rad/s³"
>
> "tau_j_max | 87 | 87 | 87 | 87 | 12 | 12 | 12 | Nm"
>
> "tau_dot_j_max | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | 1000 | Nm/s"

"FER" is the same robot the rest of this document calls Panda; the live page
also has an FR3 limits table and an FR3-labelled DH table. The FR3 limit rows
differ from FER/Panda (examples):

> "q_max | 2.9007 | 1.8361 | 2.9007 | -0.1169 | 2.8763 | 4.6216 | 3.0508 | rad"
>
> "qdot_max | 2.62 | 2.62 | 2.62 | 2.62 | 5.26 | 4.18 | 5.26 | rad/s"
>
> "qddot_max | 10 | 10 | 10 | 10 | 10 | 10 | 10 | rad/s²"
>
> "qdddot_max | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 | 5000 | rad/s³"

(Labels are normalized from the page's MathJax notation; the numeric values
are verbatim. A programmatic comparison of the live FER rows with
`hrc_sim/config.py` returned exact equality for all seven joints of
`q_min`, `q_max`, `dq_max` and `tau_max`. The `qddot`,
`qdddot` and `tau_dot` rows were likewise identical between the live FER
table and the archived Panda table (S1).)

> "The Denavit–Hartenberg parameters for the Franka Research 3 kinematic chain (following Craig's convention) are shown below:"

### S3 — Panda joint limits YAML (linked by README)
<https://raw.githubusercontent.com/frankarobotics/franka_ros/develop/franka_description/robots/panda/joint_limits.yaml>

> `joint1: limit: lower: -2.8973 upper: 2.8973 velocity: 2.1750 effort: 87.0`
>
> `joint5: limit: lower: -2.8973 upper: 2.8973 velocity: 2.6100 effort: 12.0`

(All seven entries were read; they match S1 exactly.)

### S4 — Panda URDF joint origins
<https://raw.githubusercontent.com/frankarobotics/franka_ros/develop/franka_description/robots/common/franka_arm.xacro>

> `<joint name="${arm_id}_joint1" type="revolute">` … `<origin rpy="0 0 0" xyz="0 0 0.333" />`
>
> `joint2` … `<origin rpy="${-pi/2} 0 0" xyz="0 0 0" />`
>
> `joint3` … `<origin rpy="${pi/2} 0 0" xyz="0 -0.316 0" />`
>
> `joint4` … `<origin rpy="${pi/2} 0 0" xyz="0.0825 0 0" />`
>
> `joint5` … `<origin rpy="${-pi/2} 0 0" xyz="-0.0825 0.384 0" />`
>
> `joint6` … `<origin rpy="${pi/2} 0 0" xyz="0 0 0" />`
>
> `joint7` … `<origin rpy="${pi/2} 0 0" xyz="0.088 0 0"/>`
>
> `joint8` … `<origin rpy="0 0 0" xyz="0 0 0.107"/>`

### S5 — MuJoCo Menagerie README at commit `370c439`
<https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda/README.md>

> "7. Matched inertial parameters with [inertial.yaml](https://github.com/frankaemika/franka_ros/blob/develop/franka_description/robots/common/inertial.yaml)."

(The fetched Menagerie `panda_nohand.xml` at this commit is byte-identical to
the local vendored file: SHA-256
`f05c9674dbc6081c8f096133d8537bdd977f87ec8eeb6264a77a9295ae88af09`.)

### S6 — Menagerie `panda_nohand.xml` at commit `370c439`
<https://raw.githubusercontent.com/google-deepmind/mujoco_menagerie/370c43965323c0944b433b202aac5058b5f907a9/franka_emika_panda/panda_nohand.xml>

Values are identical to the local file (same SHA); the local line numbers
quoted in the tables are from `assets/franka_emika_panda/panda_nohand.xml`.
The CHANGELOG at this commit has no inertial notes; only the README step 7
and `inertial.yaml` document the provenance.

### S7 — `inertial.yaml` (source of the Menagerie inertials)
<https://raw.githubusercontent.com/frankaemika/franka_ros/develop/franka_description/robots/common/inertial.yaml>

> "# This file does not contain official inertial properties"
>
> "# of panda robot. The values are from the identification"
>
> "# results published in: Identification of the Franka Robotics"
>
> "# PandaRobot With Retrieval of Feasible Parameters Using"
>
> "# Penalty-Based Optimization"
>
> "# by: Claudio Gaz, Marco Cognetti, Alexander Oliva,"
>
> "# Paolo Robuffo Giordano, Alessandro de Luca"

Link masses quoted in §3 are from this file (`link0` … `link7`).

### S8 — Gaz et al., RA-L 2019 (identification paper, fetched PDF)
<https://inria.hal.science/hal-02265293/file/IROS_19_Panda.pdf>

> "This robot is equipped with n = 7 revolute joints, each mounting a torque sensor, and it has a total weight of approximately 18 kg, having the possibility to handle payloads up to 3 kg."
>
> "we chose mrob,min = 16 kg and mrob,max = 20 kg"

### S9 — public paper quoting the ISO/TS 15066 body model
<https://arxiv.org/pdf/2311.13814> (Ghanbarzadeh & Najafi, 2023).

> "Table 1: Parameters of the human body model given in ISO/TS 15066 [1]."
>
> "Body Region | Maximum Permissible Force (N) | Effective Spring Constant K (N/mm) | Effective Mass (kg)"
>
> "Hands and fingers | 140 | 75 | 0.6"

### S10 — second public confirmation of the ISO values
<https://arxiv.org/pdf/2203.02706> ("ISO/TS 15066: How Different Interpretations Affect Risk Assessment").

> "a stiffness k = 75 N/mm for the human hand"
>
> "the values given in TS Appendix A (hand with mh = 0.6 kg, k = 75 N/mm)"

The ISO/TS 15066 full text itself was **not** fetched (paywalled); S9 and S10
are the fetched, citable public sources for the numbers.

### S11 — MuJoCo solver-parameter docs
<https://mujoco.readthedocs.io/en/stable/modeling.html#solver-parameters>

> "Rigid contacts usually prefer lowering it to 0.004–0.01 and loop closures to 0.004–0.005, keeping solref[0] >= 2 * timestep."
>
> "The timeconst parameter should be at least two times larger than the simulation time step, otherwise the system can become too stiff relative to the numerical integrator (especially when Euler integration is used) and the simulation can go unstable. This is enforced internally, unless the refsafe attribute of flag is set to false."
>
> "d_w (solimp[1]) only applies at penetrations approaching width (solimp[2], default 1 mm); scale width to the compliant layer or object size, and do not confuse solimp with solref"

### S12 — MuJoCo XML reference (sensor/contact conventions)
<https://mujoco.readthedocs.io/en/stable/XMLreference.html>

> force sensor: "The sensor outputs three numbers, which are the interaction force between a child and a parent body, expressed in the site frame defining the sensor."
>
> contact/pair: "The contact force vector computed by the solver and stored in mjData.efc_force points from the first towards the second geom by convention."

---

## Files/lines used for local values

`hrc_sim/config.py:12-19` (hand), `:25-42` (dt, geometry, solref/solimp,
q/dq/tau limits); `assets/panda_torque.xml:4` (option), `:8-19` (defaults and
contacts), `:90-171` (inertials/kinematics), `:171-176` (attachment, peg, TCP),
`:189-195` (motors); `assets/franka_emika_panda/panda_nohand.xml:9-10`
(default joint/actuator), `:98-187` (inertials/kinematics), `:201-210`
(position actuators and `forcerange`); `README.md:39-48, 107, 117-123`;
`hrc_sim/env.py:22-26, 44-46, 115, 125-138, 158-173, 175-182, 245-252,
274-278, 318-335`.
