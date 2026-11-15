# T4-fix validation evidence

All requested revisions were implemented except m9, explicitly delegated elsewhere; protected review/parameter-audit/report source files were not changed and no git add/commit was run.

## Plain pytest from repository root, after conda activate hrc

```text
$ pytest -q
....................................................                     [100%]
52 passed in 35.32s
```

## Modeling verification, including calibration and capture tables

```text
$ MUJOCO_GL=egl /home/eva/miniforge3/envs/hrc/bin/python scripts/verify_modeling.py
Verification: seed=6223, N=1000 random + 1000 task states
Quantity                                 Max error   Mean error    Tolerance      N  Pass
FK position (m)                          1.347e-15    3.794e-16    1.000e-10   1000  True
FK orientation (rad)                     1.033e-15    3.677e-16    1.000e-10   1000  True
Jacobian / MuJoCo                        4.108e-15    1.441e-15    1.000e-10   1000  True
Jacobian / central FD                    3.402e-10    1.416e-10    1.000e-08   1000  True
Mass (relative)                          3.035e-15    1.090e-15    1.000e-10   1000  True
Gravity (N m)                            6.395e-14    1.824e-14    1.000e-10   1000  True
Coriolis (N m)                           2.461e-14    4.122e-15    1.000e-10   1000  True
Forward EOM (N m)                        2.629e-13    4.427e-14    1.000e-10   1000  True
Inverse EOM (N m)                        6.395e-14    1.775e-14    1.000e-10   1000  True
Task acceleration                        2.931e-12    1.071e-13    1.000e-10   1000  True
Default damped acceleration              9.997e-05    7.508e-05    1.010e-04   1000  True
Task acceleration / step FD              1.055e-04    3.502e-05    2.000e-03   1000  True
Task with passive damping                2.917e-12    1.074e-13    1.000e-10   1000  True
Null acceleration                        2.822e-13    2.513e-14    1.000e-10   1000  True
Null acceleration / step FD              5.139e-06    9.669e-07    1.000e-04   1000  True
rim penetration (m)                      3.524e-05    1.225e-07    1.000e-04   3000  True
rim steady load error (N)                1.241e-07    1.118e-07    5.000e-02    500  True
inside_bottom penetration (m)            3.056e-05    1.441e-07    1.000e-04   3000  True
inside_bottom steady load error (N)      1.074e-07    9.560e-08    5.000e-02    500  True
Minimum mass eigenvalue: 1.035921e-01 kg m^2
Singularity: rank=5, sigma_min=1.237e-16, damping=1.000e-02, ||Lambda_d||=1.000e+04, null defect=4.634e-04, passed=True
Hand free-impact calibration (0.6 kg, 75 N/mm; tolerance 2%)
Speed (m/s)    Peak (N)  Elastic (N)    Error (%)
       0.10      21.296       21.213        0.391
       0.25      53.243       53.033        0.395
       0.50     106.485      106.066        0.395
Misalignment capture (raw single-step resultant; 3 mm chamfer unless sharp)
Case                         Depth (mm)    Peak (N)    Outcome
lateral_0mm                      40.000      21.006    inserted
lateral_0.5mm                    40.000      22.736    inserted
lateral_1mm                      40.000     116.426    inserted
lateral_2mm                      40.000     108.963    inserted
tilt_1deg                        39.741      22.677    inserted
tilt_2deg                        34.687      48.001    partial
yaw_1deg                         40.000      20.997    inserted
yaw_2deg                         39.524      71.129    inserted
sharp_lateral_1mm                 0.087      90.652    jammed
Artifacts: /home/eva/code/ma6223/ca3/report/figures
Overall passed: True
```

## Demo

```text
$ MUJOCO_GL=egl /home/eva/miniforge3/envs/hrc/bin/python scripts/demo_insertion.py --offscreen
Case: nominal
Final depth: 40.000 mm
Success: True (inserted)
Peak peg-hole resultant force: 21.006 N
Peak individual normal force: 12.234 N
Peak normal sum: 20.120 N
Peak wall/chamfer resultant: 0.000 N
Saved image: /home/eva/code/ma6223/ca3/outputs/demo_insertion.png
Case: offset_1.5mm
Final depth: 40.000 mm
Success: True (inserted)
Peak peg-hole resultant force: 117.875 N
Peak individual normal force: 57.833 N
Peak normal sum: 114.224 N
Peak wall/chamfer resultant: 117.875 N
Saved image: /home/eva/code/ma6223/ca3/outputs/demo_insertion_offset.png
```

## Public API changes (README is the frozen contract)

- Removed the bare `N` key from all task-space dicts; added `N_vel`, `N_tau` and `Jdot_dq` consistently.
- `HRCSim.task_space_dynamics(damping=None, include_joint_damping=False)` and `PandaDynamics.task_space_dynamics(q,dq=None,damping=None,include_joint_damping=False)` share damping semantics; the functional task-space helper defaults to damping=0.01. `PandaDynamics(...,task_damping=0.01)` adds the configurable default. All add damping squared continuously; damping=0 is explicit exact inversion.
- Public `jacobian_dot()` replaces private `_jacobian_dot()`.
- Added read-only-copy properties `joint_damping`, `last_tau_applied`, and `limits_violated(margin=0.0)`.
- `get_ext_joint_torque(measured=False)` adds the measured fault path; observation `tau_ext` uses measured=True.
- Added `get_contact_normal_sum(group=None)`.
- `set_hand(...,allow_penetration=False)` adds transactional fresh-enable placement rejection; `release_hand(velocity)` adds free flight and the `free` mode.
- New SimConfig fields: `chamfer`, `damping_compensation`, `seed`, `sensor_bias_frame`, `tau_ext_bias`, `tau_ext_noise_std`; `task_damping` defaults to 0.01 and `task_condition_limit` was removed.
- New HandConfig fields: `contact_solref`, `contact_solimp`, `solmix`, `placement_max_penetration`.
- Removed unused ReferenceControllerConfig `null_damping`; reference posture torque now uses environment `N_tau`.
- Measurement noise is cached per physics step/reset, reset(None) uses the stored deterministic seed, sensor faults default to flange sensor axes/origin, and runtime fault configuration persists across reset.
- `run_insertion(env,callback=None,lateral_offset=0.,tilt_deg=0.,yaw_deg=0.)` adds misalignment inputs and returns `outcome`, `peak_normal_sum` and `peak_wall_force`; the demo runs nominal plus 1.5 mm offset.

## Artifact checks and physical scope

IEEEtran tabular width: 212.97845 pt, column: 252 pt; six data rows, scoped footnotesize, math numbers and explicit units.
Scene: 1600x1200, white background, cropped content, world/TCP axes and peg/hole-fixture labels; the render is not guaranteed byte-reproducible across runs (independent review could not reproduce byte identity), so `scene.png` is a committed artefact, regenerated only with `scripts/verify_modeling.py --render-figure`.
Scene SHA256: `a0d161e59a5ea289ad5e4f101be6f1d60fbfe9655821644a670ea59c9068e502`.

The 2-degree tilt case remains partial at 34.687 mm; this is the measured physical capture limit for the constant-gain example, not an omitted regression. All required 1/2 mm lateral chamfer regressions insert and the 1 mm sharp-rim control jams. The free-impact calibration tolerance is ±2% (requested ±15%); maximum observed error is 0.3954%. Calibrated contact values are solref=(-125000,-1), solimp=(0.95,0.95,0.001), solmix=1000. These are calibrated constraint coefficients, not literal N/m material constants; k=75 N/mm is the fixed-table elastic reference. Kinematic and compliant modes remain prescribed-motion clamp and position spring, while release_hand selects the free transient benchmark.

Report artifacts: report/figures/modeling_verification.json, report/figures/modeling_verification_table.tex, report/figures/scene.png; derivation and synchronized tables: docs/modeling_notes.md; demo images: outputs/demo_insertion.png and outputs/demo_insertion_offset.png.
