"""Reproducible hand calibration and reference-controller capture-range probes."""
from pathlib import Path
import sys
if __package__ in (None, ''):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import mujoco
from hrc_sim import HRCSim, SimConfig
from scripts.demo_insertion import run_insertion


def verify_hand_impacts():
    """Raw per-step normal peak for a free hand hitting the fixed table.

    Capsule axis is vertical; tip starts 2 mm above the table, off the fixture.
    Gravity compensation prevents changes in incident speed during free flight.
    The elastic reference has mass .6 kg and stiffness 75000 N/m.
    """
    rows = []
    for speed in (.1, .25, .5):
        with HRCSim() as env:
            table_z = env.model.body('table').pos[2]
            h = env.config.hand
            env.set_hand([.65, .12, table_z+h.radius+h.cylinder_half_length+.002])
            env.release_hand([0., 0., -speed])
            peak = 0.
            for _ in range(150):
                env.step(np.zeros(7))
                normal = 0.
                for i, c in enumerate(env.data.contact):
                    if env.hand_geom_id in (c.geom1, c.geom2):
                        w = np.zeros(6)
                        mujoco.mj_contactForce(env.model, env.data, i, w)
                        normal += w[0]
                peak = max(peak, normal)
            reference = speed*np.sqrt(h.mass*75000.)
            rows.append(dict(speed_m_s=speed, peak_N=float(peak), reference_N=float(reference),
                             relative_error=float(abs(peak-reference)/reference), tolerance=.02))
    return dict(mass_kg=.6, stiffness_N_m=75000., dt_s=.001,
                contact_solref=list(h.contact_solref), contact_solimp=list(h.contact_solimp), solmix=h.solmix,
                rows=rows, passed=all(r['relative_error'] < r['tolerance'] for r in rows))


def verify_misalignment():
    cases = [(f'lateral_{mm:g}mm', mm/1000, 0., 0., .003) for mm in (0, .5, 1, 2)]
    cases += [(f'tilt_{deg:g}deg', 0., deg, 0., .003) for deg in (1, 2)]
    cases += [(f'yaw_{deg:g}deg', 0., 0., deg, .003) for deg in (1, 2)]
    cases += [('sharp_lateral_1mm', .001, 0., 0., 0.)]
    rows = []
    for label, offset, tilt, yaw, chamfer in cases:
        with HRCSim(SimConfig(chamfer=chamfer)) as env:
            r = run_insertion(env, lateral_offset=offset, tilt_deg=tilt, yaw_deg=yaw)
            rows.append(dict(case=label, lateral_m=offset, tilt_deg=tilt, yaw_deg=yaw,
                             chamfer_m=chamfer, **r))
    passed = all(r['outcome'] == 'inserted' for r in rows[:4]) and rows[-1]['outcome'] == 'jammed'
    return dict(rows=rows, passed=passed)


def print_scenarios(impact, alignment):
    print('Hand free-impact calibration (0.6 kg, 75 N/mm; tolerance 2%)')
    print('Speed (m/s)    Peak (N)  Elastic (N)    Error (%)')
    for r in impact['rows']:
        print(f"{r['speed_m_s']:11.2f} {r['peak_N']:11.3f} {r['reference_N']:12.3f} {100*r['relative_error']:12.3f}")
    print('Misalignment capture (raw single-step resultant; 3 mm chamfer unless sharp)')
    print('Case                         Depth (mm)    Peak (N)    Outcome')
    for r in alignment['rows']:
        print(f"{r['case']:28s} {1000*r['depth']:10.3f} {r['peak_force']:11.3f}    {r['outcome']}")


if __name__ == '__main__':
    impact, alignment = verify_hand_impacts(), verify_misalignment()
    print_scenarios(impact, alignment)
    raise SystemExit(0 if impact['passed'] and alignment['passed'] else 1)
