"""Physical regressions requested by the independent interface review."""
import numpy as np
from numpy.testing import assert_allclose
from hrc_sim import HRCSim, SimConfig


def test_free_hand_calibration(scenario_results):
    impact, _ = scenario_results
    for row in impact['rows']:
        assert row['relative_error'] < .02, row


def test_chamfer_capture_and_sharp_rim_jam(scenario_results):
    _, alignment = scenario_results
    rows = {r['case']: r for r in alignment['rows']}
    for name in ('lateral_1mm', 'lateral_2mm'):
        assert rows[name]['depth'] > .039
        assert rows[name]['peak_wall_force'] > 5.
    assert rows['sharp_lateral_1mm']['depth'] < .001
    assert rows['sharp_lateral_1mm']['outcome'] == 'jammed'


def test_hand_specific_contact_parameters_survive_global_override():
    cfg = SimConfig(solref=(.01, 1), solimp=(.8, .9, .002))
    with HRCSim(cfg) as env:
        assert_allclose(env.model.geom_solref[env.hand_geom_id], cfg.hand.contact_solref)
        assert_allclose(env.model.geom_solimp[env.hand_geom_id, :3], cfg.hand.contact_solimp)
        assert env.model.geom_solmix[env.hand_geom_id] == cfg.hand.solmix


def test_normal_metrics_are_distinct():
    with HRCSim() as env:
        from scripts.demo_insertion import run_insertion
        result = run_insertion(env, lateral_offset=.001)
        assert result['peak_wall_force'] > 5
        normals = []
        import mujoco
        for i, c in enumerate(env.data.contact):
            if env.model.geom('peg').id in (c.geom1, c.geom2):
                f = np.zeros(6); mujoco.mj_contactForce(env.model, env.data, i, f)
                normals.append(f[0])
        assert len(normals) > 1
        assert_allclose(env.get_contact_normal_sum('peg_hole'), sum(normals), atol=1e-12)
        assert_allclose(env.get_max_normal_force('peg_hole'), max(normals), atol=1e-12)
        assert env.get_contact_normal_sum('peg_hole') > env.get_max_normal_force('peg_hole')
