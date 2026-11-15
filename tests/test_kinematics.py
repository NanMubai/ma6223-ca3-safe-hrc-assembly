"""Independent Craig-DH model vs MuJoCo and central FK differences, N=1000."""
import numpy as np
import pytest
from numpy.testing import assert_allclose
from hrc_sim.kinematics import PandaKinematics, DH_PARAMETERS, modified_dh


@pytest.mark.parametrize('name', ['fk_position', 'fk_orientation', 'flange_position', 'flange_orientation'])
def test_fk_over_1000_full_range_configurations(modeling_random, name):
    metric = modeling_random['metrics'][name]
    assert metric['max'] < 1e-10, metric


@pytest.mark.parametrize('name,tolerance', [('jacobian_mujoco', 1e-10), ('jacobian_finite_difference', 1e-8), ('jacobian_dot', 1e-10)])
def test_analytic_jacobians_over_1000_configurations(modeling_random, name, tolerance):
    metric = modeling_random['metrics'][name]
    assert metric['max'] < tolerance, metric


def test_flange_tcp_offset_and_craig_order():
    kin = PandaKinematics()
    q = np.array([.2, -.3, .4, -1.7, .1, 1.8, .5])
    Tf, Tt = kin.fk_flange(q), kin.fk_tcp(q)
    assert_allclose(Tf[:3, :3], Tt[:3, :3])
    assert_allclose(Tt[:3, 3]-Tf[:3, 3], .06*Tf[:3, 2], atol=1e-15)
    # Craig order puts d3 along the already-twisted parent z, hence -world-y.
    a, alpha, d, offset = DH_PARAMETERS[2]
    A = modified_dh(a, alpha, d, offset)
    assert_allclose(A[:3, 3], [0, -.316, 0], atol=1e-15)
    with pytest.raises(ValueError):
        kin.fk_tcp(np.zeros(6))
