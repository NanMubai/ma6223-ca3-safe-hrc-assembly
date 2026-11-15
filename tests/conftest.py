"""Share the expensive 1000-state experiments across independent assertions."""
import pytest
from scripts.verify_modeling import (verify_random_samples, verify_task_space,
                                    verify_contacts, verify_singularity)


@pytest.fixture(scope='session')
def modeling_random():
    return verify_random_samples(n_samples=1000, seed=6223)


@pytest.fixture(scope='session')
def modeling_task():
    return verify_task_space(n_samples=1000, seed=6224)


@pytest.fixture(scope='session')
def modeling_contacts():
    return verify_contacts()


@pytest.fixture(scope='session')
def modeling_singularity():
    return verify_singularity()


@pytest.fixture(scope='session')
def scenario_results():
    from scripts.verify_scenarios import verify_hand_impacts, verify_misalignment
    return verify_hand_impacts(), verify_misalignment()
