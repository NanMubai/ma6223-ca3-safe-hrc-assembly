"""SI-unit configuration. Panda limits follow the FER/Panda (not FR3) datasheet."""
from dataclasses import dataclass, field
import numpy as np


def _a(values):
    return np.asarray(values, dtype=float)


@dataclass
class HandConfig:
    mass: float = 0.6  # ISO/TS 15066 effective hand mass: TBD by part A.
    radius: float = 0.04
    cylinder_half_length: float = 0.05  # Total capsule length = 0.18 m.
    parked_pos: tuple = (2.0, 0.0, 1.0)
    parked_quat: tuple = (1.0, 0.0, 0.0, 0.0)
    kinematic_solref: tuple = (0.002, 1.0)
    compliant_solref: tuple = (0.04, 1.0)
    weld_solimp: tuple = (0.95, 0.99, 0.001)
    gravity_compensation: bool = True
    contact_solref: tuple = (-125000.0, -1.0)  # Calibrated free impact: 75 N/mm at 0.6 kg.
    contact_solimp: tuple = (0.95, 0.95, 0.001)
    solmix: float = 1000.0
    placement_max_penetration: float = 0.001

    def __post_init__(self):
        if min(self.mass, self.radius, self.cylinder_half_length) <= 0:
            raise ValueError('Hand mass and dimensions must be positive')
        if self.solmix < 0 or self.placement_max_penetration < 0:
            raise ValueError('Hand solmix/placement threshold must be nonnegative')
        sr = np.asarray(self.contact_solref)
        if sr.shape != (2,) or not np.all(np.isfinite(sr)) or not (np.all(sr > 0) or np.all(sr < 0)):
            raise ValueError('contact_solref must be a positive time/damping pair or a negative direct pair')


@dataclass
class SimConfig:
    dt: float = 0.001
    gravity: tuple = (0.0, 0.0, -9.81)
    peg_half_sizes: tuple = (0.015, 0.015, 0.030)
    peg_mass: float = 0.2
    chamfer: float = 0.003  # 45-degree lead-in; zero restores the sharp rim.
    clearance: float = 0.0005  # Per side, not total diametral clearance.
    hole_depth: float = 0.040
    wall_thickness: float = 0.040
    hole_pos: tuple = (0.5, 0.0, 0.28)  # Entrance centre.
    hole_quat: tuple = (1.0, 0.0, 0.0, 0.0)
    table_half_sizes: tuple = (0.20, 0.20, 0.02)
    friction: tuple = (0.3, 0.005, 0.0001)
    solref: tuple = (0.004, 1.0)
    solimp: tuple = (0.95, 0.99, 0.001)
    q_home: np.ndarray = field(default_factory=lambda: _a([0, -0.181472146, 0, -2.121642174, 0, 1.940170028, 0.7853981633974483]))
    q_min: np.ndarray = field(default_factory=lambda: _a([-2.8973, -1.7628, -2.8973, -3.0718, -2.8973, -0.0175, -2.8973]))
    q_max: np.ndarray = field(default_factory=lambda: _a([2.8973, 1.7628, 2.8973, -0.0698, 2.8973, 3.7525, 2.8973]))
    dq_max: np.ndarray = field(default_factory=lambda: _a([2.175]*4 + [2.610]*3))
    tau_max: np.ndarray = field(default_factory=lambda: _a([87]*4 + [12]*3))
    gravity_compensation: bool = True
    damping_compensation: bool = False
    seed: int = 0
    sensor_bias_frame: str = 'sensor'
    tau_ext_bias: np.ndarray = field(default_factory=lambda: np.zeros(7))
    tau_ext_noise_std: np.ndarray = field(default_factory=lambda: np.zeros(7))
    sensor_bias: np.ndarray = field(default_factory=lambda: np.zeros(6))
    noise_std: np.ndarray = field(default_factory=lambda: np.zeros(6))
    hand: HandConfig = field(default_factory=HandConfig)
    task_damping: float = 1e-2
    camera: str = 'insertion'

    @property
    def hole_half_sizes(self):
        return np.asarray(self.peg_half_sizes[:2]) + self.clearance

    def __post_init__(self):
        for name in ('q_home', 'q_min', 'q_max', 'dq_max', 'tau_max'):
            a = np.asarray(getattr(self, name), dtype=float)
            if a.shape != (7,) or not np.all(np.isfinite(a)):
                raise ValueError(f'{name} must be a finite (7,) array')
            setattr(self, name, a.copy())
        for name in ('sensor_bias', 'noise_std', 'tau_ext_bias', 'tau_ext_noise_std'):
            a = np.broadcast_to(np.asarray(getattr(self, name), dtype=float), (7 if name.startswith('tau_ext') else 6,)).copy()
            if not np.all(np.isfinite(a)):
                raise ValueError(f'{name} must be finite')
            setattr(self, name, a)
        if np.any(self.noise_std < 0) or np.any(self.tau_ext_noise_std < 0):
            raise ValueError('noise_std must be nonnegative')
        if not isinstance(self.seed, (int, np.integer)) or self.seed < 0:
            raise ValueError('seed must be a nonnegative integer')
        if self.sensor_bias_frame not in ('sensor', 'world'):
            raise ValueError('sensor_bias_frame must be sensor or world')
        if not 0 <= self.chamfer < min(self.hole_depth, self.wall_thickness):
            raise ValueError('chamfer must be nonnegative and smaller than depth/wall thickness')
        if self.task_damping < 0 or not np.isfinite(self.task_damping):
            raise ValueError('task_damping must be finite and nonnegative')
        if self.dt <= 0 or self.solref[0] < 2*self.dt:
            raise ValueError('Require dt > 0 and contact solref time constant >= 2*dt')
        if np.any(self.q_min >= self.q_max) or np.any(self.tau_max <= 0) or np.any(self.dq_max <= 0):
            raise ValueError('Joint ranges, torque and velocity limits must be positive')
        if np.any(self.q_home < self.q_min) or np.any(self.q_home > self.q_max):
            raise ValueError('q_home outside joint limits')
        if min(self.peg_half_sizes) <= 0 or min(self.peg_mass, self.clearance, self.hole_depth, self.wall_thickness) <= 0:
            raise ValueError('Geometry dimensions, mass and clearance must be positive')


# A descriptive alias for downstream code.
HRCConfig = SimConfig


@dataclass
class ReferenceControllerConfig:
    """Constant gains for the interface example; part E supplies its own control."""
    stiffness: np.ndarray = field(default_factory=lambda: _a([1200, 1200, 800, 80, 80, 60]))
    damping: np.ndarray = field(default_factory=lambda: _a([85, 85, 65, 12, 12, 10]))
    posture_stiffness: float = 5.0
    posture_damping: float = 2.0
