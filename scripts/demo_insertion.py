#!/usr/bin/env python
"""Nominal and offset contact-rich insertion using the constant-gain example."""
import argparse
import os
from pathlib import Path
import numpy as np


def run_insertion(env, callback=None, lateral_offset=0., tilt_deg=0., yaw_deg=0.):
    from hrc_sim.reference_controller import ReferenceController
    controller = ReferenceController(env)
    env.reset(seed=0)
    start, R, _ = env.get_ee_pose()
    hole = env.data.site('hole_top').xpos.copy()
    Rh = env.data.site('hole_top').xmat.reshape(3, 3)
    axis = Rh[:, 2]
    # Square peg aligned to the square opening, local peg +z pointing down.
    a, b = np.deg2rad([tilt_deg, yaw_deg])
    Ry = np.array([[np.cos(a), 0, np.sin(a)], [0, 1, 0], [-np.sin(a), 0, np.cos(a)]])
    Rz = np.array([[np.cos(b), -np.sin(b), 0], [np.sin(b), np.cos(b), 0], [0, 0, 1]])
    Rd = Rh @ Ry @ np.diag([-1., 1., -1.]) @ Rz
    offset = lateral_offset*Rh[:, 0]
    above = hole + .02*axis + offset
    end = hole - (env.config.hole_depth+.001)*axis + offset
    peak = 0.0
    peak_normal = 0.0
    peak_normal_sum = 0.0
    peak_wall_force = 0.0
    count = 0
    # Smooth quintic position ramps; velocity feedback remains pure damping.
    for p0, p1, duration in [(start, start, .25), (start, above, 2.), (above, end, 4.), (end, end, 1.5)]:
        steps = int(round(duration/env.config.dt))
        for i in range(steps):
            u = (i+1)/steps
            blend = 10*u**3-15*u**4+6*u**5
            env.step(controller(p0+blend*(p1-p0), Rd))
            peak = max(peak, float(np.linalg.norm(env.get_contact_wrench('peg_hole')[:3])))
            peak_normal = max(peak_normal, env.get_max_normal_force('peg_hole'))
            peak_normal_sum = max(peak_normal_sum, env.get_contact_normal_sum('peg_hole'))
            wall = np.zeros(3)
            for body, point, f, t, normal in env._contacts('peg_hole'):
                # The floor is at hole_depth; every contact above it is a wall/rim.
                if (hole-point)@axis < env.config.hole_depth-.001:
                    wall += f
            peak_wall_force = max(peak_wall_force, float(np.linalg.norm(wall)))
            if callback is not None:
                callback(env, count)
            count += 1
    p, _, _ = env.get_ee_pose()
    depth = float((hole-p)@axis)
    lateral_error = float(np.linalg.norm((p-hole)-((p-hole)@axis)*axis))
    return dict(depth=depth, success=bool(depth >= .030),
                peak_force=peak, peak_normal_force=peak_normal, peak_normal_sum=peak_normal_sum, peak_wall_force=peak_wall_force,
                lateral_error=lateral_error, outcome='inserted' if depth >= .039 else ('partial' if depth >= .003 else 'jammed'))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--offscreen', action='store_true', help='Save outputs/demo_insertion.png using EGL')
    parser.add_argument('--video', action='store_true', help='Also save outputs/demo_insertion.mp4')
    args = parser.parse_args()
    if args.offscreen or args.video:
        os.environ.setdefault('MUJOCO_GL', 'egl')
    from hrc_sim import HRCSim, VideoRecorder
    out = Path(__file__).resolve().parents[1] / 'outputs'
    with HRCSim() as env:
        recorder = None
        if args.video:
            recorder = VideoRecorder(out/'demo_insertion.mp4')
        stride = max(1, int(round(1/(30*env.config.dt))))
        def callback(sim, i):
            if recorder is not None and i % stride == 0:
                recorder.capture(sim)
        try:
            results = []
            for label, lateral in [('nominal', 0.), ('offset_1.5mm', .0015)]:
                result = run_insertion(env, callback, lateral_offset=lateral)
                results.append(result)
                print(f"Case: {label}")
                print(f"Final depth: {result['depth']*1000:.3f} mm")
                print(f"Success: {result['success']} ({result['outcome']})")
                print(f"Peak peg-hole resultant force: {result['peak_force']:.3f} N")
                print(f"Peak individual normal force: {result['peak_normal_force']:.3f} N")
                print(f"Peak normal sum: {result['peak_normal_sum']:.3f} N")
                print(f"Peak wall/chamfer resultant: {result['peak_wall_force']:.3f} N")
                if args.offscreen or args.video:
                    import imageio.v2 as imageio
                    out.mkdir(parents=True, exist_ok=True)
                    path = out/('demo_insertion.png' if label == 'nominal' else 'demo_insertion_offset.png')
                    imageio.imwrite(path, env.render())
                    print(f'Saved image: {path}')
            if args.video:
                print(f'Saved video: {out / "demo_insertion.mp4"}')
        finally:
            if recorder is not None:
                recorder.close()
    return 0 if all(r['success'] for r in results) else 1


if __name__ == '__main__':
    raise SystemExit(main())
