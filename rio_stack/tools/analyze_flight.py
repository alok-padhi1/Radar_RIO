#!/usr/bin/env python3
"""
analyze_run_v2.py -- verified post-flight analyzer for the RIO/SLAM radar stack.

Replaces tools/analyze_run.py. What is different, and why:

  1. COVERAGE-AWARE.  doppler_rio.py logs only the frames it accepts. The old
     analyzer integrated RIO over the accepted frames but compared the result
     against the FULL-flight GPS path -- on your 2026-09-20 logs RIO is absent
     for 55-70% of the flight, so that comparison manufactures a 50-70%
     "distance error" that has nothing to do with velocity accuracy. Here,
     every RIO-vs-GPS comparison is computed over the identical time support.

  2. PER-AXIS, NOT SPEED-MAGNITUDE.  The old velocity check compared |v_rio|
     against |v_gps|. That test cannot see a swapped axis, a sign flip, a
     wrong tilt angle or a heading error. This one reports bias / RMSE /
     correlation / regression slope per axis in ENU *and* in body FRD, and
     then solves for the residual mount rotation directly.

  3. SLAM ERROR AT THREE LEVELS.  Raw (absolute), yaw-aligned, and full
     Kabsch shape. The old analyzer's "Absolute XY error" silently included
     an arbitrary map-yaw offset whenever slam_node ran with its shipped
     default --no-trust-imu-yaw.

  4. SELF-VERIFYING.  `--selftest` builds synthetic logs whose answers are
     known in closed form and asserts that every metric above is recovered.
     If the self-test passes, a number this tool prints is a statement about
     your flight, not about the tool.

Usage
    python3 analyze_run_v2.py --selftest
    python3 analyze_run_v2.py logs/run_XXXX.jsonl
    python3 analyze_run_v2.py logs/run_XXXX.jsonl --plot out.png --json out.json
    python3 analyze_run_v2.py logs/*.jsonl --brief          # fleet summary

Requires: numpy (mandatory), matplotlib (only for --plot)
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rio_eval import (                                      # noqa: E402
    Run, load_run, R_body_to_ned_batch, ned_to_enu, interp_vec, nearest_rpy,
    gps_velocity, path_length_multi, speed_integrated_path,
    gps_displacement_over, coverage_intervals, rio_to_enu, integrate_velocity,
    axis_stats, estimate_time_lag, estimate_misalignment, slam_alignment,
    altimeter_check, lla_to_enu, enu_scale_factors,
)

BAR = "=" * 78
SUB = "-" * 78


# ═════════════════════════════════════════════════════════════════════════
#  Analysis
# ═════════════════════════════════════════════════════════════════════════

def analyse(run: Run, cfg) -> dict:
    R = {'path': run.path, 'warnings': list(run.warnings), 'cfg': vars(cfg).copy()}
    t0 = run.t0
    R['t0'] = t0
    R['duration'] = run.duration

    # ---------------------------------------------------------------- rates
    def rate(t):
        return len(t) / max(t[-1] - t[0], 1e-6) if len(t) > 1 else 0.0

    R['rates'] = {k: rate(v) for k, v in (
        ('gps', run.gps_t), ('rio', run.rio_t), ('slam', run.slam_t),
        ('imu', run.imu_t), ('alt', run.alt_t))}
    R['counts'] = {k: int(len(v)) for k, v in (
        ('gps', run.gps_t), ('rio', run.rio_t), ('slam', run.slam_t),
        ('imu', run.imu_t), ('alt', run.alt_t))}

    # ------------------------------------------------- pipeline latency
    if len(run.rio_t) and len(run.rio_t_recv):
        lat = run.rio_t_recv - run.rio_t
        R['rio_latency'] = {'mean': float(lat.mean()),
                            'p50': float(np.median(lat)),
                            'p95': float(np.percentile(lat, 95)),
                            'max': float(lat.max())}
    if len(run.slam_t) and len(run.slam_t_recv):
        lat = run.slam_t_recv - run.slam_t
        R['slam_latency'] = {'mean': float(lat.mean()),
                             'p50': float(np.median(lat)),
                             'p95': float(np.percentile(lat, 95)),
                             'max': float(lat.max())}

    # ------------------------------------------------- GPS ground truth
    if len(run.gps_t) > 2:
        v_gps, v_ok = gps_velocity(run.gps_t, run.gps_enu, cfg.gps_window,
                                   v_native=(None if cfg.force_diff_velocity
                                             else run.gps_v_enu))
        R['gps_v'] = v_gps
        R['gps_v_ok'] = v_ok
        R['gps_v_source'] = ('GLOBAL_POSITION_INT.vx/vy/vz (native EKF velocity)'
                             if (run.gps_v_enu is not None and not cfg.force_diff_velocity)
                             else f'centred {cfg.gps_window:.2f} s difference of ENU position')
        sp = np.linalg.norm(v_gps[v_ok], axis=1)
        R['gps_speed'] = {'mean': float(sp.mean()), 'max': float(sp.max()),
                          'p95': float(np.percentile(sp, 95))}
        # noise floor: median speed over the slowest decile (i.e. while parked)
        R['gps_noise_floor_mps'] = float(np.median(np.sort(sp)[:max(1, len(sp) // 10)]))
        R['gps_paths'] = path_length_multi(run.gps_t, run.gps_enu)
        d = run.gps_enu[-1] - run.gps_enu[0]
        R['gps_disp_enu'] = d
        R['gps_disp_xy'] = float(np.linalg.norm(d[:2]))
        R['gps_disp_3d'] = float(np.linalg.norm(d))
        R['gps_extent'] = (run.gps_enu.max(0) - run.gps_enu.min(0))
        R['gps_path_int'] = speed_integrated_path(run.gps_t, run.gps_enu,
                                                  window_s=cfg.gps_window)
        R['gps_path_int_db'] = speed_integrated_path(
            run.gps_t, run.gps_enu, window_s=cfg.gps_window,
            deadband_mps=R['gps_noise_floor_mps'] * 2.0)

    # ------------------------------------------------- RIO
    if len(run.rio_t) > 2:
        iv, cov_s = coverage_intervals(run.rio_t, cfg.max_gap)
        span = run.rio_t[-1] - run.rio_t[0]
        gaps = np.diff(run.rio_t)
        big = gaps[gaps > cfg.max_gap]
        R['rio_cov'] = {
            'intervals': iv, 'covered_s': cov_s,
            'span_s': float(span),
            'frac_of_span': cov_s / span if span > 0 else 0.0,
            'frac_of_flight': cov_s / max(R['duration'], 1e-6),
            'n_gaps': int(len(big)),
            'gap_total_s': float(big.sum()) if len(big) else 0.0,
            'gap_max_s': float(big.max()) if len(big) else 0.0,
            'gaps_sorted': np.sort(big)[::-1][:8].tolist() if len(big) else [],
            'median_dt': float(np.median(gaps)),
            'implied_hz': 1.0 / float(np.median(gaps)) if np.median(gaps) > 0 else 0.0,
        }

        v_enu, v_ned, att_ok = rio_to_enu(run.rio_t, run.rio_v,
                                          run.imu_t, run.imu_rpy, cfg.imu_max_dt)
        R['rio_v_enu'] = v_enu
        R['rio_att_ok'] = att_ok
        R['rio_att_missing'] = int((~att_ok).sum())
        if att_ok.sum() == 0:
            R['warnings'].append(
                "NO usable FC attitude for ANY RIO sample"
                + (" (the log contains no 'imu' entries at all)" if len(run.imu_t) == 0
                   else " (every sample was outside --imu-max-dt)")
                + ". RIO velocity is a BODY-frame vector; without attitude it cannot "
                  "be placed in the world, so every RIO-vs-GPS number below is "
                  "undefined, NOT zero. Re-fly with imu_bridge.py running.")

        disp, L3, L2 = integrate_velocity(run.rio_t, v_enu, iv)
        R['rio_disp_enu'] = disp
        R['rio_path_3d'] = L3
        R['rio_path_2d'] = L2

        # Body-frame integral, for reference only (this is what gps_logger.py
        # and tools/plot_run.py currently report -- it is wrong under yaw).
        disp_b, L3b, _ = integrate_velocity(run.rio_t, run.rio_v, iv)
        R['rio_disp_body_WRONG'] = disp_b

        if len(run.gps_t) > 2:
            # matched-support GPS truth
            R['gps_path_matched'] = speed_integrated_path(
                run.gps_t, run.gps_enu, intervals=iv, window_s=cfg.gps_window)
            R['gps_disp_matched'] = gps_displacement_over(run.gps_t, run.gps_enu, iv)

            gv = interp_vec(run.gps_t, R['gps_v'], run.rio_t)
            inside = np.zeros(len(run.rio_t), bool)
            for a, b in iv:
                inside |= (run.rio_t >= a) & (run.rio_t <= b)
            in_gps = (run.rio_t >= run.gps_t[0]) & (run.rio_t <= run.gps_t[-1])
            use = inside & in_gps & att_ok
            R['n_vel_pairs'] = int(use.sum())

            R['vel_enu'] = [axis_stats(v_enu[use, k], gv[use, k], n)
                            for k, n in enumerate(('East', 'North', 'Up'))]
            R['vel_speed'] = axis_stats(np.linalg.norm(v_enu[use], axis=1),
                                        np.linalg.norm(gv[use], axis=1), 'speed3D')
            R['vel_hspeed'] = axis_stats(np.linalg.norm(v_enu[use, :2], axis=1),
                                         np.linalg.norm(gv[use, :2], axis=1), 'speedXY')

            # body-frame comparison: project GPS velocity into body FRD
            rpy, _ = nearest_rpy(run.imu_t, run.imu_rpy, run.rio_t, cfg.imu_max_dt)
            if rpy is not None:
                Rb = R_body_to_ned_batch(rpy)
                gv_ned = ned_to_enu(gv)                       # ENU->NED (involutive)
                gv_body = np.einsum('nji,nj->ni', Rb, gv_ned)  # R^T @ v
                R['gps_v_body'] = gv_body
                R['vel_body'] = [axis_stats(run.rio_v[use, k], gv_body[use, k], n)
                                 for k, n in enumerate(('fwd(+x)', 'right(+y)', 'down(+z)'))]
                R['misalign'] = estimate_misalignment(
                    run.rio_v[use], gv_body[use], min_speed=cfg.min_speed)

            # time lag
            R['lag'] = estimate_time_lag(
                run.rio_t[use], np.linalg.norm(v_enu[use], axis=1),
                run.gps_t, np.linalg.norm(np.nan_to_num(R['gps_v']), axis=1),
                max_lag_s=cfg.max_lag)

            # stationary-bias test: what does RIO say while GPS says parked?
            gsp = np.linalg.norm(gv, axis=1)
            still = use & np.isfinite(gsp) & (gsp < cfg.still_speed)
            if still.sum() >= 5:
                R['still'] = {
                    'n': int(still.sum()),
                    'gps_mean': float(gsp[still].mean()),
                    'rio_speed_mean': float(np.linalg.norm(v_enu[still], axis=1).mean()),
                    'rio_hspeed_mean': float(np.linalg.norm(v_enu[still, :2], axis=1).mean()),
                    'rio_hspeed_p95': float(np.percentile(
                        np.linalg.norm(v_enu[still, :2], axis=1), 95)),
                    'rio_vspeed_mean': float(np.abs(v_enu[still, 2]).mean()),
                    'phantom_m_per_min': float(
                        np.linalg.norm(v_enu[still, :2], axis=1).mean() * 60.0),
                }
            # fast-motion test: does RIO survive when the aircraft really moves?
            # Evaluated on the GPS timebase -- asking "were RIO samples inside a
            # RIO interval" is circular and always answers ~100 %.
            g_sp = np.linalg.norm(np.where(v_ok[:, None], R['gps_v'], 0.0), axis=1)
            g_inside = np.zeros(len(run.gps_t), bool)
            for a, b in iv:
                g_inside |= (run.gps_t >= a) & (run.gps_t <= b)
            fast_g = v_ok & (g_sp > cfg.fast_speed)
            slow_g = v_ok & (g_sp <= cfg.still_speed)
            if fast_g.sum() >= 5:
                R['fast'] = {'n': int(fast_g.sum()),
                             'n_covered': int((fast_g & g_inside).sum()),
                             'frac_covered': float((fast_g & g_inside).sum() / fast_g.sum())}
            if slow_g.sum() >= 5:
                R['slow_cov'] = float((slow_g & g_inside).sum() / slow_g.sum())

        R['rio_health'] = {
            'inliers_mean': float(run.rio_inliers.mean()),
            'inliers_min': int(run.rio_inliers.min()),
            'ntotal_mean': float(run.rio_ntotal.mean()),
            'inlier_ratio_mean': float(np.mean(run.rio_inliers /
                                               np.clip(run.rio_ntotal, 1, None))),
            'static_frac': float(run.rio_static.mean()),
            'airborne_frac': float(run.rio_airborne.mean()),
            'vzprior_frac': float(run.rio_vzprior.mean()),
            'cond_max': float(run.rio_cond.max()),
            'cond_p95': float(np.percentile(run.rio_cond, 95)),
        }

    # ------------------------------------------------- SLAM
    lag = R.get('lag', (0.0, 0.0))[0] if cfg.apply_lag and R.get('lag') else 0.0
    R['slam'] = slam_alignment(run, alt_ref=True, lag_s=lag)
    if len(run.slam_t) > 1:
        gaps = np.diff(run.slam_t)
        R['slam_cov'] = {
            'first_s': float(run.slam_t[0] - t0),
            'last_s': float(run.slam_t[-1] - t0),
            'span_s': float(run.slam_t[-1] - run.slam_t[0]),
            'frac_of_flight': float((run.slam_t[-1] - run.slam_t[0]) /
                                    max(R['duration'], 1e-6)),
            'median_dt': float(np.median(gaps)),
            'gap_max_s': float(gaps.max()),
            'silent_tail_s': float(R['t0'] + R['duration'] - run.slam_t[-1]),
        }

    # ------------------------------------------------- Altimeter
    R['alt'] = altimeter_check(run)

    # ------------------------------------------------- Missing instrumentation
    miss = []
    if len(run.rio_t) and run.gps_v_enu is None:
        miss.append("GPS/EKF velocity (GLOBAL_POSITION_INT.vx/vy/vz) is not logged. "
                    "Ground-truth velocity has to be differentiated from position, "
                    "which costs ~0.1 m/s of noise and ~0.3 s of bandwidth. "
                    "Log the native fields: they are already in the message.")
    if len(run.rio_t):
        miss.append("Rejected RIO frames are not logged. The log cannot distinguish "
                    "'radar saw nothing' from 'gate rejected it' from 'process died'. "
                    "Log a {'type':'rio_reject','reason':...} line per rejected frame.")
        miss.append("Per-frame RIO velocity covariance (cxx,cyy,czz) is received by "
                    "gps_logger.py and then discarded. It is the number the autopilot "
                    "EKF actually weights on -- log it.")
    if len(run.slam_t):
        miss.append("SLAM fitness / inlier_rmse / n_corr / n_observable_axes are "
                    "computed in slam_node.py but never transmitted. Without them a "
                    "bad pose is indistinguishable from a good one post-flight.")
        miss.append("Rejected SLAM keyframes and their reason codes are not logged.")
    if len(run.rio_t) and len(run.alt_t) == 0:
        miss.append("No altimeter stream: SLAM Z has no absolute reference.")
    miss.append("The radar frame's own parse timestamp is not on the wire "
                "(radar_fanout.py captures Frame.t and then never sends it), so "
                "every timestamp in this log is a receive time, not an epoch.")
    R['missing'] = miss

    return R


# ═════════════════════════════════════════════════════════════════════════
#  Report
# ═════════════════════════════════════════════════════════════════════════

def _f(x, n=3, unit=""):
    if x is None or (isinstance(x, float) and not math.isfinite(x)):
        return "   n/a"
    return f"{x:.{n}f}{unit}"


def _stat_row(s):
    if not s or s.get('n', 0) < 3:
        return f"  {s.get('label',''):<12s}  (insufficient samples)"
    return (f"  {s['label']:<12s} {s['bias']:+8.3f} {s['rmse']:8.3f} "
            f"{s['p95']:8.3f} {s['corr']:8.3f} {s['slope']:8.3f} "
            f"{s['std_est']:8.3f} {s['std_truth']:8.3f}")


def print_report(run: Run, R, cfg):
    p = print
    p(f"\n{BAR}")
    p(f"  RIO/SLAM POST-FLIGHT ANALYSIS v2   --   {os.path.basename(R['path'])}")
    p(BAR)

    for w in R['warnings']:
        p(f"  [log warning] {w}")

    c, rt = R['counts'], R['rates']
    p(f"\n[1] SESSION")
    p(f"  Duration {R['duration']:.1f} s")
    p(f"  {'stream':<8}{'count':>8}{'rate Hz':>10}")
    for k in ('gps', 'imu', 'alt', 'rio', 'slam'):
        p(f"  {k:<8}{c[k]:>8d}{rt[k]:>10.2f}")
    if 'rio_latency' in R:
        L = R['rio_latency']
        p(f"  RIO publish->log latency : p50 {L['p50']*1e3:.1f} ms  "
          f"p95 {L['p95']*1e3:.1f} ms  max {L['max']*1e3:.1f} ms")
    if 'slam_latency' in R:
        L = R['slam_latency']
        p(f"  SLAM keyframe->log latency: p50 {L['p50']*1e3:.0f} ms  "
          f"p95 {L['p95']*1e3:.0f} ms  max {L['max']*1e3:.0f} ms   "
          f"(this is your GICP compute time -- it is also pose age)")

    # ---------------------------------------------------------------- GPS
    if 'gps_paths' in R:
        p(f"\n[2] GPS / FC-EKF GROUND TRUTH")
        p("  Source: GLOBAL_POSITION_INT -> ENU metres, origin = first fix.")
        p("  NOTE: this is the autopilot's *fused* EKF position, not raw GNSS. It is")
        p("        smooth and low-latency but it is NOT independent of the IMU.")
        d = R['gps_disp_enu']
        p(f"\n  Net displacement first->last fix (ENU):")
        p(f"      East  {d[0]:+9.2f} m")
        p(f"      North {d[1]:+9.2f} m")
        p(f"      Up    {d[2]:+9.2f} m")
        p(f"      |XY|  {R['gps_disp_xy']:9.2f} m      |XYZ| {R['gps_disp_3d']:.2f} m")
        e = R['gps_extent']
        p(f"  Bounding-box extent: E {e[0]:.1f} m   N {e[1]:.1f} m   U {e[2]:.1f} m")
        p(f"\n  Path length is decimation-dependent (coastline paradox). All of these")
        p(f"  are 'the GPS path length' -- pick one deliberately, do not let a tool")
        p(f"  pick for you:")
        p(f"      {'decimation':<14}{'3D (m)':>10}{'2D (m)':>10}")
        for s, (a, b) in sorted(R['gps_paths'].items()):
            lbl = "raw (50 Hz)" if s == 0 else f"{s:.1f} s"
            p(f"      {lbl:<14}{a:>10.2f}{b:>10.2f}")
        a3, a2 = R['gps_path_int']
        p(f"      {'integral |v|':<14}{a3:>10.2f}{a2:>10.2f}   <- used for RIO comparison")
        b3, b2 = R['gps_path_int_db']
        p(f"      {'  + deadband':<14}{b3:>10.2f}{b2:>10.2f}   "
          f"(noise floor {R['gps_noise_floor_mps']:.3f} m/s removed)")
        sp = R['gps_speed']
        p(f"  Speed: mean {sp['mean']:.2f}  p95 {sp['p95']:.2f}  max {sp['max']:.2f} m/s")
        p(f"\n  HOW GPS VELOCITY IS COMPUTED HERE:")
        p(f"    source: {R.get('gps_v_source','?')}")
        if 'native' in R.get('gps_v_source', ''):
            p(f"    The autopilot's own EKF velocity, straight off the wire in cm/s.")
            p(f"    Lower noise and full bandwidth -- always preferred when logged.")
            p(f"    (--force-diff-velocity falls back to differencing, for comparison.)")
            p(f"    Caveat: it is EKF output, so it is not independent of the IMU,")
            p(f"    and during a GNSS outage it is dead reckoning, not ground truth.")
        else:
            p(f"    centred finite difference over a {cfg.gps_window:.2f} s window:")
            p(f"        v(t) = ( p(t+w/2) - p(t-w/2) ) / w      [ENU, m/s]")
            p(f"    Centred, so there is no group delay. The window low-passes")
            p(f"    position noise: raw 50 Hz differencing gives sigma_v ~ 3 m/s.")
            p(f"    Cost: motion faster than ~{1/cfg.gps_window:.1f} Hz is attenuated.")
            p(f"    NOTE: GLOBAL_POSITION_INT already carries vx/vy/vz. Log them")
            p(f"    (patched gps_logger.py does) and this whole approximation goes away.")
        p(f"    Residual noise floor measured on this flight: "
          f"{R['gps_noise_floor_mps']:.3f} m/s -- any RIO error below that is")
        p(f"    NOT measurable with this ground truth.")

    # ---------------------------------------------------------------- RIO
    if 'rio_cov' in R:
        cv = R['rio_cov']
        p(f"\n[3] RIO COVERAGE   <-- read this before any distance number")
        p(f"  Median inter-frame dt {cv['median_dt']*1e3:.0f} ms "
          f"(=> {cv['implied_hz']:.1f} Hz when publishing)")
        p(f"  Time with valid RIO  : {cv['covered_s']:.1f} s of "
          f"{R['duration']:.1f} s flight = {100*cv['frac_of_flight']:.1f} %")
        p(f"  Dropouts > {cfg.max_gap:.2f} s   : {cv['n_gaps']} gaps, "
          f"{cv['gap_total_s']:.1f} s total, longest {cv['gap_max_s']:.1f} s")
        if cv['gaps_sorted']:
            p(f"  Longest gaps (s)     : "
              + ", ".join(f"{g:.1f}" for g in cv['gaps_sorted']))
        if cv['frac_of_flight'] < 0.9:
            p(f"  !! {100*(1-cv['frac_of_flight']):.0f} % of this flight has NO RIO output at all.")
            p(f"     doppler_rio.py logs only accepted frames, so a dropout and a")
            p(f"     crashed process look identical here. Every RIO number below is")
            p(f"     computed ONLY over the {100*cv['frac_of_flight']:.0f} % that is covered.")
        if 'fast' in R:
            fa = R['fast']
            p(f"  Coverage while GPS speed > {cfg.fast_speed} m/s: "
              f"{100*fa['frac_covered']:.0f} % ({fa['n_covered']}/{fa['n']} samples)")
            if 'slow_cov' in R:
                p(f"  Coverage while GPS speed < {cfg.still_speed} m/s: "
                  f"{100*R['slow_cov']:.0f} %")
            if fa['frac_covered'] < 0.6:
                p(f"     !! RIO preferentially drops out during real motion. A velocity")
                p(f"        estimator that only works while you hover is not usable.")
            elif 'slow_cov' in R and R['slow_cov'] < 0.5 < fa['frac_covered']:
                p(f"     RIO holds up under motion but drops out at low speed. That is")
                f_ = "     the expected signature of the min-inlier / static-margin gates"
                p(f_)
                p(f"     firing when Doppler spread collapses towards zero.")

        p(f"\n[4] RIO DISTANCE & DISPLACEMENT  (matched time support)")
        rd, gd = R['rio_disp_enu'], R.get('gps_disp_matched')
        p(f"  Integration: trapezoidal, body FRD -> NED via FC attitude -> ENU.")
        p(f"  Both columns cover the SAME {R['rio_cov']['covered_s']:.0f} s.")
        p(f"  {'':<10}{'RIO':>12}{'GPS':>12}{'error':>12}{'err %':>9}")
        if gd is not None:
            for k, n in enumerate(('East', 'North', 'Up')):
                e = rd[k] - gd[k]
                pct = (f"{100*abs(e)/abs(gd[k]):>8.1f}%"
                       if abs(gd[k]) > 1.0 else f"{'--':>9}")
                p(f"  {n:<10}{rd[k]:>+12.2f}{gd[k]:>+12.2f}{e:>+12.2f}{pct}")
            p(f"  {'|XY|':<10}{np.linalg.norm(rd[:2]):>12.2f}"
              f"{np.linalg.norm(gd[:2]):>12.2f}"
              f"{np.linalg.norm(rd[:2]-gd[:2]):>12.2f}")
            p(f"  {'|XYZ|':<10}{np.linalg.norm(rd):>12.2f}"
              f"{np.linalg.norm(gd):>12.2f}{np.linalg.norm(rd-gd):>12.2f}")
            if np.linalg.norm(gd) > 1.0 and np.linalg.norm(rd) > 1e-6:
                cosang = float(rd @ gd / (np.linalg.norm(rd) * np.linalg.norm(gd)))
                p(f"  Angle between the RIO and GPS displacement vectors: "
                  f"{math.degrees(math.acos(max(-1,min(1,cosang)))):.0f} deg")
                p(f"  (magnitudes can agree while the direction is wrong -- always")
                p(f"   check this before believing a 'distance error' percentage.)")
        gm = R.get('gps_path_matched')
        if gm:
            p(f"\n  Path length over the SAME covered intervals:")
            p(f"      RIO 3D {R['rio_path_3d']:8.2f} m     GPS 3D {gm[0]:8.2f} m"
              f"     ratio {R['rio_path_3d']/max(gm[0],1e-6):.3f}")
            p(f"      RIO 2D {R['rio_path_2d']:8.2f} m     GPS 2D {gm[1]:8.2f} m"
              f"     ratio {R['rio_path_2d']/max(gm[1],1e-6):.3f}")
            full = R['gps_path_int'][0]
            p(f"\n  For contrast, the OLD analyzer's comparison "
              f"(RIO-over-covered vs GPS-over-EVERYTHING):")
            p(f"      RIO {R['rio_path_3d']:.2f} m vs GPS {full:.2f} m = "
              f"{100*abs(R['rio_path_3d']-full)/max(full,1e-6):.1f} % 'error',")
            p(f"      of which {100*(1-R['rio_cov']['frac_of_flight']):.0f} points are "
              f"pure missing-data artefact.")
        bw = R.get('rio_disp_body_WRONG')
        if bw is not None:
            p(f"\n  (gps_logger.py and tools/plot_run.py integrate RIO in BODY frame")
            p(f"   without rotating by heading. That gives |disp| = "
              f"{np.linalg.norm(bw):.2f} m here, which is")
            p(f"   meaningless as soon as the aircraft yaws. Ignore those numbers.)")
        if R.get('rio_att_missing'):
            p(f"  !! {R['rio_att_missing']} RIO samples had no attitude within "
              f"{cfg.imu_max_dt}s and were excluded.")

        # ------------------------------------------------ velocity accuracy
        if 'vel_enu' in R:
            p(f"\n[5] RIO VELOCITY ACCURACY   [{R['n_vel_pairs']} matched samples]")
            p(f"  slope = least-squares gain of RIO on GPS; 1.000 is perfect scale.")
            p(f"  {'axis':<12} {'bias':>8} {'rmse':>8} {'p95':>8} {'corr':>8} "
              f"{'slope':>8} {'sd_rio':>8} {'sd_gps':>8}")
            p(f"  ENU frame:")
            for s in R['vel_enu']:
                p(_stat_row(s))
            p(_stat_row(R['vel_speed']))
            p(_stat_row(R['vel_hspeed']))
            if 'vel_body' in R:
                p(f"  BODY FRD frame (GPS velocity projected into body):")
                for s in R['vel_body']:
                    p(_stat_row(s))

        if 'misalign' in R and R['misalign']:
            m = R['misalign']
            sig = float(np.nanstd(np.linalg.norm(
                np.nan_to_num(R.get('gps_v_body', np.zeros((1, 3)))), axis=1)))
            frac = m['resid_rms'] / max(sig, 1e-6)
            p(f"\n[6] MOUNT / CALIBRATION RESIDUAL   [{m['n']} samples above "
              f"{cfg.min_speed} m/s]")
            p(f"  Best-fit rotation taking GPS body velocity onto RIO body velocity:")
            p(f"      roll  {m['roll_deg']:+7.2f} deg")
            p(f"      pitch {m['pitch_deg']:+7.2f} deg   <- adds to --theta-tilt-deg")
            p(f"      yaw   {m['yaw_deg']:+7.2f} deg")
            p(f"      scale {m['scale']:.4f}            (1.0 = correct Doppler scale)")
            p(f"      residual RMS {m['resid_rms']:.3f} m/s "
              f"({100*frac:.0f} % of the signal's own spread)")
            if frac > 0.6:
                p(f"  >> FIT IS NOT MEANINGFUL. No single rigid rotation explains this")
                p(f"     data: the residual is as large as the signal. Read the angles")
                p(f"     above as 'the velocity error is not a mounting error' -- it is")
                p(f"     noise or hallucination. Do NOT re-shim the mount on the")
                p(f"     strength of these numbers. Re-run this section once the")
                p(f"     hover-phantom in [7] is under control; only then are the")
                p(f"     angles a calibration statement.")
            elif m['reflection']:
                p(f"  !! The best fit is a REFLECTION, not a rotation. That means an")
                p(f"     axis is inverted -- check --lateral-sign and the TiltMount P")
                p(f"     matrix. No amount of tilt tuning fixes a handedness error.")
            if frac <= 0.6:
                if abs(m['pitch_deg']) > 3:
                    p(f"  !! Pitch residual > 3 deg: your measured mount tilt is off by")
                    p(f"     about this much, or the AHRS pitch trim is. Re-measure with")
                    p(f"     an inclinometer before touching any solver gain.")
                if abs(m['yaw_deg']) > 5:
                    p(f"  !! Yaw residual > 5 deg: radar boresight is not aligned with")
                    p(f"     the airframe x-axis, or the compass is off.")
                if not (0.9 < m['scale'] < 1.1):
                    p(f"  !! Scale error {100*(m['scale']-1):+.1f} %: Doppler scaling or")
                    p(f"     the range gate is wrong -- a multiplicative distance error.")
            R['misalign_meaningful'] = bool(frac <= 0.6)

        if 'still' in R:
            s = R['still']
            p(f"\n[7] STATIONARY / HOVER BIAS   [{s['n']} samples with GPS speed < "
              f"{cfg.still_speed} m/s]")
            p(f"      GPS  speed  {s['gps_mean']:.3f} m/s   (ground truth: parked)")
            p(f"      RIO  |v|    {s['rio_speed_mean']:.3f} m/s")
            p(f"      RIO  |v_xy| {s['rio_hspeed_mean']:.3f} m/s  "
              f"(p95 {s['rio_hspeed_p95']:.3f})")
            p(f"      RIO  |v_z|  {s['rio_vspeed_mean']:.3f} m/s")
            p(f"      => phantom horizontal drift {s['phantom_m_per_min']:.1f} m per minute "
              f"of hover")
            if s['rio_hspeed_mean'] > 0.3:
                p(f"  !! This is the Vx/Vz null-direction leakage. At a {cfg.tilt_hint:.0f} deg")
                p(f"     tilt the LOS cone barely separates forward from vertical, so")
                p(f"     Doppler noise converts into phantom forward speed. Fusing this")
                p(f"     into an EKF as position aiding will walk the aircraft away.")

        if R.get('lag'):
            lg, cc = R['lag']
            p(f"\n[8] TIME ALIGNMENT")
            p(f"  Best RIO-vs-GPS speed correlation {cc:.3f} at lag {lg:+.3f} s")
            p(f"  (positive = RIO is late). Set EKF2_EV_DELAY / EK3_VIS_DELAY to")
            p(f"  approximately {max(lg,0)*1000:.0f} ms. Note the logged publish->log")
            p(f"  latency above is only the last hop; this number is end-to-end.")
            if cc < 0.7:
                p(f"  !! Correlation below 0.7 at every lag means the two signals do not")
                p(f"     describe the same motion. Do not tune a delay against this --")
                p(f"     fix the velocity first.")

        if len(run.rio_rej_t):
            from collections import Counter
            cnt = Counter(run.rio_rej_reason)
            tot = len(run.rio_rej_reason) + len(run.rio_t)
            p(f"\n[8b] RIO REJECTION BREAKDOWN   <-- why the gaps exist")
            p(f"  Accepted {len(run.rio_t)} / {tot} frames "
              f"= {100*len(run.rio_t)/max(tot,1):.1f} % accept rate")
            for reason, c in cnt.most_common():
                p(f"      {reason:<28s} {c:6d}  ({100*c/max(tot,1):5.1f} % of all frames)")
            p(f"  This is the single most useful diagnostic in the log. Tune the gate")
            p(f"  that dominates -- not every gate at once.")
        elif len(run.rio_t):
            p(f"\n[8b] RIO REJECTION BREAKDOWN: not available.")
            p(f"  Launch doppler_rio.py with --reject-ports 5015 and gps_logger.py")
            p(f"  with --reject-port 5015. Without it the {100*(1-R['rio_cov']['frac_of_flight']):.0f} % of missing")
            p(f"  timeline above has no attributable cause and you are guessing.")

        if run.rio_sigma is not None:
            sg = run.rio_sigma
            p(f"\n[8c] PUBLISHED VELOCITY COVARIANCE (what the autopilot weights on)")
            p(f"  sigma published to the EKF, m/s:")
            for k, n in enumerate(('x/fwd', 'y/right', 'z/down')):
                col = sg[:, k]
                p(f"      {n:<10s} p50 {np.median(col):.4f}  p95 "
                  f"{np.percentile(col,95):.4f}  max {col.max():.4f}")
            if 'vel_body' in R:
                emp = [st['rmse'] for st in R['vel_body']]
                p(f"  measured body-axis RMSE vs GPS, m/s:")
                p(f"      {'fwd/right/down':<10s} "
                  + "  ".join(f"{e:.3f}" for e in emp))
                ratio = [e / max(np.median(sg[:, k]), 1e-6) for k, e in enumerate(emp)]
                p(f"  actual error / claimed sigma: "
                  + "  ".join(f"{r:.1f}x" for r in ratio))
                if max(ratio) > 3:
                    p(f"  !! The solver is understating its own uncertainty by up to")
                    p(f"     {max(ratio):.0f}x. The EKF will over-trust this measurement by")
                    p(f"     roughly the square of that. Check finding R-1 (covariance")
                    p(f"     truncation) and R-7 (Huber sandwich estimator).")

        h = R['rio_health']
        p(f"\n[9] RIO SOLVER HEALTH (of the frames that were published)")
        p(f"  inliers mean {h['inliers_mean']:.1f}  min {h['inliers_min']}  "
          f"of {h['ntotal_mean']:.1f} gated points "
          f"(ratio {h['inlier_ratio_mean']:.2f})")
        p(f"  static-hypothesis frames {100*h['static_frac']:.1f} %   "
          f"airborne flag {100*h['airborne_frac']:.1f} %   "
          f"vz-prior used {100*h['vzprior_frac']:.1f} %")
        p(f"  logged condition number: p95 {h['cond_p95']:.1f}  max {h['cond_max']:.1f}")
        if h['vzprior_frac'] > 0.5:
            p(f"  !! CAUTION: when a vz prior is active, doppler_rio.py computes the")
            p(f"     logged `cond` on the 2-column (vx,vy) sub-matrix only. It is")
            p(f"     therefore structurally incapable of showing the Vx/Vz degeneracy")
            p(f"     it is supposed to guard. Treat this number as uninformative until")
            p(f"     that is fixed (see the audit report, finding R-3).")

    # ---------------------------------------------------------------- SLAM
    S = R.get('slam')
    if 'slam_cov' in R:
        sc = R['slam_cov']
        p(f"\n[10a] SLAM TEMPORAL COVERAGE")
        p(f"  First keyframe at t+{sc['first_s']:.1f} s, last at t+{sc['last_s']:.1f} s")
        p(f"  Keyframes span {sc['span_s']:.1f} s of a {R['duration']:.1f} s flight "
          f"= {100*sc['frac_of_flight']:.0f} %")
        p(f"  Median keyframe interval {sc['median_dt']:.2f} s "
          f"(target ~0.2-0.3 s), largest gap {sc['gap_max_s']:.1f} s")
        if sc['silent_tail_s'] > 5.0:
            p(f"  !! SLAM produced NOTHING for the last {sc['silent_tail_s']:.0f} s of the")
            p(f"     flight. Any SLAM error figure below describes only the opening")
            p(f"     {sc['span_s']:.0f} s. The pose chain did not merely drift -- it stopped.")
            p(f"     Check slam_node's keyframe rejection reasons "
              f"(too_sparse_after_filtering / insufficient_correspondences);")
            p(f"     they are printed to the console but never logged.")
    if len(run.slam_rej_t):
        from collections import Counter
        cnt = Counter(run.slam_rej_reason)
        tot = len(run.slam_rej_reason) + len(run.slam_t)
        p(f"  Keyframe accept rate {100*len(run.slam_t)/max(tot,1):.1f} % "
          f"({len(run.slam_t)}/{tot}); rejections:")
        for reason, c in cnt.most_common():
            p(f"      {reason:<32s} {c:6d}")
    if run.slam_fitness is not None:
        p(f"  GICP quality: fitness p50 {np.median(run.slam_fitness):.3f} "
          f"(p05 {np.percentile(run.slam_fitness,5):.3f}), "
          f"rmse p50 {np.median(run.slam_rmse):.3f} m")
        p(f"  correspondences p50 {np.median(run.slam_ncorr):.0f}, "
          f"observable translation axes p50 {np.median(run.slam_nobs):.0f}/3")
        if np.median(run.slam_nobs) < 2:
            p(f"  !! Fewer than 2 of 3 translation axes observable on a typical")
            p(f"     keyframe -- the pose is being carried by the RIO/IMU prior, not")
            p(f"     by the radar geometry. That is dead reckoning wearing a SLAM hat.")
    if S:
        p(f"\n[10] SLAM POSE ERROR vs GPS   [{S['n']} keyframes in GPS window]")
        p(f"  SLAM map frame yaw offset vs true north: "
          f"{S['map_yaw_offset_deg']:+.1f} deg")
        if abs(S['map_yaw_offset_deg']) > 10:
            p(f"     (large: slam_node was almost certainly run with the shipped")
            p(f"      default --no-trust-imu-yaw, so the map frame is 'whatever the")
            p(f"      airframe was pointing at bootstrap'. The RAW column below is")
            p(f"      then dominated by this offset and says nothing about SLAM.)")
        if S['reflection']:
            p(f"  !! The shape fit needed a reflection -- SLAM's XY handedness is "
              f"inverted.")
        p(f"\n  {'level':<26}{'mean':>9}{'p95':>9}{'max':>9}{'final':>9}")
        for key, name in (('err_raw', 'RAW  (absolute XY)'),
                          ('err_yaw', 'YAW-ALIGNED XY'),
                          ('err_shape', 'SHAPE (Kabsch XY)')):
            e = S[key]
            p(f"  {name:<26}{e.mean():>9.2f}{np.percentile(e,95):>9.2f}"
              f"{e.max():>9.2f}{e[-1]:>9.2f}")
        e = S['err_z']
        p(f"  {'Z  (vs GPS up)':<26}{e.mean():>9.2f}{np.percentile(e,95):>9.2f}"
          f"{e.max():>9.2f}{e[-1]:>9.2f}")
        if S['err_z_alt'] is not None:
            e = S['err_z_alt']
            p(f"  {'Z  (vs tilt-corr. alt)':<26}{e.mean():>9.2f}"
              f"{np.percentile(e,95):>9.2f}{e.max():>9.2f}{e[-1]:>9.2f}")
        p(f"\n  Judge the front-end on YAW-ALIGNED. RAW additionally contains the map")
        p(f"  heading offset; SHAPE additionally forgives a constant position offset.")
        p(f"  GPS distance travelled over the SLAM window: {S['gps_travel_m']:.1f} m")
        p(f"  Drift {_f(S['drift_per_m'],4)} m per metre travelled "
          f"({100*S['drift_per_m']:.1f} % of distance)" if
          math.isfinite(S['drift_per_m']) else "  Drift per metre: n/a")
        p(f"  Drift {_f(S['drift_per_s'],4)} m/s of elapsed time")
        p(f"  (m/m is the transferable number -- m/s depends on how fast you flew.)")
    elif len(R['counts']) and R['counts']['slam'] < 3:
        p(f"\n[10] SLAM: only {R['counts']['slam']} keyframes logged -- nothing to "
          f"evaluate. Target is ~5 Hz; check slam_node's rejection reasons.")

    # ---------------------------------------------------------------- Alt
    A = R.get('alt')
    if A and 'corrected' in A:
        p(f"\n[11] ALTIMETER")
        p(f"  Height change vs GPS-up change, RMSE:")
        p(f"      raw slant range        {A['rmse_raw_m']:.3f} m")
        p(f"      tilt-corrected         {A['rmse_corrected_m']:.3f} m")
        p(f"  Mean slant-vs-vertical bias {A['tilt_bias_m']:.3f} m "
          f"(max {A['tilt_bias_max_m']:.3f} m)")
        p(f"  altimeter_bridge.py publishes the RAW slant range and slam_node.py")
        p(f"  consumes it as a vertical height. nav_node.py does apply")
        p(f"  cos(roll)cos(pitch); the other two do not. That inconsistency is worth")
        p(f"  {A['tilt_bias_max_m']:.2f} m of Z error at this flight's attitudes.")

    # ---------------------------------------------------------------- Gates
    p(f"\n[12] VERDICT")
    gates = build_gates(R, cfg)
    if not gates:
        p("  No gate could be evaluated -- insufficient data.")
    width = max(len(g[0]) for g in gates) if gates else 10
    for name, ok, val, note in gates:
        icon = "PASS" if ok is True else ("FAIL" if ok is False else "----")
        p(f"  [{icon}] {name:<{width}s}  {val}")
        if note:
            p(f"          {note}")
    hard = [g for g in gates if g[1] is False]
    p(f"\n  {len(hard)} of {len(gates)} gates failed.")
    p(f"  {'NOT FLIGHT-READY' if hard else 'All evaluated gates pass'}")

    # ---------------------------------------------------------------- Missing
    p(f"\n[13] WHAT THIS LOG CANNOT TELL YOU (missing instrumentation)")
    for i, m in enumerate(R['missing'], 1):
        p(f"  {i}. {m}")
    p(BAR + "\n")
    return len(hard) == 0


def build_gates(R, cfg):
    g = []
    cv = R.get('rio_cov')
    if cv:
        ok = cv['frac_of_flight'] >= cfg.gate_coverage
        g.append(("RIO coverage", ok,
                  f"{100*cv['frac_of_flight']:.1f} % (gate >= {100*cfg.gate_coverage:.0f} %)",
                  "" if ok else "Velocity aiding with this much dropout forces the "
                                "EKF to coast on IMU for seconds at a time."))
    if 'fast' in R:
        ok = R['fast']['frac_covered'] >= cfg.gate_coverage
        g.append(("RIO coverage in motion", ok,
                  f"{100*R['fast']['frac_covered']:.1f} %",
                  "" if ok else "RIO drops out exactly when it is needed."))
    gm = R.get('gps_path_matched')
    if gm and gm[0] > 1.0:
        ratio = R['rio_path_3d'] / gm[0]
        ok = abs(ratio - 1.0) <= cfg.gate_dist
        g.append(("RIO distance (matched)", ok,
                  f"ratio {ratio:.3f} (gate 1.000 +/- {cfg.gate_dist:.2f})", ""))
    if 'vel_enu' in R:
        for s in R['vel_enu']:
            if s.get('n', 0) >= 3:
                ok = s['rmse'] <= cfg.gate_vel_rmse
                g.append((f"RIO vel RMSE {s['label']}", ok,
                          f"{s['rmse']:.3f} m/s (gate <= {cfg.gate_vel_rmse})", ""))
        for s in R['vel_enu']:
            if s.get('n', 0) >= 3 and math.isfinite(s.get('corr', np.nan)):
                ok = s['corr'] >= cfg.gate_vel_corr
                g.append((f"RIO vel corr {s['label']}", ok,
                          f"{s['corr']:.3f} (gate >= {cfg.gate_vel_corr})", ""))
    if 'still' in R:
        ok = R['still']['rio_hspeed_mean'] <= cfg.gate_still
        g.append(("Hover phantom speed", ok,
                  f"{R['still']['rio_hspeed_mean']:.3f} m/s "
                  f"(gate <= {cfg.gate_still})",
                  "" if ok else "Direct flyaway mechanism under position aiding."))
    if R.get('misalign'):
        m = R['misalign']
        if not R.get('misalign_meaningful', True):
            g.append(("Mount calibration", None,
                      "indeterminate -- residual is the size of the signal",
                      "Fix the velocity estimate first; this gate cannot be "
                      "evaluated while the error is dominated by noise."))
        else:
            ok = (abs(m['pitch_deg']) <= 3 and abs(m['yaw_deg']) <= 5
                  and abs(m['roll_deg']) <= 5 and not m['reflection'])
            g.append(("Mount calibration", ok,
                      f"r{m['roll_deg']:+.1f} p{m['pitch_deg']:+.1f} "
                      f"y{m['yaw_deg']:+.1f} deg, scale {m['scale']:.3f}", ""))
    S = R.get('slam')
    if S:
        ok = S['err_yaw'][-1] <= cfg.gate_slam_xy
        g.append(("SLAM XY (yaw-aligned)", ok,
                  f"final {S['err_yaw'][-1]:.2f} m (gate <= {cfg.gate_slam_xy})", ""))
        ok = S['err_z'][-1] <= cfg.gate_slam_z
        g.append(("SLAM Z", ok,
                  f"final {S['err_z'][-1]:.2f} m (gate <= {cfg.gate_slam_z})", ""))
        if math.isfinite(S['drift_per_m']):
            ok = S['drift_per_m'] <= cfg.gate_drift
            g.append(("SLAM drift", ok,
                      f"{100*S['drift_per_m']:.1f} % of distance "
                      f"(gate <= {100*cfg.gate_drift:.0f} %)", ""))
    if R.get('lag'):
        lg, cc = R['lag']
        g.append(("RIO/GPS correlation", cc >= cfg.gate_vel_corr,
                  f"{cc:.3f} at lag {lg:+.2f} s", ""))
    # numpy.bool_ is not Python bool, and `x is True` would silently turn every
    # numpy-derived gate into "not evaluated". Coerce explicitly.
    return [(n, (None if o is None else bool(o)), v, note) for n, o, v, note in g]


# ═════════════════════════════════════════════════════════════════════════
#  Plots
# ═════════════════════════════════════════════════════════════════════════

def make_plots(run: Run, R, out_png):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.gridspec import GridSpec

    t0 = R['t0']
    fig = plt.figure(figsize=(19, 22))
    gs = GridSpec(6, 3, figure=fig, hspace=0.42, wspace=0.26)
    fig.suptitle(f"RIO/SLAM analysis  -  {os.path.basename(R['path'])}",
                 fontsize=15, y=0.995)

    # --- 1. XY trajectory -------------------------------------------------
    ax = fig.add_subplot(gs[0, 0])
    if len(run.gps_t):
        ax.plot(run.gps_enu[:, 0], run.gps_enu[:, 1], 'k-', lw=1.6, label='GPS/EKF')
        ax.plot(*run.gps_enu[0, :2], 'go', ms=8, label='start')
        ax.plot(*run.gps_enu[-1, :2], 'rs', ms=8, label='end')
    S = R.get('slam')
    if S:
        ax.plot(S['slam_en_raw'][:, 0], S['slam_en_raw'][:, 1], ':',
                c='tab:orange', lw=1.2, label='SLAM raw')
        ax.plot(S['slam_en_yaw'][:, 0], S['slam_en_yaw'][:, 1], '-',
                c='tab:blue', lw=1.4, label='SLAM yaw-aligned')
    ax.set_title('Ground track (ENU)')
    ax.set_xlabel('East m'); ax.set_ylabel('North m')
    ax.axis('equal'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 2. RIO integrated vs GPS ----------------------------------------
    ax = fig.add_subplot(gs[0, 1])
    if 'rio_v_enu' in R and len(run.rio_t) > 2:
        iv = R['rio_cov']['intervals']
        pos = np.zeros((len(run.rio_t), 3))
        acc = np.zeros(3)
        iva = np.asarray(iv, float).reshape(-1, 2)
        for i in range(1, len(run.rio_t)):
            a, b = run.rio_t[i-1], run.rio_t[i]
            if len(iva) and np.any((iva[:, 0] <= a+1e-9) & (b <= iva[:, 1]+1e-9)):
                va = 0.5*(R['rio_v_enu'][i-1] + R['rio_v_enu'][i])
                if np.all(np.isfinite(va)):
                    acc = acc + va*(b-a)
            pos[i] = acc
        ax.plot(pos[:, 0], pos[:, 1], '-', c='tab:red', lw=1.3,
                label='RIO integrated (covered only)')
        if len(run.gps_t):
            gp = interp_vec(run.gps_t, run.gps_enu, run.rio_t) - \
                 interp_vec(run.gps_t, run.gps_enu, np.array([run.rio_t[0]]))[0]
            ax.plot(gp[:, 0], gp[:, 1], 'k-', lw=1.3, label='GPS (same span)')
    ax.set_title('RIO dead-reckoning vs GPS')
    ax.set_xlabel('East m'); ax.set_ylabel('North m')
    ax.axis('equal'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 3. Coverage timeline --------------------------------------------
    ax = fig.add_subplot(gs[0, 2])
    if len(run.gps_t):
        gsp = np.linalg.norm(np.nan_to_num(R['gps_v']), axis=1)
        ax.plot(run.gps_t - t0, gsp, 'k-', lw=1.0, label='GPS speed')
    if 'rio_cov' in R:
        for a, b in R['rio_cov']['intervals']:
            ax.axvspan(a - t0, b - t0, color='tab:green', alpha=.18, lw=0)
        ax.plot([], [], color='tab:green', alpha=.4, lw=8, label='RIO covered')
    if len(run.slam_t):
        ax.plot(run.slam_t - t0, np.zeros(len(run.slam_t)), 'v',
                c='tab:purple', ms=4, label='SLAM keyframe')
    ax.set_title('Coverage: green = RIO publishing')
    ax.set_xlabel('t s'); ax.set_ylabel('GPS speed m/s')
    ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 4-6. Velocity per axis ------------------------------------------
    for k, name in enumerate(('East', 'North', 'Up')):
        ax = fig.add_subplot(gs[1, k])
        if len(run.gps_t):
            ax.plot(run.gps_t - t0, R['gps_v'][:, k], 'k-', lw=1.2, label='GPS')
        if 'rio_v_enu' in R:
            ax.plot(run.rio_t - t0, R['rio_v_enu'][:, k], '.', c='tab:red',
                    ms=2.6, label='RIO')
        ax.set_title(f'Velocity {name}')
        ax.set_xlabel('t s'); ax.set_ylabel('m/s')
        ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 7. Velocity scatter ---------------------------------------------
    ax = fig.add_subplot(gs[2, 0])
    if 'vel_enu' in R and 'gps_v' in R:
        gv = interp_vec(run.gps_t, R['gps_v'], run.rio_t)
        cols = ('tab:blue', 'tab:orange', 'tab:green')
        for k, name in enumerate(('E', 'N', 'U')):
            ax.plot(gv[:, k], R['rio_v_enu'][:, k], '.', ms=2.4,
                    c=cols[k], label=name, alpha=.6)
        lim = np.nanpercentile(np.abs(gv), 99) * 1.3 + 0.5
        ax.plot([-lim, lim], [-lim, lim], 'k--', lw=1)
        ax.set_xlim(-lim, lim); ax.set_ylim(-lim, lim)
    ax.set_title('RIO vs GPS velocity (ideal = diagonal)')
    ax.set_xlabel('GPS m/s'); ax.set_ylabel('RIO m/s')
    ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 8. Speed error vs speed -----------------------------------------
    ax = fig.add_subplot(gs[2, 1])
    if 'vel_enu' in R:
        gv = interp_vec(run.gps_t, R['gps_v'], run.rio_t)
        gsp = np.linalg.norm(gv, axis=1)
        err = np.linalg.norm(R['rio_v_enu'] - gv, axis=1)
        ax.plot(gsp, err, '.', ms=2.6, c='tab:red')
        ax.axhline(0.3, ls='--', c='k', lw=1)
    ax.set_title('|velocity error| vs true speed')
    ax.set_xlabel('GPS speed m/s'); ax.set_ylabel('|error| m/s'); ax.grid(alpha=.3)

    # --- 9. Altitude ------------------------------------------------------
    ax = fig.add_subplot(gs[2, 2])
    if len(run.gps_t):
        ax.plot(run.gps_t - t0, run.gps_enu[:, 2] - run.gps_enu[0, 2],
                'k-', lw=1.4, label='GPS up')
    A = R.get('alt')
    if A:
        ax.plot(A['t'] - t0, A['raw'] - A['raw'][0], '-', c='tab:gray',
                lw=1.0, label='alt raw slant')
        if 'corrected' in A:
            ax.plot(A['t'] - t0, A['corrected'] - A['corrected'][0], '-',
                    c='tab:green', lw=1.2, label='alt tilt-corrected')
    if S:
        ax.plot(S['t'] - t0, S['slam_up'], '-', c='tab:blue', lw=1.2, label='SLAM up')
    ax.set_title('Altitude (relative)')
    ax.set_xlabel('t s'); ax.set_ylabel('m'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 10. SLAM XY error -----------------------------------------------
    ax = fig.add_subplot(gs[3, 0])
    if S:
        ax.plot(S['t'] - t0, S['err_raw'], ':', c='tab:orange', label='raw')
        ax.plot(S['t'] - t0, S['err_yaw'], '-', c='tab:blue', lw=1.6,
                label='yaw-aligned')
        ax.plot(S['t'] - t0, S['err_shape'], '--', c='tab:green', label='shape')
    ax.set_title('SLAM XY error')
    ax.set_xlabel('t s'); ax.set_ylabel('m'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 11. SLAM Z error -------------------------------------------------
    ax = fig.add_subplot(gs[3, 1])
    if S:
        ax.plot(S['t'] - t0, S['err_z'], '-', c='tab:blue', lw=1.5, label='vs GPS')
        if S['err_z_alt'] is not None:
            ax.plot(S['t'] - t0, S['err_z_alt'], '--', c='tab:green',
                    label='vs altimeter')
    ax.set_title('SLAM Z error')
    ax.set_xlabel('t s'); ax.set_ylabel('m'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 12. RIO health ---------------------------------------------------
    ax = fig.add_subplot(gs[3, 2])
    if len(run.rio_t):
        ax.plot(run.rio_t - t0, run.rio_ntotal, '.', ms=2.4, c='tab:gray',
                label='gated points')
        ax.plot(run.rio_t - t0, run.rio_inliers, '.', ms=2.4, c='tab:red',
                label='inliers')
        ax2 = ax.twinx()
        ax2.plot(run.rio_t - t0, run.rio_cond, '.', ms=2.0, c='tab:blue',
                 alpha=.5)
        ax2.set_ylabel('cond (blue)', color='tab:blue')
    ax.set_title('RIO inliers / conditioning')
    ax.set_xlabel('t s'); ax.set_ylabel('points'); ax.grid(alpha=.3)
    ax.legend(fontsize=7, loc='upper left')

    # --- 13. Inter-frame dt histogram ------------------------------------
    ax = fig.add_subplot(gs[4, 0])
    if len(run.rio_t) > 2:
        d = np.diff(run.rio_t)
        ax.hist(np.clip(d, 0, 2.0), bins=80, color='tab:red', alpha=.8)
        ax.axvline(0.5, ls='--', c='k', lw=1)
        ax.set_yscale('log')
    ax.set_title('RIO inter-frame dt (clipped 2 s)')
    ax.set_xlabel('dt s'); ax.set_ylabel('count'); ax.grid(alpha=.3)

    # --- 14. Attitude -----------------------------------------------------
    ax = fig.add_subplot(gs[4, 1])
    if len(run.imu_t):
        ax.plot(run.imu_t - t0, np.degrees(run.imu_rpy[:, 0]), lw=.9, label='roll')
        ax.plot(run.imu_t - t0, np.degrees(run.imu_rpy[:, 1]), lw=.9, label='pitch')
        ax.plot(run.imu_t - t0, np.degrees(run.imu_rpy[:, 2]), lw=.6, label='yaw',
                alpha=.6)
    ax.set_title('FC attitude')
    ax.set_xlabel('t s'); ax.set_ylabel('deg'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 15. Hover bias ---------------------------------------------------
    ax = fig.add_subplot(gs[4, 2])
    if 'vel_enu' in R:
        gv = interp_vec(run.gps_t, R['gps_v'], run.rio_t)
        gsp = np.linalg.norm(gv, axis=1)
        m = np.isfinite(gsp) & (gsp < 0.4)
        if m.sum() > 3:
            ax.hist(np.linalg.norm(R['rio_v_enu'][m, :2], axis=1), bins=40,
                    color='tab:red', alpha=.85)
            ax.axvline(0.15, ls='--', c='k', lw=1)
            ax.set_title(f'RIO |v_xy| while GPS says parked (n={int(m.sum())})')
        else:
            ax.set_title('RIO |v_xy| at hover (too few samples)')
    ax.set_xlabel('m/s'); ax.set_ylabel('count'); ax.grid(alpha=.3)

    # --- 16. Cumulative distance -----------------------------------------
    ax = fig.add_subplot(gs[5, 0])
    if len(run.gps_t) and 'gps_v' in R:
        gsp = np.nan_to_num(np.linalg.norm(R['gps_v'], axis=1))
        cum = np.concatenate([[0], np.cumsum(0.5*(gsp[1:]+gsp[:-1])*np.diff(run.gps_t))])
        ax.plot(run.gps_t - t0, cum, 'k-', lw=1.5, label='GPS')
    if 'rio_v_enu' in R and len(run.rio_t) > 2:
        iva = np.asarray(R['rio_cov']['intervals'], float).reshape(-1, 2)
        acc, cumr = 0.0, np.zeros(len(run.rio_t))
        for i in range(1, len(run.rio_t)):
            a, b = run.rio_t[i-1], run.rio_t[i]
            if len(iva) and np.any((iva[:, 0] <= a+1e-9) & (b <= iva[:, 1]+1e-9)):
                va = 0.5*(R['rio_v_enu'][i-1]+R['rio_v_enu'][i])
                if np.all(np.isfinite(va)):
                    acc += float(np.linalg.norm(va))*(b-a)
            cumr[i] = acc
        ax.plot(run.rio_t - t0, cumr, '-', c='tab:red', lw=1.5, label='RIO (covered)')
    ax.set_title('Cumulative distance')
    ax.set_xlabel('t s'); ax.set_ylabel('m'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 17. Body-frame velocity -----------------------------------------
    ax = fig.add_subplot(gs[5, 1])
    if len(run.rio_t):
        for k, (n, c) in enumerate((('vx fwd', 'tab:blue'),
                                    ('vy right', 'tab:orange'),
                                    ('vz down', 'tab:green'))):
            ax.plot(run.rio_t - t0, run.rio_v[:, k], '.', ms=2.0, c=c, label=n)
        if 'gps_v_body' in R:
            for k, c in enumerate(('tab:blue', 'tab:orange', 'tab:green')):
                ax.plot(run.rio_t - t0, R['gps_v_body'][:, k], '-', c=c,
                        lw=.9, alpha=.55)
    ax.set_title('Body FRD velocity: dots = RIO, lines = GPS')
    ax.set_xlabel('t s'); ax.set_ylabel('m/s'); ax.grid(alpha=.3); ax.legend(fontsize=7)

    # --- 18. Summary text -------------------------------------------------
    ax = fig.add_subplot(gs[5, 2]); ax.axis('off')
    lines = []
    if 'rio_cov' in R:
        lines.append(f"RIO coverage      {100*R['rio_cov']['frac_of_flight']:.1f} %")
        lines.append(f"longest dropout   {R['rio_cov']['gap_max_s']:.1f} s")
    gm = R.get('gps_path_matched')
    if gm and gm[0] > 1:
        lines.append(f"RIO/GPS dist      {R['rio_path_3d']/gm[0]:.3f} x")
    if 'vel_enu' in R:
        for s in R['vel_enu']:
            if s.get('n', 0) > 3:
                lines.append(f"vel {s['label']:<6} rmse {s['rmse']:.2f}  r {s['corr']:+.2f}")
    if 'still' in R:
        lines.append(f"hover phantom     {R['still']['rio_hspeed_mean']:.2f} m/s")
    if R.get('misalign'):
        m = R['misalign']
        lines.append(f"mount r/p/y  {m['roll_deg']:+.1f}/{m['pitch_deg']:+.1f}/"
                     f"{m['yaw_deg']:+.1f} deg")
        lines.append(f"Doppler scale     {m['scale']:.3f}")
    if S:
        lines.append(f"SLAM XY final     {S['err_yaw'][-1]:.2f} m")
        lines.append(f"SLAM Z final      {S['err_z'][-1]:.2f} m")
        lines.append(f"map yaw offset    {S['map_yaw_offset_deg']:+.0f} deg")
    ax.text(0.0, 1.0, "\n".join(lines), family='monospace', fontsize=10,
            va='top', ha='left', transform=ax.transAxes)
    ax.set_title('Summary')

    fig.savefig(out_png, dpi=110, bbox_inches='tight')
    plt.close(fig)
    return out_png


# ═════════════════════════════════════════════════════════════════════════
#  Self-test -- proves the ANALYZER is right, using synthetic ground truth
# ═════════════════════════════════════════════════════════════════════════

def _synth_log(path, *, dur=120.0, gps_hz=50.0, rio_hz=10.0, imu_hz=50.0,
               slam_hz=2.0, alt_hz=20.0, radius=40.0, climb=25.0, yaw_spin=True,
               rio_bias_body=(0.0, 0.0, 0.0), rio_scale=1.0,
               rio_tilt_err_deg=0.0, rio_dropout=None, slam_yaw_off_deg=0.0,
               slam_drift_frac=0.0, seed=0, v2_fields=False):
    """Write a synthetic gps_logger-format JSONL with an analytically known
    trajectory, so every metric has a closed-form expected value."""
    rng = np.random.default_rng(seed)
    t0 = 1000.0

    # truth: horizontal circle + linear climb, nose always along the velocity
    w = 2 * math.pi / dur * 2.0        # two laps
    def truth(t):
        e = radius * math.sin(w * t)
        n = radius * (1 - math.cos(w * t))
        u = climb * t / dur
        ve = radius * w * math.cos(w * t)
        vn = radius * w * math.sin(w * t)
        vu = climb / dur
        return np.array([e, n, u]), np.array([ve, vn, vu])

    def att(t):
        _, v = truth(t)
        # NOTE: atan2(-0.0, -0.0) == -pi, so a stationary synthetic vehicle would
        # otherwise flip heading by 180 deg every half lap purely from signed
        # zeros. Guard on speed. (Real AHRS output has the same hazard whenever
        # heading is derived from a velocity vector near zero speed.)
        speed = float(np.hypot(v[0], v[1]))
        yaw = math.atan2(v[0], v[1]) if (yaw_spin and speed > 1e-6) else 0.0
        return 0.0, 0.0, yaw

    rows = [{'type': 'meta', 't_start': t0, 'synthetic': True}]

    tg = np.arange(0, dur, 1.0 / gps_hz)
    for t in tg:
        p, _ = truth(t)
        row = {'type': 'gps', 't_mono': t0 + t,
               'enu': [float(x) for x in p], 'sats': 14}
        if v2_fields:
            _, vv = truth(t)
            row['v_enu'] = [float(x) for x in vv]
            row['v_ned'] = [float(vv[1]), float(vv[0]), float(-vv[2])]
        rows.append(row)

    ti = np.arange(0, dur, 1.0 / imu_hz) if imu_hz > 0 else np.zeros(0)
    for t in ti:
        r, pch, y = att(t)
        rows.append({'type': 'imu', 't_mono': t0 + t, 'roll': r, 'pitch': pch,
                     'yaw': y, 'wx': 0.0, 'wy': 0.0, 'wz': float(w)})

    cs, sn = math.cos(math.radians(rio_tilt_err_deg)), math.sin(math.radians(rio_tilt_err_deg))
    M = np.array([[cs, 0, sn], [0, 1, 0], [-sn, 0, cs]])   # extra pitch in body
    tr = np.arange(0, dur, 1.0 / rio_hz) if rio_hz > 0 else np.zeros(0)
    for t in tr:
        if rio_dropout and any(a <= t <= b for a, b in rio_dropout):
            if v2_fields:
                rows.append({'type': 'rio_reject', 't_mono': t0 + t + 0.03,
                             't_frame': t0 + t, 'reason': 'ransac_reject',
                             'n_total': 12, 'inliers': 4, 'cond': 61.0,
                             'extra': 0.0})
            continue
        _, v_enu = truth(t)
        r, pch, y = att(t)
        Rb = R_body_to_ned_batch(np.array([[r, pch, y]]))[0]
        v_ned = ned_to_enu(v_enu)                     # ENU->NED
        v_body = Rb.T @ v_ned
        v_body = rio_scale * (M @ v_body) + np.asarray(rio_bias_body)
        row = {'type': 'rio', 't_mono': t0 + t + 0.03, 't_frame': t0 + t,
               'vx': float(v_body[0]), 'vy': float(v_body[1]),
               'vz': float(v_body[2]), 'inliers': 40, 'n_total': 55,
               'is_static': False, 'airborne': True, 'vz_prior': False,
               'cond': 4.0}
        if v2_fields:
            row.update({'sx': 0.08, 'sy': 0.09, 'sz': 0.11})
        rows.append(row)

    ts = np.arange(0, dur, 1.0 / slam_hz) if slam_hz > 0 else np.zeros(0)
    cy, sy = math.cos(math.radians(slam_yaw_off_deg)), math.sin(math.radians(slam_yaw_off_deg))
    for t in ts:
        p, _ = truth(t)
        # SLAM map: x=north, y=east, z=down, rotated by -slam_yaw_off about down
        n_, e_ = p[1], p[0]
        xm = cy * n_ + sy * e_
        ym = -sy * n_ + cy * e_
        drift = slam_drift_frac * (radius * w * t)
        row = {'type': 'slam', 't_mono': t0 + t + 0.1, 't_slam': t0 + t,
               'pos': [float(xm + drift), float(ym), float(-p[2])],
               'n_map': 500, 'fwd_range': 1.0e6}
        if v2_fields:
            row.update({'n_corr': 38, 'fitness': 0.62, 'rmse': 0.44,
                        'n_obs_axes': 2, 'n_raw': 210, 'n_final': 74})
        rows.append(row)

    ta = np.arange(0, dur, 1.0 / alt_hz) if alt_hz > 0 else np.zeros(0)
    for t in ta:
        p, _ = truth(t)
        rows.append({'type': 'altimeter', 't_mono': t0 + t,
                     'range_m': float(5.0 + p[2])})

    rows.sort(key=lambda r: r.get('t_mono', -1))
    with open(path, 'w') as f:
        for r in rows:
            f.write(json.dumps(r) + '\n')
    # analytic truths
    lap = 2 * math.pi * radius * 2.0
    path3 = math.hypot(lap / dur, climb / dur) * dur
    return {'path_3d': path3, 'path_2d': lap, 'climb': climb}


def selftest(tmpdir="/tmp/rio_selftest"):
    os.makedirs(tmpdir, exist_ok=True)
    cfg = _default_cfg()
    fails = []

    def check(name, got, want, tol, unit=""):
        ok = abs(got - want) <= tol
        print(f"   {'OK  ' if ok else 'FAIL'}  {name:<42s} got {got:+9.4f}{unit}  "
              f"want {want:+.4f} +/- {tol}{unit}")
        if not ok:
            fails.append(name)

    print(f"\n{BAR}\n  ANALYZER SELF-TEST  (synthetic logs, known answers)\n{BAR}")

    # ---- case A: perfect sensors ----------------------------------------
    print("\n A. Perfect RIO + perfect SLAM, full coverage")
    p = os.path.join(tmpdir, 'perfect.jsonl')
    tr = _synth_log(p)
    R = analyse(load_run(p), cfg)
    check("coverage fraction", R['rio_cov']['frac_of_flight'], 1.0, 0.03)
    check("RIO 3D path", R['rio_path_3d'], tr['path_3d'], 0.02 * tr['path_3d'], " m")
    check("RIO/GPS path ratio", R['rio_path_3d'] / R['gps_path_matched'][0], 1.0, 0.02)
    for s in R['vel_enu']:
        check(f"vel {s['label']} rmse", s['rmse'], 0.0, 0.05, " m/s")
        check(f"vel {s['label']} slope", s['slope'], 1.0, 0.03)
    check("misalign pitch", R['misalign']['pitch_deg'], 0.0, 0.5, " deg")
    check("misalign yaw", R['misalign']['yaw_deg'], 0.0, 0.5, " deg")
    check("Doppler scale", R['misalign']['scale'], 1.0, 0.01)
    check("SLAM yaw-aligned final XY", R['slam']['err_yaw'][-1], 0.0, 0.5, " m")
    check("SLAM Z final", R['slam']['err_z'][-1], 0.0, 0.5, " m")
    check("displacement East", R['rio_disp_enu'][0],
          R['gps_disp_matched'][0], 1.0, " m")
    check("displacement North", R['rio_disp_enu'][1],
          R['gps_disp_matched'][1], 1.0, " m")
    check("displacement Up", R['rio_disp_enu'][2],
          R['gps_disp_matched'][2], 0.5, " m")

    # ---- case B: 40 % dropout, otherwise perfect -------------------------
    print("\n B. Perfect RIO but 40 % dropout -- must NOT report a distance error")
    p = os.path.join(tmpdir, 'dropout.jsonl')
    _synth_log(p, rio_dropout=[(10, 30), (60, 88)])
    R = analyse(load_run(p), cfg)
    check("coverage fraction", R['rio_cov']['frac_of_flight'], 0.60, 0.06)
    check("RIO/GPS path ratio (matched)",
          R['rio_path_3d'] / R['gps_path_matched'][0], 1.0, 0.03)
    naive = R['rio_path_3d'] / R['gps_path_int'][0]
    print(f"   info  naive (old-analyzer) ratio would be {naive:.3f} "
          f"-> a fake {100*(1-naive):.0f} % 'distance error'")
    if naive > 0.8:
        fails.append("dropout case did not reproduce the old analyzer's artefact")

    # ---- case C: known mount pitch error ---------------------------------
    print("\n C. RIO with a +6 deg mount pitch error -- must be identified")
    p = os.path.join(tmpdir, 'tilt.jsonl')
    _synth_log(p, rio_tilt_err_deg=6.0)
    R = analyse(load_run(p), cfg)
    check("recovered mount pitch", R['misalign']['pitch_deg'], 6.0, 0.6, " deg")
    check("recovered mount roll", R['misalign']['roll_deg'], 0.0, 0.6, " deg")
    check("recovered scale", R['misalign']['scale'], 1.0, 0.02)

    # ---- case D: known Doppler scale error -------------------------------
    print("\n D. RIO with a 1.15x Doppler scale error")
    p = os.path.join(tmpdir, 'scale.jsonl')
    _synth_log(p, rio_scale=1.15)
    R = analyse(load_run(p), cfg)
    check("recovered scale", R['misalign']['scale'], 1.15, 0.02)
    check("RIO/GPS path ratio", R['rio_path_3d'] / R['gps_path_matched'][0],
          1.15, 0.03)

    # ---- case E: hover-phantom bias --------------------------------------
    print("\n E. Stationary vehicle, RIO has a +0.8 m/s forward-body bias")
    p = os.path.join(tmpdir, 'bias.jsonl')
    _synth_log(p, rio_bias_body=(0.8, 0.0, 0.0), radius=0.0, climb=0.0)
    R = analyse(load_run(p), cfg)
    # parked with yaw = 0, so body-forward maps onto North, not East
    check("hover phantom |v_xy|", R['still']['rio_hspeed_mean'], 0.8, 0.05, " m/s")
    check("vel North bias", R['vel_enu'][1]['bias'], 0.8, 0.1, " m/s")
    check("vel East bias", R['vel_enu'][0]['bias'], 0.0, 0.05, " m/s")
    check("phantom drift per minute", R['still']['phantom_m_per_min'], 48.0, 3.0, " m")

    # ---- case F1: SLAM map-yaw offset, no drift --------------------------
    print("\n F1. SLAM with a 35 deg map-yaw offset and NO drift")
    p = os.path.join(tmpdir, 'slam_yaw.jsonl')
    _synth_log(p, slam_yaw_off_deg=35.0)
    R = analyse(load_run(p), cfg)
    S = R['slam']
    check("recovered map yaw offset", abs(S['map_yaw_offset_deg']), 35.0, 1.0, " deg")
    check("yaw-aligned XY error", S['err_yaw'].mean(), 0.0, 0.5, " m")
    print(f"   info  RAW XY mean {S['err_raw'].mean():.1f} m vs "
          f"YAW-ALIGNED {S['err_yaw'].mean():.2f} m")
    print(f"         SLAM is PERFECT here; the old analyzer would still have")
    print(f"         reported {S['err_raw'][-1]:.1f} m of 'absolute XY error'.")
    if S['err_raw'].mean() < 5.0:
        fails.append("map-yaw offset did not show up in the RAW metric")

    # ---- case F2: SLAM drift, no yaw offset ------------------------------
    print("\n F2. SLAM with 4 % along-track drift and no yaw offset")
    p = os.path.join(tmpdir, 'slam_drift.jsonl')
    _synth_log(p, slam_drift_frac=0.04)
    R = analyse(load_run(p), cfg)
    S = R['slam']
    # drift grows as 0.04 * (v * t); GPS travel also grows as ~v*t, so
    # error-per-metre-travelled recovers the fraction directly.
    check("drift per metre travelled", S['drift_per_m'], 0.04, 0.012, " m/m")
    print(f"   info  same run in m/s of elapsed time: {S['drift_per_s']:.4f} m/s")
    print(f"         -- that number changes if you fly the same path slower;")
    print(f"         the m/m number does not. Gate on m/m.")

    # ---- case G0: v2 log fields ------------------------------------------
    print("\n G0. Patched-logger v2 fields (native velocity, rejects, GICP quality)")
    p = os.path.join(tmpdir, 'v2.jsonl')
    _synth_log(p, v2_fields=True, rio_dropout=[(20, 45)])
    r2 = load_run(p)
    R = analyse(r2, cfg)
    check("native velocity used", 1.0 if 'native' in R['gps_v_source'] else 0.0,
          1.0, 0.0)
    check("reject frames parsed", float(len(r2.rio_rej_t)), 250.0, 20.0)
    check("published sigma parsed", float(r2.rio_sigma.shape[0]),
          float(len(r2.rio_t)), 0.0)
    check("GICP fitness parsed", float(np.median(r2.slam_fitness)), 0.62, 0.02)
    # native velocity must give the same answer as differencing, to within noise
    cfg2 = _default_cfg(); cfg2.force_diff_velocity = True
    Rd = analyse(r2, cfg2)
    check("native vs differenced path", R['gps_path_int'][0],
          Rd['gps_path_int'][0], 0.03 * Rd['gps_path_int'][0], " m")

    # ---- case G: geodesy --------------------------------------------------
    print("\n G. Geodesy: WGS-84 vs gps_logger.py's hard-coded flat-earth constants")
    for lat in (0.0, 30.0, 45.0, 60.0):
        m_lat, m_lon = enu_scale_factors(lat)
        old_lat, old_lon = 110_852.0, 111_320.0 * math.cos(math.radians(lat))
        e_lat = 100 * (old_lat - m_lat) / m_lat
        e_lon = 100 * (old_lon - m_lon) / m_lon
        print(f"   lat {lat:4.0f} deg : north scale error {e_lat:+6.3f} %   "
              f"east scale error {e_lon:+6.3f} %")
    m_lat, _ = enu_scale_factors(0.0)
    if abs(110_852.0 - m_lat) / m_lat < 0.001:
        fails.append("geodesy check did not detect the latitude-specific constant")

    # ---- case H: every report path must actually render ------------------
    # analyse() was the only thing under test, so a NameError in print_report
    # could (and did) ship. Render every synthetic case through the real
    # printer into a throwaway buffer.
    print("\n H. Report renderer smoke test (all synthetic cases)")
    import io
    import contextlib
    cases = {
        'perfect': {}, 'dropout': {'rio_dropout': [(10, 30)]},
        'tilt': {'rio_tilt_err_deg': 6.0}, 'scale': {'rio_scale': 1.15},
        'parked': {'radius': 0.0, 'climb': 0.0, 'rio_bias_body': (0.8, 0, 0)},
        'slam_yaw': {'slam_yaw_off_deg': 35.0},
        'v2': {'v2_fields': True, 'rio_dropout': [(20, 45)]},
        'no_imu': {'imu_hz': 0.0},
        'no_slam': {'slam_hz': 0.0},
        'no_alt': {'alt_hz': 0.0},
        'tiny': {'dur': 6.0},
    }
    for name, kw in cases.items():
        fp = os.path.join(tmpdir, f'smoke_{name}.jsonl')
        try:
            _synth_log(fp, **kw)
            rr = load_run(fp)
            RR = analyse(rr, cfg)
            buf = io.StringIO()
            with contextlib.redirect_stdout(buf):
                print_report(rr, RR, cfg)
            out = buf.getvalue()
            assert 'VERDICT' in out, "report truncated before the verdict"
            print(f"   OK    render '{name}' ({len(out.splitlines())} lines)")
        except Exception as e:
            print(f"   FAIL  render '{name}': {type(e).__name__}: {e}")
            fails.append(f"render:{name}")

    print(f"\n{SUB}")
    if fails:
        print(f"  SELF-TEST FAILED: {len(fails)} check(s)")
        for f in fails:
            print(f"    - {f}")
    else:
        print("  SELF-TEST PASSED -- every metric recovered its known ground truth.")
        print("  Numbers this tool prints about a real flight describe the flight,")
        print("  not the tool.")
    print(BAR + "\n")
    return 0 if not fails else 1


# ═════════════════════════════════════════════════════════════════════════
#  CLI
# ═════════════════════════════════════════════════════════════════════════

class _Cfg:
    pass


def _default_cfg():
    c = _Cfg()
    c.gps_window = 0.6
    c.max_gap = 0.5
    c.imu_max_dt = 0.2
    c.min_speed = 1.0
    c.still_speed = 0.3
    c.fast_speed = 2.0
    c.max_lag = 1.5
    c.apply_lag = False
    c.tilt_hint = 40.0
    c.force_diff_velocity = False
    c.gate_coverage = 0.90
    c.gate_dist = 0.05
    c.gate_vel_rmse = 0.30
    c.gate_vel_corr = 0.85
    c.gate_still = 0.15
    c.gate_slam_xy = 5.0
    c.gate_slam_z = 2.0
    c.gate_drift = 0.05
    return c


def main():
    ap = argparse.ArgumentParser(
        description="Verified post-flight analyzer for the RIO/SLAM radar stack.")
    ap.add_argument('log', nargs='*', help="JSONL log(s) from gps_logger.py")
    ap.add_argument('--selftest', action='store_true',
                    help="validate the analyzer against synthetic ground truth")
    ap.add_argument('--plot', default=None, help="write a PNG figure here")
    ap.add_argument('--json', default=None, help="write metrics as JSON here")
    ap.add_argument('--brief', action='store_true', help="one line per log")
    d = _default_cfg()
    ap.add_argument('--gps-window', type=float, default=d.gps_window,
                    help="centred differencing window for GPS velocity (s)")
    ap.add_argument('--max-gap', type=float, default=d.max_gap,
                    help="RIO inter-frame gap above which coverage is broken (s)")
    ap.add_argument('--imu-max-dt', type=float, default=d.imu_max_dt)
    ap.add_argument('--still-speed', type=float, default=d.still_speed)
    ap.add_argument('--fast-speed', type=float, default=d.fast_speed)
    ap.add_argument('--min-speed', type=float, default=d.min_speed)
    ap.add_argument('--tilt-hint', type=float, default=d.tilt_hint)
    ap.add_argument('--force-diff-velocity', action='store_true',
                    help="ignore logged native EKF velocity and differentiate "
                         "position instead (for comparing the two)")
    ap.add_argument('--apply-lag', action='store_true',
                    help="shift SLAM by the estimated lag before scoring")
    ap.add_argument('--gate-coverage', type=float, default=d.gate_coverage)
    ap.add_argument('--gate-vel-rmse', type=float, default=d.gate_vel_rmse)
    ap.add_argument('--gate-still', type=float, default=d.gate_still)
    ap.add_argument('--gate-slam-xy', type=float, default=d.gate_slam_xy)
    ap.add_argument('--gate-slam-z', type=float, default=d.gate_slam_z)
    a = ap.parse_args()

    if a.selftest:
        sys.exit(selftest())

    cfg = _default_cfg()
    for k in ('gps_window', 'max_gap', 'imu_max_dt', 'still_speed', 'fast_speed',
              'min_speed', 'tilt_hint', 'apply_lag', 'force_diff_velocity',
              'gate_coverage',
              'gate_vel_rmse', 'gate_still', 'gate_slam_xy', 'gate_slam_z'):
        setattr(cfg, k, getattr(a, k))

    files = []
    for pat in a.log:
        files.extend(sorted(glob.glob(pat)) or [pat])
    if not files:
        ap.error("no log file given (or use --selftest)")

    all_pass = True
    if a.brief:
        np.seterr(all='ignore')
        import warnings as _w
        _w.filterwarnings('ignore')
        print(f"{'log':<32}{'dur':>6}{'cov%':>7}{'dist':>8}{'vRMSE':>7}"
              f"{'hover':>7}{'slamXY':>8}{'slamZ':>7}  note")
        for fp in files:
            try:
                R = analyse(load_run(fp), cfg)
            except Exception as e:
                print(f"{os.path.basename(fp):<32}  ERROR {e}")
                continue
            cov = 100 * R.get('rio_cov', {}).get('frac_of_flight', float('nan'))
            gm = R.get('gps_path_matched')
            ratio = (R['rio_path_3d'] / gm[0]) if (gm and gm[0] > 1) else float('nan')
            vr = np.mean([s['rmse'] for s in R.get('vel_enu', [])
                          if s.get('n', 0) > 3]) if R.get('vel_enu') else float('nan')
            hv = R.get('still', {}).get('rio_hspeed_mean', float('nan'))
            S = R.get('slam')
            sx = S['err_yaw'][-1] if S else float('nan')
            sz = S['err_z'][-1] if S else float('nan')
            note = ""
            if R.get('rio_att_missing', 0) and R.get('rio_att_ok') is not None \
                    and R['rio_att_ok'].sum() == 0:
                note = "NO IMU -> RIO columns undefined"
            elif R.get('counts', {}).get('rio', 0) < 20:
                note = "too few RIO frames"
            elif R.get('counts', {}).get('slam', 0) < 5:
                note = "SLAM barely ran"
            print(f"{os.path.basename(fp):<32}{R['duration']:>6.0f}{cov:>7.1f}"
                  f"{ratio:>8.3f}{vr:>7.3f}{hv:>7.3f}{sx:>8.2f}{sz:>7.2f}  {note}")
        sys.exit(0)

    for fp in files:
        run = load_run(fp)
        R = analyse(run, cfg)
        ok = print_report(run, R, cfg)
        all_pass &= ok
        if a.plot:
            out = a.plot if len(files) == 1 else \
                os.path.splitext(a.plot)[0] + "_" + os.path.basename(fp) + ".png"
            try:
                make_plots(run, R, out)
                print(f"  figure -> {out}")
            except Exception as e:
                print(f"  [plot failed] {e}")
        if a.json:
            out = a.json if len(files) == 1 else \
                os.path.splitext(a.json)[0] + "_" + os.path.basename(fp) + ".json"
            def _clean(o):
                if isinstance(o, np.ndarray):
                    return o.tolist()
                if isinstance(o, (np.floating, np.integer)):
                    return o.item()
                if isinstance(o, dict):
                    return {k: _clean(v) for k, v in o.items() if not k.startswith('_')}
                if isinstance(o, (list, tuple)):
                    return [_clean(x) for x in o]
                if isinstance(o, float) and not math.isfinite(o):
                    return None
                return o
            with open(out, 'w') as f:
                json.dump(_clean(R), f, indent=1, default=str)
            print(f"  metrics -> {out}")

    sys.exit(0 if all_pass else 1)


if __name__ == '__main__':
    main()
