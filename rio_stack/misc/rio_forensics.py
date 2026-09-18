#!/usr/bin/env python3
"""rio_forensics.py -- reproduces every number in RIO_DISTANCE_ROOT_CAUSE.md.

Usage:
    python3 docs/rio_forensics.py logs/run_20260918_*.jsonl

Edit FLIGHT_TILT below if a log was flown at a mount angle not listed; the
null-direction test depends on it.

The three questions this answers:
  1. How much of the missing distance is "RIO published exactly zero"?
     -> loss attribution table
  2. Are the velocity spikes a Nyquist fold, or a slide along the
     unobservable Vx/Vz direction?
     -> null-direction alignment test (the alignment MUST track the mount
        angle; if it does, it is geometry, not aliasing)
  3. Which code branch is producing the zeros?
     -> inlier histogram on zero frames. Clustering at 8-9 is the
        doppler_ransac() static-hypothesis signature.
"""
import json, math, sys, collections
import numpy as np

# Mount angle (degrees, nose-down) per flight. Used for the null-direction test.
FLIGHT_TILT = {
    '20260918_123405': 45.0, '20260918_123656': 45.0, '20260918_123957': 45.0,
    '20260918_144805': 25.0, '20260918_145032': 25.0,
}
DEFAULT_TILT = 45.0
MEDIAN_K = 15          # GPS speed median-filter width (samples @ ~50 Hz)


def load(path):
    d = collections.defaultdict(list)
    for line in open(path):
        try:
            j = json.loads(line)
        except Exception:
            continue
        d[j.get('type')].append(j)
    return d


def medfilt(x, k):
    h = k // 2
    return np.array([np.median(x[max(0, i - h):i + h + 1]) for i in range(len(x))])


def analyse(path):
    key = path.split('/')[-1].replace('run_', '').replace('.jsonl', '')
    tilt = FLIGHT_TILT.get(key, DEFAULT_TILT)
    d = load(path)
    rio, gps, imu, alt = d['rio'], d['gps'], d['imu'], d['altimeter']
    if not rio or not gps:
        print(f"{path}: no rio/gps records"); return

    tr = np.array([r['t_frame'] for r in rio])
    v = np.array([[r['vx'], r['vy'], r['vz']] for r in rio])
    inl = np.array([r['inliers'] for r in rio])
    sp_r = np.linalg.norm(v[:, :2], axis=1)
    nrm = np.linalg.norm(v, axis=1)
    zero = (np.abs(v).sum(axis=1) == 0.0)

    tg = np.array([g['t_mono'] for g in gps]); enu = np.array([g['enu'] for g in gps])
    dtg = np.diff(tg); ok = dtg > 1e-3
    vg = np.diff(enu, axis=0)[ok] / dtg[ok, None]
    tvg = (tg[:-1] + tg[1:])[ok] / 2
    sp_gps = medfilt(np.linalg.norm(vg[:, :2], axis=1), MEDIAN_K)
    gspd = np.interp(tr, tvg, sp_gps)

    ta = np.array([a['t_mono'] for a in alt]) if alt else np.array([])
    agl = np.interp(tr, ta, [a['range_m'] for a in alt]) if len(ta) > 2 else np.full(len(tr), np.nan)

    T = tr[-1] - tr[0]
    dt = np.clip(np.diff(tr, prepend=tr[0]), 0, 0.5)
    d_rio = float(np.sum(sp_r * dt))
    d_gps = float(np.sum(np.linalg.norm(np.diff(enu[:, :2], axis=0), axis=1)))

    print("=" * 94)
    print(f"{key}   tilt {tilt:.0f} deg   {T:.1f}s   rio {len(rio)} ({len(rio)/T:.1f} Hz)")
    print(f"  radar frame dt: p50 {np.median(np.diff(tr))*1000:.1f} ms  "
          f"pipeline latency p50 {np.median([r['t_mono']-r['t_frame'] for r in rio])*1000:.1f} ms")
    if np.isfinite(agl).any():
        print(f"  AGL median {np.nanmedian(agl):.1f} m   GPS-up median "
              f"{np.median(np.interp(tr, tg, enu[:,2])):.1f} m")
    print(f"  GPS max speed: {sp_gps.max():.2f} m/s (median-{MEDIAN_K} filtered)")

    # -- 1. zero-publishing --
    print(f"\n  [1] ZERO-PUBLISHING")
    print(f"      exact-zero frames            : {zero.sum()}/{len(v)} = {100*zero.mean():.1f}%")
    print(f"      RIO path {d_rio:7.1f} m | GPS path {d_gps:7.1f} m | ratio {d_rio/d_gps:.3f} "
          f"(error {100*(1-d_rio/d_gps):+.1f}%)")
    mv = gspd > 1.0
    lost0 = float(np.sum(gspd[mv & zero] * dt[mv & zero]))
    print(f"      distance flown while v=(0,0,0): {lost0:7.1f} m ({100*lost0/d_gps:.1f}% of path)")
    nz = mv & ~zero
    if nz.sum() > 10:
        r = sp_r[nz] / np.clip(gspd[nz], 0.5, None)
        print(f"      RIO/GPS speed ratio on NON-ZERO frames: median {np.median(r):.3f} "
              f"(>1.0 means OVER-reporting -- there is no scale deficit)")
    # counterfactual: hold last good
    sp_h = sp_r.copy(); last = 0.0
    for i in range(len(sp_h)):
        if zero[i]: sp_h[i] = last
        else: last = sp_h[i]
    print(f"      COUNTERFACTUAL hold-last-good  : {np.sum(sp_h*dt):7.1f} m "
          f"(ratio {np.sum(sp_h*dt)/d_gps:.3f}) <- why R1 alone is not enough")

    # -- 2. null-direction test --
    th = math.radians(tilt)
    nhat = np.array([math.sin(th), 0.0, -math.cos(th)])
    with np.errstate(invalid='ignore'):
        align = np.abs(v @ nhat) / np.clip(nrm, 1e-9, None)
    big = (nrm > 5.0); ctrl = (nrm > 0.05) & (nrm < 3.0)
    print(f"\n  [2] NULL-DIRECTION TEST   n_hat = ({nhat[0]:+.3f}, 0, {nhat[2]:+.3f})")
    if big.sum():
        print(f"      frames |v|>5 m/s: {big.sum():3d}   mean |cos to n_hat| {align[big].mean():.3f} "
              f"median {np.median(align[big]):.3f}")
        print(f"      ... their GPS true speed: mean {gspd[big].mean():.2f} max {gspd[big].max():.2f} m/s")
        print(f"      ... their mean inliers  : {inl[big].mean():.1f}")
    if ctrl.sum():
        print(f"      CONTROL |v|<3 m/s        : mean |cos to n_hat| {align[ctrl].mean():.3f}")
    print(f"      -> spikes aligned with n_hat AND n_hat varies with tilt => GEOMETRY, not aliasing")

    # -- 3. which branch --
    i = inl[zero]
    if len(i):
        bins = [('<=3', i <= 3), ('4-5', (i >= 4) & (i <= 5)), ('6-7', (i >= 6) & (i <= 7)),
                ('8-9', (i >= 8) & (i <= 9)), ('10-14', (i >= 10) & (i <= 14)),
                ('15-24', (i >= 15) & (i <= 24)), ('>=25', i >= 25)]
        print(f"\n  [3] INLIER HISTOGRAM ON ZERO FRAMES (static-branch signature)")
        print("      " + "  ".join(f"{n}:{100*m.mean():.0f}%" for n, m in bins))
    for th_ in (8, 10, 15):
        m = inl < th_
        if m.any():
            print(f"      inliers < {th_:2d}: {100*m.mean():5.1f}% of all frames, "
                  f"{100*zero[m].mean():5.1f}% of those are exact zero")

    # -- 4. aliasing check --
    print(f"\n  [4] NYQUIST/FOLD CHECK (non-zero frames only)")
    nzz = nrm > 0.05
    for lo, hi in [(2,3),(3,4),(4,5),(5,6),(6,8),(8,12),(12,20)]:
        m = nzz & (gspd >= lo) & (gspd < hi)
        if m.sum() < 4: continue
        print(f"      GPS {lo:2d}-{hi:2d} m/s  n={m.sum():3d}  RIO {sp_r[m].mean():6.2f}  "
              f"ratio {sp_r[m].mean()/gspd[m].mean():5.2f}  inl {inl[m].mean():5.1f}")
    print(f"      -> a hardware v_max would show a CEILING and sign inversions; absence => not Nyquist")

    # -- 5. starvation --
    if np.isfinite(agl).any():
        print(f"\n  [5] SIGNAL STARVATION vs ALTITUDE")
        for lo, hi in [(0,5),(5,10),(10,15),(15,20),(20,30),(30,60)]:
            m = (agl >= lo) & (agl < hi)
            if m.sum() < 5: continue
            print(f"      AGL {lo:2d}-{hi:2d} m  n={m.sum():4d}  inliers {inl[m].mean():5.1f}  "
                  f"zero {100*zero[m].mean():5.1f}%")


if __name__ == '__main__':
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    for p in sys.argv[1:]:
        analyse(p)
