#!/usr/bin/env python3
"""NIDAR PID analysis: how well does each control loop follow its command?

    python3 tools/pid_analyze.py ~/nidar_logs/pid_20261127_210000.csv   # from pid_logger
    python3 tools/pid_analyze.py ~/PX4-Autopilot/build/px4_sitl_default/rootfs/log/2026-11-27/21_00_00.ulg
    python3 tools/pid_analyze.py <file> --plots out_dir      # also save PNG graphs

For every axis it reports
    RMS / max tracking error over the whole flight, and for every STEP in the
    command (fly the `step` mission to make clean steps):
    rise time (10 -> 90 %), overshoot %, settling time (within 5 %), steady-state error.
Then it prints which PX4 parameter to change, in plain words.

The .ulg route needs nothing extra: PX4 logs every setpoint on its own.
"""
import argparse
import csv
import math
import os
import sys

import numpy as np

# axis -> (group, unit scale for printing, unit, PX4 params to tune)
AXES = {
    "x": ("position", 1, "m", "MPC_XY_P"),
    "y": ("position", 1, "m", "MPC_XY_P"),
    "z": ("position", 1, "m", "MPC_Z_P"),
    "vx": ("velocity", 1, "m/s", "MPC_XY_VEL_P_ACC / _I_ACC / _D_ACC"),
    "vy": ("velocity", 1, "m/s", "MPC_XY_VEL_P_ACC / _I_ACC / _D_ACC"),
    "vz": ("velocity", 1, "m/s", "MPC_Z_VEL_P_ACC / _I_ACC / _D_ACC"),
    "roll": ("attitude", 180 / math.pi, "deg", "MC_ROLL_P"),
    "pitch": ("attitude", 180 / math.pi, "deg", "MC_PITCH_P"),
    "yaw": ("attitude", 180 / math.pi, "deg", "MC_YAW_P"),
    "rollrate": ("rate", 180 / math.pi, "deg/s", "MC_ROLLRATE_P / _I / _D"),
    "pitchrate": ("rate", 180 / math.pi, "deg/s", "MC_PITCHRATE_P / _I / _D"),
    "yawrate": ("rate", 180 / math.pi, "deg/s", "MC_YAWRATE_P / _I / _D"),
}
# a command jump bigger than this counts as a "step"
MAX_LISTED = 12
STEP_WINDOW = 8.0   # s, longest time a single step is judged over
STEP_MIN = {"position": 0.3, "velocity": 0.3, "attitude": math.radians(8), "rate": math.radians(40)}


# ------------------------------------------------------------------ loading
def load_csv(path):
    with open(path) as fh:
        rows = list(csv.DictReader(fh))
    t = np.array([float(r["t"]) for r in rows])
    data = {}
    for ax in AXES:
        if f"{ax}_sp" not in rows[0]:
            continue
        sp = np.array([float(r[f"{ax}_sp"]) if r[f"{ax}_sp"] else np.nan for r in rows])
        act = np.array([float(r[ax]) if r[ax] else np.nan for r in rows])
        data[ax] = (t, sp, act)
    return data


def quat_to_euler(w, x, y, z):
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1, 1))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return roll, pitch, yaw


def load_ulog(path, all_modes=False):
    try:
        from pyulog import ULog
    except ImportError:
        sys.exit("pip install pyulog   (needed to read .ulg files)")
    ulog = ULog(path)

    def topic(name):
        for d in ulog.data_list:
            if d.name == name and d.multi_id == 0:
                return d.data
        return None

    data = {}

    def put(ax, ts_sp, sp, ts_act, act):
        t = ts_act / 1e6
        # zero-order hold: a setpoint holds its value until the next one (linear
        # interpolation would turn a step into a ramp and distort the step metrics)
        idx = np.searchsorted(ts_sp, ts_act, side="right") - 1
        sp_i = np.where(idx >= 0, np.asarray(sp, float)[np.clip(idx, 0, len(sp) - 1)], np.nan)
        data[ax] = (t, sp_i, act)

    lp, lps = topic("vehicle_local_position"), topic("vehicle_local_position_setpoint")
    if lp is not None and lps is not None:
        for ax in ("x", "y", "z", "vx", "vy", "vz"):
            put(ax, lps["timestamp"], lps[ax], lp["timestamp"], lp[ax])
    att, atts = topic("vehicle_attitude"), topic("vehicle_attitude_setpoint")
    if att is not None and atts is not None:
        a = quat_to_euler(att["q[0]"], att["q[1]"], att["q[2]"], att["q[3]"])
        s = quat_to_euler(atts["q_d[0]"], atts["q_d[1]"], atts["q_d[2]"], atts["q_d[3]"])
        for i, ax in enumerate(("roll", "pitch", "yaw")):
            # unwrap yaw so +-180 deg crossings are not seen as huge errors
            sp, ac = (np.unwrap(s[i]), np.unwrap(a[i])) if ax == "yaw" else (s[i], a[i])
            put(ax, atts["timestamp"], sp, att["timestamp"], ac)
    rv, rs = topic("vehicle_angular_velocity"), topic("vehicle_rates_setpoint")
    if rv is not None and rs is not None:
        for i, ax in enumerate(("rollrate", "pitchrate", "yawrate")):
            put(ax, rs["timestamp"], rs[("roll", "pitch", "yaw")[i]], rv["timestamp"], rv[f"xyz[{i}]"])
    if not data:
        sys.exit("no setpoint/actual pairs found in this log")

    # Only judge the controllers while flying in Offboard / Position / Hold / Mission
    # (not on the ground, not during take-off or landing, where setpoints are special).
    st = topic("vehicle_status")
    if st is not None and not all_modes:
        FLYING = {2, 3, 4, 14}   # POSCTL, AUTO_MISSION, AUTO_LOITER, OFFBOARD
        ts, nav, arm = st["timestamp"], st["nav_state"], st["arming_state"]

        def flying(tq):
            i = np.clip(np.searchsorted(ts, tq * 1e6, side="right") - 1, 0, len(ts) - 1)
            return np.isin(nav[i], list(FLYING)) & (arm[i] == 2)
        kept = {}
        for ax, (t, sp, act) in data.items():
            m = flying(t)
            if m.sum() > 10:
                sp = np.where(m, sp, np.nan)
                kept[ax] = (t, sp, act)
        if kept:
            data = kept
            print("(analysing only the time spent flying in Offboard/Position/Hold/Mission; "
                  "use --all-modes for the whole log)")
    t0 = min(v[0][0] for v in data.values())
    return {k: (v[0] - t0, v[1], v[2]) for k, v in data.items()}


# ------------------------------------------------------------------ analysis
def find_steps(t, sp, min_jump):
    """Indices where the command jumps by more than min_jump within 0.1 s."""
    steps, last = [], -1e9
    for i in range(1, len(sp)):
        j = i
        while j > 0 and t[i] - t[j - 1] < 0.1:
            j -= 1
        if np.isnan(sp[i]) or np.isnan(sp[j]):
            continue
        if abs(sp[i] - sp[j]) > min_jump and t[i] - last > 1.0:
            steps.append(i)
            last = t[i]
    return steps


def step_metrics(t, sp, act, i0, i_end, min_size=0.0):
    """Rise time, overshoot, settling time and steady-state error for one step.
    Returns None for a pulse (command jumps but comes straight back): only a
    command that STAYS at its new value (by >= min_size) is a step."""
    before = act[max(0, i0 - 3)]
    seg_sp = sp[i0:i_end]
    seg_sp = seg_sp[~np.isnan(seg_sp)]
    if len(seg_sp) < 5:
        return None
    target = float(np.median(seg_sp))       # the value the command settles at
    size = target - before
    if abs(size) < max(min_size, 1e-9):
        return None
    seg_t, seg = t[i0:i_end] - t[i0], (act[i0:i_end] - before) / size   # 0 -> 1 normalised
    ok = ~np.isnan(seg)
    seg_t, seg = seg_t[ok], seg[ok]
    if len(seg) < 5:
        return None
    r10 = np.argmax(seg >= 0.1) if np.any(seg >= 0.1) else None
    r90 = np.argmax(seg >= 0.9) if np.any(seg >= 0.9) else None
    rise = seg_t[r90] - seg_t[r10] if r10 is not None and r90 is not None else math.nan
    overshoot = max(0.0, (np.max(seg) - 1) * 100)
    outside = np.where(np.abs(seg - 1) > 0.05)[0]
    settle = seg_t[outside[-1] + 1] if len(outside) and outside[-1] + 1 < len(seg) else (
        0.0 if not len(outside) else math.nan)
    tail = seg[seg_t > seg_t[-1] - 1.0]
    sse = (1 - np.mean(tail)) * size
    crossings = np.sum(np.diff(np.sign(seg - 1)) != 0)
    return dict(size=size, rise=rise, overshoot=overshoot, settle=settle, sse=sse,
                crossings=int(crossings))


def advice(group, params, m):
    tips = []
    if m["overshoot"] > 25 or m["crossings"] > 4:
        tips.append(f"overshoots/oscillates -> LOWER the P gain ({params.split(' /')[0]})"
                    + (" or raise D" if group in ("rate", "velocity") else ""))
    elif not math.isnan(m["rise"]) and m["overshoot"] < 2 and m["rise"] > {
            "position": 2.0, "velocity": 1.0, "attitude": 0.3, "rate": 0.08}[group]:
        tips.append(f"slow and no overshoot -> RAISE the P gain ({params.split(' /')[0]}) by ~20 %")
    if math.isnan(m["settle"]):
        tips.append("never settles inside 5 % -> check for oscillation, lower P")
    return tips


def analyse(data, plots=None):
    groups = {}
    for ax, (t, sp, act) in data.items():
        g, k, unit, params = AXES[ax]
        ok = ~np.isnan(sp) & ~np.isnan(act)
        if ok.sum() < 10:
            continue
        err = sp[ok] - act[ok]
        steps = find_steps(t, sp, STEP_MIN[g])
        bounds = steps + [len(t)]
        res = []
        for a, b in zip(bounds[:-1], bounds[1:]):
            # judge each step only until the command stops (NaN = other flight mode)
            # and for at most STEP_WINDOW seconds
            b = min(b, int(np.searchsorted(t, t[a] + STEP_WINDOW)))
            gap = np.where(np.isnan(sp[a:b]))[0]
            if len(gap):
                b = a + int(gap[0])
            m = step_metrics(t, sp, act, a, b, 0.5 * STEP_MIN[g])
            if m:
                m["t"] = t[a]
                res.append(m)
        groups.setdefault(g, []).append((ax, k, unit, params, err, res))

    order = ["rate", "attitude", "velocity", "position"]
    print("=" * 78)
    print("NIDAR PID report (inner loop first - always tune rate -> attitude -> velocity -> position)")
    print("=" * 78)
    for g in order:
        if g not in groups:
            continue
        print(f"\n## {g.upper()} loop")
        for ax, k, unit, params, err, res in groups[g]:
            print(f"  {ax:9s} RMS error {np.sqrt(np.mean(err ** 2)) * k:8.3f} {unit:5s} "
                  f"max {np.max(np.abs(err)) * k:8.3f} {unit}   ({len(res)} steps)")
            if len(res) > MAX_LISTED:
                # a noisy/oscillating command makes dozens of "steps": summarise them
                ov = np.median([m["overshoot"] for m in res])
                unsettled = sum(math.isnan(m["settle"]) for m in res)
                print(f"      {len(res)} command jumps (noisy or oscillating): median overshoot "
                      f"{ov:.0f}%, {unsettled} never settle")
                tips = advice(g, params, dict(overshoot=ov, crossings=99 if unsettled > len(res) / 2 else 0,
                                              rise=math.nan, settle=0.0))
                for tip in tips:
                    print(f"        -> {tip}")
                continue
            for m in res:
                print(f"      step at {m['t']:6.1f}s size {m['size'] * k:+7.2f} {unit}: "
                      f"rise {m['rise']:5.2f}s  overshoot {m['overshoot']:5.1f}%  "
                      f"settle {m['settle']:5.2f}s  steady err {m['sse'] * k:+.3f} {unit}")
                for tip in advice(g, params, m):
                    print(f"        -> {tip}")
            sse = [abs(m["sse"]) for m in res]
            if sse and np.median(sse) > 0.05 * max(abs(m["size"]) for m in res):
                # PX4's position and attitude loops are P-only: the integrator that removes
                # their steady-state error lives one loop further in
                i_gain = {"position": "MPC_XY_VEL_I_ACC / MPC_Z_VEL_I_ACC (position loop is P-only)",
                          "attitude": "MC_ROLLRATE_I / MC_PITCHRATE_I / MC_YAWRATE_I (attitude loop is P-only)"
                          }.get(g, params.split(" /")[0].rsplit("_", 1)[0] + "_I" if g == "rate"
                                else params.replace("_P_ACC / _I_ACC / _D_ACC", "_I_ACC"))
                print(f"        -> steady-state error stays -> RAISE the I gain ({i_gain})")
    if plots:
        make_plots(data, plots)


def make_plots(data, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    os.makedirs(out, exist_ok=True)
    for g in ("position", "velocity", "attitude", "rate"):
        axes = [a for a in data if AXES[a][0] == g]
        if not axes:
            continue
        fig, plts = plt.subplots(len(axes), 1, figsize=(11, 2.6 * len(axes)), sharex=True)
        plts = np.atleast_1d(plts)
        for p, ax in zip(plts, axes):
            t, sp, act = data[ax]
            k, unit = AXES[ax][1], AXES[ax][2]
            p.plot(t, sp * k, "--", color="#d9480f", lw=1.6, label="command (setpoint)")
            p.plot(t, act * k, color="#1c7ed6", lw=1.2, label="actual")
            p.set_ylabel(f"{ax} [{unit}]")
            p.grid(alpha=0.3)
        plts[0].legend(loc="upper right")
        plts[0].set_title(f"{g} loop: command vs actual")
        plts[-1].set_xlabel("time [s]")
        fig.tight_layout()
        f = os.path.join(out, f"pid_{g}.png")
        fig.savefig(f, dpi=110)
        plt.close(fig)
        print("saved", f)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("log", help=".csv from pid_logger or .ulg from PX4")
    ap.add_argument("--plots", metavar="DIR", help="save command-vs-actual PNGs here")
    ap.add_argument("--all-modes", action="store_true", help=".ulg: include ground, take-off and landing")
    a = ap.parse_args()
    data = load_ulog(a.log, a.all_modes) if a.log.endswith(".ulg") else load_csv(a.log)
    analyse(data, a.plots)


if __name__ == "__main__":
    main()
