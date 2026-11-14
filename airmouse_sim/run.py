"""AirMouse simulator entry point.

  python run.py                                         # watch the baseline
  python run.py --algo algorithms/my_algo.py --seed 42  # your algorithm, one map
  python run.py --profile hard --profile faults         # harder maps + sensor faults
  python run.py --set drone.vel_pid.kp=2.0              # override any config value
  python run.py --headless --runs 50 --jobs 8 --label v2  # batch -> leaderboard.csv
  python run.py --leaderboard                           # show the leaderboard
  python run.py --seed 42 --report reports/             # JSON + event log + PNG
  python run.py --seed 42 --export-gazebo maze42.sdf
"""
import argparse
import copy
import csv
import importlib.util
import json
import os
import statistics
import sys
import time
from concurrent.futures import ProcessPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from sim.engine import Simulation  # noqa: E402
from sim.world import generate_world  # noqa: E402

LEADERBOARD = os.path.join(HERE, "leaderboard.csv")


def deep_update(base, over):
    for k, v in over.items():
        if k.startswith("_"):
            continue
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_update(base[k], v)
        else:
            base[k] = v
    return base


def load_config(path, profiles, sets):
    with open(path) as f:
        cfg = json.load(f)
    for p in profiles:
        fp = p if p.endswith(".json") else os.path.join(HERE, "profiles", p + ".json")
        with open(fp) as f:
            deep_update(cfg, json.load(f))
    for s in sets:
        key, val = s.split("=", 1)
        node = cfg
        parts = key.split(".")
        for k in parts[:-1]:
            node = node[k]
        try:
            node[parts[-1]] = json.loads(val)
        except json.JSONDecodeError:
            node[parts[-1]] = val
    return cfg


def load_algo(path):
    spec = importlib.util.spec_from_file_location("user_algo", path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.Algorithm


def run_one(args):
    cfg, algo_path, seed = args
    world = generate_world(copy.deepcopy(cfg), seed)
    sim = Simulation(world, load_algo(algo_path)(), seed=world.seed)
    return sim.run()


def save_report(sim, out_dir):
    os.makedirs(out_dir, exist_ok=True)
    base = os.path.join(out_dir, f"mission_seed{sim.w.seed}")
    with open(base + ".json", "w") as f:
        json.dump({"score": sim.score(), "events": sim.events,
                   "survivors_reported_world": sim.reported_world(),
                   "survivors_true": [(s.id, round(s.x, 2), round(s.y, 2)) for s in sim.w.survivors]},
                  f, indent=2)
    with open(base + "_events.txt", "w") as f:
        f.writelines(f"{t:7.1f}s  {m}\n" for t, m in sim.events)
    os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
    from sim.render import Viewer
    v = Viewer(sim)
    v.show_rays = False
    v.draw()
    v.save(base + ".png")
    print("report written:", base + ".json/.png/_events.txt")


def print_leaderboard():
    if not os.path.exists(LEADERBOARD):
        print("no leaderboard yet - run a batch with --headless --runs N --label NAME")
        return
    with open(LEADERBOARD) as f:
        rows = list(csv.DictReader(f))
    rows.sort(key=lambda r: -float(r["mean_score"]))
    cols = ["label", "algo", "profiles", "runs", "mean_score", "std_score", "survivors_pct",
            "crash_pct", "exit_pct", "coverage_pct", "mean_loc_err_m", "date"]
    print(" | ".join(f"{c:>12}" for c in cols))
    for r in rows:
        print(" | ".join(f"{str(r.get(c, ''))[:12]:>12}" for c in cols))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--algo", default=os.path.join(HERE, "algorithms", "explorer.py"))
    ap.add_argument("--config", default=os.path.join(HERE, "config.json"))
    ap.add_argument("--profile", action="append", default=[], help="profiles/<name>.json, repeatable")
    ap.add_argument("--set", action="append", default=[], help="override, e.g. sensors.camera_type=depth")
    ap.add_argument("--seed", type=int, default=None)
    ap.add_argument("--headless", action="store_true")
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--jobs", type=int, default=1, help="parallel processes for batch runs")
    ap.add_argument("--csv", default=None)
    ap.add_argument("--label", default=None, help="add batch result to leaderboard.csv")
    ap.add_argument("--leaderboard", action="store_true")
    ap.add_argument("--report", default=None, metavar="DIR")
    ap.add_argument("--export-gazebo", default=None, metavar="FILE.sdf")
    a = ap.parse_args()

    if a.leaderboard:
        print_leaderboard()
        return
    cfg = load_config(a.config, a.profile, a.set)
    Algo = load_algo(a.algo)

    def new_sim(seed):
        world = generate_world(copy.deepcopy(cfg), seed)
        return Simulation(world, Algo(), seed=world.seed)

    if a.export_gazebo:
        from sim.gazebo_export import export_sdf
        w = generate_world(cfg, a.seed)
        print("wrote", export_sdf(w, a.export_gazebo), "seed", w.seed)
        return

    if not a.headless:
        from sim.render import Viewer
        sim = new_sim(a.seed)
        print("seed", sim.w.seed)
        v = Viewer(sim)
        sc = v.loop(new_sim)
        print(json.dumps(sc, indent=2))
        if a.report:
            save_report(v.sim, a.report)
        return

    if a.runs == 1 and a.report:
        sim = new_sim(a.seed)
        sim.run()
        print(json.dumps(sim.score(), indent=2))
        save_report(sim, a.report)
        return

    import random
    base_seed = a.seed if a.seed is not None else random.randrange(1_000_000)
    jobs = [(cfg, a.algo, base_seed + k) for k in range(a.runs)]
    t0 = time.time()
    results = []
    if a.jobs > 1:
        with ProcessPoolExecutor(max_workers=a.jobs) as ex:
            for k, sc in enumerate(ex.map(run_one, jobs)):
                results.append(sc)
                _print_row(k, a.runs, sc)
    else:
        for k, job in enumerate(jobs):
            sc = run_one(job)
            results.append(sc)
            _print_row(k, a.runs, sc)

    n = len(results)
    found = sum(r["survivors_found"] for r in results)
    total = sum(r["survivors_total"] for r in results)
    scores = [r["score"] for r in results]
    errs = [r["mean_loc_error_m"] for r in results if r["mean_loc_error_m"] is not None]
    summary = {
        "label": a.label or "",
        "algo": os.path.basename(a.algo),
        "profiles": "+".join(a.profile) or "default",
        "runs": n,
        "seeds": f"{base_seed}-{base_seed + n - 1}",
        "mean_score": round(statistics.mean(scores), 1),
        "std_score": round(statistics.pstdev(scores), 1),
        "survivors_pct": round(100 * found / max(1, total), 1),
        "crash_pct": round(100 * sum(r["crashed"] for r in results) / n, 1),
        "exit_pct": round(100 * sum(r["exited_and_landed"] for r in results) / n, 1),
        "coverage_pct": round(sum(r["coverage_pct"] for r in results) / n, 1),
        "mean_loc_err_m": round(statistics.mean(errs), 3) if errs else "",
        "date": time.strftime("%Y-%m-%d %H:%M"),
    }
    print(f"\nSUMMARY over {n} maps ({time.time() - t0:.0f}s): score {summary['mean_score']} "
          f"+- {summary['std_score']} | survivors {found}/{total} ({summary['survivors_pct']}%) | "
          f"crashes {summary['crash_pct']}% | exit+land {summary['exit_pct']}% | "
          f"coverage {summary['coverage_pct']}%")
    if a.csv:
        with open(a.csv, "w", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(results[0]))
            wr.writeheader()
            wr.writerows(results)
        print("wrote", a.csv)
    if a.label:
        new = not os.path.exists(LEADERBOARD)
        with open(LEADERBOARD, "a", newline="") as f:
            wr = csv.DictWriter(f, fieldnames=list(summary))
            if new:
                wr.writeheader()
            wr.writerow(summary)
        print("added to leaderboard.csv as", a.label)


def _print_row(k, n, sc):
    print(f"[{k + 1}/{n}] seed {sc['seed']:>6}  score {sc['score']:>7}  found {sc['survivors_found']}/"
          f"{sc['survivors_total']}  false {sc['false_reports']}  err {sc['mean_loc_error_m']}  "
          f"coll {sc['collisions']}  exit {sc['exited_and_landed']}  cov {sc['coverage_pct']}%  "
          f"batt {sc['battery_left_pct']}%  t {sc['time_s']}s  ({sc['end_reason']})", flush=True)


if __name__ == "__main__":
    main()
