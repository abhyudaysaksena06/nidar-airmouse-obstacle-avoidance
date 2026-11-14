#!/usr/bin/env bash
# OA-Stack in Gazebo with a live evaluation.
#   bash gazebo/run_gazebo.sh                      # mixed_clutter, seeds 207..211 (5 worlds)
#   bash gazebo/run_gazebo.sh guide_people 213 3   # scenario, first seed, number of worlds
# Needs: Gazebo Harmonic (gz sim), python3-numpy, python3-gz-transport13, python3-gz-msgs10.
set -u
cd "$(dirname "$0")/.."
SC=${1:-mixed_clutter}
SEED=${2:-207}
RUNS=${3:-5}

# --- checks ---------------------------------------------------------------------------
if ! command -v gz >/dev/null 2>&1; then
  echo "Gazebo (gz) not found. Install Gazebo Harmonic: https://gazebosim.org/docs/harmonic/install_ubuntu"; exit 1
fi
missing=""
python3 -c "import numpy" 2>/dev/null || missing="$missing python3-numpy"
python3 -c "import gz.transport13, gz.msgs10" 2>/dev/null \
  || python3 -c "import gz.transport14, gz.msgs11" 2>/dev/null \
  || missing="$missing python3-gz-transport13 python3-gz-msgs10"
if [ -n "$missing" ]; then
  echo "Missing:$missing"
  echo "Install with:  sudo apt install$missing"; exit 1
fi

CSV="results/gazebo_${SC}_$(date +%Y%m%d_%H%M%S).csv"
mkdir -p results
cleanup() { pkill -f "gz sim -r /tmp/airmouse_" >/dev/null 2>&1; }
trap 'cleanup; exit 130' INT TERM

for i in $(seq 0 $((RUNS - 1))); do
  S=$((SEED + i))
  W="/tmp/airmouse_${SC}_${S}.sdf"
  python3 gazebo/gz_world.py --scenario "$SC" --seed "$S" --out "$W" || continue
  gz sim -r "$W" >/tmp/gz_${S}.log 2>&1 &
  sleep 3
  python3 gazebo/oa_gazebo.py --scenario "$SC" --seed "$S" --run $((i + 1)) --runs "$RUNS" --csv "$CSV"
  cleanup
  sleep 2
done

echo
echo "=== Gazebo evaluation: $SC, seeds $SEED..$((SEED + RUNS - 1)) ==="
python3 - "$CSV" <<'EOF'
import csv, sys
rows = list(csv.DictReader(open(sys.argv[1])))
n = len(rows)
if not n:
    sys.exit("no runs finished")
k = lambda w: sum(r["result"].startswith(w) for r in rows)
print(f"{'seed':>6} {'result':<20} {'time s':>7} {'min clear m':>12} {'speed m/s':>10}")
for r in rows:
    print(f"{r['seed']:>6} {r['result']:<20} {r['time_s']:>7} {r['min_clearance_m']:>12} {r['mean_speed_mps']:>10}")
print(f"\nsuccess {k('success')}/{n}   collision {k('collision')}/{n}   timeout {k('timeout')}/{n}")
print("saved", sys.argv[1])
EOF
