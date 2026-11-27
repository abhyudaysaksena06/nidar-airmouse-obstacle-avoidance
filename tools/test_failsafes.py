#!/usr/bin/env python3
"""Automatic test of every NIDAR Rulebook v2.2 fail-safe (PS2 rule 8.31 + section 10) in PX4 SITL.

Needs (as in the guide, section 7.2): PX4 SITL with gz_nidar_drone, MicroXRCEAgent, and
`ros2 launch nidar_drone_description px4_sensors.launch.py` running. QGroundControl must be
CLOSED (this script plays the ground station itself for the comm-loss test).

    python3 tools/test_failsafes.py                  # all tests
    python3 tools/test_failsafes.py recall abort     # only some

Each test starts a hover mission over the open floor, triggers one fail-safe and checks the
node's reaction and that the drone ends LANDED and DISARMED. ~2-4 min per test in a slow VM.
"""
import json
import os
import signal
import subprocess
import sys
import threading
import time

PX4_DIR = os.path.expanduser(os.environ.get("PX4_DIR", "~/PX4-Autopilot"))
PXH = os.path.join(PX4_DIR, "build/px4_sitl_default/bin")


def px4(*args):
    """Run a PX4 console command (e.g. param set) from outside the pxh> prompt."""
    return subprocess.run([os.path.join(PXH, "px4-" + args[0]), *args[1:]], capture_output=True,
                          text=True, timeout=20).stdout


class Run:
    """One offboard_control process + its log."""

    def __init__(self, name, extra):
        self.name, self.lines = name, []
        cmd = ["ros2", "run", "nidar_control", "offboard_control", "--ros-args",
               "-p", "mission:=hover", "-p", "hover_time:=300.0", "-p", "altitude:=1.2"] + extra
        self.p = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
                                  preexec_fn=os.setsid)
        threading.Thread(target=self._read, daemon=True).start()

    def _read(self):
        for ln in self.p.stdout:
            msg = ln.split("]: ", 1)[-1].rstrip()
            self.lines.append(msg)
            if any(k in msg for k in ("RECALL", "LANDING", "DONE", "ABORT", "ESTOP", "armed in", "at '",
                                      "motors off")):
                print(f"    {msg}", flush=True)

    def wait_for(self, text, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if any(text in ln for ln in self.lines):
                return True
            time.sleep(0.5)
        return False

    def stop(self):
        try:
            os.killpg(self.p.pid, signal.SIGINT)
            self.p.wait(10)
        except Exception:
            os.killpg(self.p.pid, signal.SIGKILL)


def pub_empty(topic):
    subprocess.run(["ros2", "topic", "pub", "--once", "-w", "1", topic, "std_msgs/msg/Empty"],
                   capture_output=True, timeout=30)


class FakeGcs:
    """Send MAVLink GCS heartbeats to PX4 so it sees a ground station; stop() = link lost."""

    def __init__(self):
        from pymavlink import mavutil
        self.m = mavutil.mavlink_connection("udpout:127.0.0.1:18570", source_system=255)
        self.on = True
        threading.Thread(target=self._loop, daemon=True).start()

    def _loop(self):
        while self.on:
            self.m.mav.heartbeat_send(6, 8, 0, 0, 0)   # MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID
            time.sleep(0.5)

    def stop(self):
        self.on = False


def hover_then(name, action, expect, extra=(), before=None, after=None, timeout_air=240,
               final="[DONE]"):
    print(f"\n=== {name}", flush=True)
    if before:
        before()
    r = Run(name, list(extra))
    ok = r.wait_for("at 'hover'", timeout_air)
    if ok and action:
        time.sleep(3)
        action()
    ok = ok and r.wait_for(expect, 240)
    ok = ok and r.wait_for(final, 400)
    if after:
        after()
    r.stop()
    print(f"--- {name}: {'PASS' if ok else 'FAIL'}", flush=True)
    time.sleep(8)
    return ok


def main():
    only = set(sys.argv[1:])
    gcs = {}
    tests = {
        "recall": lambda: hover_then("emergency recall (/nidar/recall)",
                                     lambda: pub_empty("/nidar/recall"), "EMERGENCY RECALL (operator"),
        "abort": lambda: hover_then("mission abort (/nidar/abort)",
                                    lambda: pub_empty("/nidar/abort"), "MISSION ABORT"),
        "battery": lambda: hover_then("low battery (sim battery drains in 60 s)", None, "low battery",
                                      before=lambda: px4("param", "set", "SIM_BAT_DRAIN", "60"),
                                      after=lambda: px4("param", "set", "SIM_BAT_DRAIN", "1800")),
        "geofence": lambda: hover_then("geofence breach (fence ceiling 1.0 m, target 1.2 m)", None,
                                       "geofence breach",
                                       extra=["-p", "fence_alt_max:=1.0", "-p", "fence_margin:=-0.4"]),
        "timelimit": lambda: hover_then("mission time limit (20 s)", None, "mission time limit",
                                        extra=["-p", "mission_time_limit_s:=20.0"]),
        "commloss": lambda: hover_then("ground-station link lost", lambda: gcs["g"].stop(),
                                       "ground-station link lost",
                                       before=lambda: gcs.update(g=FakeGcs()) or time.sleep(6)),
        "estop": lambda: hover_then("emergency stop (/nidar/emergency_stop)",
                                    lambda: pub_empty("/nidar/emergency_stop"), "EMERGENCY STOP",
                                    final="motors off and disarmed"),
    }
    results = {}
    for k, fn in tests.items():
        if not only or k in only:
            results[k] = fn()
    print("\n" + json.dumps(results, indent=1))
    sys.exit(0 if all(results.values()) else 1)


if __name__ == "__main__":
    main()
