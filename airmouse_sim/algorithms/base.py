"""Interface every AirMouse algorithm implements.

All poses are in the START frame: origin = take-off point (outside the
arena, on the ground), +x = facing the entry. There is NO GPS and no ground
truth - only these sensors:

obs = {
  "t", "dt", "time_left": seconds,
  "lidar": None while the lidar is down, else
           {"angles": np.array body frame rad (CCW +), "ranges": np.array m
            (np.inf = no return / dropout), "max_range": m,
            "new": True on the step a fresh scan arrived (rate-limited)},
  "odom":  {"x", "y", "yaw"  (EKF-style integrated, drifts),
            "vx", "vy" body m/s from optical flow (None when flow is invalid),
            "yaw_rate" rad/s gyro, "flow_quality" 0..1},
  "tof":   {"down", "up", "fwd_down", "fwd_up"}  m (inf = out of range);
           fwd_* beams are pitched +-tof_fwd_pitch to catch low/hanging obstacles,
  "baro_z": m (noisy, drifting absolute altitude),
  "landed": bool,
  "detections": None while the camera is down, [] if no new frame, else
           [{"cls": "person", "bbox": [u, v, w, h] px, "confidence": 0..1,
             "depth": m (only if camera_type == "depth")}],
  "battery": {"frac": 0..1, "power_w", "energy_wh"},
  "link": bool (GCS radio link up),
  "health": {"lidar": bool, "camera": bool, "flow_quality": 0..1},
}

step() returns {"vx": m/s fwd, "vy": m/s left, "vz": m/s up,
                "yaw_rate": rad/s CCW, "state": "EXPLORE" (shown on the GCS),
                "done": True when landed back at the exit}.
"""
import math


class BaseAlgorithm:
    def reset(self, info):
        """info: dt, time_limit, drone_radius, half_height, cruise_alt,
        max_speed, max_vz, max_yaw_rate, lidar_angles, lidar_max_range,
        lidar_rate_hz, tof_fwd_pitch, tof_max_range, camera {type, width,
        height, f_px, hfov, vfov, pitch, range_hint}, battery_capacity_wh,
        start_outside (m to the entry), exit_radius."""
        self.info = info

    def step(self, obs):
        return {"vx": 0.0, "vy": 0.0, "vz": 0.0, "yaw_rate": 0.0, "done": False}

    def get_survivors(self):
        """[(x, y), ...] survivor positions in the START frame (scored)."""
        return []

    def get_map(self):
        """Optional: {"grid": 2D array of P(occupied) 0..1 (0.5 unknown),
        "res": m/cell, "origin": (x, y) of grid[0,0] in START frame}.
        Sent to the GCS over the radio link."""
        return None

    def get_pose(self):
        """Optional: your own (x, y, yaw) estimate in the START frame -
        drawn on the GCS and scored as localisation error."""
        return None

    def debug_text(self):
        """Optional: short status string shown in the side panel."""
        return ""

    # ---- helpers ----------------------------------------------------------
    @staticmethod
    def altitude_hold(obs, target, kp=1.2, vmax=0.5):
        """Simple vz command to hold height above the floor using the
        downward ToF (falls back to the barometer when out of range)."""
        h = obs["tof"]["down"]
        if not math.isfinite(h):
            h = obs["baro_z"]
        return max(-vmax, min(vmax, kp * (target - h)))

    @staticmethod
    def bbox_ground_range(bbox, cam, height):
        """Mono camera: distance along the floor to a person from the bottom
        of their bounding box (flat-floor assumption) and their bearing."""
        u, v, w, h = bbox
        f = cam["f_px"]
        vb = v + h * 0.7  # a little above the box bottom ~ body centre
        depression = cam["pitch"] + math.atan2(vb - cam["height"] / 2, f)
        bearing = -math.atan2(u + w / 2 - cam["width"] / 2, f)
        if depression <= math.radians(2):
            return None, bearing
        return max(0.2, (height - 0.15) / math.tan(depression)), bearing
