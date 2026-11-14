"""NIDAR AirMouse local obstacle avoidance: VFH+ direction selection +
Control-Barrier-Function safety filter + moving-obstacle tracking +
vertical ToF guard + failsafes."""
from .avoider import ObstacleAvoider, AvoidanceOutput
from .config import AvoidanceConfig

__all__ = ["ObstacleAvoider", "AvoidanceOutput", "AvoidanceConfig"]
