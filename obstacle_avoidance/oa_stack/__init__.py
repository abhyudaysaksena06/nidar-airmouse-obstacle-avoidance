"""OA-Stack: obstacle avoidance for the NIDAR budget sensor stack
(RPLIDAR C1 + 2x VL53L5CX UP/DOWN + MTF-01/EKF height), as defined in Drone sensing.zip."""
from .avoider import StackAvoider, StackOutput
from .baseline_guide import GuideBaseline
from .config import StackConfig

__all__ = ["StackAvoider", "StackOutput", "StackConfig", "GuideBaseline"]
