"""LeRobot teleoperator plugin adapting an SO-101 leader to a reBot B601-RS follower."""

from .config_so101_b601_leader import SO101B601LeaderConfig
from .so101_b601_leader import SO101B601Leader

__version__ = "0.2.0"

__all__ = [
    "SO101B601Leader",
    "SO101B601LeaderConfig",
    "__version__",
]
