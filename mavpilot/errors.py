"""mavpilot exception types."""


class DroneError(RuntimeError):
    """Any non-standard situation: command failure, timeout, loss-of-comm."""
