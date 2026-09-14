"""Compatibility facade for the shared spdlog backend (no ROS context required)."""


class DefaultLogger:
    def __init__(self, name='astribot.sdk'):
        self.name = name

    def __getattr__(self, method):
        # Lazy import keeps SDK module discovery possible before overlay setup.
        from astribot_logging import get_logger
        return getattr(get_logger(self.name), method)
