"""spdlog-backed Python diagnostics; importing this module has no side effects."""
import ctypes
import logging
import os
from pathlib import Path
import threading

_lock = threading.RLock()
_backend = None
_backend_pid = None
_loggers = {}
_LEVELS = {'debug': 10, 'info': 20, 'warn': 30, 'warning': 30,
           'error': 40, 'fatal': 50, 'critical': 50}


def _after_fork():
    global _lock
    _lock = threading.RLock()


os.register_at_fork(after_in_child=_after_fork)


def log_level():
    """Return a validated ROS-compatible level (explicit launch args still win)."""
    value = os.environ.get('ASTRIBOT_LOG_LEVEL', 'info').lower()
    if value not in _LEVELS:
        raise ValueError(f'Invalid ASTRIBOT_LOG_LEVEL: {value!r}')
    return {'warning': 'warn', 'critical': 'fatal'}.get(value, value)


def log_directory():
    value = (os.environ.get('ASTRIBOT_LOG_DIR') or os.environ.get('ROS_LOG_DIR') or
             str(Path(os.environ.get('ROS_HOME', str(Path.home() / '.ros'))) / 'log/astribot'))
    return Path(value).expanduser().resolve()


def _positive_int(name, default):
    value = int(os.environ.get(name, default))
    if value <= 0:
        raise ValueError(f'{name} must be positive')
    return value


def _native():
    global _backend, _backend_pid
    with _lock:
        if _backend is not None:
            if _backend_pid != os.getpid():
                raise RuntimeError('Do not fork after initializing spdlog; use multiprocessing spawn')
            return _backend
        from ament_index_python.packages import get_package_prefix
        path = Path(get_package_prefix('astribot_logging')) / 'lib/astribot_logging/libastribot_spdlog.so'
        try:
            backend = ctypes.CDLL(str(path))
        except OSError as exc:
            raise RuntimeError('Build astribot_logging and source ws_robot/install/setup.bash '
                               f'before using the SDK logger ({path})') from exc
        backend.astribot_log_initialize.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_size_t]
        backend.astribot_log_initialize.restype = ctypes.c_int
        backend.astribot_log_write.argtypes = [ctypes.c_char_p, ctypes.c_int, ctypes.c_char_p]
        backend.astribot_log_write.restype = ctypes.c_int
        backend.astribot_log_error.argtypes = []
        backend.astribot_log_error.restype = ctypes.c_char_p
        backend.astribot_file_open.argtypes = [ctypes.c_char_p, ctypes.c_size_t, ctypes.c_size_t]
        backend.astribot_file_open.restype = ctypes.c_int
        backend.astribot_file_write.argtypes = [ctypes.c_char_p, ctypes.c_char_p]
        backend.astribot_file_write.restype = ctypes.c_int
        if backend.astribot_log_initialize(
                os.fsencode(log_directory()),
                _positive_int('ASTRIBOT_LOG_MAX_BYTES', 10 * 1024 * 1024),
                _positive_int('ASTRIBOT_LOG_BACKUP_COUNT', 5)):
            raise RuntimeError(backend.astribot_log_error().decode('utf-8', 'replace'))
        _backend, _backend_pid = backend, os.getpid()
        return backend


class SpdlogHandler(logging.Handler):
    """Optional bridge for stdlib/vendor loggers; never configures the root logger."""

    def emit(self, record):
        backend = _native()
        message = self.format(record).replace('\0', '\\0')
        if backend.astribot_log_write(record.name.encode(), record.levelno,
                                     message.encode('utf-8', 'backslashreplace')):
            raise RuntimeError(backend.astribot_log_error().decode('utf-8', 'replace'))


def get_logger(name='astribot.sdk'):
    with _lock:
        if name not in _loggers:
            logger = logging.Logger(name, _LEVELS[log_level()])
            logger.addHandler(SpdlogHandler())
            logger.propagate = False
            _loggers[name] = logger
        return _loggers[name]


def configure_stdlib_logger(logger):
    """Explicitly redirect an owned vendor logger once, retaining its public API."""
    with _lock:
        if not any(isinstance(handler, SpdlogHandler) for handler in logger.handlers):
            for handler in logger.handlers[:]:
                logger.removeHandler(handler)
                handler.close()
            logger.addHandler(SpdlogHandler())
        logger.setLevel(_LEVELS[log_level()])
        logger.propagate = False
