"""File-only spdlog sinks for launch and captured process output."""
import codecs
from datetime import datetime, timezone
import os
import logging
from pathlib import Path
import threading

from . import _native, _positive_int


class FileLog:
    def __init__(self, path):
        self.path = Path(path).resolve()
        self.backend = _native()
        self.encoded_path = str(self.path).encode()
        self._check(self.backend.astribot_file_open(
            self.encoded_path, _positive_int('ASTRIBOT_LOG_MAX_BYTES', 10485760),
            _positive_int('ASTRIBOT_LOG_BACKUP_COUNT', 5)))

    def _check(self, result):
        if result:
            raise RuntimeError(self.backend.astribot_log_error().decode('utf-8', 'replace'))

    def write(self, message):
        self._check(self.backend.astribot_file_write(
            self.encoded_path, message.replace('\0', '\\0').encode('utf-8', 'backslashreplace')))


class SessionLog(FileLog):
    """One process owns the file; sources share its native, mutex-protected sink."""
    def __init__(self, path, source):
        super().__init__(path)
        self.source = source.replace('\n', ' ').replace('\r', ' ')

    def write(self, message):
        stamp = datetime.now(timezone.utc).isoformat(timespec='milliseconds')
        for line in str(message).splitlines():
            super().write(f'[{stamp}] [{self.source}] {line}')


class SessionHandler(logging.Handler):
    def __init__(self, path, source='supervisor'):
        super().__init__()
        self.file = SessionLog(path, source)
        self.setFormatter(logging.Formatter('[%(levelname)s] [%(name)s] %(message)s'))

    def emit(self, record):
        self.file.write(self.format(record))


class ConsoleOnlyLaunchHandler(logging.NullHandler):
    """launch formatting API; disk persistence belongs to the parent collector."""
    def __init__(self, filename, **kwargs):
        super().__init__()

    def setFormatterFor(self, logger, formatter):
        pass

    def unsetFormatterFor(self, logger):
        pass


class LaunchFileHandler(logging.Handler):
    """Implements launch's public per-logger-formatting handler contract."""
    def __init__(self, filename, **kwargs):
        super().__init__()
        self.file = FileLog(filename)
        self._formatters = {}

    def setFormatterFor(self, logger, formatter):
        self._formatters[logger if isinstance(logger, str) else logger.name] = formatter

    def unsetFormatterFor(self, logger):
        self._formatters.pop(logger if isinstance(logger, str) else logger.name, None)

    def emit(self, record):
        formatter = self._formatters.get(record.name, self.formatter)
        message = formatter.format(record) if formatter else record.getMessage()
        self.file.write(message.rstrip('\n'))


class ProcessOutput:
    """Drain a child's merged stdout/stderr continuously, with bounded line buffers."""
    def __init__(self, stream, path, source=None):
        self.file = SessionLog(path, source) if source is not None else FileLog(path)
        self.stream = stream
        self.error = None
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def _drain(self):
        decoder = codecs.getincrementaldecoder('utf-8')(errors='replace')
        try:
            while True:
                data = self.stream.readline(65536)
                if not data:
                    tail = decoder.decode(b'', final=True)
                    if tail:
                        self.file.write(tail)
                    break
                self.file.write(decoder.decode(data).rstrip('\n'))
        except Exception as exc:
            self.error = exc
            # Keep draining to avoid freezing a child on a full pipe after a disk error.
            while self.stream.read(65536):
                pass
        finally:
            self.stream.close()

    def check(self):
        if self.error:
            raise RuntimeError(f'Failed to capture {self.file.path}: {self.error}') from self.error

    def close(self):
        self.thread.join(timeout=5)
        if self.thread.is_alive():
            raise RuntimeError(f'Output pipe did not close: {self.file.path}')
        self.check()


def configure_launch_logging():
    """Use the public launch handler factory, including already-created handlers."""
    import os
    from launch.logging import launch_config

    # Includes ExecuteProcess in third-party launch files (Gazebo), not just ROS Node.
    captured = os.environ.get('ASTRIBOT_LOG_CAPTURE') == '1'
    os.environ['OVERRIDE_LAUNCH_PROCESS_OUTPUT'] = 'screen' if captured else 'both'
    factory = ConsoleOnlyLaunchHandler if captured else LaunchFileHandler
    if launch_config.log_handler_factory is factory:
        return
    launch_config.log_handler_factory = factory
    for filename, previous in list(launch_config.file_handlers.items()):
        replacement = factory(launch_config.get_log_file_path(filename))
        replacement.setFormatter(previous.formatter)
        if hasattr(replacement, '_formatters'):
            replacement._formatters.update(getattr(previous, '_formatters', {}))
        known = [logging.root, *logging.Logger.manager.loggerDict.values()]
        for logger in known:
            if isinstance(logger, logging.Logger) and previous in logger.handlers:
                logger.removeHandler(previous)
                logger.addHandler(replacement)
        launch_config.file_handlers[filename] = replacement
        previous.close()


def publish_session_link(root, session):
    """Index custom session directories without moving logs or changing their owner."""
    import tempfile

    root, session = Path(root).expanduser().resolve(), Path(session).resolve()
    if not (session / 'session.log').is_file():
        raise FileNotFoundError(f'Session log does not exist: {session / "session.log"}')
    root.mkdir(parents=True, exist_ok=True)
    destination = root / 'latest_sim'
    if destination.exists() and not destination.is_symlink():
        raise FileExistsError(f'Refusing to replace non-symlink: {destination}')
    with tempfile.TemporaryDirectory(prefix='.session-link-', dir=root) as temporary:
        link = Path(temporary) / 'latest_sim'
        link.symlink_to(session, target_is_directory=True)
        os.replace(link, destination)
    return destination
