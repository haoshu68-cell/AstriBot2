"""Bounded cleanup of an operation's own new-session process group."""
import os
from pathlib import Path
import select
import signal
import subprocess
import time


class OwnedCommand:
    def __init__(self, command, **kwargs):
        self.process = subprocess.Popen(command, start_new_session=True, **kwargs)
        self.handles = {}
        self.observe()

    def observe(self):
        # The root PID remains reserved until poll/wait reaps it. Keep captured
        # pidfds after that point so a surviving child cannot escape cleanup.
        table = {}
        for entry in Path('/proc').iterdir():
            if not entry.name.isdigit():
                continue
            try:
                table[int(entry.name)] = (entry/'stat').read_text().rsplit(')', 1)[1].split()
            except (FileNotFoundError, ProcessLookupError):
                continue
        owned = {pid for pid, fd in self.handles.items()
                 if not select.select([fd], [], [], 0)[0]}
        if self.process.returncode is None:
            owned.add(self.process.pid)
        while True:
            expanded = owned | {pid for pid, fields in table.items()
                if int(fields[1]) in owned or
                (self.process.returncode is None and int(fields[2]) == self.process.pid)}
            if expanded == owned:
                break
            owned = expanded
        for pid in owned - self.handles.keys():
            fd = None
            try:
                fd = os.pidfd_open(pid)
                current = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
                if current[19] == table[pid][19]:
                    self.handles[pid] = fd
                    fd = None
            except (FileNotFoundError, ProcessLookupError, KeyError):
                pass
            finally:
                if fd is not None:
                    os.close(fd)

    def live(self):
        return [fd for fd in self.handles.values() if not select.select([fd], [], [], 0)[0]]

    def poll(self):
        if self.process.returncode is not None:
            return self.process.returncode
        root = self.handles[self.process.pid]
        if not select.select([root], [], [], 0)[0]:
            return None
        # Exited but not yet reaped: the root PID still reserves its process
        # group identity while we capture children created just before exit.
        self.observe()
        return self.process.poll()

    def wait(self, timeout):
        deadline = time.monotonic() + timeout
        while True:
            self.observe()
            result = self.poll()
            if result is not None:
                return result
            if time.monotonic() >= deadline:
                raise subprocess.TimeoutExpired(self.process.args, timeout)
            time.sleep(.05)

    def stop(self, grace=6):
        self.observe()
        try:
            for sig, delay in ((signal.SIGINT, grace), (signal.SIGTERM, 2), (signal.SIGKILL, 1)):
                self.observe()
                for fd in self.live():
                    try:
                        signal.pidfd_send_signal(fd, sig)
                    except ProcessLookupError:
                        pass
                deadline = time.monotonic() + delay
                while time.monotonic() < deadline:
                    self.observe()
                    if not self.live():
                        break
                    time.sleep(.05)
                self.observe()
                if not self.live():
                    break
            if self.live():
                raise RuntimeError('Owned command children did not exit')
            self.process.wait(timeout=1)
        finally:
            for fd in self.handles.values():
                os.close(fd)
            self.handles.clear()


def run_owned_capture(command, log_path, *, env, timeout):
    with log_path.open('w') as log:
        owned = OwnedCommand(command, env=env, stdout=log, stderr=subprocess.STDOUT)
        try:
            return owned.wait(timeout)
        finally:
            owned.stop(grace=15)
