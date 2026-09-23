import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import pytest

from capture_deferred_views import validate_environment
from owned_command import OwnedCommand, run_owned_capture


def test_default_nonisolated_environment():
    validate_environment({'ROS_DOMAIN_ID': '25'}, {'ROS_DOMAIN_ID': '25'})


def test_unexpected_partition_rejected():
    with pytest.raises(RuntimeError, match='IGN_PARTITION'):
        validate_environment({'ROS_DOMAIN_ID': '25', 'IGN_PARTITION': 'other'},
                             {'ROS_DOMAIN_ID': '25'})


def test_child_surviving_wrapper_is_reclaimed(tmp_path):
    marker = tmp_path/'child'
    code = ('import subprocess,sys,time; from pathlib import Path; '
            'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"]); '
            f'Path({str(marker)!r}).write_text(str(p.pid)); time.sleep(60)')
    owned = OwnedCommand([sys.executable, '-c', code], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 3
        while not marker.exists() and time.monotonic() < deadline:
            owned.observe()
            time.sleep(.02)
        child = int(marker.read_text())
        owned.observe()
        fd = os.pidfd_open(child)
        try:
            owned.process.terminate()
            owned.process.wait(timeout=2)
            owned.stop(grace=.2)
            import select
            assert select.select([fd], [], [], 1)[0]
        finally:
            try:
                signal.pidfd_send_signal(fd, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.close(fd)
    finally:
        if owned.handles:
            owned.stop(grace=.2)


def test_timeout_allows_helper_finally(tmp_path):
    marker = tmp_path/'cleaned'
    code = ('import signal,time\nfrom pathlib import Path\n'
            'def interrupt(s,f): raise KeyboardInterrupt\n'
            'signal.signal(signal.SIGINT, interrupt)\n'
            'try: time.sleep(60)\n'
            f'finally: Path({str(marker)!r}).write_text("cleaned")\n')
    with pytest.raises(subprocess.TimeoutExpired):
        run_owned_capture([sys.executable, '-c', code], tmp_path/'log',
                          env=os.environ.copy(), timeout=.5)
    assert marker.read_text() == 'cleaned'


def test_shutdown_handler_child_is_reclaimed(tmp_path):
    marker = tmp_path/'child'
    ready = tmp_path/'ready'
    code = ('import subprocess,sys,signal,time\nfrom pathlib import Path\n'
            'def interrupt(s,f):\n'
            ' p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"])\n'
            f' Path({str(marker)!r}).write_text(str(p.pid))\n'
            ' sys.exit(0)\n'
            'signal.signal(signal.SIGINT, interrupt)\n'
            f'Path({str(ready)!r}).touch()\n'
            'time.sleep(60)\n')
    owned = OwnedCommand([sys.executable, '-c', code], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    try:
        deadline = time.monotonic() + 3
        while not ready.exists() and time.monotonic() < deadline:
            time.sleep(.02)
        assert ready.exists()
        owned.stop(grace=.2)
        child = int(marker.read_text())
        proc = Path(f'/proc/{child}/stat')
        assert not proc.exists() or proc.read_text().rsplit(')', 1)[1].split()[0] == 'Z'
    finally:
        if owned.handles:
            owned.stop(grace=.2)


def test_fork_between_observation_and_parent_exit_is_reclaimed(tmp_path):
    gate, marker = tmp_path/'gate', tmp_path/'child'
    code = ('import subprocess,sys,time\nfrom pathlib import Path\n'
            f'while not Path({str(gate)!r}).exists(): time.sleep(.001)\n'
            'p=subprocess.Popen([sys.executable,"-c","import time; time.sleep(60)"])\n'
            f'Path({str(marker)!r}).write_text(str(p.pid))\n')
    owned = OwnedCommand([sys.executable, '-c', code], stdout=subprocess.DEVNULL,
                         stderr=subprocess.DEVNULL)
    observe = owned.observe
    def release_after_scan():
        observe()
        if not gate.exists():
            gate.touch()
            deadline = time.monotonic() + 3
            while not marker.exists() and time.monotonic() < deadline:
                time.sleep(.01)
            time.sleep(.1)
    owned.observe = release_after_scan
    fd = None
    try:
        assert owned.wait(3) == 0
        fd = os.pidfd_open(int(marker.read_text()))
        owned.stop(grace=.2)
        import select
        assert select.select([fd], [], [], 1)[0]
    finally:
        if fd is not None:
            try:
                signal.pidfd_send_signal(fd, signal.SIGKILL)
            except ProcessLookupError:
                pass
            os.close(fd)
        if owned.handles:
            owned.stop(grace=.2)
