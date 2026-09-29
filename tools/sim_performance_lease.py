"""Cooperative, host-wide simulation performance reservation.

Normal supervisors take shared locks. An explicitly exclusive benchmark takes
an exclusive lock. The persistent inode must never be unlinked while in use.
"""
import fcntl
from os.path import basename
from pathlib import Path

PERFORMANCE_LEASE_PATH = Path('/tmp/astribot_sim_performance.lock')


def is_simulation_command(executable, arguments):
    """Recognize launch argv and Gazebo's rewritten title, never shell text."""
    if not arguments or executable in ('bash', 'sh', 'dash', 'timeout'):
        return False
    if executable in ('gzserver', 'gzclient') or basename(arguments[0]) in ('gzserver', 'gzclient'):
        return True
    words = arguments
    # Fortress replaces argv with one space-delimited process title on Linux.
    if arguments[0].startswith(('ign gazebo ', 'gz sim ')):
        words = arguments[0].split()
    if basename(words[0]).startswith('ruby'):
        words = words[1:]
    return len(words) >= 2 and (basename(words[0]), words[1]) in (
        ('ign', 'gazebo'), ('gz', 'sim'))


def simulation_servers():
    """Also detect direct Gazebo launches that do not participate in the lock."""
    rows = []
    for process in Path('/proc').iterdir():
        if not process.name.isdigit():
            continue
        try:
            executable = (process / 'exe').resolve().name
            arguments = [x.decode() for x in (process / 'cmdline').read_bytes().split(b'\0') if x]
            if not is_simulation_command(executable, arguments):
                continue
            rows.append({'pid': int(process.name), 'command': ' '.join(arguments)})
        except (OSError, UnicodeError):
            continue
    return rows


def acquire_performance_lease(exclusive=False, path=PERFORMANCE_LEASE_PATH):
    lease = Path(path).open('a+')
    try:
        mode = fcntl.LOCK_EX if exclusive else fcntl.LOCK_SH
        fcntl.flock(lease, mode | fcntl.LOCK_NB)
    except BlockingIOError as error:
        lease.close()
        raise RuntimeError('PERFORMANCE_LEASE_BUSY: another simulation owns the host performance reservation') from error
    except BaseException:
        lease.close()
        raise
    return lease
