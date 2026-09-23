import json
import pytest
from astribot_s1_transport.core import (TaskFailure, ResourceLease, canonical_resource_domain,
                                        check_native_resource_release)


def test_domain_aliases_cannot_bypass_shared_lock():
    assert canonical_resource_domain('20') == '20'
    for value in ('024', '094', '0x14', '-1', '233', '２０'):
        with pytest.raises(TaskFailure):
            canonical_resource_domain(value)


def test_native_unresolved_and_torn_record_deny_legacy(tmp_path):
    journal = tmp_path / 'state'
    check_native_resource_release(journal)
    for record in ('', '{}', '{bad}\n', json.dumps({'schema': 'astribot.resource/1', 'phase': 2})+'\n'):
        journal.write_text(record)
        with pytest.raises(TaskFailure):
            check_native_resource_release(journal)
    journal.write_text(json.dumps({'schema': 'astribot.resource/1', 'phase': 0})+'\n')
    check_native_resource_release(journal)


def test_legacy_marker_is_synced_before_shorter_replacement_truncates(tmp_path, monkeypatch):
    import os
    path = tmp_path / 'lock'
    with ResourceLease(path) as lease:
        lease.checkpoint({'unconfirmed_executor': True, 'old_long_field': 'x'*100})
        real_sync = os.fsync
        observed = []
        def sync(fd):
            observed.append(path.read_text())
            return real_sync(fd)
        monkeypatch.setattr(os, 'fsync', sync)
        lease.checkpoint({'unconfirmed_executor': False})
        assert observed[0]  # the previous guard is never truncated to empty
        assert len(observed[0]) > len(observed[-1])
        assert json.loads(observed[-1]) == {'unconfirmed_executor': False}


def test_interrupted_unresolved_marker_is_never_empty_or_released(tmp_path, monkeypatch):
    import os
    path = tmp_path / 'lock'
    with ResourceLease(path) as lease:
        lease.checkpoint({'unconfirmed_executor': True, 'details': 'x'*200})
        def failed_sync(fd):
            raise OSError('injected sync failure')
        monkeypatch.setattr(os, 'fsync', failed_sync)
        with pytest.raises(OSError):
            lease.checkpoint({'unconfirmed_executor': True})
        marker = path.read_text()
        assert marker
        try:
            decoded = json.loads(marker)
        except ValueError:
            pass  # malformed state denies admission in both entry points
        else:
            assert decoded['unconfirmed_executor'] is True
