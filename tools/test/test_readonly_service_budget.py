import sys
from pathlib import Path
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from run_waypoint_route import read_service


class Future:
    def __init__(self, done):
        self.ready = done
        self.cancelled = False
    def done(self):
        return self.ready
    def result(self):
        return 'value'
    def cancel(self):
        self.cancelled = True


class Client:
    srv_name = '/fixture/get_parameters'
    def __init__(self, success):
        self.success, self.calls, self.removed = success, [], []
    def call_async(self, request):
        future = Future(len(self.calls)+1 == self.success)
        self.calls.append(future)
        return future
    def remove_pending_request(self, future):
        self.removed.append(future)


def test_transient_first_read_retries_and_removes_pending_request():
    now = [0.]
    client, events = Client(2), []
    def spin(): now[0] += .1
    assert read_service(client, object(), spin, clock=lambda: now[0], events=events) == 'value'
    assert len(client.calls) == 2
    assert client.calls[0].cancelled
    assert client.removed == [client.calls[0]]
    assert [e['result'] for e in events] == ['response_timeout', 'response']


def test_retry_does_not_extend_original_deadline():
    now = [0.]
    client = Client(None)
    def spin(): now[0] += .125
    with pytest.raises(TimeoutError):
        read_service(client, object(), spin, timeout=5., clock=lambda: now[0])
    assert now[0] == 5.
    assert len(client.calls) == 3
    assert all(f.cancelled for f in client.calls)


def test_interrupt_cleans_up_and_does_not_retry():
    client = Client(None)
    def spin(): raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        read_service(client, object(), spin)
    assert len(client.calls) == 1
    assert client.calls[0].cancelled
