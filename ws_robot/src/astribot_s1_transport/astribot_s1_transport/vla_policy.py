"""Policy SDK and bounded HTTP client; inference has no ROS/controller dependencies."""
import copy
import json
import queue
import threading
import time
from urllib.parse import urlparse
import http.client
from typing import Protocol

from .vla_contract import VERSION, UNITS, IDENTITY, fail, validate_capabilities, validate_proposal


class PolicyAdapter(Protocol):
    """Implement call(endpoint, packet) in an inference environment and serve it.

    Endpoints: capabilities, reset, infer, feedback, cancel, close. An episode owns
    temporal policy state; reset MUST discard previously cached action chunks.
    Implementations must permit cancel concurrently with infer and isolate episodes.
    """
    def call(self, endpoint: str, packet: dict) -> dict: ...


class ReferencePolicy:
    """Deterministic protocol fixture. Does not load a model or infer from images."""
    def call(self, endpoint, packet):
        if endpoint == 'capabilities':
            return dict(schema=VERSION, policy_id='reference_nominal', model_revision='1',
                        normalization_id='SI_identity', robot_model='astribot_s1',
                        units=UNITS, action_types=['mtc_targets'])
        if endpoint == 'infer':
            return dict({k: packet[k] for k in IDENTITY}, units=UNITS,
                        action=copy.deepcopy(packet['nominal_action']))
        return dict(schema=VERSION, ok=True)


class HttpPolicy:
    """POST JSON v1. No redirects/retries: request identities cannot be replayed."""
    def __init__(self, endpoint, timeout_s):
        self.url = urlparse(endpoint)
        if (self.url.scheme not in ('http', 'https') or not self.url.hostname or
                self.url.username or self.url.password or self.url.query or self.url.fragment):
            fail('HTTP_ENDPOINT')
        self.timeout = timeout_s

    def call(self, endpoint, packet):
        factory = http.client.HTTPSConnection if self.url.scheme == 'https' else http.client.HTTPConnection
        connection = factory(self.url.hostname, self.url.port, timeout=self.timeout)
        try:
            body = json.dumps(packet, allow_nan=False).encode()
            if len(body) > 32 * 1024 * 1024:
                fail('HTTP_REQUEST_TOO_LARGE')
            connection.request('POST', self.url.path.rstrip('/') + '/' + endpoint, body,
                               {'Content-Type': 'application/json'})
            response = connection.getresponse()
            if response.status != 200:
                fail('HTTP_STATUS_' + str(response.status))
            data = response.read(2 * 1024 * 1024 + 1)
            if len(data) > 2 * 1024 * 1024:
                fail('HTTP_RESPONSE_TOO_LARGE')
            return json.loads(data)
        finally:
            connection.close()


class PolicySession:
    def __init__(self, config, episode_id, record, adapter=None):
        self.config, self.episode_id, self.record = config, episode_id, record
        self.adapter = adapter or (ReferencePolicy() if config['adapter'] == 'reference'
                                  else HttpPolicy(config['endpoint'], config['timeout_s']))
        self.capabilities = None
        self.ready = False
        self.used = set()
        self.sequence = 0
        self.closed = False
        self.inference_lock = threading.Lock()
        self.notifications = queue.Queue(maxsize=128)
        def dispatch():
            while True:
                endpoint, values = self.notifications.get()
                self.notify(endpoint, values)
                if endpoint == 'close':
                    return
        self.notifier = threading.Thread(target=dispatch, daemon=True)
        self.notifier.start()

    def packet(self, **values):
        return dict(schema=VERSION, episode_id=self.episode_id, **values)

    def invoke(self, endpoint, packet, check):
        if self.closed:
            fail('SESSION_CLOSED')
        result = queue.Queue(maxsize=1)
        def work():
            try: result.put((True, copy.deepcopy(self.adapter.call(endpoint, copy.deepcopy(packet)))))
            except Exception as error: result.put((False, error))
        threading.Thread(target=work, daemon=True).start()
        deadline = time.monotonic() + self.config['timeout_s']
        try:
            while True:
                check()
                if time.monotonic() >= deadline:
                    fail('INFERENCE_TIMEOUT:' + endpoint)
                try: ok, value = result.get(timeout=.02)
                except queue.Empty: continue
                check()
                if time.monotonic() >= deadline:
                    fail('INFERENCE_TIMEOUT:' + endpoint)
                if not ok:
                    fail('ADAPTER_ERROR:' + str(value))
                return value
        except Exception:
            # A late worker only writes to its private queue; it can never issue
            # robot actions or be reused by a new session. Remote cancellation is
            # best effort, since network/model interruption cannot be guaranteed.
            self.closed = True
            self.ready = False
            self.enqueue('cancel', dict(request_id=packet.get('request_id'), reason='request_aborted'))
            raise

    def start(self, check):
        if self.capabilities is not None:
            fail('SESSION_ALREADY_STARTED')
        self.capabilities = self.invoke('capabilities', self.packet(), check)
        validate_capabilities(self.capabilities, self.config['mode'])
        response = self.invoke('reset', self.packet(), check)
        if not isinstance(response, dict) or response.get('schema') != VERSION or response.get('ok') is not True:
            fail('RESET_NOT_ACKNOWLEDGED')
        self.record('session', dict(config=self.config, capabilities=self.capabilities))
        self.ready = True

    def infer(self, request, check):
        if not self.inference_lock.acquire(blocking=False):
            fail('INFERENCE_BUSY')
        try:
            return self._infer(request, check)
        finally:
            self.inference_lock.release()

    def _infer(self, request, check):
        if not self.ready or self.closed:
            fail('SESSION_NOT_READY')
        if (request.get('schema') != VERSION or request.get('episode_id') != self.episode_id or
                type(request.get('sequence')) is not int or not isinstance(request.get('request_id'), str) or
                not request['request_id']):
            fail('REQUEST_IDENTITY')
        if request['request_id'] in self.used or request['sequence'] != self.sequence:
            fail('REQUEST_REPLAY_OR_SEQUENCE')
        self.used.add(request['request_id'])
        self.sequence += 1
        self.record('request', request)
        started = time.monotonic()
        response = self.invoke('infer', request, check)
        self.record('response', dict(response=response, latency_s=time.monotonic()-started))
        return validate_proposal(request, response, self.capabilities, self.config)

    def notify(self, endpoint, values):
        # Feedback is telemetry. A failed notification must not prevent holding
        # a payload or alter the outcome of a completed physical transaction.
        try:
            self.adapter.call(endpoint, self.packet(**values))
        except Exception as error:
            self.record('notification_error', dict(endpoint=endpoint, reason=str(error)))

    def enqueue(self, endpoint, values):
        try:
            self.notifications.put_nowait((endpoint, values))
        except queue.Full:
            self.record('notification_error', dict(endpoint=endpoint, reason='QUEUE_FULL'))

    def close(self, values):
        self.closed = True
        self.ready = False
        self.enqueue('close', values)
        self.notifier.join(timeout=.3)
