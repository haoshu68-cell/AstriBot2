"""Validation-client retry decision; never changes the original request or lease."""
def retry_clock_rejection(reason, request_stamp_ns, response_stamp_ns, steady_now, deadline):
    if steady_now >= deadline:
        return False
    # The response state is stamped after admission evaluation; /clock can advance
    # in between. An equal/newer response stamp does not negate the explicit reason.
    if reason == 'REQUEST_CLOCK_AHEAD':
        return True
    return reason == 'REQUEST_EXPIRED' and 0 < request_stamp_ns-response_stamp_ns <= 20_000_000


def call_with_clock_retry(client, request, pump, *, steady_now=None,
                          retry_budget=0.2, service_timeout=1.0, on_retry=None):
    """Bounded validation call; resend the identical original request only on clock rejection."""
    import time
    steady_now = steady_now or time.monotonic
    deadline = steady_now() + retry_budget
    while True:
        future = client.call_async(request)
        wait_until = steady_now() + service_timeout
        while not future.done() and steady_now() < wait_until:
            pump(0.01)
        if not future.done():
            raise TimeoutError('Camera session service')
        result = future.result()
        req_stamp = request.header.stamp.sec*10**9 + request.header.stamp.nanosec
        res_stamp = result.state.header.stamp.sec*10**9 + result.state.header.stamp.nanosec
        if result.accepted or not retry_clock_rejection(
                result.reason_code, req_stamp, res_stamp, steady_now(), deadline):
            return result
        if on_retry:
            on_retry(req_stamp, res_stamp)
        pump(0.02)
        if steady_now() >= deadline:
            return result
