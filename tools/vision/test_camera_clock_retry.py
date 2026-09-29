"""Regression for response state sampled after a future-clock rejection."""
import unittest
from camera_clock_retry import retry_clock_rejection
class ClockRetry(unittest.TestCase):
    def test_reason_survives_response_clock_catching_up(self):
        for server in (99,100,101):
            self.assertTrue(retry_clock_rejection('REQUEST_CLOCK_AHEAD',100,server,1.,1.2))
    def test_wall_budget_is_not_extended(self):
        self.assertFalse(retry_clock_rejection('REQUEST_CLOCK_AHEAD',100,99,1.2,1.2))
    def test_other_failures_are_final(self):
        for reason in ('OWNER_MISMATCH','TOKEN_MISMATCH','ACTIVE','REQUEST_EXPIRED'):
            self.assertFalse(retry_clock_rejection(reason,100,101,1.,1.2))
    def test_legacy_expiry_only_for_bounded_future_skew(self):
        self.assertTrue(retry_clock_rejection('REQUEST_EXPIRED',100,99,1.,1.2))
        self.assertFalse(retry_clock_rejection('REQUEST_EXPIRED',30000000,1,1.,1.2))

class RequestExecution(unittest.TestCase):
    def test_all_operations_keep_same_request_on_clock_rejection(self):
        from camera_clock_retry import call_with_clock_retry
        from types import SimpleNamespace as S
        for operation in (0, 1, 2):
            request=S(operation=operation,header=S(stamp=S(sec=1,nanosec=0)))
            calls=[];clock=[0.0]
            class Future:
                def __init__(self, response):self.response=response
                def done(self):return True
                def result(self):return self.response
            class Client:
                def call_async(self, value):
                    calls.append(value)
                    return Future(S(accepted=len(calls)>1,reason_code='REQUEST_CLOCK_AHEAD' if len(calls)==1 else 'ACTIVE',state=S(header=S(stamp=S(sec=1,nanosec=0)))))
            result=call_with_clock_retry(Client(),request,lambda dt:clock.__setitem__(0,clock[0]+dt),steady_now=lambda:clock[0])
            self.assertTrue(result.accepted)
            self.assertEqual(len(calls),2)
            self.assertTrue(all(value is request for value in calls))
    def test_non_clock_failure_is_not_retried(self):
        from camera_clock_retry import call_with_clock_retry
        from types import SimpleNamespace as S
        calls=[];r=S(accepted=False,reason_code='CAMERA_BUSY',state=S(header=S(stamp=S(sec=1,nanosec=0))))
        f=S(done=lambda:True,result=lambda:r)
        client=S(call_async=lambda req:(calls.append(req) or f))
        result=call_with_clock_retry(client,S(header=S(stamp=S(sec=1,nanosec=0))),lambda dt:None)
        self.assertIs(result,r);self.assertEqual(len(calls),1)

    def test_delay_must_not_dispatch_retry_past_original_budget(self):
        from camera_clock_retry import call_with_clock_retry
        from types import SimpleNamespace as S
        clock=[0.0];calls=[]
        response=S(accepted=False,reason_code='REQUEST_CLOCK_AHEAD',state=S(header=S(stamp=S(sec=1,nanosec=0))))
        def call(req):
            calls.append(req);clock[0]=0.19 if len(calls)==1 else clock[0]
            return S(done=lambda:True,result=lambda:response)
        call_with_clock_retry(S(call_async=call),S(header=S(stamp=S(sec=1,nanosec=0))),
            lambda dt:clock.__setitem__(0,clock[0]+dt),steady_now=lambda:clock[0])
        self.assertEqual(len(calls),1)

if __name__=='__main__':unittest.main()
