"""Bounded model parameter discovery/response retries, without ROS."""
import unittest
from concurrent.futures import Future
from prepare_empty_inventory import read_model_parameters


class ModelParameterReadTest(unittest.TestCase):
    def setup_client(self, complete_on=2, discovery=0.):
        self.now=0.;self.futures=[];self.removed=[];self.spin_times=[]
        outer=self
        class Client:
            srv_name='/move_group/get_parameters'
            def service_is_ready(self):return outer.now>=discovery
            def call_async(self,request):
                result=Future();outer.futures.append(result)
                if len(outer.futures)==complete_on:result.set_result('model')
                return result
            def remove_pending_request(self,future):outer.removed.append(future)
        return Client()

    def spin(self):
        self.now+=.01
        self.spin_times.append(self.now)

    def test_discovery_continues_spinning_until_service_is_ready(self):
        client=self.setup_client(complete_on=1,discovery=.5)
        def spin():
            self.assertEqual(self.futures,[])
            self.spin()
        self.assertEqual(read_model_parameters(client,object(),spin,clock=lambda:self.now),'model')
        self.assertGreaterEqual(len(self.spin_times),50)
        self.assertLessEqual(max(b-a for a,b in zip([0.]+self.spin_times,self.spin_times)),.010001)
        self.assertEqual(len(self.futures),1);self.assertLess(self.now,.52)

    def test_first_response_lost_then_second_succeeds(self):
        client=self.setup_client();events=[]
        self.assertEqual(read_model_parameters(client,object(),self.spin,clock=lambda:self.now,events=events),'model')
        self.assertEqual(len(self.futures),2);self.assertEqual(len(self.removed),1)
        self.assertTrue(self.futures[0].cancelled());self.assertLess(self.now,2.02)
        self.assertEqual([e['result']for e in events],['response_timeout','response'])

    def test_discovery_and_three_attempts_share_original_budget(self):
        client=self.setup_client(complete_on=99,discovery=1.5)
        with self.assertRaises(TimeoutError):
            read_model_parameters(client,object(),self.spin,timeout=6.,clock=lambda:self.now)
        self.assertEqual(len(self.futures),3);self.assertEqual(len(self.removed),3)
        self.assertTrue(all(f.cancelled()for f in self.futures));self.assertLess(self.now,6.02)

    def test_discovery_exhaustion_sends_no_request(self):
        for timeout,discovery in ((1.,1.),(10.,6.)):
            with self.subTest(timeout=timeout):
                client=self.setup_client(discovery=discovery)
                with self.assertRaises(RuntimeError):
                    read_model_parameters(client,object(),self.spin,timeout=timeout,clock=lambda:self.now)
                self.assertEqual(self.futures,[])
                self.assertLess(self.now,min(5.,timeout)+.02)

    def test_non_parameter_service_cannot_use_retry(self):
        client=self.setup_client();client.srv_name='/apply_planning_scene'
        with self.assertRaisesRegex(RuntimeError,'READ_ONLY'):
            read_model_parameters(client,object(),self.spin,clock=lambda:self.now)
        self.assertEqual(self.futures,[])
        self.assertEqual(self.spin_times,[])


if __name__=='__main__':unittest.main()
