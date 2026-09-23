import copy
import math
import unittest
from analyze_constraint_deadlines import audit_proposed, audit_final, lease_within_deadline


def rows():
    wire=dict(epoch=7,sequence=3,stamp_ns=1000000000,lease_s=.1,hold=False)
    gate=dict(allowed=True,deadline_ns=1200000000,publication_ns=1000000000,
              decision_ns=980000000,decision_epoch=0,publication_epoch=0,
              constraint_epoch=7,constraint_sequence=3,constraint_stamp_ns=1000000000,
              constraint_lease_s=.1,constraint_hold=False,
              required_sources=[dict(sensor_id='scan',capture_ns=900000000,deadline_ns=1200000000)])
    return [dict(source='proposed_constraint_wire',data=wire),
            dict(source='state',data=dict(publication_gate=gate,coverage_health=[dict(sensor_id='scan',required=True,capture_ns=900000000,valid_until_ns=1200000000)],scan_timing_constraint=dict(publish_started=dict(ros_ns=1001000000))))]


class ConstraintDeadlines(unittest.TestCase):
    def final_rows(self):
        r=rows()
        r.extend([dict(source='final_constraint_wire',data=dict(epoch=2,sequence=4,stamp_ns=1010000000,lease_s=.0625,hold=False)),
                  dict(source='protection',data=dict(constraint_epoch=2,constraint_sequence=4,proposal_epoch=7,proposal_sequence=3,
                      proposal_stamp_ns=1000000000,proposal_deadline_ns=1100000000,publication_stamp_ns=1010000000,
                      effective_lease_s=.0625,hold=False,publication_inputs_fresh=True,publication_proposal_fresh=True,
                      scan_capture_ns=900000000,scan_deadline_ns=1200000000,odom_capture_ns=990000000,odom_deadline_ns=1290000000,
                      effective_deadline_ns=1100000000,output_publish_start_ns=1011000000,constraint_publish_start_ns=1012000000))])
        return r

    def test_final_good_pair(self):
        self.assertTrue(audit_final(self.final_rows())['passed'])

    def test_conservative_upstream_nanosecond_is_valid(self):
        r=self.final_rows();r[-1]['data']['proposal_deadline_ns']-=1;r[-1]['data']['effective_deadline_ns']-=1
        self.assertTrue(audit_final(r)['passed'])

    def test_reverse_positive_pairing_required(self):
        r=rows();extra=copy.deepcopy(r[-1]);extra['data']['publication_gate']['constraint_sequence']+=1;r.append(extra)
        self.assertFalse(audit_proposed(r)['passed'])
        r=self.final_rows();extra=copy.deepcopy(r[-1]);extra['data']['constraint_sequence']+=1;r.append(extra)
        self.assertFalse(audit_final(r)['passed'])

    def test_original_health_deadline_and_required_set(self):
        for change in ('deadline','missing_source','missing_health'):
            r=rows();d=r[1]['data']
            if change=='deadline':d['coverage_health'][0]['valid_until_ns']=1050000000
            if change=='missing_source':d['coverage_health'].append(dict(sensor_id='depth',required=True,capture_ns=900000000,valid_until_ns=1050000000))
            if change=='missing_health':del d['coverage_health']
            with self.subTest(change=change):self.assertFalse(audit_proposed(r)['passed'])

    def test_internal_sequence_gap_is_incomplete(self):
        r=rows();next_pair=copy.deepcopy(r);next_pair[0]['data']['sequence']+=2;next_pair[1]['data']['publication_gate']['constraint_sequence']+=2
        self.assertIn('WIRE_SEQUENCE_GAPS',audit_proposed(r+next_pair)['error_counts'])

    def test_final_renewal_and_late_publication_fail(self):
        for field,value in [('effective_lease_s',.125),('output_publish_start_ns',1100000000),('constraint_publish_start_ns',1100000000),('scan_deadline_ns',1050000000),('publication_inputs_fresh',False),('proposal_deadline_ns',1200000000)]:
            r=self.final_rows();r[-1]['data'][field]=value
            if field=='effective_lease_s':r[-2]['data']['lease_s']=value
            with self.subTest(field=field):self.assertFalse(audit_final(r)['passed'])

    def test_final_missing_pair_and_hold_only_fail(self):
        self.assertFalse(audit_final(self.final_rows()[1:])['passed'])
        self.assertFalse(audit_final(self.final_rows()[:-1])['passed'])
        r=self.final_rows();r[-2]['data']['hold']=True
        self.assertFalse(audit_final(r)['passed'])
    def test_good_pair(self):
        r=audit_proposed(rows());self.assertTrue(r['passed']);self.assertEqual(r['positive_pairs'],1)

    def test_lease_exact_float_boundaries(self):
        self.assertTrue(lease_within_deadline(100,.125,125000100))
        self.assertFalse(lease_within_deadline(100,math.nextafter(.125,math.inf),125000100))
        for lease in (0,-1,math.nan,math.inf):self.assertFalse(lease_within_deadline(100,lease,200))

    def test_empty_and_unpaired_are_not_pass(self):
        self.assertFalse(audit_proposed([])['passed'])
        self.assertFalse(audit_proposed(rows()[:1])['passed'])

    def test_republished_lease_cannot_extend_source_deadline(self):
        r=rows();r[0]['data']['lease_s']=.3;r[1]['data']['publication_gate']['constraint_lease_s']=.3
        self.assertFalse(audit_proposed(r)['passed'])

    def test_actual_publication_crossing_and_future_capture_fail(self):
        for change in ('publish','capture','epoch'):
            r=rows()
            if change=='publish':r[1]['data']['scan_timing_constraint']['publish_started']['ros_ns']=1200000000
            if change=='capture':r[1]['data']['publication_gate']['required_sources'][0]['capture_ns']=1000000001
            if change=='epoch':r[1]['data']['publication_gate']['publication_epoch']=1
            with self.subTest(change=change):self.assertFalse(audit_proposed(r)['passed'])

    def test_hold_is_not_positive_evidence(self):
        r=rows();r[0]['data']['hold']=True;r[1]['data']['publication_gate']['constraint_hold']=True
        self.assertFalse(audit_proposed(r)['passed'])

    def test_conflicting_duplicate_sequence_fails(self):
        r=rows();bad=copy.deepcopy(r[0]);bad['data']['stamp_ns']+=1;r.append(bad)
        self.assertFalse(audit_proposed(r)['passed'])

if __name__=='__main__':unittest.main()
