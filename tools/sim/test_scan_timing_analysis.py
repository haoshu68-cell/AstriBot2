"""Offline timing evidence checks; no ROS graph is initialized."""
import unittest
from analyze_scan_timing import summarize


def record(sequence=1, epoch=0):
    return dict(sequence=sequence, clock_epoch=epoch, capture_ros_ns=1_000_000_000,
                receive=dict(ros_ns=1_040_000_000, steady_ns=2_000_000_000),
                first_tf_ready=dict(ros_ns=1_090_000_000, steady_ns=2_050_000_000),
                selected=dict(ros_ns=1_100_000_000, steady_ns=2_060_000_000),
                finished=dict(ros_ns=1_180_000_000, steady_ns=2_140_000_000),
                processing_success=True)


class AnalysisTest(unittest.TestCase):
    def test_repeated_snapshots_do_not_count_as_new_scans(self):
        row=dict(source='observation', data=dict(scan_timing=dict(clock_epoch=0, successful=record(),
                 observation_pre_serialize=dict(ros_ns=1_250_000_000, steady_ns=2_210_000_000))))
        result=summarize([row,row])
        self.assertEqual(result['unique_successful_scans'],1)
        self.assertEqual(result['stages_ms']['receive_to_select']['p50'],60.)
        self.assertEqual(result['stages_ms']['select_to_finish']['p50'],80.)
        self.assertEqual(result['observation_publish_age_ms']['n'],2)
        self.assertEqual(result['observation_publish_age_ms']['p50'],250.)

    def test_clock_domains_are_not_subtracted_and_stale_is_visible(self):
        r=record();r['receive']['steady_ns']=10_000_000_000
        r['selected']['steady_ns']=10_060_000_000;r['first_tf_ready']['steady_ns']=10_050_000_000
        r['finished']['steady_ns']=10_140_000_000
        row=dict(source='state',data=dict(scan_timing_constraint=dict(successful=r,
                 publish_started=dict(ros_ns=1_400_000_000,steady_ns=10_360_000_000))))
        result=summarize([row])
        self.assertEqual(result['constraint_publish_age_ms']['p50'],400.)
        self.assertEqual(result['constraint_over_300ms_samples'],1)
        self.assertEqual(result['stages_ms']['receive_to_select']['p50'],60.)

    def test_epoch_mismatch_is_excluded_and_missing_is_explicit(self):
        result=summarize([dict(source='observation',data=dict(scan_timing=dict(clock_epoch=1,successful=record()))),
                          dict(source='state',data={})])
        self.assertEqual(result['epoch_mismatch_records'],1)
        self.assertEqual(result['unique_successful_scans'],0)
        self.assertEqual(result['diagnostic_samples'],1)
        self.assertEqual(result['constraint_publish_age_ms']['n'],0)

    def test_equal_sequences_in_different_epochs_stay_separate(self):
        rows=[dict(source='observation',data=dict(scan_timing=dict(clock_epoch=e,successful=record(epoch=e)))) for e in (0,1)]
        self.assertEqual(summarize(rows)['unique_successful_scans'],2)


if __name__=='__main__':unittest.main()
