#!/usr/bin/env python3
"""Exercise the selected real ScanTiming extension without loading ROS nodes."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import unittest


class ScanTimingBindingTest(unittest.TestCase):
    def test_unicode_frame_and_reason_remain_serializable(self):
        for char in ('é', '帧', '😀'):
            with self.subTest(char=char):
                trace=NATIVE.ScanTiming()
                sequence=trace.receive(1,2,3,char*300,0)
                trace.finish(sequence,False,4,5,char*300)
                try:
                    snapshot=trace.snapshot()
                except UnicodeDecodeError as error:
                    self.fail('bounded diagnostic text broke UTF-8: '+str(error))
                expected=char*(256//len(char.encode('utf-8')))
                self.assertEqual(snapshot['received']['frame_id'],expected)
                self.assertEqual(snapshot['finished']['reason'],expected)
                json.dumps(snapshot,ensure_ascii=False).encode('utf-8')

    def test_invalid_byte_sequences_do_not_break_snapshot(self):
        cases=((b'prefix\x80suffix','prefix?suffix'),(b'\xc0\xaf','??'),
               (b'\xe0\x80\xaf','???'),(b'\xed\xa0\x80','???'),
               (b'\xf4\x90\x80\x80','????'),(b'\xe5\xb8','??'),
               (b'\xc2A','?A'),(b'\xff'*10000,'?'*256))
        for value,expected in cases:
            with self.subTest(value=value[:8]):
                trace=NATIVE.ScanTiming();sequence=trace.receive(1,2,3,value,0)
                trace.finish(sequence,False,4,5,value)
                try:
                    snapshot=trace.snapshot()
                except UnicodeDecodeError as error:
                    self.fail('invalid diagnostic bytes escaped normalization: '+str(error))
                self.assertEqual(snapshot['received']['frame_id'],expected)
                self.assertEqual(snapshot['finished']['reason'],expected)
                json.dumps(snapshot,ensure_ascii=False).encode('utf-8')

    def test_old_epoch_receive_cannot_replace_current_success(self):
        trace=NATIVE.ScanTiming();trace.reset(1)
        old=trace.receive(100,101,1000,'old',0)
        trace.select(old,102,1001);trace.finish(old,True,103,1002,'')
        snapshot=trace.snapshot()
        self.assertIsNone(snapshot['received']);self.assertIsNone(snapshot['successful'])
        self.assertEqual(snapshot['retained'],0)
        self.assertEqual(snapshot['drop_counts']['obsolete_epoch'],1)
        self.assertEqual(snapshot['missing_updates'],2)
        current=trace.receive(1,2,1003,'current',1)
        self.assertGreater(current,old)
        trace.finish(current,True,3,1004,'');trace.finish(old,True,104,1005,'')
        snapshot=trace.snapshot()
        self.assertEqual(snapshot['successful']['sequence'],current)
        self.assertEqual(snapshot['successful']['clock_epoch'],snapshot['clock_epoch'])


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--binding',type=Path,required=True)
    args=parser.parse_args();binding=args.binding.resolve(strict=True)
    spec=importlib.util.spec_from_file_location('_navigation_math_native',binding)
    NATIVE=importlib.util.module_from_spec(spec);spec.loader.exec_module(NATIVE)
    print(json.dumps({'binding':str(binding),'sha256':hashlib.sha256(binding.read_bytes()).hexdigest()}),flush=True)
    unittest.main(argv=[sys.argv[0]],verbosity=2)
