#!/usr/bin/env python3
"""Historical valid-JSON boundary replay; current results separate fixed Integer fields from the unchanged Stamp domain."""
import importlib.util
import json
from pathlib import Path

ROOT=Path(__file__).resolve().parents[4]
TEST=ROOT/'ws_robot/src/astribot_s1_navigation_policy_native/test/test_policy_observation_adapters.py'
spec=importlib.util.spec_from_file_location('adapter_test_audit',TEST)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
fixture=module.authority.__wrapped__();authority=next(fixture)
try:
    image=dict(kind='image_box',measurement_id='wide',image_size_px=[2**63,480],box_xyxy_px=[0.,0.,10.,10.],geometry_quality=1.,provenance=['wide'])
    inputs={
        'calibration_epoch_above_int64':module.vision(calibration_epoch=2**63),
        'image_width_above_int64':module.vision(observations=[image]),
        'stamp_plus_timeout_above_int64':module.vision(stamp_ns=2**63-500000000),
    }
    result={}
    for name,packet in inputs.items():
        case={'operations':[{'packet':packet}]}
        result[name]={'input':case,'python':module.reference(case,authority),'native':module.native(case)}
    print(json.dumps(result,indent=2))
finally:
    try:next(fixture)
    except StopIteration:pass
