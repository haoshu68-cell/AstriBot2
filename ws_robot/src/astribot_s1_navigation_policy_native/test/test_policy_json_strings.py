"""Python string-code-point parity at the actual JSON adapter boundary."""
import json
import random
import pytest

from test_policy_observation_adapters import authority, check, vision, cloud, native, reference


@pytest.mark.parametrize('raw',[
    r'"\ud800"',r'"\udfff"',r'"\ud800\udc00"',
    r'"\ud800\ud800\udc00\udfff"',r'"\u0000Sd800"',
    r'"\u0000\u0000Sdead\ud800"',r'"\\ud800"',
    r'{"\ud800":1,"\u0000Sd800":2,"\ud800":3}',
    '{"\\ud800\\udc00":1,"\U00010000":2}',
    r'{"__integer_json__":"\ud800","n":18446744073709551617}',
    r'["\u0000", "\udfff", "普通文字", "\u2028"]',
    r'"\ud800\uDEZZ"',r'"\ud800',r'"\u0000\q"',
])
def test_lone_paired_null_marker_keys_and_syntax(authority,raw):
    check(dict(operations=[dict(action='json',data=raw)]),authority)


@pytest.mark.parametrize('field',['ignored','measurement_id','track_id','provenance','classes','sensor_id','frame_id'])
def test_adapter_preserves_strings_and_call_order(authority,field):
    p=vision()
    if field in ('ignored','sensor_id','frame_id'):p[field]='text\ud800'
    elif field=='provenance':p['observations'][0][field]=['origin\udfff']
    elif field=='classes':p['observations'][0][field]={'person\ud800':.6,'unknown':.3}
    else:p['observations'][0][field]='text\ud800'
    check(dict(operations=[dict(data=json.dumps(p)),dict(action='metadata')]),authority)


@pytest.mark.parametrize('offset',[-1,0,1])
def test_packet_budget_counts_original_escaped_text(authority,offset):
    p=vision(ignored='\u0000Sd800\ud800中文')
    raw=json.dumps(p,ensure_ascii=True)
    check(dict(raw_options={'max_packet_bytes':len(raw)+offset},operations=[dict(data=raw)]),authority)


def test_distinct_surrogate_and_literal_marker_measurement_ids(authority):
    p=vision();original=p['observations'][0]
    p['observations']=[dict(original,measurement_id=x,provenance=[x])
        for x in ('x\ud800','x\u0000Sd800','x\udfff','x\u0000Sdfff','x\u0000\u0000Sd800')]
    check(dict(operations=[dict(data=json.dumps(p))]),authority)


def test_cloud_sensor_option_python_repr_escapes_surrogates(authority):
    check(dict(adapter='pointcloud_boxes',raw_options={'sensor_id':['x\ud800']},
        operations=[dict(packet=cloud([(1.,2.,3.)]))]),authority)


def test_all_surrogates_and_seeded_marker_collisions(authority):
    values=[chr(cp) for cp in range(0xd800,0xe000)]
    rng=random.Random(20260921);alphabet=['\0','S','d','8','0','"','\\','中','\U00010000','\ud800','\udfff']
    values += [''.join(rng.choices(alphabet,k=rng.randrange(1,20))) for _ in range(1000)]
    check(dict(operations=[dict(action='json',data=json.dumps({value:value})) for value in values]),authority)


def test_duplicate_restored_key_keeps_first_insertion_position(authority):
    case=dict(operations=[dict(action='json',data=r'{"\ud800":1,"\u0000Sd800":2,"\ud800":3,"last":4}')])
    a=reference(case,authority);b=native(case)
    assert list(a[0]['result'].items())==list(b[0]['result'].items())
