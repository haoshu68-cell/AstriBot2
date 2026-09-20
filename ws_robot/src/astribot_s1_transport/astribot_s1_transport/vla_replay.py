"""Offline policy evaluation of a recorded request; never imports ROS or executes actions."""
import argparse
import json
from pathlib import Path
import uuid

from .vla_contract import VERSION, digest, validate_config
from .vla_policy import PolicySession


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--config', required=True)
    parser.add_argument('--request', required=True)
    parser.add_argument('--output', required=True)
    args = parser.parse_args()
    config = validate_config(json.loads(Path(args.config).read_text()))
    config['mode'] = 'shadow'
    request = json.loads(Path(args.request).read_text())
    if request.get('schema') != VERSION:
        raise SystemExit('Unsupported recorded schema')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    index = 0
    def record(kind, value):
        nonlocal index
        (output/f'{index:04d}_{kind}.json').write_text(json.dumps(value, indent=2, allow_nan=False))
        index += 1
    source_id = request['request_id']
    session = PolicySession(config, str(uuid.uuid4()), record)
    ok = False
    try:
        session.start(lambda:None)
        request.update(episode_id=session.episode_id, request_id=str(uuid.uuid4()), sequence=0)
        request['context']['execution_permitted'] = False
        request['context']['replay_source_request_id'] = source_id
        request['context_id'] = digest(request['context'])
        request['allowed_action_types'] = session.capabilities['action_types']
        action = session.infer(request, lambda:None)
        result = dict(evidence_level='recorded_observation_policy_only', execution_permitted=False,
                      source_request=str(Path(args.request).resolve()), policy=session.capabilities,
                      action_type=action['type'], source_stamp_ns=request['observation']['stamp_ns'])
        ok = True
    except Exception as error:
        result = dict(evidence_level='replay_failed', execution_permitted=False, reason=str(error))
    finally:
        session.close(dict(status='replay_finished'))
    (output/'summary.json').write_text(json.dumps(result, indent=2))
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if ok else 1)


if __name__ == '__main__':
    main()
