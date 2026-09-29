#!/usr/bin/env python3
"""Launch-time validation of a frozen integration manifest; no robot commands."""
import argparse
import hashlib
import json
import re
import subprocess
from pathlib import Path


def dependency_paths(output):
    result = {}
    for line in output.splitlines():
        words = line.split()
        if len(words) >= 3 and words[1] == '=>':
            result[words[0]] = str(Path(words[2]).resolve()) if words[2].startswith('/') else 'NOT_FOUND'
    return result


def validate(manifest, prefix_lookup, runtime_only=False):
    errors = []
    for package, expected in manifest['prefixes'].items():
        try:
            actual = prefix_lookup(package)
        except Exception as error:
            errors.append(f'PACKAGE_UNAVAILABLE {package}: {error}')
            continue
        if Path(actual).resolve() != Path(expected).resolve():
            errors.append(f'OVERLAY_MISMATCH {package}: {actual} != {expected}')
    for name, expected in manifest['files'].items():
        path = Path(name)
        if runtime_only and not any(path.is_relative_to(Path(prefix))
                                    for prefix in manifest['prefixes'].values()):
            continue
        try:
            if str(path.resolve()) != expected['resolved']:
                errors.append(f'ARTIFACT_TARGET_CHANGED {name}')
                continue
            h = hashlib.sha256()
            with path.open('rb') as stream:
                for chunk in iter(lambda: stream.read(1024*1024), b''):
                    h.update(chunk)
            if h.hexdigest() != expected['sha256']:
                errors.append(f'ARTIFACT_CHANGED {name}')
        except OSError as error:
            errors.append(f'ARTIFACT_UNAVAILABLE {name}: {error}')
    for binary, expected in manifest.get('dependencies', {}).items():
        try:
            actual = subprocess.run(['ldd', binary], capture_output=True, text=True, timeout=5)
            expected_paths, actual_paths = dependency_paths(expected), dependency_paths(actual.stdout)
            if 'NOT_FOUND' in expected_paths.values() or 'NOT_FOUND' in actual_paths.values():
                errors.append(f'LIBRARY_DEPENDENCY_MISSING {binary}')
            if actual.returncode or actual_paths != expected_paths:
                errors.append(f'LIBRARY_RESOLUTION_CHANGED {binary}')
        except (OSError, subprocess.TimeoutExpired) as error:
            errors.append(f'LIBRARY_RESOLUTION_FAILED {binary}: {error}')
    return errors


def verify_manifest_file(path, prefix_lookup=None, runtime_only=False):
    """Read-only launch preflight; receipt binds the exact checked JSON bytes.

    This does not prove what a subsequently spawned process actually loads.
    The session owner must separately inspect its loaded libraries before goals.
    """
    path = Path(path).expanduser().resolve()
    receipt = {'passed': False, 'manifest': str(path), 'errors': []}
    try:
        raw = path.read_bytes()
        receipt['sha256'] = hashlib.sha256(raw).hexdigest()
        manifest = json.loads(raw)
        prefixes, files = manifest['prefixes'], manifest['files']
        dependencies = manifest.get('dependencies', {})
        if not isinstance(prefixes, dict) or not prefixes or not isinstance(files, dict) or not files:
            raise ValueError('nonempty prefixes and files required')
        if not isinstance(dependencies, dict) or not set(dependencies).issubset(files):
            raise ValueError('dependencies must refer to hashed artifacts')
        for package, prefix in prefixes.items():
            if not isinstance(package, str) or not package or not isinstance(prefix, str) or not Path(prefix).is_absolute():
                raise ValueError('package prefixes must be absolute paths')
        for name, item in files.items():
            if (not isinstance(name, str) or not Path(name).is_absolute() or
                    not isinstance(item, dict) or not isinstance(item.get('resolved'), str) or
                    not Path(item['resolved']).is_absolute() or
                    not isinstance(item.get('sha256'), str) or
                    re.fullmatch('[0-9a-f]{64}', item['sha256']) is None):
                raise ValueError('artifact requires absolute path, resolved target and SHA256')
        if any(not isinstance(value, str) for value in dependencies.values()):
            raise ValueError('dependency records must be ldd output strings')
        if prefix_lookup is None:
            from ament_index_python.packages import get_package_prefix
            prefix_lookup = get_package_prefix
        receipt.update(prefixes=prefixes, artifact_count=len(files),
                       errors=validate(manifest, prefix_lookup, runtime_only))
        receipt['passed'] = not receipt['errors']
    except (OSError, ValueError, TypeError, KeyError) as error:
        receipt['errors'].append('INVALID_RUNTIME_MANIFEST: ' + str(error))
    return receipt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--manifest', type=Path, required=True)
    parser.add_argument('--runtime-only', action='store_true',
                        help='check frozen installed artifacts; source drift remains a separate report')
    args = parser.parse_args()
    receipt = verify_manifest_file(args.manifest, runtime_only=args.runtime_only)
    print(json.dumps(receipt, indent=2))
    return 0 if receipt['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
