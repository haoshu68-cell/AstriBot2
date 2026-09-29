#!/usr/bin/env python3
"""Build an opt-in Fortress renderer; never replace the system installation."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shlex
import subprocess
import urllib.request

TAG = 'ignition-rendering6_6.6.4'
ARCHIVE_SHA256 = '6610084357cd1af051c84967f25d561ced2130f8e43a65d786d340a5fdb5d659'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True, help='New isolated directory')
    parser.add_argument('--archive', type=Path, help='Optional cached official source archive')
    parser.add_argument('--jobs', type=int, default=2)
    args = parser.parse_args()
    if not 1 <= args.jobs <= 8:
        parser.error('jobs must be 1..8')
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    archive = output / 'upstream.tar.gz'
    if args.archive:
        archive.write_bytes(args.archive.read_bytes())
    else:
        urllib.request.urlretrieve(
            f'https://github.com/gazebosim/gz-rendering/archive/refs/tags/{TAG}.tar.gz', archive)
    if hashlib.sha256(archive.read_bytes()).hexdigest() != ARCHIVE_SHA256:
        raise RuntimeError('Source archive checksum mismatch; refusing extraction')
    here = Path(__file__).resolve().parent
    patch = here / 'patches' / f'{TAG}-worker-count.patch'
    source = output / f'gz-rendering-{TAG}'
    build, install = output / 'build', output / 'install'

    def run(command, **kwargs):
        subprocess.run([str(v) for v in command], check=True, **kwargs)

    run(['tar', '-xzf', archive, '-C', output])
    run(['patch', '--batch', '--fuzz=0', '-p1', '-i', patch], cwd=source)
    run(['g++', '-std=c++14', '-I', source / 'ogre2/src',
         here / 'test_render_worker_count.cpp', '-o', output / 'test_worker_count'])
    run([output / 'test_worker_count'])
    build.mkdir()
    run(['cmake', '-S', source, '-B', build, '-DCMAKE_BUILD_TYPE=Release',
         '-DBUILD_TESTING=OFF', f'-DCMAKE_INSTALL_PREFIX={install}'], cwd=build)
    run(['cmake', '--build', build, '--target', 'ignition-rendering6-ogre2',
         '--parallel', args.jobs])
    # Fortress searches compiled-in plugin paths before lazy environment paths.
    # Install its same-version core too, so that default path points to this prefix.
    run(['cmake', '--install', build / 'src'])
    run(['cmake', '--install', build / 'ogre2/src'])
    library = install / 'lib/ign-rendering-6/engine-plugins/libignition-rendering6-ogre2.so.6.6.4'
    runtime_env = dict(os.environ, LD_LIBRARY_PATH=str(install / 'lib') +
                       ':' + os.environ.get('LD_LIBRARY_PATH', ''))
    linkage = subprocess.run(['ldd', str(library)], check=True, text=True,
                             capture_output=True, env=runtime_env).stdout
    (output / 'ldd.txt').write_text(linkage)
    if 'not found' in linkage:
        raise RuntimeError('Candidate has unresolved shared libraries')
    (output / 'env.sh').write_text(
        '# Opt-in renderer path only; choose worker count explicitly before launching.\n'
        'export IGN_RENDERING_PLUGIN_PATH=' + shlex.quote(str(library.parent)) +
        '${IGN_RENDERING_PLUGIN_PATH:+:$IGN_RENDERING_PLUGIN_PATH}\n'
        'export IGN_RENDERING_RESOURCE_PATH=' +
        shlex.quote(str(install / 'share/ignition/ignition-rendering6')) + '\n'
        'export LD_LIBRARY_PATH=' + shlex.quote(str(install / 'lib')) +
        '${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}\n')
    (output / 'manifest.json').write_text(json.dumps({
        'tag': TAG, 'source_sha256': ARCHIVE_SHA256,
        'patch_sha256': hashlib.sha256(patch.read_bytes()).hexdigest(),
        'library': str(library), 'library_sha256': hashlib.sha256(library.read_bytes()).hexdigest(),
        'core_sha256': hashlib.sha256((install / 'lib/libignition-rendering6.so.6.6.4').read_bytes()).hexdigest(),
        'worker_policy_tests': 'passed',
        'scope': 'Build and link checks only; simulator behavior and timing require validation',
    }, indent=2) + '\n')
    print(f'Candidate ready: {output / "env.sh"}')


if __name__ == '__main__':
    main()
