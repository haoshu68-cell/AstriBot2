#!/usr/bin/env python3
"""Fetch pinned, blob-verified optional simulation sources into a private prefix."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import json
from pathlib import Path, PurePosixPath
import urllib.request


def fetch(url):
    request = urllib.request.Request(url, headers={'User-Agent': 'astribot-social-sim-bootstrap'})
    with urllib.request.urlopen(request, timeout=60) as response:
        return response.read()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--destination', required=True, type=Path)
    args = parser.parse_args()
    destination = args.destination.expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    (destination / 'COLCON_IGNORE').touch()
    lock = json.loads(Path(__file__).with_name('dependencies.json').read_text())
    records = []
    for repo in lock['repositories']:
        revision = repo['commit']
        base = destination / 'source' / repo['name']
        base.mkdir(parents=True, exist_ok=True)
        tree_path = destination / (repo['name'] + '.tree.json')
        url = f"https://api.github.com/repos/{repo['repository']}/git/trees/{revision}?recursive=1"
        if not tree_path.exists():
            tree_path.write_bytes(fetch(url))
        tree = json.loads(tree_path.read_text())
        if tree.get('truncated'):
            raise RuntimeError('Incomplete upstream tree: ' + repo['name'])
        entries = [entry for entry in tree['tree'] if entry['type'] == 'blob' and
                   any(entry['path'].startswith(prefix) if prefix.endswith('/') or not prefix
                       else entry['path'] == prefix for prefix in repo['paths'])]

        def download(entry):
            relative = PurePosixPath(entry['path'])
            if relative.is_absolute() or '..' in relative.parts or entry['mode'] == '120000':
                raise ValueError('Unsupported upstream path: ' + str(relative))
            path = base / relative
            def digest(data):
                return hashlib.sha1(b'blob ' + str(len(data)).encode() + b'\0' + data).hexdigest()
            data = path.read_bytes() if path.exists() else b''
            if digest(data) != entry['sha']:
                data = fetch(f"https://raw.githubusercontent.com/{repo['repository']}/{revision}/{relative}")
                if digest(data) != entry['sha']:
                    raise RuntimeError('Upstream blob digest mismatch: ' + str(relative))
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(data)
            return {'path': str(path.relative_to(destination)), 'git_blob': entry['sha'],
                    'sha256': hashlib.sha256(data).hexdigest()}

        with ThreadPoolExecutor(max_workers=6) as pool:
            files = list(pool.map(download, entries))
        records.append(dict(repo, files=files))
        print(f"{repo['name']}: {len(files)} verified files @ {revision}", flush=True)
    (destination / 'sources.json').write_text(json.dumps(records, indent=2) + '\n')


if __name__ == '__main__':
    main()
