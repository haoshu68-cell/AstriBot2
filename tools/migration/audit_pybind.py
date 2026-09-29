#!/usr/bin/env python3
"""Read-only migration gate; disabled targets and optional imports still count.

This is a source/dependency inventory, not a proof about arbitrary opaque ELF
libraries. It never imports a project module or executes a vendor binary.
"""
import argparse
import ast
from collections import Counter
import json
from pathlib import Path
import re
import xml.etree.ElementTree as ET


HISTORICAL_MODULES = {
    '_chassis_math_native', '_navigation_math_native', '_geometry_native',
    '_wheel_math_native',
}
CPP_SUFFIXES = {'.c', '.cc', '.cpp', '.cxx', '.h', '.hh', '.hpp', '.hxx'}
SKIP_DIRS = {'__pycache__', '.git', 'build', 'install', 'log'}


def files_under(directory):
    if directory.is_dir():
        for path in sorted(directory.rglob('*')):
            if path.is_file() and not SKIP_DIRS.intersection(path.relative_to(directory).parts):
                yield path


def strip_cpp_comments(source):
    # Keep strings and newlines so includes and source line numbers survive.
    token = r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\n]*|/\*[\s\S]*?\*/'
    return re.sub(token, lambda m: re.sub(r'[^\n]', ' ', m[0])
                  if m[0].startswith(('//', '/*')) else m[0], source)


def audit(root, install_roots=()):
    root = Path(root).resolve()
    findings = []
    modules = set()
    python_sources = []
    scanned = 0

    def add(kind, path, line=1, symbol='', scope='project'):
        try:
            name = str(path.relative_to(root))
        except ValueError:
            name = str(path)
        item = dict(kind=kind, path=name, line=line, symbol=symbol, scope=scope)
        if item not in findings:
            findings.append(item)

    if not any((root / directory).is_dir() for directory in ('ws_robot/src', 'astribot_sdk')):
        add('read_error', root, symbol='no expected project source scope exists under this root')
    for directory, scope in ((root / 'ws_robot/src', 'project'),
                             (root / 'astribot_sdk', 'vendor')):
        for path in files_under(directory):
            if '.so' in path.name:
                if 'pybind' in path.name.lower():
                    add('vendor_binary' if scope == 'vendor' else 'source_binary',
                        path, symbol=path.name, scope=scope)
                continue
            if path.suffix not in CPP_SUFFIXES | {'.py', '.cmake'} and path.name not in {
                    'CMakeLists.txt', 'package.xml'}:
                continue
            scanned += 1
            try:
                source = path.read_text(encoding='utf-8')
            except (OSError, UnicodeError) as error:
                add('read_error', path, symbol=str(error), scope=scope)
                continue
            if path.suffix == '.py':
                python_sources.append((path, source, scope))
            elif path.name == 'package.xml':
                try:
                    manifest = ET.fromstring(source)
                    for element in manifest.iter():
                        value = (element.text or '').strip()
                        if 'depend' in element.tag and 'pybind' in value.lower():
                            add('package_dependency', path, symbol=value, scope=scope)
                except ET.ParseError as error:
                    add('parse_error', path, symbol=str(error), scope=scope)
            elif path.suffix in CPP_SUFFIXES:
                source = strip_cpp_comments(source)
                for match in re.finditer(r'PYBIND11_MODULE\s*\(\s*(\w+)', source):
                    modules.add(match[1])
                    add('binding_definition', path, source.count('\n', 0, match.start()) + 1,
                        match[1], scope)
                for match in re.finditer(r'^\s*#\s*include\s*[<"]([^>"\n]*pybind[^>"\n]*)',
                                         source, re.MULTILINE | re.IGNORECASE):
                    add('binding_include', path, source.count('\n', 0, match.start()) + 1,
                        match[1], scope)
            else:
                # CMake bracket comments are removed before ordinary line comments.
                source = re.sub(r'#\[(=*)\[[\s\S]*?\]\1\]',
                                lambda m: re.sub(r'[^\n]', ' ', m[0]), source)
                uncommented = []
                for line, value in enumerate(source.splitlines(), 1):
                    value = value.split('#', 1)[0]
                    uncommented.append(value)
                    if re.search(r'pybind', value, re.IGNORECASE):
                        add('build_dependency', path, line, value.strip(), scope)
                for match in re.finditer(r'pybind11_add_module\s*\(\s*(\w+)', '\n'.join(uncommented),
                                         re.IGNORECASE):
                    modules.add(match[1])

    known = modules | HISTORICAL_MODULES

    def binding_import(name):
        return any(part in known or 'pybind' in part.lower() for part in name.split('.'))

    for path, source, scope in python_sources:
        try:
            tree = ast.parse(source, filename=str(path))
        except SyntaxError as error:
            add('parse_error', path, error.lineno or 1, str(error), scope)
            continue
        for node in ast.walk(tree):
            names = []
            if isinstance(node, ast.Import):
                names = [alias.name for alias in node.names]
            elif isinstance(node, ast.ImportFrom):
                names = [node.module or ''] + [alias.name for alias in node.names]
            elif isinstance(node, ast.Call):
                fn = node.func
                if ((isinstance(fn, ast.Name) and fn.id == '__import__') or
                        (isinstance(fn, ast.Attribute) and fn.attr == 'import_module')):
                    if node.args and isinstance(node.args[0], ast.Constant):
                        if isinstance(node.args[0].value, str):
                            names = [node.args[0].value]
            for name in names:
                if binding_import(name):
                    add('python_consumer', path, node.lineno, name, scope)

    installs = [Path(path).resolve() for path in install_roots]
    for directory in installs:
        if not directory.is_dir():
            add('read_error', directory, symbol='requested install root does not exist')
            continue
        # Install trees are explicitly requested; do not apply source skip rules.
        for path in sorted(directory.rglob('*')):
            if path.is_file() and '.so' in path.name:
                stem = path.name.split('.', 1)[0]
                if stem in known or 'pybind' in path.name.lower():
                    add('installed_extension', path, symbol=path.name, scope='install')

    findings.sort(key=lambda item: (item['path'], item['line'], item['kind'], item['symbol']))
    return {
        'schema': 'astribot.pybind-audit/1', 'root': str(root),
        'clean': not findings, 'binding_modules': sorted(modules),
        'source_files_scanned': scanned,
        'install_roots': [str(path) for path in installs],
        'counts': dict(sorted(Counter(item['kind'] for item in findings).items())),
        'findings': findings,
        'limits': [
            'Static scan of ws_robot/src and astribot_sdk; optional and disabled code counts.',
            'Historical docs and vendored third_party sources are excluded; their enabled build targets require separate review.',
            'Opaque binaries are recognized by binding module names only; this cannot prove their internal dependencies.',
            'No modules are imported and no binaries are executed.',
        ],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--install-root', type=Path, action='append', default=[])
    parser.add_argument('--assert-clean', action='store_true')
    args = parser.parse_args()
    report = audit(args.root, args.install_root)
    print(json.dumps(report, indent=2, ensure_ascii=False))
    return 1 if args.assert_clean and not report['clean'] else 0


if __name__ == '__main__':
    raise SystemExit(main())
