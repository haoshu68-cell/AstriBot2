"""The removal gate must detect disabled builds, hidden fallbacks and stale installs."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys

SCRIPT = Path(__file__).resolve().parents[1] / 'migration/audit_pybind.py'


def module():
    assert SCRIPT.exists(), 'Provide the dependency audit before declaring pybind removal'
    spec = importlib.util.spec_from_file_location('audit_pybind', SCRIPT)
    loaded = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(loaded)
    return loaded


def write(root, name, text):
    path = root / name
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text)
    return path


def test_disabled_target_and_optional_import_are_still_dependencies(tmp_path):
    write(tmp_path, 'ws_robot/src/example/CMakeLists.txt',
          'option(ASTRIBOT_BUILD_PYBIND "temporary" OFF)\nif(ASTRIBOT_BUILD_PYBIND)\n'
          'pybind11_add_module(_example_native binding.cpp)\nendif()\n')
    write(tmp_path, 'ws_robot/src/example/binding.cpp',
          '#include <pybind11/pybind11.h>\nPYBIND11_MODULE(_example_native, m) {}\n')
    write(tmp_path, 'ws_robot/src/example/client.py',
          'try:\n    from example import _example_native\nexcept ImportError:\n    pass\n')
    report = module().audit(tmp_path)
    assert not report['clean']
    assert '_example_native' in report['binding_modules']
    assert any(f['kind'] == 'python_consumer' for f in report['findings'])


def test_docs_comments_and_python_launch_are_not_binding_implementations(tmp_path):
    write(tmp_path, 'docs/history.md', 'pybind11_add_module(_old, old.cpp)')
    write(tmp_path, 'ws_robot/src/example/CMakeLists.txt', '# find_package(pybind11 REQUIRED)\n')
    write(tmp_path, 'ws_robot/src/example/launch/run.launch.py', 'from launch import LaunchDescription\n')
    write(tmp_path, 'ws_robot/src/example/node.cpp', '// PYBIND11_MODULE(_old, m) {}\nint main(){}\n')
    assert module().audit(tmp_path)['clean']


def test_vendor_binary_is_reported_without_importing_or_executing(tmp_path):
    write(tmp_path, 'astribot_sdk/core/vendor_pybind11.cpython-310-x86_64-linux-gnu.so', 'not executable')
    report = module().audit(tmp_path)
    assert not report['clean']
    assert any(f['kind'] == 'vendor_binary' for f in report['findings'])


def test_stale_installed_extension_prevents_clean_claim(tmp_path):
    install = tmp_path / 'candidate_install'
    write(install, 'lib/python3.10/site-packages/pkg/_geometry_native.cpython-310-x86_64-linux-gnu.so', '')
    report = module().audit(tmp_path, [install])
    assert not report['clean']
    assert any(f['kind'] == 'installed_extension' for f in report['findings'])


def test_manifest_dependency_is_a_residual_even_without_binding_sources(tmp_path):
    write(tmp_path, 'ws_robot/src/example/package.xml',
          '<package><name>example</name><build_depend>pybind11-dev</build_depend></package>')
    assert any(f['kind'] == 'package_dependency' for f in module().audit(tmp_path)['findings'])


def test_parse_errors_do_not_silently_pass(tmp_path):
    write(tmp_path, 'ws_robot/src/example/broken.py', 'from nope import (')
    assert any(f['kind'] == 'parse_error' for f in module().audit(tmp_path)['findings'])


def test_commented_cmake_target_does_not_create_a_fake_consumer(tmp_path):
    write(tmp_path, 'ws_robot/src/example/CMakeLists.txt', '# pybind11_add_module(_gone module.cpp)\n')
    write(tmp_path, 'ws_robot/src/example/client.py', 'from unrelated import _gone\n')
    assert module().audit(tmp_path)['clean']


def test_dynamic_import_is_counted_and_missing_install_is_not_clean(tmp_path):
    write(tmp_path, 'ws_robot/src/example/client.py',
          'import importlib\nimportlib.import_module("pkg._geometry_native")\n')
    report = module().audit(tmp_path, [tmp_path / 'missing_install'])
    assert any(f['kind'] == 'python_consumer' for f in report['findings'])
    assert any(f['kind'] == 'read_error' for f in report['findings'])


def test_wrong_repository_root_cannot_produce_an_empty_clean_claim(tmp_path):
    report=module().audit(tmp_path)
    assert not report['clean']
    assert any(f['kind']=='read_error' for f in report['findings'])


def test_cli_assert_clean_returns_failure_with_json_evidence(tmp_path):
    write(tmp_path, 'ws_robot/src/example/package.xml',
          '<package><build_depend>pybind11-dev</build_depend></package>')
    module()
    run = subprocess.run([sys.executable, str(SCRIPT), '--root', str(tmp_path), '--assert-clean'],
                         capture_output=True, text=True, timeout=5)
    assert run.returncode == 1
    assert json.loads(run.stdout)['clean'] is False
