#!/usr/bin/env python3
"""Read-only source/entry/import inventory; never import or start robot code.

Writes JSON only when --output is explicitly supplied. Classifications are a
reviewed 2026-09-21 policy, not a filename-only detector. Unknown executable
source is kept visible for review. Source discovery uses rg's ignore rules;
generated build/install trees and external vendors are excluded from own counts.
"""
import argparse
import ast
from collections import Counter, defaultdict, deque
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import re
import subprocess


TOOL_RUNTIME = {
    'tools/sim_stack_supervisor.py', 'tools/sim_isolation.py',
    'tools/robot/hardware_exploration.py',
    'tools/robot/hardware_sensors.py', 'tools/robot/robot_task_control.py',
    'tools/joy_tools.py',
}
VALIDATION_MODULES = {
    'astribot_s1_navigation.explore_metrics_recorder_node',
    'astribot_s1_navigation.path_tracking_diagnostics_node',
    'astribot_trajectory_bridge.joint_map_probe',
    'astribot_s1_transport.vla_probe', 'astribot_s1_transport.vla_replay',
}
LAUNCH_MODULES = {
    'astribot_logging.launch', 'astribot_logging.launch_entry',
    'astribot_s1_perception.map_source_config',
    'astribot_s1_perception.map_start_cell_check',
}
CPP_REPLACEMENTS = {
    'astribot_s1_chassis_effort_drive.omni_effort_drive_node': 'astribot_s1_chassis_effort_drive_native/omni_effort_drive_cpp',
    'astribot_s1_navigation.arm_speed_limiter_node': 'astribot_trajectory_bridge_native/arm_speed_limiter_cpp',
    'astribot_s1_navigation.arm_reach_metric': 'astribot_trajectory_bridge_native/chassis_math; arm_speed_limiter_cpp',
    'astribot_s1_navigation.cmd_vel_body_to_world_node': 'astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp',
    'astribot_s1_navigation.posture_monitor_policy': 'astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp',
    'astribot_s1_navigation_policy.cmd_vel_math': 'astribot_s1_navigation_policy_native/cmd_vel_body_to_world_cpp',
    'astribot_s1_navigation_policy.costmap_scan_node': 'astribot_s1_navigation_policy_native/costmap_scan_cpp',
    'astribot_s1_navigation_policy.envelope_node': 'astribot_s1_navigation_policy_native/envelope_coordinator_cpp + fixed_envelope_cpp',
    'astribot_s1_navigation_policy.fixed_envelope_node': 'astribot_s1_navigation_policy_native/fixed_envelope_cpp',
    'astribot_s1_navigation_policy.fixed_envelope': 'astribot_s1_navigation_policy_native/fixed_envelope_core',
    'astribot_s1_navigation_policy.protection_node': 'astribot_s1_navigation_policy_native/final_protection_cpp',
    'astribot_s1_navigation_policy.control_time': 'astribot_s1_navigation_policy_native/navigation_math ControlTime; final_protection_cpp',
    'astribot_s1_navigation_policy.scan_occupancy': 'astribot_s1_robot_geometry/geometry_core scan_occupied_cells; observer uses native functions directly',
    'astribot_s1_robot_geometry.node': 'astribot_s1_robot_geometry/geometry_state',
    'astribot_s1_robot_geometry.model': 'astribot_s1_robot_geometry/geometry_core robot_model.hpp',
    'astribot_s1_robot_geometry.state': 'astribot_s1_robot_geometry/geometry_core joint_snapshot.hpp',
    'astribot_s1_robot_geometry.projection_contract': 'astribot_s1_robot_geometry/geometry_state projection contract',
    'astribot_s1_transport.camera_calibration_postprocess': 'astribot_s1_perception_components/camera_calibration_postprocess',
    'astribot_s1_navigation_policy.task_arbiter_node': 'astribot_s1_task_arbiter_native/task_arbiter_cpp',
    'astribot_s1_dynamics_coupling.arm_chassis_speed_coupling_node': 'astribot_s1_dynamics_coupling/arm_chassis_speed_coupling_node (C++ ELF)',
    'astribot_s1_dynamics_coupling.arm_reach_metric': 'astribot_s1_dynamics_coupling/core.hpp',
    'astribot_s1_perception.map_domain_relay': 'astribot_s1_perception_native/map_domain_relay (C++ ELF)',
    'astribot_s1_perception.map_odom_tf_node': 'astribot_s1_perception_native/map_odom_tf_node (C++ ELF)',
    'astribot_s1_perception.map_odom_decompose': 'astribot_s1_perception_native/map_odom_core.hpp',
}

REFERENCE_PREFIX = ('tools', 'migration', 'python_reference')

# Boundary/validation fields belong to a migration unit, inherited by each file.
GROUPS = {
    'policy': dict(priority='P1', cpp_status='Native cores and direct observer candidate exist; JSON integer/string boundaries have isolated differential and ROS evidence; production observer/controller entries remain Python', boundary='Preserve capture time, epoch, leases, risk, all P2-P5/H2 phases, path ownership, candidate budgets and abort semantics', validation='14 CTest, 54 paired observer ROS scenarios (108 cases) and 1600 measured output frames; Stamp domains/failure stage, controller phases and isolated Nav2 closed loop remain required', blocker='Observer Stamp representation/failure-stage equivalence and complete controller remain unfinished; imported math bindings still needed'),
    'arbiter': dict(priority='P1', cpp_status='Native task_arbiter_cpp is the sole production entry; Python is a frozen validation reference', boundary='One backend action owner; replacement must await canceled backend terminal result; preserve task ID/sequence/front aliases', validation='77 installed-ELF protocol/entry cases and 1 ownership CTest; paired fake-Nav2 overhead measurements', blocker='Real Nav2 integration and physical stopping/whole-robot acceptance remain untested'),
    'dynamics': dict(priority='P1', cpp_status='Same-name C++ Twist coupling ELF is the sole production entry; Python is a frozen validation reference', boundary='Twist in/out + JointState + TF reach/velocity scaling, timeout/EMA/cache behavior', validation='10027 core inputs, 30 ROS cases, 3 installed smoke cases; 8000 paired performance messages matched', blocker='Whole-stack drive/long-duration/hardware acceptance remains untested; inherited nonfinite-parameter behavior documented'),
    'bridge': dict(priority='P0', cpp_status='bridge_runtime/chassis_math C++ cores exist; ROS adapters, session/write ownership remain Python', boundary='Single SDK session, heartbeat/control rights, write gate, enable/reset, filtered/direct/stop/hold, leases, units and arm/gripper action results', validation='Fake SessionPort trace parity then ROS service/action fault/cancel and manufacturer hardware acceptance', blocker='Manufacturer native SDK headers/library or documented equivalent control protocol unavailable in reviewed source'),
    'transport': dict(priority='P1', cpp_status='MTC planner/execution guard and geometry C++ exist; task transaction/ROS orchestration remains Python', boundary='MTC as planner, task lease, ordered single-use plans, attach/detach ledger, physical confirmation, fixed hold and cancellation cleanup', validation='Complete pick/carry/place and failures: one arm failure, timeout, cancel, payload/version changes, source expiry', blocker='No complete C++ TransportTask/RosBackend; geometry source inbox still bound'),
    'vla': dict(priority='P2', cpp_status='No equivalent C++ protocol/HTTP adapter identified; only model-agnostic reference policy available', boundary='astribot.vla/1 request/proposal schema, TTL, size/time budgets, observation identity; never give model direct actuator authority', validation='Protocol fixtures, malformed/late/oversized response, replay and task cancellation; trained model/hardware remain separate', blocker='Must preserve inference adapter configurability; no trained model acceptance claimed'),
    'camera': dict(priority='P1', cpp_status='C++ RGB-D/detector/health nodes exist; they are not proof of legacy orange-box JSON contract replacement', boundary='Paired capture stamps, encodings/stride, calibration epoch, TF-at-capture, object JSON and camera health', validation='Image/CameraInfo/TF replay incl stale/NaN/wrong encoding; compare transport consumer contract', blocker='Transport support still launches camera_observer; C++ vision pipeline emits different typed interfaces'),
    'geometry': dict(priority='P1', cpp_status='geometry_core/geometry_state C++ exist; polygon API remains imported by live policy/transport', boundary='Same conservative hull, float canonical order/hash, bounds, payload/source identity and timing', validation='Polygon differential and source timestamp/epoch/attachment invariants; delete binding only after all import consumers migrate', blocker='Live Python consumer closure'),
    'perception': dict(priority='P2', cpp_status='Native SLAM exists, but these ROS/operational adapters have no verified like-for-like C++ replacement', boundary='map/odom TF single writer, jump/clock handling, cross-domain QoS, session save completeness; patrol Twist safety gates', validation='TF and map capture replay, two-domain isolation, map session save/reload; patrol closed loop separately', blocker='ROS adapter and operational CLI contracts require preservation'),
    'social': dict(priority='P2', cpp_status='C++ physics/proxy components exist; social observation adapters/provider remain Python', boundary='Explicit simulation-truth labeling, observation validation/TF, no motion authority; GetAgents service', validation='HuNav fixture + stale/out-of-order/transform/identity cases; no hardware truth inference', blocker='Not a human tracking model; simulation-only acceptance'),
    'supervisor': dict(priority='P2', cpp_status='No equivalent C++ supervisor identified; C++ nodes do not replace process/session orchestration', boundary='Owned PID/start identity, locks, signal handling, readiness, control enable/stop, ROS/Gazebo isolation and authoritative session log', validation='Fake subprocess/unit ownership cases + separately owned integration; never stop shared sessions to validate', blocker='Hybrid launch/runtime file: launch descriptions may remain Python, ongoing supervision is migratable'),
    'logging': dict(priority='P3', cpp_status='spdlog C ABI shared library already C++; Python logging/output adapter uses ctypes, not pybind', boundary='Per-session sink, rotation, fork safety, stdout capture and shutdown flush', validation='Output/log rotation tests + source/installation dependency audit', blocker='Needed while Python launch/validation remain; not required to eliminate own pybind'),
    'joy': dict(priority='P3', cpp_status='No C++ equivalent identified', boundary='Joy subscription, deadman/button mapping; examples are callers', validation='Recorded Joy inputs and disconnect/default-zero behavior', blocker='Depends on external astribot_ros_middleware; only examples call it in reviewed graph'),
}


def module_name(path):
    parts = list(path.parts)
    if parts[:2] == ['ws_robot', 'src']:
        package = parts[2]
        parts = parts[3:]
        if parts[:2] == ['test', 'reference']:
            parts = parts[2:]
            # New native packages may host oracles in the old Python namespace.
            # A flat oracle inherits its enclosing package; an explicit package
            # directory retains its own identity for dependency/import auditing.
            if not (len(parts) > 1 and parts[0].startswith('astribot_')):
                parts.insert(0, package)
    elif tuple(parts[:3]) == REFERENCE_PREFIX:
        parts = parts[3:]
    parts[-1] = Path(parts[-1]).stem
    if parts[-1] == '__init__': parts.pop()
    return '.'.join(parts)


def group_for(path, module):
    if path == 'tools/joy_tools.py': return 'joy'
    if path in TOOL_RUNTIME: return 'supervisor'
    if module.startswith('astribot_logging'): return 'logging'
    if module.endswith('task_arbiter_node'): return 'arbiter'
    if module.startswith('astribot_s1_dynamics_coupling'): return 'dynamics'
    if module.startswith('astribot_trajectory_bridge'): return 'bridge'
    if module.startswith('astribot_s1_robot_geometry'): return 'geometry'
    if module.startswith('astribot_s1_perception.'): return 'perception'
    if module.startswith('astribot_s1_social_navigation'): return 'social'
    if module.startswith('astribot_s1_transport.vla_'): return 'vla'
    if module.startswith('astribot_s1_transport.camera') or module.endswith('.rgbd'): return 'camera'
    if module.startswith('astribot_s1_transport'): return 'transport'
    return 'policy'


def index_modules(rows):
    """An executable source must not disappear behind its frozen test oracle."""
    modules = {}
    for path, row in rows.items():
        old = modules.get(row['module'])
        if old is None or rows[old]['category'] == 'cpp_validation_reference':
            modules[row['module']] = path
        elif row['category'] != 'cpp_validation_reference':
            modules[row['module']] = path
    return modules


def initial_class(path, module, tree):
    parts = Path(path).parts
    reference = parts[:3] == REFERENCE_PREFIX or (
        parts[:2] == ('ws_robot', 'src') and parts[3:5] == ('test', 'reference'))
    if reference and module in CPP_REPLACEMENTS:
        return 'cpp_validation_reference'
    if parts[0] == 'astribot_sdk': return 'vendor_sdk'
    if parts[0] == 'examples': return 'vendor_example'
    if set(parts) & {'test', 'tests'}: return 'excluded_validation'
    if module in VALIDATION_MODULES or '.explore_metrics.' in module: return 'excluded_validation'
    if module in LAUNCH_MODULES or 'launch' in parts or Path(path).name == 'setup.py': return 'excluded_launch_build'
    if path == 'ws_robot/src/astribot_eigen_vendor/verify_source.py': return 'excluded_launch_build'
    if path == 'ws_robot/src/astribot_object_pose_core/tools/generate_reference_cad.py': return 'excluded_launch_build'
    if parts[0] == 'tools' and path not in TOOL_RUNTIME: return 'excluded_validation_or_setup_tool'
    if path.endswith('/scripts/plot_traj.py'): return 'excluded_validation'
    if Path(path).name == '__init__.py' and all(isinstance(n, ast.Expr) and isinstance(n.value, ast.Constant) for n in tree.body): return 'package_marker'
    return 'candidate'


def const(node, env):
    if isinstance(node, ast.Constant): return node.value
    if isinstance(node, ast.Name): return env.get(node.id)
    if isinstance(node, (ast.List, ast.Tuple)): return [const(v, env) for v in node.elts]
    if isinstance(node, ast.Dict): return {const(k, env): const(v, env) for k, v in zip(node.keys, node.values)}
    if isinstance(node, ast.BinOp) and isinstance(node.op, ast.Add):
        left, right = const(node.left, env), const(node.right, env)
        if isinstance(left, str) and isinstance(right, str): return left + right
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=Path(__file__).resolve().parents[2])
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(); root = args.root.resolve()
    if not (root / 'ws_robot/src/astribot_s1_navigation_policy/setup.py').is_file():
        parser.error('root does not contain expected repository source')
    discovered = subprocess.run(['rg', '--files', 'ws_robot/src', 'tools', 'astribot_sdk', 'examples',
                                 '-g', '*.py', '-g', '!**/build/**', '-g', '!**/install/**',
                                 '-g', '!**/log/**'], cwd=root, text=True, check=True, capture_output=True).stdout.splitlines()
    paths = sorted(p for p in discovered if not p.startswith('ws_robot/src/') or Path(p).parts[2].startswith('astribot'))
    rows = {}; trees = {}; entries = []; launch_nodes = []; errors = []
    for path in paths:
        raw = (root/path).read_bytes()
        try: tree = ast.parse(raw, filename=path)
        except (SyntaxError, UnicodeError) as exc:
            errors.append({'path': path, 'error': str(exc)}); continue
        trees[path] = tree; module = module_name(Path(path)); env = {}
        for node in tree.body:
            if isinstance(node, ast.Assign) and len(node.targets) == 1 and isinstance(node.targets[0], ast.Name):
                env[node.targets[0].id] = const(node.value, env)
        imports = []; interfaces = []; params = []; bindings = []; sdk_calls=[]
        package = module if Path(path).name == '__init__.py' else module.rpartition('.')[0]
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imports.extend({'module': n.name, 'line': node.lineno} for n in node.names)
            elif isinstance(node, ast.ImportFrom):
                base = node.module or ''
                if node.level:
                    anchor = package.split('.')
                    if node.level > 1: anchor = anchor[:-(node.level-1)]
                    base = '.'.join(anchor + ([base] if base else []))
                imports.append({'module': base, 'line': node.lineno})
                imports.extend({'module': base + '.' + n.name, 'line': node.lineno} for n in node.names)
            if not isinstance(node, ast.Call): continue
            name = ast.unparse(node.func).split('.')[-1]
            function=ast.unparse(node.func)
            if any(token in function for token in ('.session.', '._session.', '.bot.', '._bot.', '.astribot_interface.', '.sdk.', '._sdk.')) or name=='Astribot':
                sdk_calls.append({'line':node.lineno,'expression':ast.unparse(node)})
            if name in {'create_publisher', 'create_subscription', 'create_service', 'create_client', 'ActionClient', 'ActionServer', 'TransformBroadcaster', 'TransformListener'}:
                interfaces.append({'kind': name, 'line': node.lineno, 'expression': ast.unparse(node)})
            if name == 'declare_parameter' and node.args:
                params.append({'line': node.lineno, 'name': ast.unparse(node.args[0]), 'default': ast.unparse(node.args[1]) if len(node.args)>1 else None})
            if name == 'setup':
                for kw in node.keywords:
                    if kw.arg == 'entry_points':
                        value = const(kw.value, env) or {}
                        for entry in value.get('console_scripts', []):
                            if not isinstance(entry, str) or '=' not in entry: continue
                            executable, target = (v.strip() for v in entry.split('=', 1))
                            entries.append({'package': Path(path).parent.name, 'executable': executable,
                                            'target': target, 'module': target.split(':')[0],
                                            'source': path, 'line': kw.value.lineno})
            if name == 'Node':
                values = {kw.arg: const(kw.value, env) for kw in node.keywords}
                if values.get('package') and values.get('executable'):
                    launch_nodes.append({'source': path, 'line': node.lineno, 'package': values['package'],
                                         'executable': values['executable'],
                                         'condition': next((ast.unparse(k.value) for k in node.keywords if k.arg=='condition'), None)})
        for imp in imports:
            if any(s in imp['module'] for s in ('_native', 'pybind')):
                bindings.append(imp)
        rows[path] = dict(path=path, module=module, sha256=hashlib.sha256(raw).hexdigest(),
                          lines=len(raw.splitlines()), category=initial_class(path,module,tree),
                          role=(ast.get_docstring(tree) or '').split('\n')[0],
                          classes=[n.name for n in tree.body if isinstance(n, ast.ClassDef)],
                          public_api=[n.name for n in tree.body if isinstance(n,(ast.FunctionDef,ast.AsyncFunctionDef,ast.ClassDef)) and not n.name.startswith('_')],
                          imports=imports, interfaces=interfaces, parameters=params, binding_imports=bindings,
                          sdk_calls=sdk_calls)
    modules = index_modules(rows); reverse = defaultdict(set)
    edges = defaultdict(set)
    for path, row in rows.items():
        for imp in row['imports']:
            name=imp['module']; target=modules.get(name)
            # Local script imports (e.g. hardware_sensors) are relative to script directory.
            if target is None:
                local = str(Path(path).parent / (name.replace('.', '/')+'.py'))
                if local in rows: target=local
            if target is None and '.' not in name:
                tool_alias='tools/'+name+'.py'
                if tool_alias in rows: target=tool_alias
            if target and target != path:
                edges[path].add(target); reverse[target].add(path)
    roots = {p for p in TOOL_RUNTIME if p in rows and p != 'tools/joy_tools.py'}
    for entry in entries:
        path=modules.get(entry['module']); entry['path']=path
        entry['launch_callers']=[n for n in launch_nodes if (n['package'],n['executable'])==(entry['package'],entry['executable'])]
        if path and rows[path]['category']=='candidate': roots.add(path)
    reached=set(); todo=deque(roots)
    while todo:
        path=todo.popleft()
        if path in reached: continue
        reached.add(path)
        todo.extend(p for p in edges[path] if rows[p]['category'] not in {'excluded_validation','excluded_launch_build','excluded_validation_or_setup_tool','vendor_example','cpp_validation_reference'})
    for path,row in rows.items():
        module=row['module']; replacement=CPP_REPLACEMENTS.get(module)
        if row['category']=='candidate':
            if path in reached:
                row['category']='runtime_remaining'
                if replacement: row['category']='cpp_replaced_but_python_reachable'
            elif replacement: row['category']='cpp_replaced_reference_source'
            elif path=='tools/joy_tools.py' or module=='astribot_s1_transport.vla_examples': row['category']='optional_runtime_example_support'
            else: row['category']='unreached_runtime_candidate'
        row['installed_source_entries']=[e for e in entries if e['path']==path]
        row['direct_importers']=sorted(reverse[path])
        row['production_importers']=sorted(p for p in reverse[path] if p in reached)
        row['local_dependencies']=sorted(edges[path])
        if row['category'] in {'runtime_remaining','cpp_replaced_but_python_reachable','cpp_replaced_reference_source','cpp_validation_reference','optional_runtime_example_support','unreached_runtime_candidate'}:
            group=group_for(path,module); row['migration_group']=group; row.update(GROUPS[group])
            if replacement: row['cpp_status']=replacement
        if row['category'] == 'cpp_validation_reference':
            package, name = module.split('.', 1)
            row['previous_production_path'] = f'ws_robot/src/{package}/{package}/{name.replace(".", "/")}.py'
            row['installed_by_production_package'] = False
    installed=[]; current_entries={(e['package'],e['executable']) for e in entries}
    for prefix in ('ws_robot/install','install'):
        if not (root/prefix).exists(): continue
        for path in sorted((root/prefix).glob('astribot*/lib/astribot*/*')):
            if not path.is_file() or path.name.endswith(('.so','.a')): continue
            with path.open('rb') as stream: magic=stream.read(32768)
            kind='ELF' if magic.startswith(b'\x7fELF') else 'python_script' if magic.startswith(b'#!') and b'python' in magic.split(b'\n')[0] else 'other'
            if kind not in {'python_script','ELF'}: continue
            installed.append({'path':str(path.relative_to(root)), 'kind':kind, 'resolved_path':str(path.resolve()),
                              'current_source_python_entry':(path.parent.name,path.name) in current_entries,
                              'entry_target_hints':re.findall(r"astribot[A-Za-z0-9_.:-]+",magic.decode('utf-8','ignore'))[:12] if kind=='python_script' else []})
    external_roots=['third_party','ws_robot/third_party','ws_robot/deps','ws_robot/src/livox_ros_driver2','ws_robot/src/aws-robomaker-small-warehouse-world','astribot_msgs/local']
    build_manifest=[]
    for path in sorted((root/'ws_robot/src').glob('astribot*/CMakeLists.txt')):
        raw=path.read_bytes(); lines=raw.decode().splitlines()
        build_manifest.append({'path':str(path.relative_to(root)),'sha256':hashlib.sha256(raw).hexdigest(),
                               'entry_evidence':[{'line':n,'text':s.strip()} for n,s in enumerate(lines,1)
                                                 if re.search(r'add_executable|install\(PROGRAMS|ament_python_install_package|install\(TARGETS',s)]})
    vendor_binaries=[{'path':str(p.relative_to(root)),'bytes':p.stat().st_size} for p in sorted((root/'astribot_sdk').rglob('*.so'))]
    out=dict(schema='astribot.python-runtime-inventory/1', generated_at=datetime.now(timezone.utc).isoformat(),
             root=str(root), method='AST source entries + literal launch Node declarations + local import closure; no imports or runtime execution',
             limitations=['Imports inside optional/functions counted conservatively, not proof every branch executes.',
                          'Dynamic imports, generated launch expressions, subprocess shell construction require review; known supervisor roots are explicit.',
                          'Source discovery respects rg ignore rules; generated build/install copies excluded from source counts.',
                          'Existing install directories are read-only historical snapshots; no source-to-install equivalence claimed.',
                          'No ROS/Gazebo/hardware validation performed.'],
             counts=dict(Counter(r['category'] for r in rows.values())),
             runtime_group_counts=dict(Counter(r.get('migration_group') for r in rows.values() if r['category']=='runtime_remaining')),
             roots=sorted(roots), source_console_entries=entries, literal_launch_nodes=launch_nodes,
             existing_install_executables=installed, build_manifests=build_manifest,
             vendor_sdk_binaries=vendor_binaries, third_party_roots_excluded=external_roots,
             errors=errors, files=list(rows.values()))
    encoded=json.dumps(out,ensure_ascii=False,indent=2)+'\n'
    if args.output:
        args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(encoded)
        print(json.dumps({'counts':out['counts'],'runtime_group_counts':out['runtime_group_counts'],'entries':len(entries),'errors':errors},ensure_ascii=False,indent=2))
    else: print(encoded,end='')
    return 1 if errors else 0


if __name__=='__main__': raise SystemExit(main())
