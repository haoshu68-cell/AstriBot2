#!/usr/bin/env python3
"""Offline tests for the actual simulation-initialization validator and xacro.

Does not import launch, initialize ROS, or start Gazebo. Source xacro is expanded
through a temporary source package index; no installed package is modified.
"""
import argparse
import ast
import copy
import hashlib
import json
import math
import numbers
import os
from pathlib import Path
import sys
import tempfile
import traceback
import xml.etree.ElementTree as ET
import yaml

REPO=Path('/home/yjh/WorkSpace/astribot_sdk_ros2')
ROOT=Path(__file__).resolve().parent


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--repo',type=Path,default=REPO)
    parser.add_argument('--before',type=Path,default=ROOT/'before_default.urdf')
    parser.add_argument('--mappings',type=Path,default=ROOT/'before_mappings.json')
    parser.add_argument('--profile-argument',default=None)
    parser.add_argument('--output',type=Path,default=ROOT/'offline_result.json')
    args=parser.parse_args()
    description=args.repo/'ws_robot/src/astribot_s1_description'
    launch=args.repo/'ws_robot/src/astribot_s1_gazebo_bringup/launch/warehouse_sim.launch.py'
    profile=description/'config/sim_initial_transport_ready.yaml'
    robot=description/'urdf/astribot_s1.xacro'
    report=dict(scope='Offline AST validator and source-xacro equivalence; not simulator birth-pose or live Hold acceptance',
                cases=[],passed=False)
    def check(name,operation):
        try:
            details=operation()
            report['cases'].append(dict(name=name,passed=True,details=details))
        except Exception as error:
            report['cases'].append(dict(name=name,passed=False,error=repr(error),traceback=traceback.format_exc()))
    try:
        before_text=args.before.read_text()
        if not before_text.strip():raise RuntimeError('before_default.urdf is not ready')
        before=ET.fromstring(before_text)
        tree=ast.parse(launch.read_text())
        function=next((n for n in tree.body if isinstance(n,ast.FunctionDef) and n.name=='_validate_sim_initial_positions'),None)
        if function is None:raise RuntimeError('Production pure validator is not ready')
        namespace=dict(math=math,ET=ET,ElementTree=ET,numbers=numbers)
        imports=[]
        for node in tree.body:
            if isinstance(node,ast.Import) and all(n.name in ('math','numbers','xml.etree.ElementTree') for n in node.names):imports.append(node)
            if isinstance(node,ast.ImportFrom) and node.module in ('math','numbers','xml.etree.ElementTree'):imports.append(node)
        exec(compile(ast.Module(body=imports+[function],type_ignores=[]),str(launch),'exec'),namespace)
        validate=namespace['_validate_sim_initial_positions']
        positions=yaml.safe_load(profile.read_text())['initial_positions']
        mappings=json.loads(args.mappings.read_text()) if args.mappings.is_file() else {}
        candidates=[e.get('name') for e in ET.parse(robot).getroot().findall('{http://www.ros.org/wiki/xacro}arg')
                    if 'initial' in e.get('name','')]
        argument=args.profile_argument
        if argument is None:
            if len(candidates)!=1:raise RuntimeError('Specify the production initial-position xacro argument: '+repr(candidates))
            argument=candidates[0]
        import xacro
        previous=os.environ.get('AMENT_PREFIX_PATH')
        with tempfile.TemporaryDirectory(prefix='ready_source_index_') as tmp:
            prefix=Path(tmp);index=prefix/'share/ament_index/resource_index/packages';index.mkdir(parents=True)
            (index/'astribot_s1_description').touch()
            (prefix/'share/astribot_s1_description').symlink_to(description,target_is_directory=True)
            os.environ['AMENT_PREFIX_PATH']=str(prefix)+(os.pathsep+previous if previous else '')
            try:
                default_text=xacro.process_file(str(robot),mappings=mappings).toxml()
                ready_mappings=dict(mappings);ready_mappings[argument]=str(profile)
                ready_text=xacro.process_file(str(robot),mappings=ready_mappings).toxml()
            finally:
                if previous is None:os.environ.pop('AMENT_PREFIX_PATH',None)
                else:os.environ['AMENT_PREFIX_PATH']=previous
        (ROOT/'after_default.urdf').write_text(default_text)
        (ROOT/'after_ready.urdf').write_text(ready_text)
        default=ET.fromstring(default_text);ready=ET.fromstring(ready_text)
        command_joints={j.get('name'):j for c in ready.findall('ros2_control') for j in c.findall('joint')
                        if j.find("command_interface[@name='position']") is not None}
        report.update(profile_argument=argument,mappings=mappings,position_joint_names=sorted(command_joints),
            inputs={str(p):hashlib.sha256(p.read_bytes()).hexdigest() for p in
                    (launch,robot,description/'urdf/astribot_s1_ros2_control.xacro',profile,args.before)})
        def normalized(element):
            return (element.tag,sorted(element.attrib.items()),(element.text or '').strip(),
                    [normalized(child) for child in element])
        def assert_equal(actual,expected,reason):
            if actual!=expected:raise AssertionError(reason)
        def xml_with(name,value):
            out=ET.fromstring(ready_text)
            targets=[j for c in out.findall('ros2_control') for j in c.findall('joint') if j.get('name')==name]
            if len(targets)!=1:raise AssertionError('fixture joint identity')
            initial=targets[0].find("state_interface[@name='position']/param[@name='initial_value']")
            if initial is None:raise AssertionError('profile omitted initial_value for '+name)
            initial.text=str(value)
            return ET.tostring(out,encoding='unicode')
        def reject(xml,values):
            try:validate(xml,values)
            except (ValueError,TypeError,RuntimeError) as error:
                return dict(rejection_type=type(error).__name__,reason=str(error))
            raise AssertionError('Invalid input accepted')
        check('profile_exactly_22_position_commands',lambda:assert_equal(set(positions),set(command_joints),
            'profile keys differ from actual position-command joints') or assert_equal(len(command_joints),22,'not 22 command joints'))
        check('normal_ready_profile_accepted',lambda:validate(ready_text,dict(positions)))
        check('default_source_unchanged',lambda:assert_equal(normalized(default),normalized(before),
            'default expanded description changed'))
        def profile_only_initial_values():
            stripped=copy.deepcopy(ready);changed=[]
            defaults={j.get('name'):j for c in default.findall('ros2_control') for j in c.findall('joint')}
            for control in stripped.findall('ros2_control'):
                for joint in control.findall('joint'):
                    name=joint.get('name');state=joint.find("state_interface[@name='position']")
                    if state is None:continue
                    params=state.findall("param[@name='initial_value']")
                    if name not in positions:
                        if params:raise AssertionError('initial_value changed for non-position-command joint '+name)
                        continue
                    if len(params)!=1 or float(params[0].text)!=positions[name]:raise AssertionError('wrong initial_value '+name)
                    old=defaults[name].find("state_interface[@name='position']/param[@name='initial_value']")
                    changed.append(dict(joint=name,before=None if old is None else old.text,after=params[0].text))
                    state.remove(params[0])
                    if old is not None:state.append(copy.deepcopy(old))
            assert_equal(len(changed),22,'profile must specify all 22 position state values')
            assert_equal(normalized(stripped),normalized(default),'profile altered more than 22 allowed initial_value fields')
            return changed
        check('profile_changes_only_22_initial_values',profile_only_initial_values)
        name=next(iter(positions))
        missing=dict(positions);missing.pop(name)
        unknown=dict(positions);unknown['unknown_joint']=0.
        check('unknown_joint_rejected',lambda:reject(ready_text,unknown))
        check('missing_joint_rejected',lambda:reject(ready_text,missing))
        for label,value in [('nan',float('nan')),('positive_inf',float('inf')),('negative_inf',float('-inf')),
                            ('bool_true',True),('bool_false',False)]:
            values=dict(positions);values[name]=value
            check(label+'_rejected',lambda values=values:reject(ready_text,values))
        hard={j.get('name'):j.find('limit') for j in ready.findall('joint') if j.get('name') in positions}
        bounded=next(key for key,limit in hard.items() if limit is not None and
                     limit.get('lower') is not None and limit.get('upper') is not None)
        low=float(hard[bounded].get('lower'));high=float(hard[bounded].get('upper'))
        for label,value in [('below_hard_limit',low-.01),('above_hard_limit',high+.01)]:
            values=dict(positions);values[bounded]=value
            check(label+'_rejected',lambda value=value,values=values:reject(xml_with(bounded,value),values))
        for label,value in [('at_lower_limit',low),('at_upper_limit',high)]:
            values=dict(positions);values[bounded]=value
            check(label+'_accepted',lambda value=value,values=values:validate(xml_with(bounded,value),values))
        mismatch=(positions[bounded]+.001 if positions[bounded]+.001<=high else positions[bounded]-.001)
        check('state_initial_value_mismatch_rejected',lambda:reject(xml_with(bounded,mismatch),dict(positions)))
        report['passed']=all(row['passed'] for row in report['cases'])
    except Exception as error:
        report['infrastructure_error']=repr(error);report['traceback']=traceback.format_exc()
    finally:
        args.output.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(dict(passed=report['passed'],cases=len(report['cases']),
        failures=[row['name'] for row in report['cases'] if not row['passed']],
        infrastructure_error=report.get('infrastructure_error'),output=str(args.output)),indent=2))
    return 0 if report['passed'] else 1

if __name__=='__main__':sys.exit(main())
