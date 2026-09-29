"""C++ URDF/mesh geometry compared with the retained Python analysis oracle."""
import json
from pathlib import Path
import subprocess
import numpy as np
import pytest
from astribot_s1_robot_geometry.model import RobotModel, Shape
from astribot_s1_robot_geometry.polygon import contains

URDF='''<robot name="fixture"><link name="base"><collision><geometry><box size="0.62 0.62 0.2"/></geometry></collision></link>
<joint name="arm" type="revolute"><parent link="base"/><child link="arm_link"/><origin xyz="0 0.3 0.5"/><axis xyz="0 0 1"/><limit lower="-3" upper="3"/></joint>
<link name="arm_link"><collision><origin xyz="0.3 0 0"/><geometry><box size="0.6 0.06 0.06"/></geometry></collision></link></robot>'''

@pytest.fixture(scope='module',params=['-O2','-O3'])
def probe(tmp_path_factory,request):
    root=Path(__file__).parents[1];exe=tmp_path_factory.mktemp('model_cpp')/'probe'
    subprocess.run(['g++','-std=c++17',request.param,'-Wall','-Wextra','-Werror','-I/usr/include/eigen3',
                    '-I'+str(root/'include'),str(root/'test/robot_model_probe.cpp'),'-ltinyxml2','-lcrypto','-o',str(exe)],check=True)
    p=subprocess.Popen([str(exe)],stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True)
    def call(**args):
        p.stdin.write(json.dumps(args)+'\n');p.stdin.flush();r=json.loads(p.stdout.readline())
        if 'error' in r:raise ValueError(r['error'])
        return r
    yield call
    p.stdin.close();p.wait(timeout=5)

def compare(probe,urdf,q,errors,attachments=()):
    model=RobotModel(urdf,'base');expected=model.geometry(q,errors,attachments)
    serialized=[dict(link=s.link,kind=s.kind,dimensions=s.dimensions.tolist(),pose=s.pose.tolist()) for s in attachments]
    actual=probe(urdf=urdf,q=q,errors=errors,attachments=serialized)
    assert actual['required']==list(model.required)
    assert actual['revision']==model.revision
    for name in ('physical','reserved'):
        assert contains(actual[name],expected[name],2e-9) and contains(expected[name],actual[name],2e-9)
    np.testing.assert_allclose([actual['height'],actual['z_min']],[expected['height'],expected['z_min']],atol=1e-12)
    assert len(actual['slices'])==len(expected['slices'])
    for a,b in zip(actual['slices'],expected['slices']):
        assert (a['z_min'],a['z_max'])==(b['z_min'],b['z_max'])
        assert contains(a['footprint'],b['footprint'],2e-9) and contains(b['footprint'],a['footprint'],2e-9)

def test_cpp_joint_mimic_primitives_and_payload_preserved(probe):
    from scipy.spatial.transform import Rotation
    for kind,geometry in [('box','box size=".2 .1 .15"'),('sphere','sphere radius=".08"'),('cylinder','cylinder radius=".1" length=".4"')]:
        urdf=URDF.replace('</robot>',f'''<joint name="finger" type="revolute"><parent link="arm_link"/><child link="tip"/><origin xyz=".6 0 0" rpy=".2 .5 .7"/><mimic joint="arm" multiplier="-2" offset=".1"/></joint><link name="tip"><collision><origin xyz=".02 .03 0"/><geometry><{geometry}/></geometry></collision></link></robot>''')
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('xyz',[.2,.4,.1]).as_matrix();pose[:3,3]=[.4,-.2,.1]
        payload=Shape('tip','box',[.3,.2,.1],pose)
        for angle in (-.2,0.,.6):compare(probe,urdf,{'arm':angle},{'arm':.003},[payload])

def test_cpp_scaled_rotated_mesh_bounds_and_hash(probe,tmp_path):
    mesh=tmp_path/'shape.obj';mesh.write_text('  v -1 -2 -.3\nv 2 .5 .7\n\tv -.1 1 .2\n')
    urdf=URDF.replace('box size="0.6 0.06 0.06"',f'mesh filename="{mesh}" scale=".2 -.3 .4"')
    compare(probe,urdf,{'arm':.4},{'arm':.01})

def test_cpp_rejects_missing_joint_invalid_shape_and_mimic_cycle(probe):
    with pytest.raises(ValueError):probe(urdf=URDF,q={},errors={'arm':.003})
    with pytest.raises(ValueError):probe(urdf=URDF.replace('box size="0.6 0.06 0.06"','cone radius=".1"'),q={'arm':0.},errors={'arm':.003})
    with pytest.raises(ValueError):probe(urdf=URDF.replace('<limit lower="-3" upper="3"/>','<mimic joint="arm"/>'),q={'arm':0.},errors={'arm':.003})

def test_cpp_stl_ascii_binary_and_prismatic_bounds(probe,tmp_path):
    import struct
    vertices=[(-1.,-.2,-.3),(.2,.3,.4),(.5,-.1,.2)]
    ascii_mesh=tmp_path/'a.stl'
    ascii_mesh.write_text('solid model\n'+''.join(' vertex %s %s %s\n'%v for v in vertices)+'endsolid\n')
    binary_mesh=tmp_path/'b.stl'
    binary_mesh.write_bytes(bytes(80)+struct.pack('<I',1)+struct.pack('<12fH',0.,0.,1.,*(x for v in vertices for x in v),0))
    for mesh in (ascii_mesh,binary_mesh):
        urdf=URDF.replace('type="revolute"','type="prismatic"').replace('box size="0.6 0.06 0.06"',f'mesh filename="{mesh}" scale=".2 .3 .4"')
        compare(probe,urdf,{'arm':.4},{'arm':.01})

def test_cpp_invariant_wheel_is_geometrically_proven_and_attachment_rejects(probe):
    urdf=URDF.replace('</robot>','''<joint name="wheel" type="continuous"><parent link="base"/><child link="wheel_link"/><axis xyz="0 0 1"/></joint><link name="wheel_link"><collision><geometry><cylinder radius=".1" length=".06"/></geometry></collision></link></robot>''')
    compare(probe,urdf,{'arm':.2},{'arm':.003})
    payload=dict(link='wheel_link',kind='box',dimensions=[.1,.1,.1],pose=np.eye(4).tolist())
    with pytest.raises(ValueError,match='invariant joint'):
        probe(urdf=urdf,q={'arm':.2},errors={'arm':.003},attachments=[payload])

def test_cpp_package_mesh_uri_uses_injected_package_resolver(probe,tmp_path):
    mesh=tmp_path/'shape.obj';mesh.write_text('v -1 -.2 -.3\nv .2 .3 .4\nv .5 -.1 .2\n')
    template=URDF.replace('box size="0.6 0.06 0.06"','mesh filename="%s" scale=".2 .3 .4"')
    direct=probe(urdf=template%str(mesh),q={'arm':.2},errors={'arm':.003})
    packaged=probe(urdf=template%'package://fixture/shape.obj',packages={'fixture':str(tmp_path)},q={'arm':.2},errors={'arm':.003})
    assert packaged['physical']==direct['physical'] and packaged['reserved']==direct['reserved']
