"""URDF FK and conservative supports for boxes, cylinders, spheres and mesh AABBs."""
import hashlib
import itertools
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
from .polygon import hull, inflate


def origin(element):
    result=np.eye(4)
    if element is not None:
        result[:3,3]=list(map(float,element.get('xyz','0 0 0').split()))
        result[:3,:3]=Rotation.from_euler('xyz',list(map(float,element.get('rpy','0 0 0').split()))).as_matrix()
    if not np.isfinite(result).all():raise ValueError('nonfinite model origin')
    return result


def mesh_bounds(filename):
    if filename.startswith('package://'):
        from ament_index_python.packages import get_package_share_directory
        package,relative=filename[10:].split('/',1)
        filename=str(Path(get_package_share_directory(package))/relative)
    data=Path(filename).read_bytes()
    count=struct.unpack_from('<I',data,80)[0] if len(data)>=84 else 0
    if Path(filename).suffix.lower()=='.obj':
        rows=[]
        for line in data.decode().splitlines():
            fields=line.split('#',1)[0].split()
            if not fields or fields[0]!='v':continue
            if len(fields)<4:raise ValueError('invalid mesh vertex: '+filename)
            # Ignoring indented vertices can shrink the bounding box. Parse
            # every vertex; reject unsupported homogeneous coordinates rather
            # than silently interpreting a weighted vertex as Cartesian XYZ.
            if len(fields)==5 and float(fields[4])!=1.:
                raise ValueError('unsupported homogeneous mesh vertex: '+filename)
            rows.append(list(map(float,fields[1:4])))
        vertices=np.array(rows)
    elif count and len(data)==84+50*count:
        vertices=np.frombuffer(data,dtype=np.dtype([('n','<f4',(3,)),('v','<f4',(3,3)),('a','<u2')]),offset=84)['v'].reshape(-1,3)
    else:
        vertices=np.array([list(map(float,l.split()[1:])) for l in data.decode('ascii').splitlines() if l.strip().startswith('vertex ')])
    if vertices.ndim!=2 or vertices.shape[1]!=3 or not len(vertices) or not np.isfinite(vertices).all():
        raise ValueError('invalid mesh: '+filename)
    return vertices.min(0),vertices.max(0),hashlib.sha256(data).hexdigest()


class Shape:
    def __init__(self, link, kind, dimensions, pose):
        self.link,self.kind,self.dimensions,self.pose=link,kind,np.asarray(dimensions,dtype=float),np.asarray(pose)
        expected={'box':3,'sphere':1,'cylinder':2}
        if kind not in expected or len(self.dimensions)!=expected[kind] or not np.isfinite(self.dimensions).all() or np.any(self.dimensions<=0):
            raise ValueError('unsupported/invalid shape')
        if self.pose.shape!=(4,4) or not np.isfinite(self.pose).all():raise ValueError('invalid shape pose')

    @property
    def radius(self):
        if self.kind=='sphere':return self.dimensions[0]
        if self.kind=='cylinder':return math.hypot(self.dimensions[0],self.dimensions[1]/2)
        return np.linalg.norm(self.dimensions)/2

    def support(self, directions, transform):
        pose=transform@self.pose
        local=directions@pose[:3,:3]
        if self.kind=='box':extent=np.abs(local)@(self.dimensions/2)
        elif self.kind=='sphere':extent=self.dimensions[0]*np.linalg.norm(local,axis=1)
        else:extent=self.dimensions[0]*np.linalg.norm(local[:,:2],axis=1)+self.dimensions[1]/2*np.abs(local[:,2])
        return directions@pose[:3,3]+extent

    def polygon(self, transform, sides=32):
        pose=transform@self.pose
        if self.kind=='box':
            points=np.array(list(itertools.product((-1,1),repeat=3)))*self.dimensions/2
            return hull((points@pose[:3,:3].T+pose[:3,3])[:,:2])
        theta=np.arange(sides)*2*math.pi/sides
        directions=np.column_stack((np.cos(theta),np.sin(theta),np.zeros(sides)))
        supports=self.support(directions,transform)
        # Intersection of consecutive supporting half-planes circumscribes the
        # projection. In particular spheres never become a two-corner diagonal.
        return hull([np.linalg.solve(directions[[i,(i+1)%sides],:2],supports[[i,(i+1)%sides]]) for i in range(sides)])


class RobotModel:
    def __init__(self, urdf, base_frame='astribot_torso_base'):
        self.base_frame=base_frame;self.joints={};self.parents={};self.shapes=[];mesh_hashes=[]
        root=ET.fromstring(urdf)
        links={l.get('name') for l in root.findall('link')}
        if base_frame not in links:raise ValueError('missing base frame')
        for j in root.findall('joint'):
            name=j.attrib['name'];kind=j.attrib['type'];child=j.find('child').get('link')
            if kind not in ('fixed','revolute','continuous','prismatic'):raise ValueError('unsupported joint '+kind)
            axis=j.find('axis');axis=np.array(list(map(float,axis.get('xyz').split()))) if axis is not None else np.array([1.,0.,0.])
            if not np.isfinite(axis).all() or np.linalg.norm(axis)<1e-9:raise ValueError('invalid joint axis')
            mimic=j.find('mimic');limit=j.find('limit')
            self.joints[name]=dict(name=name,kind=kind,parent=j.find('parent').get('link'),child=child,
                origin=origin(j.find('origin')),axis=axis/np.linalg.norm(axis),
                mimic=(mimic.get('joint'),float(mimic.get('multiplier','1')),float(mimic.get('offset','0'))) if mimic is not None else None,
                limits=(float(limit.get('lower','-inf')),float(limit.get('upper','inf'))) if limit is not None and kind!='continuous' else (-math.inf,math.inf))
            if child in self.parents:raise ValueError('duplicate joint parent')
            self.parents[child]=name
        for link in root.findall('link'):
            name=link.attrib['name']
            for collision in link.findall('collision'):
                pose=origin(collision.find('origin'));g=list(collision.find('geometry'))[0];kind=g.tag
                if kind=='box':dimensions=list(map(float,g.get('size').split()))
                elif kind=='sphere':dimensions=[float(g.get('radius'))]
                elif kind=='cylinder':dimensions=[float(g.get('radius')),float(g.get('length'))]
                elif kind=='mesh':
                    low,high,sha=mesh_bounds(g.get('filename'));mesh_hashes.append(sha)
                    scale=np.array(list(map(float,g.get('scale','1 1 1').split())))
                    pose[:3,3]+=pose[:3,:3]@((low+high)/2*scale)
                    kind='box';dimensions=(high-low)*np.abs(scale)
                else:raise ValueError('unsupported collision '+kind)
                self.chain(name)
                self.shapes.append(Shape(name,kind,dimensions,pose))
        if not self.shapes:raise ValueError('model has no collision geometry')
        # A rotating leaf with only coaxial spheres/cylinders has invariant
        # collision geometry (e.g. drive wheels). Prove this from geometry,
        # never from a joint-name allowlist; these coordinates need no hold.
        self.invariant_joints=set()
        for joint in self.joints.values():
            bodies=[s for s in self.shapes if s.link==joint['child']]
            leaf=not any(j['parent']==joint['child'] for j in self.joints.values())
            def invariant(shape):
                centered=np.linalg.norm(np.cross(shape.pose[:3,3],joint['axis']))<1e-10
                aligned=np.linalg.norm(np.cross(shape.pose[:3,2],joint['axis']))<1e-10
                return centered and (shape.kind=='sphere' or (shape.kind=='cylinder' and aligned))
            if leaf and bodies and joint['kind'] in ('continuous','revolute') and all(invariant(s) for s in bodies):
                self.invariant_joints.add(joint['name'])
                joint['geometry_invariant']=True
        required=set()
        def source(name,seen=()):
            if name in seen:raise ValueError('mimic cycle')
            j=self.joints[name]
            if j['mimic']:return source(j['mimic'][0],seen+(name,))
            return name
        for shape in self.shapes:
            for name in self.chain(shape.link):
                if self.joints[name]['kind']!='fixed' and name not in self.invariant_joints:required.add(source(name))
        self.required=tuple(sorted(required))
        self.revision=hashlib.sha256((urdf+'\n'+'\n'.join(mesh_hashes)).encode()).hexdigest()

    def chain(self, link):
        chain=[];seen=set()
        while link!=self.base_frame:
            if link in seen or link not in self.parents:raise ValueError('collision link not rooted at base: '+link)
            seen.add(link);name=self.parents[link];chain.append(name);link=self.joints[name]['parent']
        return list(reversed(chain))

    def joint_value(self, name, q, errors=False, seen=()):
        if name in seen:raise ValueError('mimic cycle')
        joint=self.joints[name]
        if joint['kind']=='fixed':return 0.
        if joint['mimic']:
            source,multiplier,offset=joint['mimic']
            value=self.joint_value(source,q,errors,seen+(name,))
            return abs(multiplier)*value if errors else multiplier*value+offset
        if name not in q or not math.isfinite(q[name]):raise ValueError('missing/nonfinite joint '+name)
        return q[name]

    def fk(self, q):
        cache={self.base_frame:np.eye(4)}
        def lookup(link):
            if link not in cache:
                j=self.joints[self.parents[link]];value=0. if j['name'] in self.invariant_joints else self.joint_value(j['name'],q)
                if not j['limits'][0]-.01<=value<=j['limits'][1]+.01:raise ValueError('joint outside model limit: '+j['name'])
                motion=np.eye(4)
                if j['kind'] in ('revolute','continuous'):motion[:3,:3]=Rotation.from_rotvec(j['axis']*value).as_matrix()
                elif j['kind']=='prismatic':motion[:3,3]=j['axis']*value
                cache[link]=lookup(j['parent'])@j['origin']@motion
            return cache[link]
        return lookup

    def error_radius(self, shape, q, errors):
        radius=shape.radius+np.linalg.norm(shape.pose[:3,3]);bound=0.
        for name in reversed(self.chain(shape.link)):
            j=self.joints[name];error=0. if name in self.invariant_joints else self.joint_value(name,errors,True)
            if not 0<=error<=.1:raise ValueError('invalid joint hold error')
            if j['kind']=='prismatic':
                bound+=error;radius+=abs(self.joint_value(name,q))+error
            elif j['kind'] in ('revolute','continuous'):bound+=2*radius*math.sin(error/2)
            radius+=np.linalg.norm(j['origin'][:3,3])
        return bound

    def geometry(self, q, errors, attachments=(), model_padding=.01, floor_rectangle=(.31,.31)):
        if not math.isfinite(model_padding) or not 0<=model_padding<=.1:raise ValueError('invalid model padding')
        for shape in attachments:
            if self.invariant_joints.intersection(self.chain(shape.link)):
                raise ValueError('attachment changes an invariant joint; rebuild collision model')
        lookup=self.fk(q);physical=[];reserved=[];bodies=[]
        for shape in list(self.shapes)+list(attachments):
            transform=lookup(shape.link)
            footprint=shape.polygon(transform)
            z=shape.support(np.array([[0.,0.,-1.],[0.,0.,1.]]),transform)
            error=model_padding+self.error_radius(shape,q,errors)
            region=inflate(footprint,error)
            physical.extend(footprint);reserved.extend(region)
            bodies.append(dict(link=shape.link,z_min=-z[0]-error,z_max=z[1]+error,footprint=region))
        x,y=floor_rectangle;floor=np.array([[-x,-y],[x,-y],[x,y],[-x,y]])
        physical.extend(floor);reserved.extend(inflate(floor,model_padding))
        # Each slice conservatively includes a body's complete XY projection if
        # that body intersects the slab. This is not a 3-D clearance certificate.
        z_min=min(b['z_min'] for b in bodies);z_max=max(b['z_max'] for b in bodies)
        edges=np.arange(math.floor(z_min/.25)*.25,math.ceil(z_max/.25)*.25+.125,.25)
        slices=[]
        for low,high in zip(edges,edges[1:]):
            parts=[b['footprint'] for b in bodies if b['z_min']<=high and b['z_max']>=low]
            if parts:slices.append(dict(z_min=float(low),z_max=float(high),footprint=hull(np.concatenate(parts))))
        return dict(physical=hull(physical),reserved=hull(reserved),height=float(z_max),z_min=float(z_min),slices=slices)
