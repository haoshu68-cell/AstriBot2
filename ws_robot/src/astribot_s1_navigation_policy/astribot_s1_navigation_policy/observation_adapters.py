"""Replaceable observation adapters; no motion or planning authority."""
import json
import math
import time
import importlib
from rclpy.time import Time
from .contracts import BearingCone, Covariance3, ImageBox, MetricBox, Observation, Stamp, Vec3


def rotate(v,q):
    tx=2*(q.y*v.z-q.z*v.y);ty=2*(q.z*v.x-q.x*v.z);tz=2*(q.x*v.y-q.y*v.x)
    return Vec3(v.x+q.w*tx+q.y*tz-q.z*ty,v.y+q.w*ty+q.z*tx-q.x*tz,v.z+q.w*tz+q.x*ty-q.y*tx)


class VisionAdapter:
    def __init__(self, profile, tf, now, options=None):
        self.profile=profile;self.tf=tf;self.now=now;self.options=options or {}
        self.last_packet=None

    @staticmethod
    def point(xyz,transform):
        v=rotate(Vec3(*xyz),transform.transform.rotation);t=transform.transform.translation
        return v.x+t.x,v.y+t.y,v.z+t.z

    def normalize(self, packet):
        if len(packet.data)>int(self.options.get('max_packet_bytes',1048576)):raise ValueError('packet over byte budget')
        data=json.loads(packet.data)
        if len(data.get('observations',[]))>int(self.options.get('max_observations',128)):raise ValueError('observation budget exceeded')
        now=self.now();self.clock_id=now.clock;self.epoch=now.epoch
        source=self.options.get('sensor_id')
        if source and data.get('sensor_id')!=source:raise ValueError('sensor identity mismatch')
        self.last_packet=data
        if data.get('schema_version')!=1:raise ValueError('vision schema_version')
        now=self.now();capture=Stamp(int(data['stamp_ns']),self.clock_id,self.epoch)
        observations=[]
        for item in data['observations']:
            frame=data['frame_id'];kind=item['kind']
            if kind=='metric_box':
                transform=self.tf.lookup_transform(self.profile.tracking_frame,frame,Time(nanoseconds=capture.ns))
                center=self.point(item['center_m'],transform);size=Vec3(*item['size_m']);q=transform.transform.rotation
                if min(size.x,size.y,size.z)<=0:raise ValueError('positive metric dimensions required')
                axes=[rotate(Vec3(size.x,0,0),q),rotate(Vec3(0,size.y,0),q),rotate(Vec3(0,0,size.z),q)]
                extent=Vec3(*[sum(abs(getattr(v,k)) for v in axes) for k in ('x','y','z')])
                # A scalar isotropic uncertainty is explicitly supplied, not inferred from class confidence.
                variance=float(item['position_variance_m2'])
                velocity=None;velocity_covariance=None
                if 'velocity_m_s' in item or 'velocity_variance_m2_s2' in item:
                    velocity=rotate(Vec3(*item['velocity_m_s']),q)
                    vv=float(item['velocity_variance_m2_s2'])
                    velocity_covariance=Covariance3((vv,0.,0.,0.,vv,0.,0.,0.,vv))
                geometry=MetricBox(Vec3(*center),extent,Covariance3((variance,0.,0.,0.,variance,0.,0.,0.,variance)),
                                   velocity,velocity_covariance)
                frame=self.profile.tracking_frame
            elif kind=='image_box':
                geometry=ImageBox(data['sensor_id'],*item['image_size_px'],*item['box_xyxy_px'])
            elif kind=='bearing_cone':geometry=BearingCone(Vec3(*item['direction']),item['half_angle_rad'])
            else:raise ValueError('unsupported vision geometry')
            observations.append(Observation(data['sensor_id'],item['measurement_id'],item.get('track_id'),capture,
                Stamp(time.monotonic_ns(),'steady',0),Stamp(capture.ns+int(self.profile.sensor_timeout_s*1e9),self.clock_id,self.epoch),
                frame,int(data['calibration_epoch']),geometry,float(item['geometry_quality']),
                tuple((str(k),float(v)) for k,v in item.get('classes',{}).items()),tuple(item['provenance']),
                velocity_observable=bool(item.get('track_id'))))
        return tuple(observations)


class PointCloudBoxAdapter(VisionAdapter):
    """Bounded conservative box from an externally segmented obstacle cloud.

    Ground removal and segmentation belong to perception. This adapter does not
    interpret an empty cloud as proof of free space.
    """
    def normalize(self, packet):
        from sensor_msgs_py.point_cloud2 import read_points
        from std_msgs.msg import String
        limit=int(self.options.get('max_points',32768))
        if not 0<packet.width*packet.height<=limit:raise ValueError('cloud empty or over point budget')
        points=list(read_points(packet,field_names=('x','y','z'),skip_nans=True))
        if not points:raise ValueError('no finite depth')
        bounds=[(min(float(p[i]) for p in points),max(float(p[i]) for p in points)) for i in range(3)]
        ns=packet.header.stamp.sec*10**9+packet.header.stamp.nanosec
        sensor=self.options['sensor_id']
        data={'schema_version':1,'sensor_id':sensor,'stamp_ns':ns,'frame_id':packet.header.frame_id,
              'calibration_epoch':int(self.options.get('calibration_epoch',0)),
              'observations':[{'kind':'metric_box','measurement_id':str(ns),
                 'center_m':[(a+b)/2 for a,b in bounds],
                 'size_m':[max(.02,b-a) for a,b in bounds],
                 'position_variance_m2':float(self.options.get('position_variance_m2',.0025)),
                 'geometry_quality':1.,'provenance':[f'{sensor}:{ns}']}]}
        return super().normalize(String(data=json.dumps(data)))


_BUILTINS={'vision_json':VisionAdapter,'pointcloud_boxes':PointCloudBoxAdapter}


def adapter_class(name):
    if name in _BUILTINS:return _BUILTINS[name]
    module,attribute=name.split(':',1)
    return getattr(importlib.import_module(module),attribute)


def make_adapter(name,profile,tf,now,options):
    return adapter_class(name)(profile,tf,now,options)


def adapter_message_type(name):
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    if name=='pointcloud_boxes':return PointCloud2
    if name=='vision_json':return String
    return adapter_class(name).message_type
