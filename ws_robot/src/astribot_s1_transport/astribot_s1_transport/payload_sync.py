"""Simulation attachment identity and capture-time confirmation contract."""
import math
import xml.etree.ElementTree as ET


def physical_parent(urdf,tcp):
    """URDF-to-SDF reduces fixed joints; find the retained articulated link."""
    robot=ET.fromstring(urdf)
    links={link.attrib['name'] for link in robot.findall('link')}
    joints={joint.find('child').attrib['link']:joint for joint in robot.findall('joint')}
    if tcp not in links:raise ValueError('PAYLOAD_TCP_NOT_IN_MODEL')
    seen=set();link=tcp
    while link in joints and joints[link].attrib['type']=='fixed':
        if link in seen:raise ValueError('PAYLOAD_FIXED_CHAIN_CYCLE')
        seen.add(link);link=joints[link].find('parent').attrib['link']
        if link not in links:raise ValueError('PAYLOAD_PARENT_NOT_IN_MODEL')
    return link


def confirmed_state(state,command_id,attached,now,radius):
    if not isinstance(state,dict):return False
    try:
        age=now-state['stamp_ns']*1e-9
        position=state['position_error_m'];rotation=state['rotation_error_rad']
        return (isinstance(state['stamp_ns'],int) and state['stamp_ns']>0 and
            state['command_id']==command_id and state['attached'] is attached and not state['error'] and
            all(math.isfinite(v) for v in (age,position,rotation,radius)) and
            0<=age<=.3 and position>=0 and rotation>=0 and radius>0 and
            position+radius*rotation<=.006)
    except (KeyError,TypeError,ValueError):return False
