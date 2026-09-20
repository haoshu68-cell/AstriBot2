"""Opt-in identities for concurrent copies of the canonical simulation world."""
from dataclasses import dataclass
from pathlib import Path
import re


def is_stack_process(arguments):
    """Match executable argv, never incidental text inside a tool shell."""
    if not arguments:return False
    executable=Path(arguments[0]).name
    if executable in ('controller_server','robot_state_publisher','gzserver','ign gazebo server'):
        return True
    return (executable.startswith('ruby') and len(arguments)>2 and
            Path(arguments[1]).name=='ign' and arguments[2]=='gazebo')


@dataclass(frozen=True)
class SimulationIsolation:
    instance: str = ''
    domain: int = 25

    def __post_init__(self):
        if self.instance:
            if not re.fullmatch(r'[a-zA-Z0-9][a-zA-Z0-9_-]{0,47}', self.instance):
                raise ValueError('instance must be 1-48 letters, digits, underscores or hyphens')
            if not 1 <= self.domain <= 101 or self.domain == 25:
                raise ValueError('isolated instance requires an explicit unused ROS domain in 1-101, excluding 25')
        elif self.domain != 25:
            raise ValueError('non-default domain requires --instance')

    @property
    def partition(self):
        return 'astribot_' + self.instance if self.instance else ''

    def environment(self):
        values={'ROS_DOMAIN_ID':str(self.domain),'ROS_LOCALHOST_ONLY':'1'}
        if self.instance:
            values.update(IGN_PARTITION=self.partition,GZ_PARTITION=self.partition,
                IGN_DISCOVERY_MSG_PORT=str(18000+2*self.domain),
                IGN_DISCOVERY_SRV_PORT=str(18001+2*self.domain),
                IGN_IP='127.0.0.1',GZ_IP='127.0.0.1',ASTRIBOT_SIM_INSTANCE=self.instance)
        return values

    def lock_paths(self):
        paths=[f'/tmp/astribot_sim_domain{self.domain}.lock']
        if self.instance:paths.append(f'/tmp/astribot_sim_partition_{self.partition}.lock')
        return paths

    def conflict(self,env):
        if env.get('ROS_DOMAIN_ID','0')==str(self.domain):return 'ROS_DOMAIN_IN_USE'
        if self.instance:
            if self.partition in (env.get('IGN_PARTITION'),env.get('GZ_PARTITION')):
                return 'GAZEBO_PARTITION_IN_USE'
            for key in ('IGN_DISCOVERY_MSG_PORT','IGN_DISCOVERY_SRV_PORT'):
                if env.get(key)==self.environment()[key]:return 'GAZEBO_DISCOVERY_PORT_IN_USE'
        return ''
