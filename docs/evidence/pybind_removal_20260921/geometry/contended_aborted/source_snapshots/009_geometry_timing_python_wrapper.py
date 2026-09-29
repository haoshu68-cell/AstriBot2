#!/usr/bin/env python3
"""Diagnostic instrumentation only; execute the frozen original Python node."""
import json
import sys
import time
from types import SimpleNamespace
from astribot_s1_robot_geometry.node import GeometryNode, main, ns


def trace(event,source,ros):
    print('GEOMETRY_TIMING '+json.dumps(dict(event=event,source_ns=source,ros_ns=ros,steady_ns=time.monotonic_ns())),file=sys.stderr,flush=True)


original_init=GeometryNode.__init__
original_tick=GeometryNode.tick
original_calculate=GeometryNode.calculate


def init(self):
    original_init(self)
    original_publish=self.pub.publish
    original_submit=self.worker.submit
    def publish(message):
        if message.complete:trace('publish',ns(message.header.stamp),ns(message.published_at))
        return original_publish(message)
    def submit(function,*args,**kwargs):
        trace('submit',ns(args[-1].header.stamp),self.get_clock().now().nanoseconds)
        return original_submit(function,*args,**kwargs)
    self.pub=SimpleNamespace(publish=publish)
    self.worker.submit=submit


def calculate(*args):
    source=ns(args[-1].header.stamp);trace('compute_begin',source,-1)
    result=original_calculate(*args);trace('compute_end',source,-1);return result


def tick(self):
    trace('tick',0,self.get_clock().now().nanoseconds)
    return original_tick(self)


GeometryNode.__init__=init
GeometryNode.tick=tick
GeometryNode.calculate=staticmethod(calculate)
main()
