from types import SimpleNamespace as NS
from astribot_s1_transport.source_inbox import SourceInbox
from astribot_s1_robot_geometry._geometry_native import select_source_sample
import pytest


def message(ns):
    return NS(header=NS(stamp=NS(sec=ns//10**9,nanosec=ns%10**9)))


def test_clock_delivery_reordering_retains_valid_acquisition():
    inbox=SourceInbox(.5);old=message(980_000_000);future=message(1_016_000_000)
    inbox.receive(old);inbox.receive(future)
    assert inbox.select(1_000_000_000) is old
    assert inbox.select(1_006_000_000) is future
    assert inbox.select(1_516_000_000) is future
    assert inbox.select(1_516_000_001) is None


def test_future_stream_cannot_extend_age_or_relabel_stamp():
    inbox=SourceInbox(.3);old=message(1_000_000_000)
    inbox.receive(old);inbox.receive(message(9_000_000_000))
    assert inbox.select(1_300_000_000) is old
    assert inbox.select(1_300_000_001) is None
    assert old.header.stamp.sec==1 and old.header.stamp.nanosec==0


def test_out_of_order_and_capacity_fail_closed():
    inbox=SourceInbox(.3)
    newest=message(1_000_000_000);inbox.receive(newest);inbox.receive(message(990_000_000))
    assert inbox.select(1_010_000_000) is newest
    for i in range(100):inbox.receive(message(10_000_000_000+i))
    assert len(inbox.messages)==64 and inbox.select(1_010_000_000) is None


def test_exact_boundaries_and_zero_source():
    assert select_source_sample([0,-1],0,300_000_000)==-1
    assert select_source_sample([1_010_000_000],1_000_000_000,300_000_000)==0
    assert select_source_sample([1_010_000_001],1_000_000_000,300_000_000)==-1
    assert select_source_sample([700_000_000],1_000_000_000,300_000_000)==0
    assert select_source_sample([699_999_999],1_000_000_000,300_000_000)==-1
    with pytest.raises(ValueError):select_source_sample([1],1,300_000_000,10_000_001)
