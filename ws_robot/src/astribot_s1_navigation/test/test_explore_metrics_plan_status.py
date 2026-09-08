"""规划 action 状态台账的单测。

这一列（planning_time）此前**整列都是退化口径** —— 因为没有任何代码往
plan_requests 里写东西，主路径是死代码，而落盘的那一列还带着
'fallback:' 前缀，看起来像是"偶尔退化"。这里的用例覆盖三个真实坑：
累积话题的重复状态、TRANSIENT_LOCAL 锁存的旧状态、台账无界增长。
"""

import pytest

from astribot_s1_navigation.explore_metrics import plan_status as ps


class TestStatusCodes:

    def test_matches_upstream_action_msgs(self):
        """抄的常量必须与真 GoalStatus 逐条相等。

        状态码错位会让 ABORTED 被数成 SUCCEEDED，而表里每一行都正常。
        """
        GoalStatus = pytest.importorskip(
            'action_msgs.msg', reason='需要 action_msgs 才能核对上游状态码'
        ).GoalStatus
        assert ps.verify_status_codes(GoalStatus) == []

    def test_verify_reports_every_mismatch_not_just_the_first(self):
        class Wrong:
            STATUS_UNKNOWN = 0
            STATUS_ACCEPTED = 9          # 错
            STATUS_EXECUTING = 2
            STATUS_CANCELING = 3
            STATUS_SUCCEEDED = 4
            STATUS_CANCELED = 5
            STATUS_ABORTED = 7           # 也错
        bad = ps.verify_status_codes(Wrong)
        assert len(bad) == 2, bad
        assert any('ACCEPTED' in b for b in bad)
        assert any('ABORTED' in b for b in bad)

    def test_missing_constant_is_a_mismatch_not_a_crash(self):
        class Empty:
            pass
        assert len(ps.verify_status_codes(Empty)) == 7

    def test_canceled_is_terminal_but_not_a_planning_failure(self):
        """CANCELED 要收尾，但不能算规划失败。

        round_metrics 只按 'ABORTED' 数失败率；把取消混进去，抢占频繁的
        跑次会凭空多出一堆"规划失败"。
        """
        assert ps.STATUS_CANCELED in ps.TERMINAL
        assert ps.STATUS_NAMES[ps.STATUS_CANCELED] == 'CANCELED'
        assert ps.STATUS_NAMES[ps.STATUS_CANCELED] != 'ABORTED'


class TestLedger:

    def test_accept_comes_from_the_message_not_the_receipt_time(self):
        """受理时刻取 goal_info.stamp，不是收报时刻。

        话题是 TRANSIENT_LOCAL：一连上就会收到一条锁存的旧状态。拿收报
        时刻当受理时刻，那条旧请求会被算成"刚刚发生"、落进当时那一轮 ——
        这正是把陈旧读数当成当前值。
        """
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=100.0, now=137.0)
        assert led.entries[0]['accept'] == 100.0
        assert led.stamp_fallbacks == 0

    def test_zero_stamp_falls_back_to_receipt_and_is_counted(self):
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_ACCEPTED, stamp=0.0, now=12.5)
        led.observe(b'g2', ps.STATUS_ACCEPTED, stamp=None, now=13.5)
        assert [e['accept'] for e in led.entries] == [12.5, 13.5]
        assert led.stamp_fallbacks == 2, '口径退化必须记数，不能静默'

    def test_repeated_status_does_not_create_duplicates(self):
        """这个话题是累积的：同一个 goal_id 会被反复看到。"""
        led = ps.PlanRequestLedger()
        for _ in range(20):
            led.observe(b'g1', ps.STATUS_EXECUTING, stamp=5.0, now=6.0)
        assert len(led) == 1

    def test_terminal_records_end_and_status(self):
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=5.0, now=5.1)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=5.0, now=5.4)
        e = led.entries[0]
        assert e['end'] == 5.4
        assert e['status'] == 'SUCCEEDED'
        assert e['end'] - e['accept'] == pytest.approx(0.4)

    def test_first_terminal_wins_so_duration_is_not_stretched(self):
        """终态只记第一次。累积话题会把终态一直重播，每次都覆盖 end
        的话，耗时会随重播时长无限变长。"""
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=5.0, now=5.1)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=5.0, now=5.4)
        for now in (6.0, 30.0, 300.0):
            led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=5.0, now=now)
        assert led.entries[0]['end'] == 5.4

    def test_small_future_stamp_within_tolerance_is_recorded_as_is(self):
        """容差内的"消息时刻略晚于收报时刻"照实记，不在本层夹 max()。

        夹掉之后那条异常就再也看不见了。超出容差的才由未来判据退回 now
        （见 TestClockSkewGuard）；容差内留给 round_metrics 的
        end >= accept 过滤兜住（见 test_explore_metrics_round.py 的
        test_request_with_end_before_accept_is_dropped）。
        """
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=10.5, now=10.0)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=10.5, now=10.2)
        e = led.entries[0]
        assert e['accept'] == 10.5 and e['end'] == 10.2
        assert led.stamp_in_future == 0, '0.5s 在 1.0s 容差内，不该计数'

    def test_window_filters_by_accept_time(self):
        led = ps.PlanRequestLedger()
        led.observe(b'old', ps.STATUS_SUCCEEDED, stamp=1.0, now=1.2)
        led.observe(b'mine', ps.STATUS_SUCCEEDED, stamp=10.0, now=10.3)
        led.observe(b'later', ps.STATUS_SUCCEEDED, stamp=99.0, now=99.1)
        got = led.window(8.0, 20.0)
        assert len(got) == 1 and got[0]['accept'] == 10.0

    def test_window_boundaries_are_inclusive(self):
        led = ps.PlanRequestLedger()
        led.observe(b'lo', ps.STATUS_ACCEPTED, stamp=8.0, now=8.0)
        led.observe(b'hi', ps.STATUS_ACCEPTED, stamp=20.0, now=20.0)
        assert len(led.window(8.0, 20.0)) == 2

    def test_window_returns_copies_not_live_entries(self):
        """已经落盘的那一行不能被之后写入的终态改掉。"""
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=5.0, now=5.1)
        snapshot = led.window(0.0, 100.0)
        led.observe(b'g1', ps.STATUS_ABORTED, stamp=5.0, now=7.0)
        assert snapshot[0]['end'] is None, '落盘快照被事后改写了'
        assert led.entries[0]['end'] == 7.0

    def test_pruning_bounds_both_containers(self):
        """无界增长会拖垮长跑；只裁列表不裁字典等于没裁。"""
        led = ps.PlanRequestLedger(keep=10)
        for i in range(500):
            led.observe(('g%d' % i).encode(), ps.STATUS_SUCCEEDED,
                        stamp=float(i), now=float(i) + 0.1)
        assert len(led.entries) == 10
        assert len(led.by_id) == 10, 'by_id 没跟着裁，字典自己无界增长'
        assert [e['accept'] for e in led.entries] == [float(i)
                                                      for i in range(490, 500)]

    def test_pruned_id_reappearing_is_treated_as_new(self):
        """被裁掉的 id 再出现时必须当成新条目。

        不从 by_id 摘掉的话，它会被当成"已见过"，那条请求的终态就永远
        写不进去 —— 表现为规划耗时缺格，而不是报错。
        """
        led = ps.PlanRequestLedger(keep=2)
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=1.0, now=1.1)
        led.observe(b'g2', ps.STATUS_EXECUTING, stamp=2.0, now=2.1)
        led.observe(b'g3', ps.STATUS_EXECUTING, stamp=3.0, now=3.1)
        assert b'g1' not in led.by_id
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=9.0, now=9.2)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=9.0, now=9.5)
        again = [e for e in led.entries if e['accept'] == 9.0]
        assert len(again) == 1, '被裁掉的 id 再出现要落成一条新条目'
        assert again[0]['end'] == 9.5, (
            '没从 by_id 摘掉的话，终态会写到已出列的旧条目上，entries 里看不到')

    def test_keep_must_be_positive(self):
        with pytest.raises(ValueError):
            ps.PlanRequestLedger(keep=0)

    def test_skew_must_be_positive(self):
        with pytest.raises(ValueError):
            ps.PlanRequestLedger(max_future_skew_sec=0.0)


class TestClockSkewGuard:
    """accept 来自消息、end 来自收报时刻 —— 两个时钟。

    🔴 判据只能是"受理时刻落在**未来**"，不能是"差得远"。
    我第一版写 |stamp-now|>300s，实测把 192/192 条全判成不同轴 —— 而
    直接探针证明 stamp 与节点时钟本来同轴（都是仿真时间，1.861~543.647s
    vs now=1905.4s），它们只是**旧**：探索早已 PAUSED。锁存话题必然给
    历史状态，旧是合法的，归轮时按 window() 筛掉即可。
    """

    def test_stamp_in_the_future_is_caught(self):
        """反向配错（本节点仿真时钟、消息带墙钟）才是这里能抓的。"""
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=1.788e9, now=42.0)
        assert led.stamp_in_future == 1
        assert led.stamp_fallbacks == 0, '这不是"上游没填"，别混进那个计数'
        assert led.entries[0]['accept'] == 42.0, '不同轴时要退回同轴的收报时刻'

    def test_duration_stays_plausible_when_clocks_disagree(self):
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=1.788e9, now=42.0)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=1.788e9, now=42.35)
        e = led.entries[0]
        assert e['end'] - e['accept'] == pytest.approx(0.35), (
            '不同轴的两个时刻相减，落盘的就是天文数字')

    def test_stale_but_same_axis_stamp_is_kept_as_is(self):
        """旧 ≠ 不同轴。实测的 543.6s vs now=1905.4s 必须原样保留。"""
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=543.647, now=1905.428)
        assert led.stamp_in_future == 0, (
            '把"旧"判成"不同轴"，就是我犯过的那个 192/192 全误判')
        assert led.entries[0]['accept'] == pytest.approx(543.647)

    def test_normal_small_lag_is_not_flagged(self):
        """同轴时的正常滞后（收报比消息晚几十毫秒）不能被当成不同轴。"""
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=100.0, now=100.04)
        assert led.stamp_in_future == 0
        assert led.entries[0]['accept'] == 100.0

    def test_threshold_is_configurable_and_respected(self):
        led = ps.PlanRequestLedger(max_future_skew_sec=1.0)
        led.observe(b'a', ps.STATUS_ACCEPTED, stamp=10.9, now=10.0)
        led.observe(b'b', ps.STATUS_ACCEPTED, stamp=11.5, now=10.0)
        assert led.stamp_in_future == 1, '0.9s 在容差内，1.5s 超出'
        assert led.by_id[b'a']['accept'] == pytest.approx(10.9)
        assert led.by_id[b'b']['accept'] == 10.0


class TestTerminalOnFirstSight:
    """第一眼就是终态：状态可信，耗时不可知。

    锁存话题订阅上来的第一帧全是这种条目（实测 192 条）。若照常
    end=now，耗时就是"我此刻才知道"减"它很久以前受理"—— 凭空造的数。
    """

    def test_first_sight_terminal_has_status_but_no_duration(self):
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_ABORTED, stamp=543.647, now=1905.428)
        e = led.entries[0]
        assert e['status'] == 'ABORTED', '状态码是可信的，要留着数失败率'
        assert e['accept'] == pytest.approx(543.647), '受理时刻照实记'
        assert e['end'] is None, (
            '1905.428-543.647=1361.8s 不是规划耗时，是我们订阅得晚')
        assert led.terminal_on_first_sight == 1

    def test_observed_running_then_terminal_does_get_a_duration(self):
        led = ps.PlanRequestLedger()
        led.observe(b'g1', ps.STATUS_EXECUTING, stamp=100.0, now=100.02)
        led.observe(b'g1', ps.STATUS_SUCCEEDED, stamp=100.0, now=100.44)
        e = led.entries[0]
        assert e['end'] == pytest.approx(100.44)
        assert e['end'] - e['accept'] == pytest.approx(0.44)
        assert led.terminal_on_first_sight == 0


class TestCumulativeFrameLongerThanKeep:
    """实测坑：每帧 status_list 长 450，而 keep 曾是 400。

    keep 比一帧还短 ⇒ 每来一帧都把上一帧还在的条目裁掉，下一帧同样的
    goal_id 又被当成新条目重建。实测 46 条消息造出 20069 条"第一眼终态"
    （≈436/帧），台账里同一目标重复多份，归轮的请求数与失败数放大数百倍。
    """

    @staticmethod
    def _frame(n, base=100.0):
        return [(b'g%03d' % i, ps.STATUS_SUCCEEDED, base + i * 0.01)
                for i in range(n)]

    def test_repeated_frame_does_not_recreate_entries(self):
        led = ps.PlanRequestLedger(keep=10)
        frame = self._frame(50)
        for _ in range(5):
            led.observe_batch(frame, now=200.0)
        assert led.terminal_on_first_sight == 50, (
            '同一帧重复五次只该有 50 个目标；重建的话会是 50×5')
        assert len(led) == 50, '台账里不该出现同一目标的多份副本'

    def test_window_does_not_double_count_a_repeated_frame(self):
        """归轮统计直接受害：重复条目会把请求数与失败数一起放大。"""
        led = ps.PlanRequestLedger(keep=10)
        frame = self._frame(30)
        for _ in range(4):
            led.observe_batch(frame, now=200.0)
        got = led.window(0.0, 1000.0)
        assert len(got) == 30, '窗口里 30 个目标被数成了 %d 个' % len(got)

    def test_keep_floor_follows_the_longest_frame_seen(self):
        led = ps.PlanRequestLedger(keep=10)
        led.observe_batch(self._frame(50), now=200.0)
        assert led.keep_raised_to == 51, '下限要留一条余量，便于放进下一个新目标'

    def test_frames_shorter_than_keep_do_not_raise_the_floor(self):
        led = ps.PlanRequestLedger(keep=400)
        led.observe_batch(self._frame(5), now=200.0)
        assert led.keep_raised_to == 6

    def test_pruning_still_bounds_growth_across_distinct_frames(self):
        """下限抬高不等于不再裁剪 —— 无界增长是另一个 bug。"""
        led = ps.PlanRequestLedger(keep=10)
        for k in range(6):
            led.observe_batch(
                [(b'f%d-%03d' % (k, i), ps.STATUS_SUCCEEDED, 100.0 + i)
                 for i in range(20)], now=200.0 + k)
        assert len(led) <= 21, '每帧 20 条，台账应稳定在一帧多一点，实际 %d' % len(led)
        assert len(led.by_id) == len(led), 'by_id 必须跟着 entries 一起裁'
