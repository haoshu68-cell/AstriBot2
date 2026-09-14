"""规划 action 状态的台账。

规划耗时这一列的唯一正经口径是 `/compute_path_to_pose/_action/status`
（action_msgs/GoalStatusArray）。这个模块只做纯逻辑：喂进
(goal_id, status_code, stamp) 三元组，出 accept/end/终态的台账条目。

为什么不直接在节点里做：
  · explore_metrics/ 整个包刻意不依赖 rclpy —— 事后离线复算能在任何
    装了 numpy 的机器上跑，不必有 ROS。
  · 这段逻辑有三个容易错的地方（累积话题去重、锁存旧状态、台账无界增长），
    每一个都必须能单测到；要起一个真 Node 才能测的逻辑，实际上就是测不到。
"""

STATUS_UNKNOWN = 0
STATUS_ACCEPTED = 1
STATUS_EXECUTING = 2
STATUS_CANCELING = 3
STATUS_SUCCEEDED = 4
STATUS_CANCELED = 5
STATUS_ABORTED = 6

STATUS_NAMES = {
    STATUS_UNKNOWN: 'UNKNOWN',
    STATUS_ACCEPTED: 'ACCEPTED',
    STATUS_EXECUTING: 'EXECUTING',
    STATUS_CANCELING: 'CANCELING',
    STATUS_SUCCEEDED: 'SUCCEEDED',
    STATUS_CANCELED: 'CANCELED',
    STATUS_ABORTED: 'ABORTED',
}

TERMINAL = (STATUS_SUCCEEDED, STATUS_CANCELED, STATUS_ABORTED)


def verify_status_codes(goal_status_cls):
    """拿真 action_msgs.msg.GoalStatus 核对本模块抄的常量。

    返回不一致的项（空列表 = 一致）。节点侧不一致就拒绝启动：状态码错位
    会让 ABORTED 被数成 SUCCEEDED，而那张表每一行看起来都正常。
    """
    expect = {
        'STATUS_UNKNOWN': STATUS_UNKNOWN,
        'STATUS_ACCEPTED': STATUS_ACCEPTED,
        'STATUS_EXECUTING': STATUS_EXECUTING,
        'STATUS_CANCELING': STATUS_CANCELING,
        'STATUS_SUCCEEDED': STATUS_SUCCEEDED,
        'STATUS_CANCELED': STATUS_CANCELED,
        'STATUS_ABORTED': STATUS_ABORTED,
    }
    bad = []
    for name, mine in expect.items():
        theirs = getattr(goal_status_cls, name, None)
        if theirs != mine:
            bad.append('%s: 本模块=%s 上游=%s' % (name, mine, theirs))
    return bad


class PlanRequestLedger:
    """按 goal_id 去重的规划请求台账。

    条目形状与 round_metrics 约定一致：{'accept': float, 'end': float|None,
    'status': str|None}。accept 用**消息自带的** goal_info.stamp，end 只能用
    收报时刻（消息里没有终态发生时刻这个字段）。
    """

    def __init__(self, keep=400, max_future_skew_sec=1.0):
        if keep < 1:
            raise ValueError('keep 必须 >= 1，拿到 %r' % (keep,))
        if max_future_skew_sec <= 0.0:
            raise ValueError('max_future_skew_sec 必须 > 0，拿到 %r'
                             % (max_future_skew_sec,))
        self.keep = int(keep)
        self.max_future_skew_sec = float(max_future_skew_sec)
        self.entries = []
        self.by_id = {}
        self.stamp_fallbacks = 0
        self.stamp_in_future = 0
        self.terminal_on_first_sight = 0
        self._min_keep = 0
        self.keep_raised_to = None

    def __len__(self):
        return len(self.entries)

    def observe(self, goal_id, status, stamp, now):
        """看到一条状态。

        goal_id: 可哈希的目标标识（节点侧传 uuid 的 bytes）。
        status:  状态码。
        stamp:   消息里的 goal_info.stamp，秒；<=0 或 None 视为"上游没填"。
        now:     收报时刻，秒。

        同一个 goal_id 会被反复看到 —— 这个话题是**累积**的，nav2 把最近
        若干个目标一直挂在 status_list 里。所以只记第一次受理、第一次终态。
        """
        entry = self.by_id.get(goal_id)
        fresh = entry is None
        if fresh:
            accept = stamp
            if accept is None or accept <= 0.0:
                accept = now
                self.stamp_fallbacks += 1
            elif accept > now + self.max_future_skew_sec:
                accept = now
                self.stamp_in_future += 1
            entry = {'accept': float(accept), 'end': None, 'status': None}
            self.by_id[goal_id] = entry
            self.entries.append(entry)
            self._prune()
        if status in TERMINAL and entry['end'] is None:
            entry['status'] = STATUS_NAMES.get(status, str(status))
            if fresh:
                self.terminal_on_first_sight += 1
            else:
                entry['end'] = float(now)
        return entry

    def newest_accept(self):
        """台账里最新的受理时刻；空台账返回 None。

        用来回答"规划器最近一次被问是多久以前"。实测过一次
        newest=543.6s 而当时 now=1905.4s —— 距今 1362s，因为探索早已
        PAUSED（rejected=120、validate_fail=4/4）。这个读数能一眼看出
        "台账里全是历史"，而不必去猜时间轴对不对。
        """
        return max((e['accept'] for e in self.entries), default=None)

    def window(self, lo, hi):
        """取受理时刻落在 [lo, hi] 闭区间内的条目，返回**副本**。

        返回副本而不是内部条目：调用方是归轮统计，拿到后会自己加字段，
        写回内部条目会污染下一轮的台账。

        台账里的条目"旧"是合法的 —— 这个话题是 TRANSIENT_LOCAL 锁存的，
        订阅上来第一帧就带着历史全量。按时间窗筛掉即可，不要在 observe()
        里以"太旧"为由丢弃（见 [[frozen-counter-read-as-current-value]]）。
        """
        return [dict(e) for e in self.entries
                if e['accept'] is not None and lo <= e['accept'] <= hi]

    def observe_batch(self, items, now):
        """看到一整条 GoalStatusArray 消息。items: (goal_id, status, stamp) 序列。

        🔴 必须按"整帧"喂，不能只逐条 observe()。这个话题是**累积**的：
        实测每帧 status_list 长度 450（450 个 goal_id 全不重复）。keep=400
        比一帧还短 ⇒ 每来一帧都把上一帧还在的条目裁掉，下一帧同样的
        goal_id 又被当成新条目重建。实测 46 条消息造出 **20069** 条
        "第一眼终态"（≈436/帧），台账里同一个目标重复多份，归轮时的
        请求数与失败数都会被放大好几百倍。

        所以裁剪下限跟着实测帧长走：见过多长的帧，就至少留那么长。
        """
        items = list(items)
        need = len(items) + 1
        if need > self._min_keep:
            self._min_keep = need
            self.keep_raised_to = need
        for goal_id, status, stamp in items:
            self.observe(goal_id, status, stamp, now)
        return len(items)

    def _prune(self):
        """只留最近 max(keep, 见过的最长一帧) 条。

        丢的时候必须**同时**从 by_id 摘掉：不摘，那个字典自己无界增长，
        而且被摘的旧 id 再出现时会被当成"已见过"，从而丢掉它的终态。

        下限不能低于一帧的长度，否则退化成"每帧全部重建"（见
        observe_batch 的实测数字）。
        """
        excess = len(self.entries) - max(self.keep, self._min_keep)
        if excess <= 0:
            return
        del self.entries[:excess]
        live = {id(e) for e in self.entries}
        for gid in [k for k, v in self.by_id.items() if id(v) not in live]:
            del self.by_id[gid]
