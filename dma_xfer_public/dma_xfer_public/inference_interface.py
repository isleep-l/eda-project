#!/usr/bin/env python3
"""
dma_xfer_public - inference_interface.py

官方 InferenceInterface 骨架（对齐大赛框架 4.6 节 / 附件 B.3）：

    class InferenceInterface:
        def __init__(self, dut_spec_path: str, covergroup_path: str)
        def predict(self, coverage_state: np.ndarray,
                    step: int, max_steps: int) -> np.ndarray

评测闭环（4.3.4）：基础镜像内置两种基线供参赛者参考
  1. 纯随机激励生成（policy="random"，默认）
  2. 简单贪心搜索（policy="greedy"）：固定"编程-启动"任务池，逐个通道/配置
     序列重复，命中新 bin 的任务保留复用

动作空间（15 维，见 dut_spec.md §6）：
  [ch_sel, conf_wr, conf_field, d0, d1, d2, d3, start, pad0..pad6]
  - ch_sel 0..3；conf_field 0=saddr 1=daddr 2={burst,dmode,len}（packed）
  - d0..d3 小端拼 32-bit；start 按通道位掩码上升沿启动传输
  - 典型编程序列：写 saddr -> 写 daddr -> 写 packed -> start 上升沿

与 harness.py 集成：
  h = DmaXferHarness()
  state = h.reset()
  for step in range(max_steps):
      action = inference.predict(state, step, max_steps)   # np.float32, shape (15,)
      state, reward, done, info = h.step(action)
"""

import json
import os
from collections import deque

import numpy as np


class InferenceInterface:
    DIMS = 15
    # 每维上界（np.float32，必须用 float32 可精确表示的值）
    # [ch_sel, conf_wr, conf_field, d0..d3, start, pad0..pad6]
    BOUNDS = np.array([4, 2, 3, 256, 256, 256, 256, 2,
                       1, 1, 1, 1, 1, 1, 1], dtype=np.float32)
    _REPEAT = 8
    _PERTURB = [(3, 8), (4, 8), (5, 8), (6, 8)]  # 对 d0..d3 小幅扰动

    # ------------------------------------------------------------------ #
    # 确定性任务池：每任务 = (ch, saddr, daddr, packed)
    # packed = len[15:0] | dmode[20:16] | burst[23:21]（bit3 为方向位）
    # 地址取 2 的幂或小值，保证 float32 可精确表示。
    # ------------------------------------------------------------------ #
    _TASKS = [
        (0, 0, 0, 0x00001), (0, 0, 0, 0x10001), (0, 0, 0, 0x02100),
        (0, 0, 0, 0x10000), (0, 0, 0, 0x1FFFF), (0, 0, 0, 0x120000),
        (0, 0, 0, 0x120001), (0, 0, 0, 0x120100), (0, 0, 0, 0x100001),
        (1, 0, 0, 0x00001), (1, 0, 0, 0x10001), (1, 0, 0, 0x02100),
        (1, 0, 0, 0x10000), (1, 0, 0, 0x1FFFF), (1, 0, 0, 0x120000),
        (1, 0, 0, 0x120001), (1, 0, 0, 0x120100), (1, 0, 0, 0x100001),
        (2, 0, 0, 0x00001), (2, 0, 0, 0x10001), (2, 0, 0, 0x02100),
        (2, 0, 0, 0x10000), (2, 0, 0, 0x1FFFF), (2, 0, 0, 0x120000),
        (2, 0, 0, 0x120001), (2, 0, 0, 0x120100), (2, 0, 0, 0x100001),
        (3, 0, 0, 0x00001), (3, 0, 0, 0x10001), (3, 0, 0, 0x02100),
        (3, 0, 0, 0x10000), (3, 0, 0, 0x1FFFF), (3, 0, 0, 0x120000),
        (3, 0, 0, 0x120001), (3, 0, 0, 0x120100), (3, 0, 0, 0x100001),
    ]

    def __init__(self, dut_spec_path=None, covergroup_path=None,
                 policy="random", seed=7):
        self.dut_spec_path = dut_spec_path
        self.covergroup_path = covergroup_path
        self.policy = policy
        self.rng = np.random.RandomState(seed)
        self._total_bins = self._read_total_bins()
        self.reset()

    # ------------------------------------------------------------------ #
    # 初始化辅助
    # ------------------------------------------------------------------ #
    def _read_total_bins(self):
        """从 covergroup 同目录的 coverage_meta.json 读 total_bins；
        读不到则退化为 DIMS（仅供自检占位，不影响接口契约）。"""
        if self.covergroup_path:
            meta = os.path.join(
                os.path.dirname(os.path.abspath(self.covergroup_path)),
                "coverage_meta.json")
            if os.path.exists(meta):
                try:
                    with open(meta, encoding="utf-8") as f:
                        n = int(json.load(f).get("total_bins", 0))
                    if n > 0:
                        return n
                except Exception:
                    pass
        return self.DIMS

    @property
    def total_bins(self):
        return self._total_bins

    def reset(self):
        self._tasks = deque(self._TASKS)
        self._phase = 0    # 0=saddr 1=daddr 2=packed 3=start(上升沿) 4=评估
        self._n = 0
        self._begin = 0

    # ------------------------------------------------------------------ #
    # 官方接口
    # ------------------------------------------------------------------ #
    def predict(self, coverage_state, step, max_steps):
        coverage_state = np.asarray(coverage_state, dtype=np.float32).reshape(-1)
        if self.policy == "greedy":
            return self._greedy_action(int(np.sum(coverage_state)))
        return self._random_action()

    # ------------------------------------------------------------------ #
    # 基线 1：纯随机
    # ------------------------------------------------------------------ #
    def _random_action(self):
        a = (self.rng.uniform(0.0, 1.0, self.DIMS) * self.BOUNDS)
        return a.astype(np.float32)

    # ------------------------------------------------------------------ #
    # 基线 2：贪心"编程-启动"任务循环
    # ------------------------------------------------------------------ #
    @staticmethod
    def _pack4(v):
        v = int(v) & 0xFFFFFFFF
        return [float((v >> (8 * i)) & 0xFF) for i in range(4)]

    def _task_action(self, task, phase):
        ch, sa, da, pk = task
        a = np.zeros(self.DIMS, dtype=np.float32)
        if phase == 0:                       # 写 saddr
            a[0], a[1], a[2] = ch, 1.0, 0.0
            a[3:7] = self._pack4(sa)
        elif phase == 1:                     # 写 daddr
            a[0], a[1], a[2] = ch, 1.0, 1.0
            a[3:7] = self._pack4(da)
        elif phase == 2:                     # 写 packed {burst,dmode,len}
            a[0], a[1], a[2] = ch, 1.0, 2.0
            a[3:7] = self._pack4(pk)
        else:                                # phase == 3：start 上升沿
            a[0], a[7] = ch, 1.0
        return a

    def _greedy_action(self, covered):
        if self._n == 0 and self._phase == 0:
            self._begin = covered
        if self._phase == 4:
            # 上一任务已完整执行：评估收益，轮换任务池
            gain = covered - self._begin
            cur = self._tasks[0]
            self._tasks.rotate(-1)
            if gain > 0:
                self._tasks.appendleft(cur)  # 命中新 bin，保留复用
            self._phase = 0
            self._n = 0
            self._begin = covered
        task = self._tasks[0]
        self._n += 1
        if self._n >= self._REPEAT:
            self._n = 0
            self._phase += 1
        return self._perturb(self._task_action(task, self._phase))

    def _perturb(self, action):
        for dim, half in self._PERTURB:
            v = int(action[dim]) + self.rng.randint(-half, half + 1)
            action[dim] = float(min(max(v, 0), int(self.BOUNDS[dim]) - 1))
        return action


if __name__ == "__main__":
    for p in ("random", "greedy"):
        inf = InferenceInterface(policy=p)
        s = np.zeros(inf.total_bins, dtype=np.float32)
        outs = [inf.predict(s, i, 1000) for i in range(4)]
        for a in outs:
            assert a.shape == (inf.DIMS,)
            assert a.dtype == np.float32
            assert np.all(a >= 0)
        print(f"[{p}] sample shapes={[a.shape for a in outs]}")
    print("InferenceInterface smoke OK")
