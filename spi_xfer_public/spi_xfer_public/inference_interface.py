#!/usr/bin/env python3
"""
spi_xfer_public - inference_interface.py

官方 InferenceInterface 骨架（对齐大赛框架 4.6 节 / 附件 B.3）：

    class InferenceInterface:
        def __init__(self, dut_spec_path: str, covergroup_path: str)
        def predict(self, coverage_state: np.ndarray,
                    step: int, max_steps: int) -> np.ndarray

评测闭环（4.3.4）：基础镜像内置两种基线供参赛者参考
  1. 纯随机激励生成（policy="random"，默认）
  2. 简单贪心搜索（policy="greedy"）：固定候选动作池，逐个周期重复，命中新 bin 的候选保留复用

动作空间（12 维，见 dut_spec.md §6）：
  [reg_we, reg_addr, reg_wdata, reg_re, rxd, ss_in_n, rst_n, pad0..pad4]
  - reg_addr 有效 0x0-0x9（掩码 0xF），reg_wdata 为 32-bit 寄存器写数据
  - DR 写 = TX FIFO 入队，reg_re@DR = RX FIFO 出队

与 harness.py 集成：
  h = SpiXferHarness()
  state = h.reset()
  for step in range(max_steps):
      action = inference.predict(state, step, max_steps)   # np.float32, shape (12,)
      state, reward, done, info = h.step(action)

注意：coverage_state 为 float32 二值向量；本骨架不假设其数值精度以外的任何
内部实现，仅按官方接口消费/返回 numpy 数组。
"""

import json
import os
from collections import deque

import numpy as np


class InferenceInterface:
    DIMS = 12
    # 每维上界（np.float32，必须用 float32 可精确表示的值）
    # [reg_we, reg_addr, wdata, reg_re, rxd, ss_in_n, rst_n, pad0..pad4]
    BOUNDS = np.array([2, 16, 1 << 32, 2, 2, 2, 2, 1, 1, 1, 1, 1], dtype=np.float32)
    # 贪心基线：每个候选动作连续重复的周期数
    _REPEAT = 8
    # 贪心基线：邻域探索 (维号, 半幅)。仅对 wdata 做小幅扰动，余下保持候选语义
    _PERTURB = [(2, 0x10000)]

    # ------------------------------------------------------------------ #
    # 确定性候选动作池：完整 12 维动作行 [we, addr, wdata, re, rxd, ss, rst, pad*5]
    # reg_wdata 一律取 float32 可精确表示的值（< 2^24 或 2 的幂），避免
    # harness 内 int() 截断后失真（如 0xFFFFFFFF 会塌缩为 0）。
    # ------------------------------------------------------------------ #
    _CANDIDATES = [
        # 复位段：ss 拉高、rst 拉低一拍
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        # ---- CTRLR0 (addr 0) ----
        [1, 0, 0x0807, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SPI, 8bit, TMOD0
        [1, 0, 0x0A07, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TMOD1
        [1, 0, 0x0C07, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TMOD2
        [1, 0, 0x0887, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SCPH
        [1, 0, 0x0907, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SCPOL
        [1, 0, 0x0827, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SSP  (FRF=1)
        [1, 0, 0x0847, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # Microwire (FRF=2)
        [1, 0, 0x1807, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SSTE
        [1, 0, 0x0803, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=3 (4bit)
        [1, 0, 0x080F, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=15 (16bit)
        [1, 0, 0x0800, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=0（钳位到有效最小）
        [1, 0, 0x0802, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=2（钳位到有效最小）
        # ---- 从设备选择 / 波特率 / FIFO 阈值 / 使能 ----
        [1, 3, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=1
        [1, 3, 0x0005, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=5
        [1, 3, 0x000F, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=15
        [1, 4, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=1
        [1, 4, 0x0002, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=2
        [1, 4, 0x0003, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=3
        [1, 4, 0x0008, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=8
        [1, 5, 0x0000, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TXFTLR=0
        [1, 5, 0x0007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TXFTLR=7
        [1, 2, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SSIENR=1
        # ---- DR 写（TX FIFO 入队，float32 精确数据）----
        [1, 9, 0x000000FF, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xFF
        [1, 9, 0x0000AA55, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xAA55
        [1, 9, 0x00DEADBE, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xDEADBE
        [1, 9, 0x00FFFFFF, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xFFFFFF
        [1, 9, 0x01000000, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0x1000000
    ]

    def __init__(self, dut_spec_path=None, covergroup_path=None,
                 policy="random", seed=7):
        self.dut_spec_path = dut_spec_path
        self.covergroup_path = covergroup_path
        self.policy = policy
        self.rng = np.random.RandomState(seed)
        self._total_bins = self._read_total_bins()
        self._cands = deque([np.asarray(c, dtype=np.float32)
                             for c in self._CANDIDATES])
        self._n = 0            # 当前候选已重复周期计数
        self._begin = 0        # 进入当前候选时的已命中 bin 数
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
        self._cands = deque([np.asarray(c, dtype=np.float32)
                             for c in self._CANDIDATES])
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
    # 基线 2：贪心候选循环
    # ------------------------------------------------------------------ #
    def _greedy_action(self, covered):
        if self._n == 0:
            self._begin = covered
        cand = self._cands[0]
        if self._n >= self._REPEAT:
            # 当前候选已重复够本：评估收益，轮换候选池
            gain = covered - self._begin
            self._cands.rotate(-1)
            if gain > 0:
                self._cands.appendleft(cand)  # 命中新 bin，保留复用
            self._n = 0
            self._begin = covered
        self._n += 1
        return self._perturb(cand.copy())

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
