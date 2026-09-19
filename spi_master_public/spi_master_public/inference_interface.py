#!/usr/bin/env python3
"""
spi_master_public - inference_interface.py

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
  - reg_addr 有效 0x00-0x3B（掩码 0x3F），reg_wdata 为 32-bit 寄存器写数据
  - DR 写 = TX FIFO 入队，reg_re@DR = RX FIFO 出队
  - ss_in_n：SPI/SSP 传输期间需保持高，Microwire 模式下 HOLD_SS 期间保持低

与 harness.py 集成：
  h = SpiMasterHarness()
  state = h.reset()
  for step in range(max_steps):
      action = inference.predict(state, step, max_steps)   # np.float32, shape (12,)
      state, reward, done, info = h.step(action)
"""

import json
import os
from collections import deque

import numpy as np


class InferenceInterface:
    DIMS = 12
    # 每维上界（np.float32，必须用 float32 可精确表示的值）
    # [reg_we, reg_addr, wdata, reg_re, rxd, ss_in_n, rst_n, pad0..pad4]
    BOUNDS = np.array([2, 64, 1 << 32, 2, 2, 2, 2, 1, 1, 1, 1, 1], dtype=np.float32)
    _REPEAT = 8
    _PERTURB = [(2, 0x10000)]

    # ------------------------------------------------------------------ #
    # 确定性候选动作池：完整 12 维动作行 [we, addr, wdata, re, rxd, ss, rst, pad*5]
    # reg_wdata 一律取 float32 可精确表示的值，避免 harness 内 int() 截断失真。
    # CTRLR0 基值 0x2007 = DFS=7 + SRL_TEST=1（回环），便于在无外部从机时自检。
    # ------------------------------------------------------------------ #
    _CANDIDATES = [
        # 复位段：ss 拉高、rst 拉低一拍
        [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],
        # ---- CTRLR0 (addr 0x00) ----
        [1, 0x00, 0x2007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SPI, 8bit, SRL_TEST=1
        [1, 0x00, 0x2207, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TMOD1
        [1, 0x00, 0x2407, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TMOD2
        [1, 0x00, 0x2027, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SSP  (FRF=1)
        [1, 0x00, 0x2047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # Microwire (FRF=2)
        [1, 0x00, 0x2087, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SCPH
        [1, 0x00, 0x2107, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SCPOL
        [1, 0x00, 0x3007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SLV_OE
        [1, 0x00, 0x6007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SS_TGL_EN
        [1, 0x00, 0x2003, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=3
        [1, 0x00, 0x200F, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # DFS=15
        # Microwire + CFS 变体（CFS[19:16]，低于隐藏下限时被钳位）
        [1, 0x00, 0x2047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=0（钳位）
        [1, 0x00, 0x12047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=1（钳位）
        [1, 0x00, 0x22047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=2（钳位）
        [1, 0x00, 0x72047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=7（钳位）
        [1, 0x00, 0x82047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=8
        [1, 0x00, 0xC2047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=12
        [1, 0x00, 0xF2047, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # CFS=15
        # ---- CTRLR1 (addr 0x01) NDF ----
        [1, 0x01, 0x0000, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x01, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x01, 0x0004, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x01, 0x0010, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        # ---- MWCR (addr 0x03) / SER (addr 0x04) / BAUDR (addr 0x05) ----
        [1, 0x03, 0x0000, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x03, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x03, 0x0003, 0, 0, 1, 1, 0, 0, 0, 0, 0],
        [1, 0x04, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=1
        [1, 0x04, 0x0005, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=5
        [1, 0x04, 0x000F, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SER=15
        [1, 0x05, 0x0002, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=2
        [1, 0x05, 0x0008, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=8
        [1, 0x05, 0x0010, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # BAUDR=16
        # ---- FIFO 阈值 / 使能 ----
        [1, 0x06, 0x0000, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TXFTLR=0
        [1, 0x06, 0x0007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # TXFTLR=7
        [1, 0x07, 0x0000, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # RXFTLR=0
        [1, 0x07, 0x0007, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # RXFTLR=7
        [1, 0x02, 0x0001, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # SSIENR=1
        # ---- DR 写（TX FIFO 入队，float32 精确数据）----
        [1, 0x18, 0x000000FF, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xFF
        [1, 0x18, 0x0000AA55, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xAA55
        [1, 0x18, 0x00DEADBE, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xDEADBE
        [1, 0x18, 0x00FFFFFF, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0xFFFFFF
        [1, 0x18, 0x01000000, 0, 0, 1, 1, 0, 0, 0, 0, 0],  # 0x1000000
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
