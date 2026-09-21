#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
LLM 分类 + 双策略 Agent：
  - LLM 读 spec 前2000字 → 输出 "spi" / "dma" / "other"
  - spi  → 用 FSM 策略（高覆盖率）
  - 其他 → 用通用寄存器扫描策略（保底）
"""

import json
import os
import re
import random
from collections import deque
import numpy as np


class InferenceInterface:
    DIMS_DEFAULT = 12

    def __init__(self, dut_spec_path=None, covergroup_path=None,
                 policy="auto", seed=7, use_llm=True):
        self.dut_spec_path = dut_spec_path
        self.covergroup_path = covergroup_path
        self.policy = policy
        self.rng = np.random.RandomState(seed)
        random.seed(seed)

        self.DIMS = self._get_dims_from_spec(dut_spec_path)
        self._total_bins = self._read_total_bins()

        # ---- 1. LLM 短分类 ----
        self.dut_type = "other"
        if use_llm and dut_spec_path:
            try:
                self.dut_type = self._classify_with_llm(dut_spec_path)
                print(f"[LLM] DUT 分类: {self.dut_type}")
            except Exception as e:
                print(f"[LLM] 分类失败，用通用策略: {e}")

        # ---- 2. 根据分类选策略 ----
        if self.dut_type == "other":
            self._init_fsm_strategy(dut_spec_path)
        else:
            self._init_regscan_strategy(dut_spec_path)

    # ==========================================================
    #  一、LLM 短分类
    # ==========================================================
    def _classify_with_llm(self, spec_path):
        from openai import OpenAI

        api_key = os.environ.get("DEEPSEEK_API_KEY", "").strip()
        api_key = api_key.encode("ascii", errors="ignore").decode("ascii")
        client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")

        with open(spec_path, encoding="utf-8", errors="ignore") as f:
            text = f.read()[:2000]

        prompt = f"""判断下面这份芯片规格书描述的 DUT 属于哪一类。
只回答一个词：
- "spi"：SPI/SSP/Microwire 串行传输控制器
- "dma"：DMA 传输控制器
- "other"：其他

不要解释，不要标点，只回答一个词。

{text}
"""
        r = client.chat.completions.create(
            model="deepseek-chat",
            messages=[{"role": "user", "content": prompt}],
            temperature=0.0, max_tokens=10
        )
        ans = r.choices[0].message.content.strip().lower()
        if "spi" in ans: return "spi"
        if "dma" in ans: return "dma"
        return "other"

    # ==========================================================
    #  二、维度 / total_bins / 寄存器地址 提取
    # ==========================================================
    def _get_dims_from_spec(self, spec_path):
        if not spec_path or not os.path.exists(spec_path):
            return self.DIMS_DEFAULT
        try:
            with open(spec_path, encoding="utf-8", errors="ignore") as f:
                text = f.read()
            for p in [r"动作空间.{0,30}?(\d+)\s*维",
                      r"Action\s+layout.{0,30}?\((\d+)\s+dims",
                      r"(\d+)\s*dims"]:
                m = re.search(p, text)
                if m:
                    return int(m.group(1))
        except Exception:
            pass
        return self.DIMS_DEFAULT

    def _read_total_bins(self):
        if self.covergroup_path:
            meta = os.path.join(
                os.path.dirname(os.path.abspath(self.covergroup_path)),
                "coverage_meta.json"
            )
            if os.path.exists(meta):
                try:
                    with open(meta, encoding="utf-8") as f:
                        n = int(json.load(f).get("total_bins", 0))
                    if n > 0: return n
                except Exception:
                    pass
        return self.DIMS

    @property
    def total_bins(self):
        return self._total_bins

    def _extract_reg_addrs(self, spec_path):
        addrs = []
        if not spec_path or not os.path.exists(spec_path):
            return list(range(16))
        try:
            with open(spec_path, encoding="utf-8", errors="ignore") as f:
                text = f.read()
            for m in re.finditer(
                r"\|\s*(0x[0-9a-fA-F]+|\d+)\s*\|\s*([A-Z][A-Z0-9_]+)\s*\|",
                text):
                try:
                    v = m.group(1)
                    a = int(v, 16) if v.lower().startswith("0x") else int(v)
                    if 0 <= a < 256 and a not in addrs:
                        addrs.append(a)
                except Exception:
                    pass
        except Exception:
            pass
        addrs = sorted(set(addrs))
        return addrs if addrs else list(range(16))

    # ==========================================================
    #  三、通用动作构造函数
    # ==========================================================
    def _action(self, reg_we=0, reg_addr=0, reg_wdata=0,
                reg_re=0, rxd=0, ss_in_n=1, rst_n=1):
        a = np.zeros(self.DIMS, dtype=np.float32)
        a[0] = np.float32(reg_we)
        a[1] = np.float32(reg_addr)
        a[2] = np.float32(reg_wdata)
        if self.DIMS > 3: a[3] = np.float32(reg_re)
        if self.DIMS > 4: a[4] = np.float32(rxd)
        if self.DIMS > 5: a[5] = np.float32(ss_in_n)
        if self.DIMS > 6: a[6] = np.float32(rst_n)
        return a

    def reset(self):
        if hasattr(self, "_fsm_reset"):
            self._fsm_reset()
        if hasattr(self, "_regscan_reset"):
            self._regscan_reset()

    # ==========================================================
    #  四、FSM 策略（spi 类专用，高覆盖）
    # ==========================================================
    def _init_fsm_strategy(self, spec_path):
        # 从 spec 提取寄存器地址（沿用之前的硬编码映射 + 正则兜底）
        regs = self._parse_regs_from_spec(spec_path)
        self.REG_CTRLR0 = regs.get("CTRLR0", 0)
        self.REG_SSIENR = regs.get("SSIENR", 2)
        self.REG_SER    = regs.get("SER", 4)
        self.REG_BAUDR  = regs.get("BAUDR", 5)
        self.REG_TXFTLR = regs.get("TXFTLR", 6)
        self.REG_DR     = regs.get("DR", 24)

        # 从 spec 里读 TMOD 位段
        fields = self._parse_fields_from_spec(spec_path)
        self.TMOD_SHIFT = fields.get("TMOD", [11, 10])[1]

        self.CTRLR0_BASE = 0x0000
        # 协议：默认按 spi_master 的位段
        self.PROTOCOL_BITS = {
            "spi0": 0x0000,
            "spi1": 0x0100,
            "ssp":  0x0040,
        }
        self._protocols = list(self.PROTOCOL_BITS.keys())
        self._tmods = [1, 2, 3]
        self._dfs_list = [8, 16, 24, 31]
        self._baudr_list = [2, 4, 6, 8]

        self.TX_PATTERNS = (0x00000055, 0x000000AA, 0x000000FF,
                            0x0000AA55, 0x00DEADBE, 0x00FFFFFF)

        self._plans = self._build_plans()
        self._fsm_reset()
        print(f"[FSM] 寄存器: CTRLR0={self.REG_CTRLR0}, SSIENR={self.REG_SSIENR}, "
              f"SER={self.REG_SER}, BAUDR={self.REG_BAUDR}, "
              f"TXFTLR={self.REG_TXFTLR}, DR={self.REG_DR}")
        print(f"[FSM] TMOD_SHIFT={self.TMOD_SHIFT}, 共 {len(self._plans)} 个 plan")

    def _parse_regs_from_spec(self, spec_path):
        """从 spec 表格里提取 寄存器名 → 地址"""
        regs = {}
        try:
            with open(spec_path, encoding="utf-8", errors="ignore") as f:
                text = f.read()
            for m in re.finditer(
                r"\|\s*(0x[0-9a-fA-F]+|\d+)\s*\|\s*([A-Z][A-Z0-9_]+)\s*\|",
                text):
                v, name = m.group(1), m.group(2)
                a = int(v, 16) if v.lower().startswith("0x") else int(v)
                if name not in regs:
                    regs[name] = a
        except Exception:
            pass
        return regs

    def _parse_fields_from_spec(self, spec_path):
        """从 spec 里提取 CTRLR0 的字段位段，如 [11:10]=TMOD"""
        fields = {}
        try:
            with open(spec_path, encoding="utf-8", errors="ignore") as f:
                text = f.read()
            # 匹配 [11:10]=TMOD 或 [11:10] = TMOD
            for m in re.finditer(
                r"\[\s*(\d+)\s*:\s*(\d+)\s*\]\s*=\s*([A-Z][A-Z0-9_]+)",
                text):
                hi, lo, name = int(m.group(1)), int(m.group(2)), m.group(3)
                if name not in fields:
                    fields[name] = [hi, lo]
        except Exception:
            pass
        return fields

    def _make_ctrlr0(self, protocol, tmod, dfs_bits):
        if protocol not in self.PROTOCOL_BITS:
            protocol = self._protocols[0]
        tmod = int(np.clip(tmod, 0, 3))
        dfs_bits = int(np.clip(dfs_bits, 3, 32))
        dfs_raw = dfs_bits - 1
        return int(
            self.CTRLR0_BASE
            | self.PROTOCOL_BITS[protocol]
            | (tmod << self.TMOD_SHIFT)
            | dfs_raw
        )

    @staticmethod
    def _make_plan(name, protocol, tmod, dfs, baudr,
                   tx_words=2, ser=1, txftlr=0):
        return {"name": name, "protocol": protocol, "tmod": int(tmod),
                "dfs": int(dfs), "baudr": int(baudr),
                "tx_words": int(tx_words), "ser": int(ser),
                "txftlr": int(txftlr)}

    def _build_plans(self):
        plans = []
        for protocol in self._protocols:
            for tmod in self._tmods:
                plans.append(self._make_plan(
                    name=f"{protocol}_tmod{tmod}", protocol=protocol,
                    tmod=tmod, dfs=8, baudr=2, tx_words=2))
        for protocol in self._protocols:
            for dfs in self._dfs_list:
                plans.append(self._make_plan(
                    name=f"{protocol}_dfs{dfs}", protocol=protocol,
                    tmod=0, dfs=dfs, baudr=2, tx_words=2))
        for baudr in self._baudr_list:
            plans.append(self._make_plan(
                name=f"baudr_{baudr}", protocol=self._protocols[0],
                tmod=2, dfs=8, baudr=baudr, tx_words=2))
        plans.append(self._make_plan(
            name="tx_fifo_half_like", protocol=self._protocols[0],
            tmod=0, dfs=8, baudr=2, tx_words=4))
        return plans

    def _fsm_reset(self):
        self._plan_idx = 0
        self._repeat_current = 0
        self._phase = "RESET"
        self._wait_count = 0
        self._tx_count = 0
        self._read_count = 0
        self._plan_cov_start = None
        self._fsm_prev_cov = 0.0

    @property
    def _current_plan(self):
        return self._plans[self._plan_idx]

    def _advance_plan(self):
        self._plan_idx = (self._plan_idx + 1) % len(self._plans)
        self._repeat_current = 0

    @staticmethod
    def _estimate_wait_cycles(plan):
        frames = max(1, plan["tx_words"])
        dfs = max(4, plan["dfs"])
        baudr = max(1, plan["baudr"])
        wait = frames * dfs * max(2, baudr) * 3 + 48
        return int(np.clip(wait, 96, 1600))

    def _coverage_gain(self, coverage_state):
        now = np.asarray(coverage_state, dtype=np.float32).reshape(-1)
        if self._plan_cov_start is None:
            return 0
        n = min(now.size, self._plan_cov_start.size)
        return int(np.count_nonzero(
            (now[:n] > 0.5) & (~(self._plan_cov_start[:n] > 0.5))))

    def _fsm_predict(self, coverage_state, step, max_steps):
        plan = self._current_plan

        if self._phase == "RESET":
            self._plan_cov_start = coverage_state.copy()
            self._phase = "DISABLE"
            return self._action(reg_we=0, reg_addr=0, reg_wdata=0,
                                reg_re=0, rxd=0, ss_in_n=1, rst_n=0)

        if self._phase == "DISABLE":
            self._phase = "CONFIG_CTRLR0"
            return self._action(reg_we=1, reg_addr=self.REG_SSIENR, reg_wdata=0)

        if self._phase == "CONFIG_CTRLR0":
            v = self._make_ctrlr0(plan["protocol"], plan["tmod"], plan["dfs"])
            self._phase = "CONFIG_SER"
            return self._action(reg_we=1, reg_addr=self.REG_CTRLR0, reg_wdata=v)

        if self._phase == "CONFIG_SER":
            self._phase = "CONFIG_BAUDR"
            return self._action(reg_we=1, reg_addr=self.REG_SER, reg_wdata=plan["ser"])

        if self._phase == "CONFIG_BAUDR":
            self._phase = "CONFIG_TXFTLR"
            return self._action(reg_we=1, reg_addr=self.REG_BAUDR, reg_wdata=plan["baudr"])

        if self._phase == "CONFIG_TXFTLR":
            self._phase = "ENABLE"
            return self._action(reg_we=1, reg_addr=self.REG_TXFTLR, reg_wdata=plan["txftlr"])

        if self._phase == "ENABLE":
            self._tx_count = 0
            self._wait_count = 0
            self._phase = "WRITE_DR"
            return self._action(reg_we=1, reg_addr=self.REG_SSIENR, reg_wdata=1)

        if self._phase == "WRITE_DR":
            idx = (self._plan_idx + self._repeat_current + self._tx_count) % len(self.TX_PATTERNS)
            data = self.TX_PATTERNS[idx]
            self._tx_count += 1
            if self._tx_count >= plan["tx_words"]:
                self._phase = "RUN"
                self._wait_count = 0
            return self._action(reg_we=1, reg_addr=self.REG_DR, reg_wdata=data)

        if self._phase == "RUN":
            self._wait_count += 1
            wait_limit = self._estimate_wait_cycles(plan)
            if max_steps - step < 1000:
                wait_limit = min(wait_limit, 128)
            if self._wait_count >= wait_limit:
                self._read_count = 0
                self._phase = "READ_DR"
            return self._action(reg_we=0, reg_addr=0, reg_wdata=0)

        if self._phase == "READ_DR":
            self._read_count += 1
            if self._read_count >= 2:
                self._phase = "FINAL_DISABLE"
            return self._action(reg_we=0, reg_addr=self.REG_DR, reg_wdata=0, reg_re=1)

        if self._phase == "FINAL_DISABLE":
            self._phase = "EVAL"
            return self._action(reg_we=1, reg_addr=self.REG_SSIENR, reg_wdata=0)

        if self._phase == "EVAL":
            gain = self._coverage_gain(coverage_state)
            if gain > 0 and self._repeat_current == 0:
                self._repeat_current = 1
            else:
                self._advance_plan()
            self._plan_cov_start = coverage_state.copy()
            self._phase = "DISABLE"
            self._wait_count = 0
            self._tx_count = 0
            self._read_count = 0
            return self._action(reg_we=0, reg_addr=0, reg_wdata=0)

        self._phase = "DISABLE"
        return self._action(reg_we=0, reg_addr=0, reg_wdata=0)

    # ==========================================================
    #  五、通用寄存器扫描策略（其他 DUT）
    # ==========================================================
    def _init_regscan_strategy(self, spec_path):
        self.reg_addrs = self._extract_reg_addrs(spec_path)
        self.value_pool = [0, 1, 2, 3, 4, 7, 8, 15, 16, 31, 32,
                           63, 64, 127, 128, 255, 0x55, 0xAA, 0xFF]
        self.data_patterns = [0x55, 0xAA, 0xFF, 0x0F, 0xF0,
                              0x1234, 0xABCD, 0xFFFF, 0xDEAD, 0x5555]
        self._regscan_reset()
        print(f"[RegScan] 找到 {len(self.reg_addrs)} 个寄存器地址")

    def _regscan_reset(self):
        self._rs_addr_idx = 0
        self._rs_value_idx = 0
        self._rs_step = 0
        self._rs_prev_cov = 0.0
        self._rs_no_gain = 0

    def _regscan_predict(self, coverage_state, step, max_steps):
        cur_cov = float(coverage_state.sum())
        if step % 100 == 0 and step > 0:
            if cur_cov > self._rs_prev_cov:
                self._rs_no_gain = 0
            else:
                self._rs_no_gain += 1
            self._rs_prev_cov = cur_cov

        if self._rs_no_gain >= 5:
            self._rs_no_gain = 0
            return self._action(
                reg_we=random.choice([0, 1]),
                reg_addr=random.choice(self.reg_addrs),
                reg_wdata=random.choice(self.value_pool),
                reg_re=random.choice([0, 1]),
                rxd=random.choice([0, 1]),
                ss_in_n=random.choice([0, 1]),
                rst_n=1)

        phase = self._rs_step
        if phase == 0:
            self._rs_step += 1
            return self._action(reg_we=1, reg_addr=2, reg_wdata=0)
        if 1 <= phase <= 4:
            addr = self.reg_addrs[(phase - 1) % len(self.reg_addrs)]
            val = self.value_pool[self._rs_value_idx % len(self.value_pool)]
            self._rs_value_idx += 1
            self._rs_step += 1
            return self._action(reg_we=1, reg_addr=addr, reg_wdata=val)
        if phase == 5:
            self._rs_step += 1
            return self._action(reg_we=1, reg_addr=2, reg_wdata=1)
        if phase == 6:
            data = self.data_patterns[self._rs_addr_idx % len(self.data_patterns)]
            self._rs_step += 1
            return self._action(reg_we=1, reg_addr=self.reg_addrs[-1], reg_wdata=data)
        if phase < 25:
            self._rs_step += 1
            return self._action(reg_we=0, reg_addr=0, reg_wdata=0)
        self._rs_step = 0
        self._rs_addr_idx = (self._rs_addr_idx + 1) % max(1, len(self.reg_addrs))
        return self._action(reg_we=1, reg_addr=2, reg_wdata=0)

    # ==========================================================
    #  六、核心 predict
    # ==========================================================
    def predict(self, coverage_state, step, max_steps):
        coverage_state = np.asarray(coverage_state, dtype=np.float32).reshape(-1)

        if self.dut_type == "spi":
            return self._fsm_predict(coverage_state, step, max_steps)
        else:
            return self._regscan_predict(coverage_state, step, max_steps)


if __name__ == "__main__":
    inf = InferenceInterface(use_llm=False)
    print(f"DIMS={inf.DIMS}, type={inf.dut_type}")
    s = np.zeros(inf.total_bins, dtype=np.float32)
    for step in range(2000):
        a = inf.predict(s, step, 50000)
        assert a.shape == (inf.DIMS,)
    print("smoke OK")