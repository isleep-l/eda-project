#!/usr/bin/env python3
"""
dma_xfer_public - Black-box simulator (local_sim.py).
Python mirror of dma_top.v for local development and evaluation.
Hidden parameters (arb_prio, done_ok_modes, xfer_hold_n) are injected
externally; the public default placeholder has no "correct" modes so done_ok
is inert unless secrets are injected.
"""

from typing import Dict, List, Optional

ADDR_W = 32
LEN_W = 16
N_CH = 4

# public placeholder values
PLACEHOLDER_ARB_PRIO = [0, 1, 2, 3]       # ascending priority
PLACEHOLDER_DONE_OK_MODES = [0, 0, 0, 0]  # none correct
PLACEHOLDER_XFER_HOLD_N = 16

IDLE = 0
ACTIVE = 1
DONE = 2


def sign_mask(v: int, bits: int) -> int:
    return v & ((1 << bits) - 1)


class DmaXferPublic:
    def __init__(self,
                 arb_prio: Optional[List[int]] = None,
                 done_ok_modes: Optional[List[int]] = None,
                 xfer_hold_n: Optional[int] = None):
        # hidden params (injected)
        self.arb_prio = list(arb_prio) if arb_prio else list(PLACEHOLDER_ARB_PRIO)
        self.done_ok_modes = list(done_ok_modes) if done_ok_modes else list(PLACEHOLDER_DONE_OK_MODES)
        self.xfer_hold_n = xfer_hold_n if xfer_hold_n is not None else PLACEHOLDER_XFER_HOLD_N

        # per-channel config
        self.saddr = [0] * N_CH
        self.daddr = [0] * N_CH
        self.len = [0] * N_CH
        self.dmode = [0] * N_CH
        self.burst = [0] * N_CH
        self.fsm = [IDLE] * N_CH
        self.remain = [0] * N_CH
        self.got = [False] * N_CH
        self.done_evt = [False] * N_CH

        # arbitration / transfer state
        self.arb_winner = 0
        self.arb_valid = False
        self.grant = [False] * N_CH

        # done-hold window
        self.hold = False
        self.hld_state = IDLE
        self.hld_cnt = 0
        self.hld_class = 0
        self.done_ok_valid = False
        self.done_ok = 0

        # coverage context (last-granted channel snapshot)
        self.cov_active_ch = 0
        self.cov_dir = 0
        self.cov_burst = 0
        self.cov_saddr = 0
        self.cov_daddr = 0
        self.cov_len = 0

        self.start_prev = [False] * N_CH

        # ---- temporal sequence monitor state ----
        self.seq_evts = {}            # this-cycle events fed before step
        self.cfg_write_evt = False    # a config write happened this cycle
        self.start_evt = [False] * N_CH  # per-channel start rising this cycle

        # sequence flags (valid for the *current* cycle, consumed by seq monitor)
        self._seq_ok = {}

        # persistent monitors
        self.s_active_cont = [0] * N_CH
        self.s_idle_cont = 0
        self.s_arb_cont = 0
        self.s_hold_cont = 0
        self.s_conf_then_any = 0
        self._hsw_cfg = False
        self._hsw_start = False
        self._hsw_active = False
        self._s3_started = False
        self._s3_cfg_seen = False
        self._hold_prev = False
        self._conf2 = 0
        self._a2d_cnt = [0] * N_CH
        self._pw = None
        self._pw_valid = False

    def inject_hidden_params(self, arb_prio, done_ok_modes, xfer_hold_n):
        self.arb_prio = list(arb_prio)
        self.done_ok_modes = list(done_ok_modes)
        self.xfer_hold_n = xfer_hold_n

    def reset(self):
        self.saddr = [0] * N_CH
        self.daddr = [0] * N_CH
        self.len = [0] * N_CH
        self.dmode = [0] * N_CH
        self.burst = [0] * N_CH
        self.fsm = [IDLE] * N_CH
        self.remain = [0] * N_CH
        self.got = [False] * N_CH
        self.done_evt = [False] * N_CH
        self.hold = False
        self.hld_state = IDLE
        self.hld_cnt = 0
        self.done_ok_valid = False
        self.done_ok = 0
        self.start_prev = [False] * N_CH

        self.seq_evts = {}
        self.cfg_write_evt = False
        self.start_evt = [False] * N_CH
        self._seq_ok = {}
        self.s_active_cont = [0] * N_CH
        self.s_idle_cont = 0
        self.s_arb_cont = 0
        self.s_hold_cont = 0
        self.s_conf_then_any = 0
        self._hsw_cfg = False
        self._hsw_start = False
        self._hsw_active = False
        self._s3_started = False
        self._s3_cfg_seen = False
        self._hold_prev = False
        self._conf2 = 0
        self._a2d_cnt = [0] * N_CH
        self._pw = None
        self._pw_valid = False

    def apply_config(self, ch: int, field: int, data: int):
        """Direct register configuration (same as conf_wr/conf_field).
        field 0 = saddr, field 1 = daddr,
        field 2 = packed {burst[23:21], dmode[20:16], len[15:0]}."""
        if self.got[ch] or self.fsm[ch] != IDLE or self.hold:
            return
        m = (1 << ADDR_W) - 1
        if field == 0:
            self.saddr[ch] = data & m
        elif field == 1:
            self.daddr[ch] = data & m
        elif field == 2:
            self.len[ch] = data & ((1 << LEN_W) - 1)
            self.remain[ch] = self.len[ch]
            self.dmode[ch] = (data >> 16) & 0xF
            self.burst[ch] = (data >> 21) & 0x7
        self.cfg_write_evt = True
        # coverage cookie reflects the most recently programmed registers
        self.cov_active_ch = ch
        self.cov_dir = self.dmode[ch]
        self.cov_burst = self.burst[ch]
        self.cov_saddr = self.saddr[ch]
        self.cov_daddr = self.daddr[ch]
        self.cov_len = self.len[ch]

    def tick_start(self, start_mask: int):
        """Feed start pins for one cycle (before step)."""
        mask = [bool(start_mask & (1 << i)) for i in range(N_CH)]
        for i in range(N_CH):
            if mask[i] and not self.start_prev[i] and not self.got[i]:
                self.got[i] = True
                self.remain[i] = self.len[i]
                self.fsm[i] = DONE if self.len[i] == 0 else ACTIVE
                self.start_evt[i] = True
        self.start_prev = mask

    def step(self):
        """Advance one clock cycle. Call apply_config/tick_start before this."""
        # capture per-cycle events (async to the step); sequential monitor runs
        # at the end so it sees the fresh done_ok_valid / hold / fsm state.
        cfg_evt = self.cfg_write_evt
        start_evt = self.start_evt
        self.cfg_write_evt = False
        self.start_evt = [False] * N_CH

        self.done_ok_valid = False

        # auto-return channels that spent a cycle in DONE back to IDLE
        for i in range(N_CH):
            if self.fsm[i] == DONE and not self.got[i]:
                self.fsm[i] = IDLE

        # ---- arbitration ----
        req = [self.got[i] and self.fsm[i] == ACTIVE for i in range(N_CH)]
        req_no_hold = [req[i] and not self.hold for i in range(N_CH)]
        self.grant = [False] * N_CH
        self.arb_winner = 0
        self.arb_valid = False
        if any(req_no_hold):
            for ch in self.arb_prio:
                if ch >= N_CH:
                    continue
                if req_no_hold[ch]:
                    self.arb_winner = ch
                    self.arb_valid = True
                    self.grant[ch] = True
                    break

        # ---- beat consumption ----
        self.done_evt = [False] * N_CH
        if not self.hold:
            for bi in range(N_CH):
                if self.got[bi] and self.grant[bi] and self.fsm[bi] == ACTIVE:
                    if self.remain[bi] == 1:
                        self.fsm[bi] = DONE
                        self.got[bi] = False
                        self.remain[bi] = 0
                        self.done_evt[bi] = True
                    else:
                        self.remain[bi] -= 1

        # ---- completion detection (policy) and done-hold window ----
        completed = [i for i in range(N_CH) if self.done_evt[i]]
        self.done_evt = [False] * N_CH
        # The completed dir_mode to classify: use the most recently granted
        # channel among completed; simplest: first completed channel.
        completing = None
        if completed:
            # pick the one that matches current arb winner if possible else lowest
            completing = min(completed)

        if self.hld_state == IDLE:
            if completing is not None:
                dm = self.dmode[completing]
                if dm in self.done_ok_modes:
                    self.hld_class = self.done_ok_modes.index(dm)
                    self.hld_state = ACTIVE
                    self.hld_cnt = self.xfer_hold_n
                    self.hold = True
        elif self.hld_state == ACTIVE:
            if self.hld_cnt == 1:
                self.done_ok_valid = True
                self.done_ok = self.hld_class
                self.hld_state = DONE
                self.hold = False
            else:
                self.hld_cnt -= 1
        elif self.hld_state == DONE:
            self.hld_state = IDLE

        # ---- coverage context (arb grant snapshot) ----
        if self.arb_valid and not self.hold:
            self.cov_active_ch = self.arb_winner
            self.cov_dir = self.dmode[self.arb_winner]
            self.cov_burst = self.burst[self.arb_winner]
            self.cov_saddr = self.saddr[self.arb_winner]
            self.cov_daddr = self.daddr[self.arb_winner]
            self.cov_len = self.len[self.arb_winner]

        # ---- temporal sequence monitor (read live post-step state) ----
        self._monitor_step(cfg_evt, start_evt)

    def _arb_conflict_val(self) -> int:
        return 1 if sum(1 for i in range(N_CH)
                        if self.got[i] and self.fsm[i] == ACTIVE) > 1 else 0

    def _monitor_step(self, cfg_evt: bool, start_evt: list):
        """Advance the temporal-sequence monitors for the sampled cycle.
        Uses live (post-step) state so done_ok_valid / hold / fsm reflect the
        cycle read_signals will present."""
        s = self._seq_ok
        for k in list(s.keys()):
            s[k] = False
        fsm = list(self.fsm)
        got = list(self.got)
        hold = bool(self.hold)
        any_start = any(start_evt)
        any_active = any(fsm[i] == ACTIVE for i in range(N_CH))
        any_done = any(fsm[i] == DONE and not got[i] for i in range(N_CH))

        # ---- Q1: cfg -> start -> active -> done (complete handshake) ----
        if cfg_evt:
            self._hsw_cfg = True
            self._hsw_start = False
            self._hsw_active = False
        elif self._hsw_cfg and not self._hsw_start and any_start:
            self._hsw_start = True
        elif self._hsw_cfg and self._hsw_start and not self._hsw_active and any_active:
            self._hsw_active = True
        elif self._hsw_cfg and self._hsw_start and self._hsw_active and any_done:
            s["seq1"] = True
            self._hsw_cfg = False

        # ---- Q2: same channel ACTIVE for >=3 consecutive cycles ----
        for i in range(N_CH):
            if fsm[i] == ACTIVE:
                self.s_active_cont[i] += 1
                if self.s_active_cont[i] >= 3:
                    s["seq2"] = True
            else:
                self.s_active_cont[i] = 0

        # ---- Q3: clean correct transfer (start->done_ok, no config interrupt) ----
        if cfg_evt:
            self._s3_started = False
            self._s3_cfg_seen = False
        if any_start:
            self._s3_started = True
            self._s3_cfg_seen = False
        if self._s3_started and cfg_evt:
            self._s3_cfg_seen = True
        if self._s3_started and self.done_ok_valid:
            if not self._s3_cfg_seen:
                s["seq3"] = True
            self._s3_started = False

        # ---- Q4: hold for >=4 consecutive cycles ----
        if hold:
            self.s_hold_cont += 1
            if self.s_hold_cont >= 4:
                s["seq4"] = True
        else:
            self.s_hold_cont = 0

        # ---- Q5: hold 1->0 then done_ok ----
        if self.done_ok_valid and getattr(self, "_hold_prev", False):
            s["seq5"] = True
        self._hold_prev = hold

        # ---- Q6: resolved conflict (arb within <=2 cycles after conflict) ----
        if self._arb_conflict_val():
            self._conf2 = 2
        if self._conf2 > 0:
            if self.arb_valid:
                s["seq6"] = True
            self._conf2 -= 1

        # ---- Q7: arbitration active for >=2 consecutive cycles ----
        if self.arb_valid:
            self.s_arb_cont += 1
            if self.s_arb_cont >= 2:
                s["seq7"] = True
        else:
            self.s_arb_cont = 0

        # ---- Q8: DONE observed (channel completes) ----
        if any_done:
            s["seq8"] = True

        # ---- Q9: global idle (no request, no hold) for >=3 cycles ----
        if (all(fsm[i] == IDLE for i in range(N_CH))
                and not any(got) and not hold):
            self.s_idle_cont += 1
            if self.s_idle_cont >= 3:
                s["seq9"] = True
        else:
            self.s_idle_cont = 0

        # ---- Q10: ACTIVE->DONE within <=2 cycles and no hold ----
        for i in range(N_CH):
            if fsm[i] == ACTIVE and not hold:
                self._a2d_cnt[i] += 1
            elif fsm[i] == DONE and not got[i] and not hold:
                if 1 <= self._a2d_cnt[i] <= 2:
                    s["seq10"] = True
                self._a2d_cnt[i] = 0
            else:
                self._a2d_cnt[i] = 0

        # ---- Q11: two different channels granted in consecutive cycles ----
        if self.arb_valid and not self.hold:
            w = self.arb_winner
            if getattr(self, "_pw_valid", False) and self._pw is not None and self._pw != w:
                s["seq11"] = True
            self._pw = w
            self._pw_valid = True
        else:
            self._pw_valid = False

        # ---- Q12: done_ok class consistent with active channel ----
        if self.done_ok_valid:
            if self.cov_active_ch == self.done_ok:
                s["seq12"] = True

    def read_signals(self) -> Dict:
        return {
            "ch0_state": self.fsm[0],
            "ch1_state": self.fsm[1],
            "ch2_state": self.fsm[2],
            "ch3_state": self.fsm[3],
            "arb_winner": self.arb_winner,
            "arb_valid": self.arb_valid,
            "active_ch": self.cov_active_ch,
            "dir_dir": (self.cov_dir >> 3) & 0x1,
            "cfg_burst": self.cov_burst,
            "cfg_saddr": self.cov_saddr,
            "cfg_daddr": self.cov_daddr,
            "cfg_len": self.cov_len,
            "arb_conflict": self._arb_conflict_val(),
            "hold": 1 if self.hold else 0,
            "done_ok_valid": self.done_ok_valid,
            "done_ok": self.done_ok,
            # temporal sequence coverpoint hit flags (1 if the sequence fired)
            "seq1": 1 if self._seq_ok.get("seq1") else 0,
            "seq2": 1 if self._seq_ok.get("seq2") else 0,
            "seq3": 1 if self._seq_ok.get("seq3") else 0,
            "seq4": 1 if self._seq_ok.get("seq4") else 0,
            "seq5": 1 if self._seq_ok.get("seq5") else 0,
            "seq6": 1 if self._seq_ok.get("seq6") else 0,
            "seq7": 1 if self._seq_ok.get("seq7") else 0,
            "seq8": 1 if self._seq_ok.get("seq8") else 0,
            "seq9": 1 if self._seq_ok.get("seq9") else 0,
            "seq10": 1 if self._seq_ok.get("seq10") else 0,
            "seq11": 1 if self._seq_ok.get("seq11") else 0,
            "seq12": 1 if self._seq_ok.get("seq12") else 0,
        }


if __name__ == "__main__":
    dut = DmaXferPublic()
    print("dma_xfer_public local_sim loaded")
