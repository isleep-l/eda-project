"""Run Quan's HFSM agent against the provided SPI Xfer RTL harness."""
import argparse
import hashlib
import json
from pathlib import Path

from harness import default_secrets, run_episode
from inference_interface import InferenceInterface


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--steps", type=int, default=20000)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--steps must be positive")

    here = Path(__file__).resolve().parent
    agent = InferenceInterface(
        dut_spec_path=str(here / "dut" / "dut_spec.md"),
        covergroup_path=str(here / "dut" / "covergroup.svh"),
        policy="hfsm",
        seed=7,
    )
    agent_sha256 = hashlib.sha256(
        (here / "inference_interface.py").read_bytes()
    ).hexdigest()
    print(f"START policy=hfsm backend=verilator steps={args.steps}", flush=True)
    print(f"agent_sha256={agent_sha256}", flush=True)
    curve = run_episode(
        agent,
        secrets=default_secrets(),
        max_steps=args.steps,
        interval=1000,
        backend="verilator",
    )
    for step, coverage in curve:
        print(f"cycle_index={step} coverage={float(coverage):.6f}")
    summary = {
        "policy": "hfsm",
        "backend": "verilator",
        "steps": args.steps,
        "seed": 7,
        "secrets_source": "harness.default_secrets (public local test)",
        "agent_sha256": agent_sha256,
        "final_coverage": float(curve[-1][1]),
        "curve": [[int(step), float(cov)] for step, cov in curve],
    }
    (here / "edatest_result.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    print("PASS: HFSM RTL simulation completed", flush=True)


if __name__ == "__main__":
    main()
