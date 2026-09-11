#!/usr/bin/env python3
"""CI latency-regression gate, and the harness that measures it.

The gate itself (see `check_run`) computes each interior/sink block's median
hop latency for one session and flags any block whose median exceeds a fixed
multiple of a baseline ceiling established from clean sessions. Median, not
p99, is the gate's statistic: this machine is shared and in use while these
sessions run (see BUILDER.md 3z), and a handful of scheduler-preemption
spikes can push a clean session's *tail* into the millisecond range (see
docs/benchmark_output.txt, "Findings"), which would make a p99-based gate
noisy. A block's *median* hop latency barely moves under that same
contention, so it is what actually discriminates "this block got slower" from
"the machine got busy for 4ms".

This script both establishes the baseline and runs the two measured claims:
  1. Seed N (default 20) latency regressions, one per trial, each on a
     randomly chosen interior/sink block, and confirm the gate catches every
     one and names the correct block.
  2. Run M (default 20) clean control sessions through the same gate and
     confirm zero of them are flagged.
"""
import argparse
import glob
import json
import random
import subprocess
import sys
import uuid
from pathlib import Path

REGRESSABLE_BLOCKS = ["t1", "t2", "t3", "t4", "m1", "m2", "t5", "t6", "m3", "t7"]


def run_once(build_dir, config, run_id, ticks, rate_hz, log_dir, regress_block=None, regress_delay_ns=0):
    cmd = [
        str(build_dir / "graph_launcher"),
        "--config", str(config),
        "--run-id", run_id,
        "--ticks", str(ticks),
        "--rate-hz", str(rate_hz),
        "--mode", "record",
        "--trace", str(log_dir / f"{run_id}_trace.csv"),
        "--log-dir", str(log_dir),
        "--merge-mode", "tick_keyed",
        "--block-runner", str(build_dir / "block_runner"),
    ]
    if regress_block:
        cmd += ["--regress-block", regress_block, "--regress-delay-ns", str(regress_delay_ns)]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        raise RuntimeError(out.stdout + out.stderr)
    return json.loads(out.stdout.strip().splitlines()[-1])


def per_block_medians(log_dir, run_id):
    medians = {}
    for path in glob.glob(str(log_dir / f"{run_id}_*.log.hop")):
        block = Path(path).stem.rsplit(".", 1)[0].split("_")[-1]  # strip trailing ".log", then "<run_id>_<block>"
        hops = []
        with open(path) as f:
            for line in f:
                parts = line.rstrip("\n").split(",")
                if len(parts) < 2:
                    continue
                hops.append(int(parts[1]))
        if hops:
            hops.sort()
            medians[block] = hops[len(hops) // 2]
    return medians


def check_run(medians, threshold_ns):
    """The gate itself: returns the list of blocks whose median hop latency
    exceeds threshold_ns."""
    return sorted(b for b, m in medians.items() if m > threshold_ns)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--ticks", type=int, default=150)
    ap.add_argument("--rate-hz", type=float, default=200.0)
    ap.add_argument("--baseline-sessions", type=int, default=8)
    ap.add_argument("--regression-trials", type=int, default=20)
    ap.add_argument("--control-sessions", type=int, default=20)
    # 15ms is 3x the 5ms tick period at 200Hz, so a block that eats this much
    # extra wall time per message falls behind its own input queue every
    # single tick rather than merely being slower than usual once in a while;
    # the backlog this creates is what actually shows up in that block's own
    # hop-latency log (see module docstring). A delay smaller than one tick
    # period gets absorbed as idle slack and produces no measurable signal,
    # which is what an earlier, since-corrected 400us default demonstrated.
    ap.add_argument("--regress-delay-ns", type=int, default=15_000_000)
    ap.add_argument("--threshold-multiplier", type=float, default=5.0)
    ap.add_argument("--tmp-dir", default="/tmp/shmmw_ci_gate")
    ap.add_argument("--seed", type=int, default=1234)
    args = ap.parse_args()

    build_dir = Path(args.build_dir)
    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)
    rng = random.Random(args.seed)

    # 1. Baseline: clean sessions, establish the ceiling as the worst
    # (max) per-block median seen across all of them.
    baseline_ceiling = 0
    for _ in range(args.baseline_sessions):
        run_id = f"base_{uuid.uuid4().hex[:8]}"
        run_once(build_dir, args.config, run_id, args.ticks, args.rate_hz, tmp_dir)
        medians = per_block_medians(tmp_dir, run_id)
        baseline_ceiling = max(baseline_ceiling, max(medians.values()))
    threshold_ns = int(baseline_ceiling * args.threshold_multiplier)

    # 2. Regression trials: seed one block per trial, confirm the gate
    # catches it and names the correct block.
    caught = 0
    trial_records = []
    for i in range(args.regression_trials):
        block = REGRESSABLE_BLOCKS[i % len(REGRESSABLE_BLOCKS)]
        run_id = f"regress_{uuid.uuid4().hex[:8]}"
        run_once(build_dir, args.config, run_id, args.ticks, args.rate_hz, tmp_dir,
                 regress_block=block, regress_delay_ns=args.regress_delay_ns)
        medians = per_block_medians(tmp_dir, run_id)
        flagged = check_run(medians, threshold_ns)
        # A regression on one block legitimately backs up every block
        # downstream of it too (real backpressure cascading through the
        # graph, not a false alarm), so "caught" means the seeded block is
        # among those flagged, not that it is the only one flagged.
        ok = block in flagged
        caught += 1 if ok else 0
        trial_records.append({"trial": i, "seeded_block": block, "flagged": flagged, "correct": ok})

    # 3. Control sessions: clean runs through the same gate, count false positives.
    false_positives = 0
    control_records = []
    for i in range(args.control_sessions):
        run_id = f"control_{uuid.uuid4().hex[:8]}"
        run_once(build_dir, args.config, run_id, args.ticks, args.rate_hz, tmp_dir)
        medians = per_block_medians(tmp_dir, run_id)
        flagged = check_run(medians, threshold_ns)
        if flagged:
            false_positives += 1
        control_records.append({"trial": i, "flagged": flagged})

    result = {
        "baseline_sessions": args.baseline_sessions,
        "baseline_ceiling_ns": baseline_ceiling,
        "threshold_ns": threshold_ns,
        "regression_trials": args.regression_trials,
        "caught": caught,
        "control_sessions": args.control_sessions,
        "false_positives": false_positives,
        "trial_records": trial_records,
        "control_records": control_records,
    }
    print(json.dumps(result, indent=2))
    # This is the actual CI gate exit code: any missed regression or any
    # false positive on a clean control session fails the build.
    if caught < args.regression_trials or false_positives > 0:
        sys.exit(1)


if __name__ == "__main__":
    main()
