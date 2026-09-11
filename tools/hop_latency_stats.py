#!/usr/bin/env python3
"""Runs the 12-block graph once and computes hop-latency percentiles across
every non-source block's log rows (each row's last column is the hop_ns this
block measured on receipt: now() - upstream send_ts_ns). One run's p99 across
all interior/sink blocks is the number the README calls "p99 hop latency".

Usage: hop_latency_stats.py --build-dir build --config configs/graph_12block.cfg
"""
import argparse
import glob
import json
import subprocess
import sys
import uuid
from pathlib import Path


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
        return None, out.stdout + out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1]), None


def hop_percentiles(log_dir, run_id):
    hops = []
    for path in glob.glob(str(log_dir / f"{run_id}_*.log.hop")):
        with open(path) as f:
            for line in f:
                parts = line.rstrip("\n").split(",")
                if len(parts) < 2:
                    continue
                hops.append(int(parts[1]))
    if not hops:
        return None
    hops.sort()
    def pct(p):
        idx = min(len(hops) - 1, int(len(hops) * p))
        return hops[idx]
    return {
        "n": len(hops),
        "p50_ns": pct(0.50),
        "p95_ns": pct(0.95),
        "p99_ns": pct(0.99),
        "max_ns": hops[-1],
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--ticks", type=int, default=2000)
    ap.add_argument("--rate-hz", type=float, default=200.0)
    ap.add_argument("--tmp-dir", default="/tmp/shmmw_hoplat")
    args = ap.parse_args()

    build_dir = Path(args.build_dir)
    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)

    run_id = f"hoplat_{uuid.uuid4().hex[:8]}"
    result, err = run_once(build_dir, args.config, run_id, args.ticks, args.rate_hz, tmp_dir)
    if err:
        print(json.dumps({"error": err}))
        sys.exit(1)
    stats = hop_percentiles(tmp_dir, run_id)
    print(json.dumps({"run": result, "hop_latency": stats}))


if __name__ == "__main__":
    main()
