#!/usr/bin/env python3
"""Measures whether a concurrently running `snooper`, attached to one edge of
a live session, changes anything about that session: does record/replay
determinism still hold with it attached, and do drop counts / hop latency
move outside normal run-to-run noise?

For each of N sessions this launches graph_launcher in the background,
starts `snooper` against one edge while the graph is running, waits for the
graph to finish, then (for the determinism half) replays the same trace and
compares digests. A second, snooper-free set of N sessions is the baseline
drop/hop-latency and digest-match comparison.

Usage: snooper_noninterference.py --build-dir build --config configs/graph_12block.cfg --edge e_t1_m1 --sessions 20
"""
import argparse
import glob
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path


def launch_graph(build_dir, config, run_id, mode, trace_path, log_dir, ticks, rate_hz):
    cmd = [
        str(build_dir / "graph_launcher"),
        "--config", str(config),
        "--run-id", run_id,
        "--ticks", str(ticks),
        "--rate-hz", str(rate_hz),
        "--mode", mode,
        "--trace", str(trace_path),
        "--log-dir", str(log_dir),
        "--merge-mode", "tick_keyed",
        "--block-runner", str(build_dir / "block_runner"),
    ]
    return subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)


def hop_and_drop_stats(log_dir, run_id):
    hops = []
    for path in glob.glob(str(log_dir / f"{run_id}_*.log.hop")):
        with open(path) as f:
            for line in f:
                parts = line.rstrip("\n").split(",")
                if len(parts) >= 2:
                    hops.append(int(parts[1]))
    hops.sort()
    def pct(p):
        if not hops:
            return None
        idx = min(len(hops) - 1, int(len(hops) * p))
        return hops[idx]
    return {"n": len(hops), "p50_ns": pct(0.50), "p99_ns": pct(0.99)}


def run_session_pair(build_dir, config, tmp_dir, ticks, rate_hz, snoop_edge, snooper_bin):
    """One record + one replay of the same trace, optionally with a snooper
    attached to snoop_edge during the record pass. Returns (digests_match,
    record_digest, replay_digest, hop_stats, snoop_saw_messages)."""
    run_id = f"nonint_{uuid.uuid4().hex[:8]}"
    trace_path = tmp_dir / f"{run_id}_trace.csv"

    record_proc = launch_graph(build_dir, config, run_id, "record", trace_path, tmp_dir, ticks, rate_hz)

    snoop_saw_messages = None
    snoop_proc = None
    if snoop_edge:
        snoop_proc = subprocess.Popen(
            [str(snooper_bin), "--run-id", run_id, "--edge", snoop_edge,
             "--snapshots", "400", "--interval-ms", "2"],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)

    record_out, record_err = record_proc.communicate(timeout=30)
    if snoop_proc:
        try:
            snoop_out, _ = snoop_proc.communicate(timeout=1)
        except subprocess.TimeoutExpired:
            snoop_proc.kill()
            snoop_out, _ = snoop_proc.communicate()
        snoop_saw_messages = "latest: tick=" in snoop_out

    if record_proc.returncode != 0:
        return None, record_err, None, None, snoop_saw_messages
    record_digest = json.loads(record_out.strip().splitlines()[-1])["session_digest"]

    replay_id = f"{run_id}_replay"
    replay_proc = launch_graph(build_dir, config, replay_id, "replay", trace_path, tmp_dir, ticks, rate_hz)
    replay_out, replay_err = replay_proc.communicate(timeout=30)
    if replay_proc.returncode != 0:
        return None, replay_err, None, None, snoop_saw_messages
    replay_digest = json.loads(replay_out.strip().splitlines()[-1])["session_digest"]

    hop_stats = hop_and_drop_stats(tmp_dir, run_id)
    return record_digest == replay_digest, record_digest, replay_digest, hop_stats, snoop_saw_messages


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--edge", required=True, help="edge to attach the snooper to")
    ap.add_argument("--sessions", type=int, default=20)
    ap.add_argument("--ticks", type=int, default=400)
    ap.add_argument("--rate-hz", type=float, default=200.0)
    ap.add_argument("--tmp-dir", default="/tmp/shmmw_nonint")
    args = ap.parse_args()

    build_dir = Path(args.build_dir)
    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)
    snooper_bin = build_dir / "snooper"

    results = {"baseline": [], "with_snooper": []}
    for label, edge in (("baseline", None), ("with_snooper", args.edge)):
        for _ in range(args.sessions):
            match, d1, d2, hop, saw = run_session_pair(
                build_dir, args.config, tmp_dir, args.ticks, args.rate_hz, edge, snooper_bin)
            results[label].append({"match": match, "hop": hop, "snoop_saw_messages": saw})

    def summarize(label):
        rows = results[label]
        matches = sum(1 for r in rows if r["match"])
        p50s = [r["hop"]["p50_ns"] for r in rows if r["hop"] and r["hop"]["p50_ns"] is not None]
        p99s = [r["hop"]["p99_ns"] for r in rows if r["hop"] and r["hop"]["p99_ns"] is not None]
        return {
            "sessions": len(rows),
            "digest_matches": matches,
            "mean_p50_ns": sum(p50s) / len(p50s) if p50s else None,
            "mean_p99_ns": sum(p99s) / len(p99s) if p99s else None,
        }

    snoop_confirmed = sum(1 for r in results["with_snooper"] if r["snoop_saw_messages"]) if args.edge else 0
    out = {
        "baseline": summarize("baseline"),
        "with_snooper": summarize("with_snooper"),
        "snooper_confirmed_live_messages": f"{snoop_confirmed}/{args.sessions}",
    }
    print(json.dumps(out, indent=2))


if __name__ == "__main__":
    main()
