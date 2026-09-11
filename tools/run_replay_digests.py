#!/usr/bin/env python3
"""Measures the headline replay-determinism claim: record a live session,
replay it from the recorded source trace, and check the two runs produce an
identical SHA-256 session digest. Repeats N times (default 100) and reports
how many sessions matched.

Each iteration uses a fresh --run-id so shared-memory segment names never
collide between iterations, and a fresh trace/log directory under /tmp.
"""
import argparse
import json
import subprocess
import sys
import time
import uuid
from pathlib import Path


def run_session(build_dir, config, run_id, ticks, rate_hz, mode, trace_path, log_dir, merge_mode):
    cmd = [
        str(build_dir / "graph_launcher"),
        "--config", str(config),
        "--run-id", run_id,
        "--ticks", str(ticks),
        "--rate-hz", str(rate_hz),
        "--mode", mode,
        "--trace", str(trace_path),
        "--log-dir", str(log_dir),
        "--merge-mode", merge_mode,
        "--block-runner", str(build_dir / "block_runner"),
    ]
    out = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if out.returncode != 0:
        return None, out.stdout + out.stderr
    return json.loads(out.stdout.strip().splitlines()[-1]), None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--build-dir", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--sessions", type=int, default=100)
    ap.add_argument("--ticks", type=int, default=200)
    ap.add_argument("--rate-hz", type=float, default=200.0)
    ap.add_argument("--merge-mode", default="tick_keyed")
    ap.add_argument("--tmp-dir", default="/tmp/shmmw_replay")
    ap.add_argument("--out", default=None, help="write raw JSON results here")
    args = ap.parse_args()

    build_dir = Path(args.build_dir)
    config = Path(args.config)
    tmp_dir = Path(args.tmp_dir)
    tmp_dir.mkdir(parents=True, exist_ok=True)

    matches = 0
    results = []
    t_start = time.time()
    for i in range(args.sessions):
        uid = uuid.uuid4().hex[:8]
        record_id = f"rec_{uid}"
        replay_id = f"rep_{uid}"
        trace_path = tmp_dir / f"trace_{uid}.csv"

        rec, err = run_session(build_dir, config, record_id, args.ticks, args.rate_hz,
                                "record", trace_path, tmp_dir, args.merge_mode)
        if rec is None:
            print(f"session {i}: RECORD FAILED: {err}", file=sys.stderr)
            results.append({"i": i, "ok": False, "stage": "record", "error": err})
            continue

        rep, err = run_session(build_dir, config, replay_id, args.ticks, args.rate_hz,
                                "replay", trace_path, tmp_dir, args.merge_mode)
        if rep is None:
            print(f"session {i}: REPLAY FAILED: {err}", file=sys.stderr)
            results.append({"i": i, "ok": False, "stage": "replay", "error": err})
            continue

        match = rec["session_digest"] == rep["session_digest"]
        matches += match
        results.append({
            "i": i, "ok": True, "match": match,
            "record_digest": rec["session_digest"], "replay_digest": rep["session_digest"],
        })
        if not match:
            print(f"session {i}: DIGEST MISMATCH record={rec['session_digest']} replay={rep['session_digest']}")

    elapsed = time.time() - t_start
    summary = {
        "sessions": args.sessions,
        "matches": matches,
        "merge_mode": args.merge_mode,
        "ticks": args.ticks,
        "rate_hz": args.rate_hz,
        "elapsed_sec": round(elapsed, 2),
    }
    print(json.dumps(summary))
    if args.out:
        Path(args.out).write_text(json.dumps({"summary": summary, "results": results}, indent=2))
    return 0 if matches == args.sessions else 1


if __name__ == "__main__":
    sys.exit(main())
