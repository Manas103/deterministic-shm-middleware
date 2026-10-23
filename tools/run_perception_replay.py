#!/usr/bin/env python3
"""Runs perception_bench --mode digest `--sessions` times with the same
seed and frame count, and checks every run produces the identical SHA-256
digest. This is the replay-determinism measurement reported in the
perception-extension README section.

Usage: python tools/run_perception_replay.py <build_dir> [--frames F] [--sessions S] [--seed SEED]
"""
import argparse
import subprocess
import sys
from pathlib import Path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("build_dir")
    ap.add_argument("--frames", type=int, default=300)
    ap.add_argument("--sessions", type=int, default=100)
    ap.add_argument("--seed", type=int, default=777)
    args = ap.parse_args()

    binary = Path(args.build_dir) / "perception_bench"
    if not binary.exists():
        print(f"perception_bench not found under {args.build_dir}", file=sys.stderr)
        sys.exit(1)

    digests = []
    for i in range(args.sessions):
        out = subprocess.run(
            [str(binary), "--mode", "digest", "--frames", str(args.frames), "--seed", str(args.seed)],
            capture_output=True, text=True, check=True,
        )
        digests.append(out.stdout.strip())

    unique = set(digests)
    matches = sum(1 for d in digests if d == digests[0])
    print(f"sessions={args.sessions} frames={args.frames} seed={args.seed}")
    print(f"unique digests: {len(unique)}")
    print(f"digest: {digests[0]}")
    print(f"{matches} of {args.sessions} sessions matched the first session's digest")
    if len(unique) != 1:
        sys.exit(1)


if __name__ == "__main__":
    main()
