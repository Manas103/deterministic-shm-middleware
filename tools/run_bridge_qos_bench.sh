#!/bin/bash
# Runs one FIFO-or-EDF trial of bridge_qos_bench across the hosta/hostb
# network namespaces tools/setup_netns_bridge.sh created. Starts the
# receiver in the background, waits for it to exit (it exits itself on the
# sender's "done" sentinel), and prints its JSON result. Kills nothing by
# PID pattern-matching; the only backgrounded process is the one this
# script itself started and $!-captured.
set -e
cd "$(dirname "$0")/.."
POLICY="${1:-fifo}"
DURATION="${2:-5}"
PORT="${3:-9500}"

OUT=$(mktemp)
sudo ip netns exec hostb ./build/bridge_qos_bench --role receiver --policy "$POLICY" --peer-ip 10.77.0.1 --port "$PORT" > "$OUT" 2>&1 &
RECV_PID=$!

sleep 0.3

sudo ip netns exec hosta ./build/bridge_qos_bench --role sender --policy "$POLICY" --peer-ip 10.77.0.2 --port "$PORT" --duration-sec "$DURATION"

wait "$RECV_PID" || true
cat "$OUT"
rm -f "$OUT"
