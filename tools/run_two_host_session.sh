#!/bin/bash
# Runs the unmodified 12-block graph (configs/graph_12block.cfg) split
# across the hosta/hostb network namespaces at the m3_t7 edge: host A runs
# blocks S..m3 (10 blocks), host B runs t7,sink (2 blocks), the crossing
# edge goes over UDP (udp_edge_link.hpp/mixed_transport.hpp) instead of
# shared memory. Prints the one combined session digest a single-host run
# of the same config and trace would have produced.
set -e
cd "$(dirname "$0")/.."
RUN_ID="${1:-twohost1}"
TICKS="${2:-100}"
MODE="${3:-record}"
TRACE="${4:-/tmp/${RUN_ID}.trace}"

HOST_A_BLOCKS="S,t1,t2,t3,t4,m1,m2,t5,t6,m3"
HOST_B_BLOCKS="t7,sink"

sudo ip netns exec hostb ./build/graph_launcher \
  --config configs/graph_12block.cfg --run-id "${RUN_ID}_b" --ticks "$TICKS" --rate-hz 200 \
  --block-runner ./build/block_runner \
  --only-blocks "$HOST_B_BLOCKS" --log-dir /tmp \
  --bridge-edge m3_t7 --bridge-role receiver --bridge-listen-port 9900 \
  > /tmp/${RUN_ID}_b.out 2>&1 &
HOSTB_PID=$!

sleep 0.3

sudo ip netns exec hosta ./build/graph_launcher \
  --config configs/graph_12block.cfg --run-id "${RUN_ID}_a" --ticks "$TICKS" --rate-hz 200 \
  --block-runner ./build/block_runner \
  --mode "$MODE" --trace "$TRACE" \
  --only-blocks "$HOST_A_BLOCKS" --log-dir /tmp \
  --bridge-edge m3_t7 --bridge-role sender --bridge-peer-ip 10.77.0.2 --bridge-peer-port 9900 \
  > /tmp/${RUN_ID}_a.out 2>&1

wait "$HOSTB_PID" || true
cat /tmp/${RUN_ID}_a.out
cat /tmp/${RUN_ID}_b.out

./build/combine_digest configs/graph_12block.cfg \
  "$HOST_A_BLOCKS" "${RUN_ID}_a" /tmp \
  "$HOST_B_BLOCKS" "${RUN_ID}_b" /tmp
