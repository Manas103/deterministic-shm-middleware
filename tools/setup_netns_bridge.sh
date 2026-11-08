#!/bin/bash
# Creates two network namespaces (hosta, hostb) connected by a veth pair,
# each side shaped to 1Gbps by tc, standing in for the "two hosts" the
# in-vehicle transport extension's bridge crosses. Idempotent: deletes any
# existing namespaces/links of the same name first. Run with sudo.
set -e

sudo ip netns del hosta 2>/dev/null || true
sudo ip netns del hostb 2>/dev/null || true
sudo ip link del veth-a 2>/dev/null || true

sudo ip netns add hosta
sudo ip netns add hostb

sudo ip link add veth-a type veth peer name veth-b
sudo ip link set veth-a netns hosta
sudo ip link set veth-b netns hostb

sudo ip netns exec hosta ip addr add 10.77.0.1/24 dev veth-a
sudo ip netns exec hostb ip addr add 10.77.0.2/24 dev veth-b

sudo ip netns exec hosta ip link set veth-a up
sudo ip netns exec hostb ip link set veth-b up
sudo ip netns exec hosta ip link set lo up
sudo ip netns exec hostb ip link set lo up

# Shape both directions to 1Gbps (tbf: token bucket filter). burst/latency
# chosen generously so a correctly-paced 1Gbps stream is not itself
# dropped by the qdisc; what the bench measures is queuing delay under the
# shaped rate, not qdisc-induced loss.
sudo ip netns exec hosta tc qdisc add dev veth-a root tbf rate 1000mbit burst 256kbit latency 400ms
sudo ip netns exec hostb tc qdisc add dev veth-b root tbf rate 1000mbit burst 256kbit latency 400ms

echo "hosta=10.77.0.1 hostb=10.77.0.2, both shaped to 1Gbps"
