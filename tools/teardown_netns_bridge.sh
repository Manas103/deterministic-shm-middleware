#!/bin/bash
sudo ip netns del hosta 2>/dev/null || true
sudo ip netns del hostb 2>/dev/null || true
echo "torn down"
