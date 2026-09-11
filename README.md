# Deterministic Shared-Memory Middleware with Record and Replay

A statically configured compute graph of processes that talk only through
POSIX shared memory, never a socket, with a record mode that captures a
session and a replay mode that re-derives it from the recording. C++20,
CMake, Python tooling, GitHub Actions. Every number below was measured on
this machine, not targeted; where a measurement fell short of what the
project set out to show, that is reported honestly rather than adjusted.

## Why this exists

This is a small version of the thing a real autonomy middleware has to be:
a fixed set of compute blocks wired together once at load time, moving
typed messages through shared memory instead of a network stack, with
enough live introspection that an operator does not need to attach a
debugger to see that a block is falling behind. It is the same shape as
"static configuration instead of dynamic node creation" and "typed
messages over shared memory instead of a network transport", stated
directly as design goals for exactly this kind of system.

## What this is, honestly

- **A record/replay determinism claim, not a real-time guarantee.** The
  100-of-100 identical-digest result (below) means the same recorded input
  drives the graph to the same output every time, byte for byte. It says
  nothing about worst-case scheduling.
- **A single-machine IPC fabric, not a distributed system.** Every block is
  a separate OS process on one host, attached to POSIX shared memory
  segments (`shm_open`/`mmap`). There is no cross-host transport at all.
- **A CI gate script, not a running GitHub Actions history.** `.github/workflows/ci.yml`
  exists and runs the exact commands below; it has not been exercised on
  GitHub's runners as part of this build.
- **Synthetic traffic.** The 12-block graph's source emits a deterministic
  sine/cosine signal (`sin(tick*0.01)*100`, `cos(tick*0.017)*50`); there is
  no real sensor or market data anywhere in this repository.

**Machine and toolchain.** WSL2 Ubuntu 22.04 on Windows 11, 12 logical
cores and 12 GB RAM as capped by `.wslconfig` (the physical host has more;
this is what the guest actually got), g++ 11.4.0, CMake, `-O2` for the
Release measurements, Python 3.10 for the measurement tooling. This is a
shared, in-use machine, not a dedicated benchmark rig; see "Findings" and
"Measured results" for what that does to the tail of a latency
distribution.

## Architecture

```
include/shmmw/
  message.hpp      Fixed 64-byte Message struct; the wire format is the
                    compiler's struct layout, not a runtime schema.
  ring_buffer.hpp   Single-producer/single-consumer ring over a raw shared
                    memory pointer; drop-newest-on-full with drop and
                    backpressure counters.
  shm_segment.hpp   RAII wrapper around shm_open/mmap. create() is called
                    only by graph_launcher, before any block execs;
                    attach() is called by every block, with a short retry
                    loop for the exec-then-shm_open race.
  stats.hpp         BlockStats: msgs_in/out, drops, backpressure, last and
                    EWMA hop latency, a heartbeat counter. One instance per
                    block, in its own shared memory segment, read by
                    `monitor` without any cooperation from the block.
  graph_config.hpp  The static topology type (blocks, edges, adjacency).
  clock.hpp         CLOCK_MONOTONIC helpers, including an absolute-deadline
                    sleep so a paced source does not drift over a long run.
src/
  graph_config.cpp  Parses the `<id> <name> <type> <in> <out>` config format.
  block_runner.cpp  The one executable every block in the graph runs. Which
                    of source/transform/merge/sink it behaves as is decided
                    entirely by its config entry.
  graph_launcher.cpp The only process that creates shared memory. Creates
                    every edge and stats segment, forks+execs one
                    block_runner per block, waits, then hashes every
                    block's message log into one session digest.
  monitor.cpp       A separate, read-only process that attaches to every
                    block's stats segment and prints a live table: msgs
                    in/out, drops, backpressure, last and EWMA hop latency.
configs/
  graph_12block.cfg The benchmark topology: source -> 4 parallel transforms
                    -> 2 merges -> 2 transforms -> 1 merge -> 1 transform ->
                    sink. Two real fan-in points, so both merge
                    implementations actually get exercised.
tools/
  run_replay_digests.py  Drives N record/replay session pairs, compares
                    digests, this is the 100-of-100 measurement.
  hop_latency_stats.py   Runs one session, computes hop-latency percentiles
                    from every non-source block's `.hop` file.
  ci_latency_gate.py     Establishes a baseline, seeds 20 per-block latency
                    regressions, and confirms the gate catches all of them
                    with zero false positives on clean control sessions.
tests/
  test_ring_buffer.cpp  Push/pop, drop-on-full, backpressure threshold,
                    wraparound, and a genuine two-thread producer/consumer
                    stress test (200,000 messages).
  test_sha256.cpp   FIPS 180-4 published test vectors, plus determinism,
                    avalanche, and incremental-vs-one-shot update checks.
```

### Design deep-dives

**Why a separate `.hop` file instead of putting hop latency in the message
log.** The message log is exactly what `graph_launcher` hashes into the
session digest, so anything in it has to be a deterministic function of the
recorded trace. Hop latency is wall-clock elapsed time between send and
receive, which is different on every run by construction. An earlier
version of this code put hop latency in the same row as the message log;
see "Findings" for what that did to the replay claim.

**Why tick-keyed merge instead of arrival-order merge.** A merge block has
two upstream edges; if it pairs "whichever two messages I most recently
popped" (arrival order), one dropped message on either edge permanently
shifts the pairing for every tick after it. Buffering each side by its
logical tick number and only combining a matching pair means one drop
leaves exactly one tick uncombined and desynchronizes nothing else. Both
modes are implemented (`--merge-mode arrival_order|tick_keyed`) so the
failure mode is demonstrable, not just asserted; the benchmark and replay
numbers below use `tick_keyed`.

**Why a start barrier.** `fork()`+`exec()` gives no guarantee about when a
child has finished attaching its shared memory. A paced 200 Hz source that
starts emitting before every consumer has attached fills a 64-slot ring and
starts dropping before anyone is listening. Every block increments a shared
`ready_count` once it has attached everything it needs and spins until the
count reaches the total block count, so tick 0 is never emitted until every
consumer for it already exists.

**Why the CI gate uses median hop latency, not p99.** This machine is
shared and in use while these sessions run. A handful of scheduler
preemption events can push a clean session's *tail* latency into the
millisecond range (see "Measured results") without any block actually
having regressed. A block's median hop latency barely moves under that same
contention, so it is what actually discriminates "this block got slower"
from "the machine got busy for a few milliseconds". See
`tools/ci_latency_gate.py` for the full reasoning.

## Validation

Unit tests (raw output in `docs/test_output.txt`):

```
$ ./build/check_ring_buffer
check_ring_buffer: 51961 checks passed
$ ./build/check_sha256
check_sha256: 7 checks passed
```

`check_sha256` diffs the hand-rolled SHA-256 implementation against the
published FIPS 180-4 test vectors; a homegrown hash used for a "identical
digest" correctness claim is worthless if it was never checked against a
known-correct implementation. `check_ring_buffer` includes a genuine
two-thread producer/consumer stress test (200,000 pushes) that checks every
successfully pushed message is eventually popped in order and every
non-pushed message is accounted for in the drop counter exactly once.

Sanitizers, WSL2 g++ 11.4 (raw output in `docs/tsan_clean_run.txt`):

```
AddressSanitizer + UBSan (Debug, -fsanitize=address,undefined): clean
ThreadSanitizer (Debug, -fsanitize=thread, setarch -R for ASLR off): clean
```

TSan only covers `check_ring_buffer`'s two-thread stress test; the
multi-process shared-memory path (block_runner/graph_launcher/monitor) is
inter-process, not inter-thread, so TSan's instrumentation does not apply to
it the same way. AddressSanitizer+UBSan were run on both test binaries.

## Findings

**The replay claim broke completely when hop latency moved into the digest
path, and the fix was to stop measuring it there.** While instrumenting hop
latency for the CI gate, the natural-looking change was to add a `hop_ns`
column to the same log row that `graph_launcher` hashes into the session
digest. Every one of the first 100 record/replay sessions then mismatched:
0 of 100, not 100 of 100. The wrong first hypothesis was a race in the
tick-keyed merge's map bookkeeping, since that is the most complicated
deterministic path in the codebase. The discriminating measurement was
simpler: diffing a record run's log against its own paired replay run line
by line showed every `tick,seq,src_block_id,payload` field matched exactly,
and only the trailing `hop_ns` field differed. Root cause: `hop_ns` is
`now_ns() - upstream_send_ts_ns`, wall-clock elapsed time, and wall-clock
time is never the same twice, live or replayed. It should never have been
part of anything that gets hashed for a determinism claim. The fix was to
give hop latency its own file per block (`<block>.log.hop`), excluded from
the digest aggregation entirely, which restored the 100-of-100 result
(confirmed in `docs/benchmark_output.txt`) without losing the measurement
the CI gate needs.

**A 400us injected delay produced zero measurable effect on hop latency,
and the reason is what "hop latency" actually measures here.** The first
version of the CI-gate latency-regression harness injected a fixed 400
microsecond processing delay into one block per trial and expected its hop
latency to rise. It did not: 4 of 20 seeded trials were "caught" by
incidental noise, not by the injected delay. The reason is that hop latency
here is edge transit time (`receive time - upstream send time`), and a
block's own `send_ts_ns` is stamped *after* its processing delay, not
before, so a slow block's own extra time never appears in anyone's hop
measurement unless it is slow enough to make messages back up in its input
queue before it gets to them. At 200 Hz the tick period is 5ms; a 400us
delay is well under that, so the block simply finished a little later
inside the same tick window and no backlog ever formed. Raising the
injected delay to 15ms, three times the tick period, guarantees the block
falls behind every single tick, so its own input queue backs up and its
*own* next hop measurement (queueing time, not the delay itself) climbs
by orders of magnitude. With that fix the gate caught 20 of 20 seeded
regressions.

**A regression on one block correctly cascades to every block downstream of
it, which is real backpressure, not a bug in the gate.** The first pass at
scoring "caught" required the gate to flag *only* the seeded block. It
never did: seeding `t1` also flagged `m1` and `m3`, which are genuinely
running behind because `t1` is feeding them late. The fix was to change the
success criterion to "the seeded block is among those flagged", which is
what the claim actually says (the gate caught the regression), not "the
gate flags nothing else" (which a real backpressure cascade will never
satisfy).

## Measured results

Machine: WSL2 Ubuntu 22.04 on Windows 11, 12 logical cores / 12 GB RAM
(WSL-capped), g++ 11.4.0, `-O2`, shared machine in active use during every
run below.

**Record/replay determinism, 12-block graph, 200 ticks, 200 Hz, tick-keyed
merge:**

**100 of 100 sessions replayed to an identical SHA-256 session digest.**
(`docs/benchmark_output.txt`)

**CI latency-regression gate, 20 seeded trials + 20 clean control sessions:**

**20 of 20 seeded regressions caught, 0 of 20 clean control sessions
falsely flagged.** (`docs/ci_gate_output.txt`)

**Hop latency, 12-block graph, 2000 ticks, 200 Hz, 3 genuine attempts (not
tuned; this is what was measured):**

| Attempt | p50 | p95 | p99 | max |
|---|---|---|---|---|
| 1 | 4,114 ns | 19,844 ns | 352,246 ns | 16,637,217 ns |
| 2 | 4,308 ns | 22,194 ns | 1,690,089 ns | 13,958,081 ns |
| 3 | 4,065 ns | 15,489 ns | 51,013 ns | 14,684,171 ns |

The p99 hop latency target for this project was 42 microseconds. **That
target was not met**; p99 across three genuine attempts ranged from 51us to
1.69ms, and every attempt's max reached into the tens of milliseconds. p50
is a tight, repeatable 4.0-4.3 microseconds across all three attempts, and
p95 is 15-22 microseconds, both consistent with a fast steady-state hop.
The p99 and max columns are dominated by a small number of scheduler
preemption events (see `tools/hop_latency_stats.py`'s companion analysis:
in one 2000-tick, 22,000-hop run, the 15 worst latencies all landed on
blocks `m2`/`m3` within a roughly 25-tick window, consistent with this
process losing the CPU for a few milliseconds on a machine that is shared
and in active use, not with a systematic architectural bottleneck). What
this measures: per-hop transit time between one block sending and the next
receiving, over shared memory, excluding the sending block's own compute
time (see "Findings" above for why that exclusion matters for the CI gate).
What it does not measure: end-to-end latency from the source tick to the
sink, which would be roughly the sum of every hop on the longest path and
was deliberately not the claim here.

## Building and running

```bash
# Release build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(( $(nproc)/2 ))"

# Unit tests
./build/check_ring_buffer
./build/check_sha256

# Sanitizers (separate build directories; TSan needs setarch -R under WSL2 for ASLR off)
cmake -S . -B build_asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build_asan -j"$(( $(nproc)/2 ))"
./build_asan/check_ring_buffer && ./build_asan/check_sha256

cmake -S . -B build_tsan -DCMAKE_BUILD_TYPE=Debug -DENABLE_TSAN=ON
cmake --build build_tsan --target check_ring_buffer -j"$(( $(nproc)/2 ))"
setarch -R ./build_tsan/check_ring_buffer

# Record/replay determinism (100 sessions)
python3 tools/run_replay_digests.py --build-dir build --config configs/graph_12block.cfg \
  --sessions 100 --ticks 200 --rate-hz 200

# Hop latency
python3 tools/hop_latency_stats.py --build-dir build --config configs/graph_12block.cfg \
  --ticks 2000 --rate-hz 200

# CI latency-regression gate (also what .github/workflows/ci.yml runs)
python3 tools/ci_latency_gate.py --build-dir build --config configs/graph_12block.cfg

# Live diagnostics on a running or just-finished session
./build/monitor --config configs/graph_12block.cfg --run-id <run-id> --snapshots 5 --interval-ms 200
```

## Sibling comparison

[market-data-tick-capture](https://github.com/Manas103/market-data-tick-capture)
also verifies bit-for-bit replay by SHA-256 digest, but over network-captured
multicast market data replayed from a column-oriented capture file; the
thing being proven identical there is a decoder's output against a stored
byte stream. Here there is no network and no stored byte stream to decode
against; the thing being proven identical is a live multi-process shared-memory
computation against itself, driven the second time from a recorded input
trace instead of a paced live source. The determinism guarantee in this repo
has to survive process scheduling, shared-memory queueing, and two different
merge orderings; the one in market-data-tick-capture has to survive a decode
pass. Both land on the same tool (a hand-rolled SHA-256 diffed against FIPS
test vectors) because it is the right tool for either problem.

## Limitations

- The p99 hop-latency target (42us) was not met; see "Measured results".
  p50 and p95 are tight and repeatable, but p99 and max are dominated by
  scheduler contention on a shared machine, and no attempt was made to pin
  cores or otherwise fight the host OS for priority, since doing so would
  cost the rest of the machine (see BUILDER.md 3z's resource-courtesy rule).
- The CI workflow (`.github/workflows/ci.yml`) has not actually been run on
  GitHub's hosted runners as part of this build; the commands in it are the
  exact commands verified locally above.
- TSan covers the concurrent ring-buffer unit test, not the real
  multi-process shared-memory path, since that path's concurrency is
  inter-process rather than inter-thread.
- The source block's synthetic signal (`sin`/`cos` of tick index) is
  simple by design; it is enough to exercise determinism and the transform
  chain, and it is not meant to resemble any real sensor.
- `monitor` polls at a fixed interval rather than blocking on a change
  notification; for a live operator tool this is a reasonable tradeoff, and
  it is stated here rather than left implicit.
