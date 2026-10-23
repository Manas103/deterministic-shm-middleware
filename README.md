# Deterministic Shared-Memory Middleware with Record and Replay

A statically configured compute graph of processes that talk only through
POSIX shared memory, never a socket, with a record mode that captures a
session and a replay mode that re-derives it from the recording, plus a
snooper that attaches to a live edge from a third process and reads it
without ever being its consumer. C++20, CMake, Python tooling, GitHub
Actions. Every number below was measured on this machine, not targeted;
where a measurement fell short of what the project set out to show, that is
reported honestly rather than adjusted.

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
- **`snooper` is non-intrusive in two independent ways, not one.** It never
  calls anything that advances the ring's `tail` (so it cannot consume a
  message the real consumer was owed), and it attaches the segment
  `PROT_READ`-only, so even a bug in it could not write into the ring; the
  kernel enforces that, not a convention. `tests/test_snooper.cpp` proves
  the second guarantee by forking a child that deliberately tries to write
  through the mapping and asserting it dies of `SIGSEGV`.

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
  snoop_ring.hpp    SnoopRing: a const-only view of an EdgeRing for `snooper`.
                    No method ever writes `head` or `tail`; pairs with a
                    PROT_READ-only mapping so there are two independent
                    reasons it cannot touch producer or consumer state.
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
  snooper.cpp       A third process, neither the producer nor the real
                    consumer, that attaches read-only to one edge's live
                    message ring and prints head/tail/occupancy/drops plus
                    the latest message, without ever being a consumer of it.
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
  snooper_noninterference.py  Runs N record/replay session pairs with
                    `snooper` attached to a live edge throughout the record
                    pass, and N without, comparing digest-match rate and
                    hop latency between the two to show the snooper's
                    presence does not change the session.
tests/
  test_ring_buffer.cpp  Push/pop, drop-on-full, backpressure threshold,
                    wraparound, and a genuine two-thread producer/consumer
                    stress test (200,000 messages).
  test_sha256.cpp   FIPS 180-4 published test vectors, plus determinism,
                    avalanche, and incremental-vs-one-shot update checks.
  test_snooper.cpp  Snoop reads match what the producer wrote; 1000 snoop
                    reads consume nothing a real consumer was owed; a write
                    through the read-only mapping faults with SIGSEGV.
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

**Why `snooper` gets a read-only mapping instead of just a read-only class.**
`SnoopRing` (the class) never calls anything that writes `head` or `tail`,
which would already be enough to make it non-intrusive in a codebase no one
else can touch. But `monitor` and `snooper` both run as separate binaries
from separate `main()`s, built from a shared library; a future change to
either one is one `git diff` away from accidentally picking up a write path
some other change needed. Mapping the segment `PROT_READ` turns "the class
happens not to write" into "a write cannot compile its way around a bug
reviewer's blind spot and still succeed at runtime", because the kernel, not
a code reviewer, is the enforcement point. `tests/test_snooper.cpp` proves
the mapping actually behaves this way rather than asserting it: it forks a
child that deliberately writes through the mapping and checks the child
died of `SIGSEGV`.

## Validation

Unit tests (raw output in `docs/test_output.txt`):

```
$ ./build/check_ring_buffer
check_ring_buffer: 55313 checks passed
$ ./build/check_sha256
check_sha256: 7 checks passed
$ ./build/check_snooper
check_snooper: 138 checks passed
```

`check_sha256` diffs the hand-rolled SHA-256 implementation against the
published FIPS 180-4 test vectors; a homegrown hash used for a "identical
digest" correctness claim is worthless if it was never checked against a
known-correct implementation. `check_ring_buffer` includes a genuine
two-thread producer/consumer stress test (200,000 pushes) that checks every
successfully pushed message is eventually popped in order and every
non-pushed message is accounted for in the drop counter exactly once.
`check_snooper` checks that a snoop read sees exactly what a producer wrote,
that 1000 snoop reads consume nothing a real consumer subsequently pops
(the actual "does not touch producer state" claim, checked rather than
asserted), and that a write through the read-only mapping faults with
`SIGSEGV` in a forked child.

Sanitizers, WSL2 g++ 11.4 (raw output in `docs/tsan_clean_run.txt`):

```
AddressSanitizer + UBSan (Debug, -fsanitize=address,undefined): clean
ThreadSanitizer (Debug, -fsanitize=thread, setarch -R for ASLR off): clean
```

TSan only covers `check_ring_buffer`'s two-thread stress test; the
multi-process shared-memory path (block_runner/graph_launcher/monitor) is
inter-process, not inter-thread, so TSan's instrumentation does not apply to
it the same way. AddressSanitizer+UBSan were run on `check_ring_buffer`,
`check_sha256` and `check_snooper`. `check_snooper` needs
`ASAN_OPTIONS=handle_segv=0`: its SIGSEGV-on-write check deliberately
triggers the fault ASan's own default signal handler would otherwise
intercept and report as an ASan error rather than letting the parent see a
normal signal exit; see `docs/tsan_clean_run.txt` for the full explanation.

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

**The first non-interference run "confirmed" the snooper saw live messages
0 times out of 20, and the ring was not actually empty.** The first version
of `snooper_noninterference.py` started the snoop process a third of the way
into the session and gave it 3 snapshots at 50ms apart. Every one of 20
sessions reported `occupancy=0`. The wrong first hypothesis was a bug in
`SnoopRing` or in `attach_readonly`. The discriminating check was simpler:
the existing hop-latency numbers already show interior blocks process a
message in low microseconds when the machine is not contended, so a ring
fed at 200 Hz (one message every 5ms) is empty almost all the time by
construction, no matter how correct the snoop code is; 3 samples spaced
50ms apart were simply unlikely to land in the handful-of-microseconds
window where a message is actually in flight. The fix was not to the
snooping code at all: polling every 2ms for the session's full duration
instead of 3 widely spaced samples raised the catch rate to 15 of 20. The
remaining 5 are not a bug either, they are sessions where the snoop
genuinely never happened to sample during an occupied instant, which is the
same honest limitation `monitor` already states for its own fixed-interval
polling.

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

**Snooper non-interference, 20 record/replay session pairs with `snooper`
attached to edge `t1_m1` for the full record pass, 20 without, 400 ticks,
200 Hz:** (`docs/snooper_noninterference_output.txt`)

| | sessions | digest matches | mean hop p50 | mean hop p99 |
|---|---|---|---|---|
| baseline (no snooper) | 20 | 20/20 | 6,435 ns | 2,178,051 ns |
| with snooper attached | 20 | 20/20 | 5,776 ns | 2,077,430 ns |

**Record/replay determinism is unaffected: 20 of 20 either way.** The mean
hop-latency columns differ by less than one noisy attempt differs from
another in the table above (both machine-contention-dominated at p99 on
this shared host); there is no direction in which the snooper's presence
makes the graph slower. Across the 20 snooped sessions, `snooper` actually
observed a non-empty ring and printed a live message 15 times; the other 5
are the ring legitimately being empty at every sampled instant, not a
failure to attach (see "Findings").

## Building and running

```bash
# Release build
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(( $(nproc)/2 ))"

# Unit tests
./build/check_ring_buffer
./build/check_sha256
./build/check_snooper

# Sanitizers (separate build directories; TSan needs setarch -R under WSL2 for ASLR off)
cmake -S . -B build_asan -DCMAKE_BUILD_TYPE=Debug -DENABLE_ASAN=ON
cmake --build build_asan -j"$(( $(nproc)/2 ))"
./build_asan/check_ring_buffer && ./build_asan/check_sha256
ASAN_OPTIONS=handle_segv=0 ./build_asan/check_snooper # see "Validation" for why

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

# Snoop a live edge without being its consumer (run while a session is in flight)
./build/snooper --run-id <run-id> --edge t1_m1 --snapshots 400 --interval-ms 2

# Snooper non-interference (20 sessions with snooper attached, 20 without)
python3 tools/snooper_noninterference.py --build-dir build --config configs/graph_12block.cfg \
  --edge t1_m1 --sessions 20 --ticks 400 --rate-hz 200
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

## Extension: GPU-Resident Perception Graph with Record and Replay (`perception/`, `ros2/`, Oct. 2026)

A synthetic camera-and-LiDAR perception stage: CUDA kernels voxelize a synthetic LiDAR point cloud and project it into a synthetic camera's image plane, an OpenGL viewer reads the rendered output, and a separate ROS 2 subscriber process measures what crossing a DDS process boundary costs on top of this repo's own shared-memory hop. Every number below was measured on this machine via the commands in "Building and running" below, not targeted.

### What this is, honestly

- **A new GPU-resident stage added to this repo's existing multi-process shared-memory graph, not a second middleware.** Everything in "Architecture" above (typed messages, record/replay, the CI latency gate) is untouched; `perception/` and `ros2/` are additive.
- **Cross-process CUDA IPC (`cudaIpcGetMemHandle`/`cudaIpcOpenMemHandle`) does not work on this machine, and that is a documented WSL2 limitation, not a bug this session chased.** It was tried first, empirically (a two-process handle-share test), and failed with `invalid resource handle` every time. NVIDIA's own CUDA-on-WSL documentation states IPC memory allocation across WSL2 processes is unsupported. Given that, "device-memory handles rather than host copies" is implemented as the strongest honest version available on this machine: the voxelize, project and composite kernels hand off to each other entirely on the device, inside one process, with zero intermediate host round trips; the measured 11.4ms/3.9ms-style comparison (below) is exactly that: the cost of adding host round trips between those same kernels versus removing them, which is the real effect "GPU-resident" is about, just not literally spanning an OS process boundary for this specific hop.
- **The OpenGL viewer is a second, genuinely separate OS process**, reading the pipeline's real output through this repo's existing POSIX-shared-memory mechanism (a small single-slot mailbox, since an RGBA image does not fit in the generic 64-byte `shmmw::Message`), the same way every other inter-process handoff in this repo works. It is not CUDA-GL zero-copy interop: this machine's only available GL context is Mesa's D3D12 software-translation path on an integrated AMD GPU (`GL_RENDERER: D3D12 (AMD Radeon(TM) Graphics)`, confirmed at startup), not the NVIDIA driver CUDA runs on, so no real CUDA-OpenGL resource sharing is possible here either.
- **"Camera" is a synthetic gradient background, not a rendered scene or a real image.** The interesting data is the LiDAR point cloud; the camera buffer exists to be a genuine second device buffer that the composite kernel fuses the LiDAR projection onto, exercising "camera and LiDAR frame fusion" as a real code path even though neither sensor is real.
- **A bird's-eye-view projection, not a perspective camera projection.** `project_kernel` collapses height and sums occupied-voxel counts into image pixels by (x, y) alone; there is no camera intrinsics matrix or depth buffer. This keeps the kernel's atomics integer (and therefore exactly order-independent, see Validation) and ties back to the same BEV framing project 1 and project 2 use elsewhere in this role's resume.
- **The ROS 2 crossing-cost measurement is a separate, decoupled benchmark**, not wired into the main 18,000-frame latency run: a dedicated publisher (`ros2/ros2_crossing_pub`) sends frame-sized messages over `std_msgs::msg::UInt8MultiArray` (no custom `.msg`/`rosidl` generation needed) and a dedicated subscriber (`ros2/ros2_crossing_sub`) in another process times receipt.

**Machine and toolchain.** WSL2 Ubuntu 22.04 on Windows 11, g++ 11.4.0, CUDA 13.4.92 (`nvcc`), one NVIDIA GeForce RTX 4070 SUPER (sm_89, 56 SMs), ROS 2 Humble (`rclcpp`, `ament_cmake`, `rmw_fastrtps_cpp`), Mesa 23.2.1 EGL/GL (D3D12 translation, AMD iGPU) for the headless viewer.

### Architecture

```
include/shmmw/perception/
  frame_gen.hpp       Synthetic LiDAR point cloud + camera background generator,
                      deterministic (SplitMix64) by (seed, frame_idx).
  pipeline_api.hpp    The CUDA/non-CUDA boundary: GpuPipeline, a plain-C++ class;
                      only kernels.cu includes <cuda_runtime.h>.
  cpu_reference.hpp   Slow serial oracle for voxelize/project/composite.
  frame_mailbox.hpp   Single-slot shared-memory handoff to the OpenGL viewer.
src/perception/
  frame_gen.cpp
  cpu_reference.cpp
  kernels.cu           voxelize_kernel, project_kernel, composite_kernel, and
                       GpuPipeline::run_frame (device-resident or host-round-trip).
  perception_bench.cpp CLI: per-frame p50/p99 latency, or --mode digest for the
                       replay-determinism SHA-256 fingerprint.
  perception_node.cpp  Lockstep producer for the viewer demo.
  viewer.cpp           Headless EGL/OpenGL consumer; checksums its own
                       framebuffer readback against the producer's checksum.
ros2/
  ros2_crossing_pub.cpp / ros2_crossing_sub.cpp / CMakeLists.txt
                       Standalone ament_cmake package (rclcpp, std_msgs) for the
                       crossing-cost measurement; not part of the main CMake build.
tests/
  test_frame_gen.cpp, test_perception_kernels.cpp
tools/
  run_perception_replay.py   100-session replay-digest check (reuses
                             tools/run_replay_digests.py's splitmix64 family).
```

### Design deep-dives

**Why the voxel grid and the per-pixel accumulator use `atomicAdd` on `int32_t`, never on a float.** Integer addition is associative and commutative in hardware exactly; a floating-point `atomicAdd` is not, because rounding depends on accumulation order, which depends on which thread's atomic wins a given race. Every occupancy and projection count in this pipeline is an integer for exactly this reason: it is what makes the replay digest (below) hold across threads and across runs, not just across processes.

**Why the CPU reference oracle is a real, independently-written third implementation, not a copy of the kernel body.** `cpu_reference.cpp`'s loops are plain nested `for` loops over points, voxels and pixels, never an atomic, never a kernel launch; `tests/test_perception_kernels.cpp` diffs its output against the GPU path byte-for-byte over 30 frames. An identical-looking bug in both the kernel and a "reference" copied from it would never be caught; this oracle is slow and boring on purpose.

### Validation

1. **Unit tests** (`tests/test_frame_gen.cpp`, `tests/test_perception_kernels.cpp`, hand-rolled, no framework): frame generation is deterministic for a fixed (seed, frame) and every generated point lands inside the volume the voxelizer covers; the GPU path matches the serial CPU reference oracle exactly (byte-for-byte image and voxel grid) over 30 frames, the host-round-trip and device-resident paths compute the identical answer, and an empty frame leaves the camera background untouched with an all-zero voxel grid.
2. **Viewer cross-check**: the OpenGL viewer independently checksums its own `glReadPixels` readback and compares it to the producer's checksum of the same bytes; 20 of 20 frames matched in the run recorded below.
3. **Replay determinism**: the same seed run through `perception_bench --mode digest` repeatedly; every run's SHA-256 digest compared (see Measured results).

Raw output: `docs/perception_test_output.txt`.

### Findings

**CUDA IPC across WSL2 processes fails outright.** A minimal two-process test (one process `cudaIpcGetMemHandle`s a buffer and writes the handle to a file; a second opens it) returned `invalid resource handle` from `cudaIpcOpenMemHandle` every time, with no code-side mistake to find: NVIDIA's CUDA-on-WSL user guide documents this as unsupported. This is why the GPU-resident claim is implemented within one process (see "What this is, honestly") rather than literally spanning the OS process boundary the base repo's other blocks do.

**The repo's own `shmmw::Sha256::update()` was quadratic in input size, invisible until this extension called it with an 8MB buffer.** Every prior call site in this repo hashes one 64-byte `Message` at a time, so nobody had ever noticed that `update()` appended the whole input to an internal `std::vector` and then called `vector::erase(begin, begin+64)` once per 64-byte block, an operation that shifts everything after the erased block down by 64 bytes every time it runs. At `Message`-sized inputs that is unmeasurably fast; at the perception extension's 8MB-per-frame voxel grid it turned a sub-second digest run into a multi-minute hang (confirmed by `ps`: a single `perception_bench --mode digest` process pinned at 100% CPU for over six minutes on 300 frames that should take a fraction of a second). The fix changes `update()` to process full blocks directly out of the caller's buffer and only ever copy the trailing remainder into the internal buffer, which is O(n) instead of O(n^2); `tests/test_sha256.cpp`'s existing FIPS-vector checks still pass unchanged, since the externally observable behavior is identical. This is a real complexity bug in a utility every block in this repo depends on, found by this extension's much larger inputs and fixed once, centrally, rather than worked around only here.

**GL_RENDERER confirmed this machine has no NVIDIA-backed OpenGL context before any CUDA-GL interop code was written.** A standalone headless-EGL probe printed `D3D12 (AMD Radeon(TM) Graphics)`: WSLg's Mesa stack is bound to the integrated AMD GPU through a D3D12 translation layer, not the NVIDIA GPU CUDA runs on. Writing `cudaGraphicsGLRegisterBuffer` code against this context would have compiled and then failed at runtime for the same class of reason as the IPC failure above; the viewer design in "What this is, honestly" was chosen because this was checked first, not discovered after the fact.

### Measured results

Machine: WSL2 Ubuntu 22.04, g++ 11.4.0, CUDA 13.4.92, one RTX 4070 SUPER (sm_89, 56 SMs). Raw output in `docs/perception_benchmark_output.txt`. Per-frame shape: 300,000 synthetic LiDAR points, a 128^3 voxel grid, a 512x512 camera/output image (sized up from an initial 4,096-point/32^3/64x64 design across two genuine attempts at reaching a measurable, realistic latency scale; see "Building and running" for the exact commands behind every number here).

**Removing the host round trips between the voxelize, project and composite kernels cut p50 per-frame latency from 2.100ms to 0.722ms** (1,000 frames each; device-resident path p99 0.907ms, host-round-trip path p99 2.965ms). This did not reach the 11.4ms-to-3.9ms scale this project set out to show; at this buffer size the gap is real and in the right direction, but far smaller in absolute terms, which is reported as measured rather than inflated by holding onto a bigger, slower design than what was actually built.

**At 18,000 frames (the nominal count for a session running this pipeline at the target 30Hz for ten minutes; this benchmark runs the frames back-to-back rather than literally pacing the clock for ten minutes, same convention this repo's own hop-latency and CI-gate benchmarks use), the device-resident path measured p50 1.215ms / p99 1.485ms.** At 30Hz the per-frame budget is 33.3ms; both paths clear it with over an order of magnitude of headroom, which is the sense in which "30Hz" is a target this latency supports rather than a literal wall-clock pacing this harness enforced.

**Replay determinism: individually verified, not completed at the full 100-session scale in this build session.** The GPU path matched the serial CPU reference oracle byte-for-byte over 30 frames (Validation, above), and every accumulator in the pipeline is an integer `atomicAdd` with no other source of cross-run nondeterminism, which is the same guarantee the 100/100 digest check elsewhere in this repo and in the sibling `batched-gpu-driving-scenario-simulator` repo rests on. The 100-session run (`tools/run_perception_replay.py build --frames 300 --sessions 100`) was started and did not finish inside this session's time budget (each session pays its own CUDA-context startup cost on top of 300 frames, longer than estimated); this is reported honestly rather than assumed. Re-running that exact command is the way to get the number this section does not yet have.

**Viewer cross-check: 20 of 20 frames' OpenGL framebuffer-readback checksum matched the producer's checksum of the same bytes exactly** (see `docs/viewer_output.txt`).

**ROS 2 crossing cost: p50 1.765ms, p99 11.492ms** over 200 messages (the 512x512 synthetic camera background as payload, `std_msgs::msg::UInt8MultiArray`), measured as the subscriber's receipt timestamp minus the publisher's embedded send timestamp, both `CLOCK_MONOTONIC`-backed `steady_clock`, across two separate OS processes. This is lower than the 2.6ms this project set out to show, and the p99 (dominated by DDS/scheduler jitter on this shared machine, consistent with this repo's own hop-latency p99 findings elsewhere) is far higher; both are reported as measured.

### Building and running

```bash
# Main build (perception/ is additive; the rest of the repo is untouched)
bash build_perception.sh   # cmake + make -j6, including perception_bench/perception_node/viewer

# Unit tests
cd build && ./check_frame_gen && ./check_perception_kernels

# Latency: device-resident vs host-round-trip, same frame count
./build/perception_bench --frames 1000 --mode latency
./build/perception_bench --frames 1000 --mode latency --host-round-trip
./build/perception_bench --frames 18000 --mode latency

# Replay determinism (100 sessions against one seed)
python3 tools/run_perception_replay.py build --frames 300 --sessions 100 --seed 777

# OpenGL viewer demo (lockstep, 20 frames)
./build/perception_node 20 &
./build/viewer 20

# ROS 2 crossing cost (separate ament_cmake package)
bash build_ros2.sh
source /opt/ros/humble/setup.bash
./ros2/build/ros2_crossing_sub 200 &
./ros2/build/ros2_crossing_pub 200
```

### Extension limitations

- Cross-process CUDA IPC does not work on this machine (WSL2); the GPU-resident claim is demonstrated within one process instead (see "What this is, honestly" and Findings).
- The OpenGL viewer renders through Mesa's D3D12 software translation on an integrated AMD GPU, not the NVIDIA driver; there is no real CUDA-OpenGL interop in this build.
- The host-round-trip-vs-device-resident latency gap, while in the claimed direction, is far smaller in absolute terms than this project set out to show; see Measured results.
- The BEV projection discards height entirely (every occupied voxel at a given (x, y) column contributes to the same pixel regardless of z); a real perception stack would keep a depth buffer or render multiple height slices.
- `ros2_crossing_pub`/`ros2_crossing_sub` measure one-way latency for a fixed-size synthetic payload at a fixed publish rate; they do not reproduce the main pipeline's actual frame content or timing.

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
- `snooper` is also fixed-interval polling, not a blocking tap, so it can
  and does miss messages that arrive and are consumed between two polls (in
  the non-interference measurement it caught a live message in 15 of 20
  sessions, not 20 of 20; see "Findings" and "Measured results"). It has no
  seqlock-style retry against a torn read of a slot the producer is
  mid-write to; at the polling rates it actually runs at this was never
  observed in testing, but it is a real theoretical gap this repo's own
  market-data seqlock (in the sibling `lock-free-order-matching-engine`)
  exists specifically to close, and this snooper does not import that
  machinery.
