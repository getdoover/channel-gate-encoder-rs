# channel-gate-encoder (Rust)

A Rust rewrite of the Python [`channel-gate-encoder`](../channel-gate-encoder)
device app, on [doover-rs](https://github.com/getdoover/doover-rs) **0.1.3**.

Two proximity sensors watch a toothed target driven by a channel gate, mounted a
quarter-pitch apart so their outputs sit ~90° out of phase (channels A and B).
The app subscribes to **the rising edge of each pin, one subscription per pin**,
decodes 2×, maintains an absolute position homed against a limit switch, and
publishes position + direction to tags on a configurable timer (default 500 ms)
that is completely decoupled from the edge rate.

The Python app is the specification. This port is behaviour-for-behaviour with
it, and drop-in on config: `doover_config.json`'s `config_schema` keys, titles,
types, defaults and bounds are identical, and so are all 14 UI element names — an
existing `app_config.json` and an existing dashboard both keep working.

## Read this first

`src/quadrature.rs` — everything about the decode is a consequence of having no
falling edges and no level information:

- **Decoding is 2×, not 4×.** Two counts per full tooth cycle, so `mm_per_count`
  is **tooth pitch / 2** and the position granularity *is* `mm_per_count`. The
  shipped default is **2.0 mm** (a 4 mm tooth pitch). This is double the old
  both-edge value for the same target — halve the pitch to get the resolution
  back.
- **Direction is a TIMING measurement, not an ordering one.** Rising edges
  alternate A, B, A, B in *both* directions; reversing the target only swaps
  which of the two inter-channel gaps is the short one. On each edge the decoder
  compares `gap` (since the *other* channel rose) against `period` (since the
  *same* channel rose): `2·gap < period` means the other channel led. At 15
  rising edges/s per sensor that is discriminating **16.7 ms from 50.0 ms**, with
  the flip at 33.3 ms. That timing budget is the whole fragility of the design.
- **Ambiguity is reported, never guessed.** A gap within ±⅛ cycle of half a
  period carries no direction information (tolerating mountings from ~45° to
  ~135° of phase). Such an edge is still counted — travel did happen — but signed
  by the *held* direction and tallied in `AmbiguousEdges`. A growing
  `AmbiguousEdges` during motion is the signature of silent position corruption:
  no pulses lost, sign possibly wrong.
- **Cold start holds, it does not guess.** The first two edges of a run have no
  same-channel period to measure against, so they are held out of the count and
  retro-signed the moment a direction lands. Guessing would inject a systematic
  2-count error into every run that starts by closing.
- **A single lost edge would invert direction** — the "period" across the
  discontinuity spans three quarter cycles instead of four, flipping the gap
  comparison. The decoder therefore discards its timing state on any
  same-channel repeat and re-measures.
- **The `period` term comes from the firmware, not from host clocks.** On
  `doovit_fw` 1.9.1, `dt_secs` in rising-only mode *is* the full tooth period,
  and it is measured by a PIO state machine with no CPU-tick jitter
  (`dio.py:300-304`). The app reads it off the wire
  (`RisingEdgeDecoder::edge_with_period`). `dt_secs == 0.0` means "the firmware
  had none" — the first edge of a pin, or after a dropped transition — and is
  treated as *no period*, never as zero seconds.
- **A reversal costs 2 counts with the PIO period, 4 without.** The edge right
  after a turnaround has no *host* same-channel interval (the decoder just
  discarded its timing state) but it does have a real PIO-measured one, so it can
  be signed instead of held. That halves the cost, and 2 counts is exactly the
  irreducible geometric bound — a channel's rising edge sits at a different
  *physical* position depending on travel direction
  (`RisingEdgeDecoder::reversal_backlash_counts`), so no decoder can do better.
  The remaining error is deterministic, rate-independent, and does not self-heal
  because the count integrates. Only a home clears it.

## Shape

```text
 platform interface (gRPC :50053)
       │  startPulseCounter(di=A, edge="rising")   15 rising/s
       │  startPulseCounter(di=B, edge="rising")   15 rising/s
       ▼
 subscribe_di_pulses -> raw stream              one task per pin
       │  synchronous body, no awaits, no dt<=0 filter
       ▼
 EncoderState { RisingEdgeDecoder, counters }   one Mutex, ~100 ns per pulse
       ▲
       │  read once per publish
 EncoderCore::publish(now) → Snapshot → tags + /state
       ▲
       │  gated at tag_publish_interval_s (default 500 ms)
 Application::main_loop     runner ticks it, then commit_tags()
```

| file | role |
|---|---|
| `src/quadrature.rs` | pure decode + rate helpers, zero dependencies |
| `src/state.rs` | the state a pulse callback may touch, + the published snapshot |
| `src/core.rs` | subscriptions, polled ingest, publish derivation |
| `src/app.rs` | the `Application` impl: setup, publish timer, buttons |
| `src/debug_server.rs` | localhost `/state` endpoint (no HTTP framework) |
| `src/config.rs` / `src/tags.rs` / `src/ui.rs` | the declarative schemas |

## Three upstream bugs, and how this app survives them

**1. The platform interface never sets `value` on `pulseCounterResponse.`**
`doover-platform-interface/src/doover_platform_interface/platform_iface_base.py:268-281`
builds the message without it on every yield, so it is proto3-absent on **every
driver including real hardware**. doover-rs surfaces this as `DiPulse::value` /
`PulseCounterUpdate::value`, which is therefore `false` on every pulse forever.

This app never reads it. **The channel is bound from which subscription
delivered the pulse** (`core.rs`, the closure in `start_pulse_listeners`), which
is information the wire cannot corrupt. The `pulses_reporting_high` counter in
`/state` keeps score on-device: if it is ever non-zero, upstream has been fixed.

The same fix is applied to the **home switch**, which the Python app gets wrong:
it subscribes once with `edge="both"` and compares `di_value` against the active
level (`application.py:361`), so with `home_switch_active_low=false` homing never
triggers from an edge at all. This app subscribes **twice** — rising and falling
— and takes the level from whichever fired.

**2. The 0.2 s pulse grace period.** pydoover discards every pulse in the first
0.2 s after a listener starts; at 30 rising edges/s that is 6 counts of silently
lost distance. **doover-rs did port it** (`docker/platform.rs:944`,
`PULSE_GRACE_PERIOD = 0.2`), but only inside
`PulseCounter::start_listener_pulses` (`:1034`), where it is a private `const`
with no setter — a Rust app *cannot* zero it the way the Python app does
(`counter.pulse_grace_period = 0.0`). So this app never constructs a
`PulseCounter`. `tests/fidelity.rs` pins that down.

**3. `dt_secs <= 0` is treated as "not a pulse", but on this firmware it is a
real one.** `doover-rs`'s `start_di_pulse_listener` skips any pulse with
`dt_secs <= 0` (`docker/platform.rs:850-852`, *"pydoover only counts pulses with
dt > 0"*). `doovit_fw` 1.9.1 emits `dt_out = 0.0` whenever the PIO has no period
to report (`dio.py:304`) — which is **the first edge of a pin** (one lost count
per channel per stream connect) and **after a dropped transition** (i.e. exactly
when the count is already at risk). On the home switch it is worse than a lost
count: the first edge on that pin *is the gate first reaching home*, so the first
homing event of a boot would never fire.

So this app consumes the `startPulseCounter` stream itself
(`EncoderCore::spawn_raw_listener`) and distinguishes the two cases properly: the
platform opens every stream with a header-only frame that leaves `dt_secs`
**absent** (`platform_iface_base.py:268-271`), whereas a real pulse always sets
it, even to `0.0`. `None` = not a pulse, `Some(_)` = a pulse — proto3 explicit
presence makes that survive the wire. Owning the stream means owning the
reconnect loop, which is what `stream_reconnects` in `/state` reports.

One transport change arrived with doover-rs 0.1.3: streaming calls now ride a
channel with a 60 s / 5 s HTTP/2 keepalive and `keep_alive_while_idle`
(`grpc.rs:85-89`), replacing 0.1.2's 10 s cadence — which sat *under* the
platform interface's 30 s C-core `min_ping_interval_without_data` floor and
could earn the stream a GOAWAY. In the shipped default (`use_event_polling =
true`) only the two home-switch listeners are streams; A/B are polled and
unaffected. The trade: a spurious-reconnect source is gone, but a silently
half-open stream now takes ~65 s to surface instead of ~15 s — during that
window the tell is `pulse_callbacks` flat while `stream_reconnects` is *also*
flat, not a rising `stream_reconnects`.

## Tests

```sh
cargo test                                    # decoder + fidelity (fast)
cargo test --release --test fidelity -- --ignored --nocapture   # the 2 min soak
```

`tests/fidelity.rs` drives the app's **real** ingest path — real tonic server on
a real socket speaking `platform_iface.proto`, real doover-rs `PlatformClient`,
real `startPulseCounter` stream, real listener tasks. Only the edge source is
synthetic, because the shipped platform-interface simulator cannot carry this
workload: it **level-samples DI on a 50 ms timer**
(`drivers/platform_iface_sim.py:93-100`), which cannot represent a 16.7 ms
transition at all, and its `getDIEvents` returns `"rising"`/`"falling"`/`"both"`
where hardware returns `DI_R`/`DI_F` (`:302-312`). See
`tests/common/platform_double.rs` for the full reasoning; the double deliberately
reproduces both upstream behaviours above.

## Build

The Doovit has no `rustc`, is `linux/arm64`, and has 1.8 GB of RAM, so nothing is
built on the device. One native `rustc` on the build host cross-compiles every
architecture with `cargo-zigbuild`, following doover-rs's own `Dockerfile`:

```sh
docker buildx build --platform linux/arm64 \
  -t ghcr.io/getdoover/channel-gate-encoder-rs:main --load .
```

The result is a static musl binary in a `FROM scratch` image — no base image, no
libc, no `curl`. `HEALTHCHECK` therefore calls the binary's own
`channel-gate-encoder healthcheck` subcommand, which probes
`127.0.0.1:$HEALTHCHECK_PORT` with the same semantics as the Python app's `curl
-f`.

Regenerate `doover_config.json` after touching `config.rs` / `ui.rs`:

```sh
cargo run --bin channel-gate-encoder -- export doover_config.json \
  --app-name channel_gate_encoder
```

## What this port does and does not fix

It **does** remove host-side jitter from the direction decode, which matters
because direction is now a timing comparison. Measured over a 2-minute soak at 30
rising callbacks/s, worst deviation of the short A→B gap from its 16.7 ms nominal
was **~1.5 ms**, with **zero** gaps reaching the 25.0 ms ambiguity-band edge.
The Python app measured 14.4 ms worst-case on the same nominal and intermittently
landed inside the band. It also removes the grace-period loss and the broken
home-switch level comparison.

One honest caveat found while measuring. There is **one gRPC stream per pin**, each
with its own reader task, so cross-channel *ordering* is a scheduling question,
not a guarantee — in either language (the Python simulator's notes say the same
about its per-callback task spawn). On an idle host this never bit: three
2-minute soaks, zero same-channel repeats. On a host contended by a parallel
Docker build it did: the injector itself stalled 65.9 ms, 4 of 3600 edges (0.11%)
arrived out of cross-channel order, and the decoder logged
`missed_edges = 4`/`ambiguous_edges = 14`. Position was still exactly right,
because a same-channel repeat makes the decoder discard its timing state and hold
the last direction — which is correct on a one-way run. Around a *reversal* it
would not be. So: don't co-schedule heavy work with this app on a device, and
treat `missed_edges` on a one-way run as a host-load alarm.

It also **halves the reversal error** (4 counts → 2) by using the firmware's
PIO-measured period, and it stops silently discarding pulses: doover-rs's
`start_di_pulse_listener` drops any pulse with `dt_secs <= 0`, which on this
firmware is a *legitimate* pulse (see below), so this app consumes the raw stream
itself.

### The firmware is not the problem it used to be

An earlier read of `doovit_fw` was **104 commits stale**. The device runs
**1.9.1**, where `check_interrupts` (`dio.py:266`) drains **every** confirmed
edge from a PIO debouncer and emits each one individually. The
`round(num_risen/(num_fallen+num_risen))` majority vote that destroyed 68% of
rising edges **no longer exists** — edge detection and debounce moved into PIO1
SM0–3. Per-edge timestamps also survive the 50 ms sweep, because each event's
time is back-computed from its leading-edge tick (`dio.py:297`): the sweep delays
delivery, it does not move the timestamp. So the old "≤ 7 rising edges/s per
sensor, 15 is unreachable" conclusion is void, and the mode Jarrod enabled
(`count_event_live`) is documented as the one to use for ~100 Hz event counting.

### What is still genuinely open

The remaining limitation is **not** in the firmware — it is in the transport
between the firmware and the app, and this app cannot fix it either:

`startPulseCounter` is **one stream per pin**, and doover-rs gives each its own
gRPC channel (`platform.rs:818`, `stream_channel()`). A 50 ms sweep releases a
clump of events at once, and two independent streams draining a simultaneous
clump have no defined interleaving — so the A,B,A,B alternation, and with it the
16.7 ms `gap` term that direction is measured from, is scrambled **at the
client**. Measured through the 1.9.1 sweep model: zero pulses dropped, but ~23%
of edges become same-channel repeats and position goes badly wrong. The decoder
reports every bit of that (`missed`, `ambiguous`) rather than hiding it, which is
the property that matters — but it is still wrong.

**The fix is one upstream field.** The firmware already computes a per-edge
timestamp (`ev_epoch`, `dio.py:297`) and doovitd forwards it; the platform
interface throws it away, yielding only `payload["dt_secs"]`
(`doovit_platform_iface.py:597-598`), because `pulseCounterResponse` has nowhere
to put it. **Add a timestamp field to `pulseCounterResponse`** and batching
becomes completely harmless: both the gap *and* the period would be
firmware-measured, and host scheduling would stop mattering entirely. That is a
far smaller ask than a PIO quadrature counter, and it fixes the actual problem.

Until then, on a device: the `count_event_live` mode this needs also **disables
the event log** (no flash record, not retrievable afterwards — `dio.py:26-32`),
so the polled-ingest path, which *does* recover cross-pin ordering from the
global `event_id` sequence and the firmware's back-computed timestamps, is
mutually exclusive with the throughput mode. Choosing between them is a real
trade-off, not an oversight — see the runbook.
