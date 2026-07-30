# TEST-RIG — the live closed-loop sim rig (agent runbook)

Operational state of the running test setup, written for agents (human or AI)
who need to drive or debug it. The *design* of the rig is in
[README.md](README.md); this file is the "what is actually running right now,
and how do I poke it" companion. Set up 2026-07-29/30.

## The device

| | |
|---|---|
| Device | `doovit-9b8363` (CM4 Doovit, dev unit) |
| LAN | `10.144.226.119` — **flaky**: drops for minutes at a time, see Quirks |
| Agent / device id | `160200521116876558` (same snowflake for both) |
| Org | The Dojo |
| Cloud page | https://dojo.doover.com/agent/160200521116876558 |
| SSH | `ssh doovit@10.144.226.119` (key auth is set up; password `doovit` works too) |
| SSH fallback | ngrok tunnel `1.tcp.au.ngrok.io:20806` (tunnel id `208192253611516672`, activate via control API — the released doover CLI's tunnel commands are stubs) |

## What is running where

| Component | How it runs | Endpoints / names |
|---|---|---|
| platform-iface **sim** | docker `pulse_sim_plt_iface` (image `doover_platform_iface:pulse-sim`) | `:8080` HTTP API + sim web UI, `:50053` gRPC |
| dda (device agent) | docker `160200521116876558__doover-device-agent` | `:50051` gRPC |
| channel-gate-encoder (this repo, Rust) | **bare binary** `/home/doovit/channel-gate-encoder`, setsid/nohup, `APP_KEY=channel_gate_encoder_sim`, `CONFIG_FP=/home/doovit/app_config.json`, `HEALTHCHECK_PORT=49201`, log `/home/doovit/encoder.log` | debug `/state` on device-local `127.0.0.1:8765` |
| channel-gate-control | docker `channel_gate_control_sim` (ghcr image, shipped by `docker save\|load`) | config `/home/doovit/gate_control_config.json`; shows *unhealthy* — cosmetic, see Quirks |
| motor-sim (this repo) | **bare python** `/home/doovit/motor-sim/.venv/bin/python main.py`, setsid/nohup, `DOOVIT_HOST=127.0.0.1`, log `/home/doovit/motor-sim/ui.log` | **`http://10.144.226.119:8090`** — the page humans use, and the REST API below |

The two bare processes do **not** survive a reboot — relaunch recipes are in
README.md. Never run a second encoder against the same dda (double-publishing
under one app key).

## App keys and tags

- `channel_gate_encoder_sim` — Height (mm), PercentOpen, RawCount, Direction,
  Speed (mm/min), Homed, HomeSwitch, Revolutions, RPM, RotationDirection,
  MissedEdges, AmbiguousEdges, Heartbeat (unix s, publishes every **0.25 s**).
- `channel_gate_control_sim` — GateHeight (mirror of Height), TargetHeight,
  Error, Moving, RaiseOutput/LowerOutput/PumpOutput, Mode (auto/hold), Status,
  Fault, FaultReason, HeightValid, EStopActive.

Scale: 0.7 mm/count, 2 counts per pulse-train cycle → gate speed =
`1.4 mm × train Hz` (15 Hz → 21 mm/s). Travel 0–1000 mm.

Platform-side install records exist for both app keys (create-only, **never
deploy them** — a deployment would make the device's app-controller start
competing containers against the hand-run ones).

## Driving the rig

Everything below works from any machine that can reach the device (or from the
device itself against `localhost`).

```sh
# What's the current state? (sim pins, train, follower, all encoder tags)
curl -s http://10.144.226.119:8090/api/status | jq

# Manual pulse train: 10 Hz opening; direction "close" lowers; cycles optional
curl -s -X POST http://10.144.226.119:8090/api/train \
  -H 'content-type: application/json' \
  -d '{"frequency_hz": 10, "direction": "open", "cycles": 100}'
curl -s -X POST http://10.144.226.119:8090/api/train/stop

# Home (zeroes the count — one rising edge on DI2)
curl -s -X POST http://10.144.226.119:8090/api/home/pulse

# Motor follower (the thing that makes the cloud HMI actually move the gate)
curl -s -X POST http://10.144.226.119:8090/api/follow \
  -H 'content-type: application/json' -d '{"enabled": true, "frequency_hz": 15}'
```

**Cloud-path move** (exactly what the HMI slider does — requires the follower
enabled):

```sh
doover channel publish ui_cmds \
  '{"channel_gate_control_sim": {"target": 300, "mode": "auto"}}' \
  --agent 160200521116876558
# ...gate drives itself there and stops. Put it back to hold afterwards:
doover channel publish ui_cmds \
  '{"channel_gate_control_sim": {"mode": "hold"}}' --agent 160200521116876558
```

**Reading tags cloud-side:**

```sh
doover channel get tag_values --agent 160200521116876558
```

## Quirks that will bite you

- **LAN flaps.** `10.144.226.119` disappears for minutes while the device's
  cloud uplink stays up. Retry, or go via the ngrok tunnel. Split risky ssh
  work into small idempotent commands.
- **`pkill -f` self-match.** In an ssh one-liner, `pkill -f "main.py"` matches
  the remote shell running your own command (its argv contains the pattern)
  and kills the session mid-chain. Use `pkill -f "[m]ain.py"` — and don't put
  the literal string later in the same command line.
- **Observation-gated cloud publishing.** Apps push tag aggregates to the
  cloud every **900 s** unless the site marks them watched in `dv-ui-sub`
  (then 3 s). A "frozen" cloud with live on-device tags is usually this, not a
  fault. Fixed properly only because both configs carry `APP_KEY` +
  `APP_DISPLAY_NAME` — see the doover-facts entry
  `app-development/hand-run-apps-need-app-key-metadata.md`. Never remove those
  keys from the sim configs.
- **Overshoot vs deadband.** Feedback latency is ~0.25 s tag publish + 0.25 s
  control loop + 0.2 s follower poll. At 15 Hz (21 mm/s) the gate can coast
  ~5–15 mm past target; controller re-engages only beyond deadband+hysteresis
  (10 mm). Occasional near-threshold parks are normal; sustained
  drive-stop-reverse cycling means widen `deadband_mm`/`hysteresis_mm`.
- **First train edge can be mis-signed** (sim seeds `dt` from sim start, not
  0.0) — ≤2 counts, self-corrects; home pulse clears it.
- **Controller container reads "unhealthy".** Its `docker run` lacks
  `HEALTHCHECK_PORT`, so the baked-in `curl -f 127.0.0.1:$HEALTHCHECK_PORT`
  probes an empty port string. Harmless; fix by re-creating the container with
  `-e HEALTHCHECK_PORT=49200` (the encoder binary owns 49201).
- **Move-timeout and stall detection are disabled** in the controller's sim
  config. In Auto with the follower disabled, a solenoid will stay energised
  forever — nothing faults. Re-enable them before copying this config anywhere
  near real hardware.
