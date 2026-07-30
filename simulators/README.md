# Simulators — testing against the platform-iface sim

Drives the app with **simulated quadrature pulses** from
[doover-platform-interface](../../../doover-platform-interface)'s `--type sim`
driver, and watches the published tags change on the dda. The sim driver
dispatches DI edges inline (no 50 ms sampling tick) and generates the A/B
waveform in-process, so phase relationships survive to the app.

## The stack

Everything runs on the Doovit; you just open a browser.

```
 Browser (anywhere on LAN)            Doovit 10.144.226.119
 ────────────────────────             ─────────────────────────────
 http://10.144.226.119:8090  ──▶      motor-sim :8090
                                        │ HTTP            │ gRPC
                                        ▼                 ▼
                                      platform-iface sim  dda :50051
                                      :8080 (web) :50053    ▲
                                             │ startPulseCounter
                                             ▼              │ tags every 0.5 s
                                      channel-gate-encoder app
```

The encoder app should run **on the Doovit** — decoding on the device keeps
the A/B arrival timing intact. Cross-build the static binary, copy it up, run
it detached (`setsid`, or it dies with the ssh session):

```sh
docker buildx build --platform linux/arm64 --target bin --output type=local,dest=./dist .
scp dist/channel-gate-encoder simulators/app_config.json doovit@10.144.226.119:/home/doovit/
ssh doovit@10.144.226.119 'cd /home/doovit && setsid env \
  APP_KEY=channel_gate_encoder_sim CONFIG_FP=/home/doovit/app_config.json \
  nohup ./channel-gate-encoder > encoder.log 2>&1 < /dev/null &'
```

It can also run locally against the Doovit's dda + sim, which is handy for a
fast edit loop but costs decode quality — LAN jitter scrambles the cross-channel
arrival timing that direction is measured from (observed: ~100 missed / ~130
ambiguous edges in a minute over WiFi, versus 1 / 0 on-device):

```sh
APP_KEY=channel_gate_encoder_sim \
CONFIG_FP=simulators/app_config.json \
DDA_URI=10.144.226.119:50051 \
PLT_URI=10.144.226.119:50053 \
cargo run
```

(`PLT_URI` is read directly in `app.rs` — `REMOTE_DEV` alone is not enough.)

## The control UI

Runs on the Doovit at **http://10.144.226.119:8090**. Deploy/update it with:

```sh
cd simulators
tar czf /tmp/motor-sim.tgz --exclude .venv --exclude __pycache__ motor-sim
scp /tmp/motor-sim.tgz doovit@10.144.226.119:/home/doovit/
ssh doovit@10.144.226.119 'cd /home/doovit && tar xzf motor-sim.tgz \
  && cd motor-sim && ~/.local/bin/uv sync \
  && pkill -f "[m]ain.py"; setsid env DOOVIT_HOST=127.0.0.1 \
     nohup .venv/bin/python main.py > ui.log 2>&1 < /dev/null &'
# the [m] stops pkill -f matching this ssh command's own remote shell

```

(uv was installed on the device via `curl -LsSf https://astral.sh/uv/install.sh | sh`.)

It also runs fine locally against the Doovit — `cd simulators/motor-sim &&
uv run main.py` → http://localhost:8090.

Start/stop the pulse train, watch Height / RawCount / Direction / RPM update
from the dda's `tag_values` channel, pulse the home switch to zero the count.
Point it elsewhere with `DOOVIT_HOST=<ip>` (or `SIM_URL` / `DDA_URI`
individually); pins with `A_PIN`/`B_PIN`/`HOME_PIN` (default 0/1/2, matching
`app_config.json`); listen address with `PORT` / `BIND`.

Direction convention (`src/quadrature.rs`): **B's rise leading A's = +count**
(height up, "open"). The UI encodes that as B at phase 270° for open, 90° for
close, with A at 0°.

## Sim quirks worth knowing

* **First-edge direction transient.** The sim seeds each pin's last-edge
  timestamp at *sim startup*, so the first pulse of a stream carries a huge
  `dt_secs` instead of the firmware's `0.0` ("no period"). Worst case the first
  edge or two of a fresh train are mis-signed (≤ 2 counts) before the decoder
  re-measures; a home pulse clears it. Position tracking is exact from the
  second cycle on.
* **Finite trains don't clear their spec.** After `cycles=N` completes, the
  sim still reports the train spec until you press Stop — "train: …" in the UI
  means "last started", not necessarily "currently emitting".
* **Frequency ceiling.** `gap` (the A→B quarter-cycle spacing) is measured at
  delivery time by the app, so network jitter eats into the direction margin —
  over a LAN keep the train ≤ ~15 Hz when the app runs off-device; on-device
  the usual 15 Hz design point holds.

## The gate controller

[channel-gate-control](../../channel-gate-control) runs alongside as a Docker
container (`channel_gate_control_sim`) and closes the tag loop: it reads the
encoder's `Height` / `Homed` / `Heartbeat` tags (configured via
`height_app_key: channel_gate_encoder_sim`), republishes the reading as its own
live `GateHeight` tag, and carries the cloud HMI (target slider, height
readout, mode select) — so absolute position reaches the cloud through both
apps' `tag_values`.

Deployed with (image is private on ghcr, so it's pulled here and shipped over):

```sh
docker pull --platform linux/arm64 ghcr.io/getdoover/channel-gate-control:main
docker save ghcr.io/getdoover/channel-gate-control:main | gzip | \
  ssh doovit@10.144.226.119 'gunzip | docker load'
scp simulators/gate_control_config.json doovit@10.144.226.119:/home/doovit/
ssh doovit@10.144.226.119 'docker run -d --name channel_gate_control_sim \
  --network host --restart unless-stopped \
  -v /home/doovit/gate_control_config.json:/app/sim_config.json:ro \
  -e APP_KEY=channel_gate_control_sim -e CONFIG_FP=/app/sim_config.json \
  ghcr.io/getdoover/channel-gate-control:main'
```

`gate_control_config.json` notes: pump on DO0, raise solenoid DO1, lower
solenoid DO2; **move-timeout and stall detection are disabled** (`0`) — the
"hydraulics" are simulated, and with them enabled a hiccup in the loop would
latch a "gate not moving" fault. Default mode is Hold (outputs off, still
mirrors height).

**The loop is closed**: motor-sim's follower polls the sim's DOs every 200 ms
and drives the pulse train to match — DO0+DO1 → train "open" (height up),
DO0+DO2 → train "close", anything else → stop. So dragging the target slider
in the cloud HMI (mode Auto) genuinely moves the simulated gate: controller
energises solenoids → motor-sim runs the train → encoder counts → controller
stops at target (measured: ±1 mm of a 300 mm move at the default 15 Hz).
The follower is a toggle + speed in the motor-sim page ("Motor — follows gate
controller"); env overrides `PUMP_DO`/`RAISE_DO`/`LOWER_DO`/`FOLLOW_FREQ_HZ` (default 15).
A follower stop clears *any* running train, including manual ones.

## Files

| | |
|---|---|
| `app_config.json` | encoder config for sim runs: A=DI0, B=DI1, home=DI2 (active high), streaming ingest, 0.7 mm/count |
| `gate_control_config.json` | channel-gate-control config for sim runs: reads `channel_gate_encoder_sim`, drives DO0/1/2, stall detection off |
| `motor-sim/` | the control web app (FastAPI + single page; `uv run main.py`) |
