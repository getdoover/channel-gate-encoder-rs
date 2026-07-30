"""Motor sim control UI.

A small local web app that drives the platform-interface *sim* driver's pulse
train (quadrature A/B on two DIs) and shows the channel-gate-encoder's tags as
they land on the dda — so you can start/stop simulated motion and watch the
absolute position move, end to end through the real stack.

It talks to two things, both usually on the Doovit running the sim:

* the sim driver's HTTP API (`:8080`) — `/di/pulse_train` start/stop, `/di/{n}`
  for the home switch, `/di` + `/do` + `/status` for pin state;
* the dda's gRPC (`:50051`) — `tag_values` channel aggregate, where the app's
  published tags live under its app key.

It is also the "motor": a follower loop watches the DOs the gate controller
drives (DO0 = pump, DO1 = raise solenoid, DO2 = lower solenoid) and runs the
pulse train to match — DO0+DO1 high → train "open" (height up), DO0+DO2 high →
train "close", anything else → train stops. That closes the physical loop, so
dragging the target slider in the cloud HMI actually moves the simulated gate.

Direction convention (see `src/quadrature.rs`): **B's rise leading A's is the
positive sense** (+count → height up → "opening"). With channel A at phase 0°,
B at 270° makes B's rise land a quarter-cycle before A's next rise → B leads →
open. B at 90° → A leads → close.

Run:  uv run main.py           (from this directory)
Env:  DOOVIT_HOST (default 10.144.226.119; use 127.0.0.1 when running ON the
      Doovit), SIM_URL, DDA_URI, A_PIN/B_PIN/HOME_PIN (default 0/1/2),
      PORT (default 8090), BIND (default 0.0.0.0)
"""

from __future__ import annotations

import asyncio
import logging
import os
import time
from contextlib import asynccontextmanager
from pathlib import Path

import httpx
import uvicorn
from fastapi import FastAPI
from fastapi.responses import FileResponse, JSONResponse
from pydantic import BaseModel, Field
from pydoover.docker.device_agent import DeviceAgentInterface

DOOVIT_HOST = os.environ.get("DOOVIT_HOST", "10.144.226.119")
SIM_URL = os.environ.get("SIM_URL", f"http://{DOOVIT_HOST}:8080")
DDA_URI = os.environ.get("DDA_URI", f"{DOOVIT_HOST}:50051")
A_PIN = int(os.environ.get("A_PIN", "0"))
B_PIN = int(os.environ.get("B_PIN", "1"))
HOME_PIN = int(os.environ.get("HOME_PIN", "2"))
PORT = int(os.environ.get("PORT", "8090"))
BIND = os.environ.get("BIND", "0.0.0.0")

# DO-follower ("motor") wiring — must match the gate controller's config.
PUMP_DO = int(os.environ.get("PUMP_DO", "0"))
RAISE_DO = int(os.environ.get("RAISE_DO", "1"))
LOWER_DO = int(os.environ.get("LOWER_DO", "2"))
FOLLOW_FREQ_HZ = float(os.environ.get("FOLLOW_FREQ_HZ", "15"))
FOLLOW_POLL_S = 0.2

STATIC_DIR = Path(__file__).parent / "static"

# B relative to A (at 0°). 270° puts B's rise a quarter-cycle BEFORE A's next
# rise → B leads → +count ("open"); 90° → A leads → -count ("close").
PHASE_FOR_DIRECTION = {"open": 270.0, "close": 90.0}

log = logging.getLogger("motor-sim")


class TrainRequest(BaseModel):
    frequency_hz: float = Field(default=10.0, gt=0, le=100)
    direction: str = Field(default="open", pattern="^(open|close)$")
    # None runs until stopped. One cycle = 2 counts = 2 x mm_per_count of travel.
    cycles: int | None = Field(default=None, gt=0)


class HomeRequest(BaseModel):
    value: bool


class FollowRequest(BaseModel):
    enabled: bool | None = None
    frequency_hz: float | None = Field(default=None, gt=0, le=100)


async def _start_train(direction: str, frequency_hz: float):
    body = {
        "pins": [
            {"pin": A_PIN, "phase_deg": 0.0},
            {"pin": B_PIN, "phase_deg": PHASE_FOR_DIRECTION[direction]},
        ],
        "frequency_hz": frequency_hz,
        "duty": 0.5,
        "cycles": None,
    }
    r = await app.state.http.post("/di/pulse_train", json=body)
    r.raise_for_status()


async def _do_follow_loop():
    """The motor: mirror the gate controller's solenoid outputs onto the train.

    pump+raise → open (height up), pump+lower → close, anything else → stop.
    Acts only on TRANSITIONS, so a manually-started train is left alone while
    the controller is idle — but a controller move always takes the train over
    (start_pulse_train replaces whatever is running), and the move ending stops
    it.
    """
    st = app.state.follow
    while True:
        await asyncio.sleep(FOLLOW_POLL_S)
        if not st["enabled"]:
            continue
        try:
            r = await app.state.http.get("/do")
            do = r.json()
            pump, raise_on, lower_on = do[PUMP_DO], do[RAISE_DO], do[LOWER_DO]
        except Exception:
            continue
        if pump and raise_on and not lower_on:
            want = "open"
        elif pump and lower_on and not raise_on:
            want = "close"
        else:
            want = None
        if want == st["driving"]:
            continue
        try:
            if want is None:
                await app.state.http.post("/di/pulse_train/stop")
                log.info("motor: outputs dropped — train stopped")
            else:
                await _start_train(want, st["frequency_hz"])
                log.info("motor: driving %s at %.1f Hz", want, st["frequency_hz"])
            st["driving"] = want
        except Exception as e:
            log.warning("motor: train command failed: %s", e)


@asynccontextmanager
async def lifespan(app: FastAPI):
    app.state.http = httpx.AsyncClient(base_url=SIM_URL, timeout=5.0)
    app.state.dda = DeviceAgentInterface(app_key="motor_sim", dda_uri=DDA_URI)
    app.state.follow = {"enabled": True, "frequency_hz": FOLLOW_FREQ_HZ, "driving": None}
    follow_task = asyncio.create_task(_do_follow_loop())
    log.info(
        "sim: %s   dda: %s   pins A=%d B=%d home=%d   motor: pump=DO%d raise=DO%d lower=DO%d @ %.1f Hz",
        SIM_URL, DDA_URI, A_PIN, B_PIN, HOME_PIN, PUMP_DO, RAISE_DO, LOWER_DO, FOLLOW_FREQ_HZ,
    )
    try:
        yield
    finally:
        follow_task.cancel()
        await app.state.http.aclose()


app = FastAPI(lifespan=lifespan)


@app.get("/")
async def index():
    return FileResponse(STATIC_DIR / "index.html")


async def _sim_state(http: httpx.AsyncClient) -> dict:
    train, di, do = await asyncio.gather(
        http.get("/di/pulse_train"), http.get("/di"), http.get("/do")
    )
    train.raise_for_status()
    di.raise_for_status()
    do.raise_for_status()
    return {"train": train.json(), "di": di.json(), "do": do.json()}


async def _dda_tags(dda: DeviceAgentInterface) -> dict:
    agg = await asyncio.wait_for(dda.fetch_channel_aggregate("tag_values"), timeout=4.0)
    data = getattr(agg, "data", None) or {}
    # Encoder instances are the sub-dicts publishing a RawCount tag.
    encoders = {k: v for k, v in data.items() if isinstance(v, dict) and "RawCount" in v}
    return {"encoders": encoders, "all_keys": sorted(data.keys())}


@app.get("/api/status")
async def status():
    out: dict = {
        "ts": time.time(),
        "pins": {"a": A_PIN, "b": B_PIN, "home": HOME_PIN},
        "dos": {"pump": PUMP_DO, "raise": RAISE_DO, "lower": LOWER_DO},
        "follow": dict(app.state.follow),
        "sim_ok": False,
        "dda_ok": False,
        "errors": [],
    }
    sim, tags = await asyncio.gather(
        _sim_state(app.state.http), _dda_tags(app.state.dda), return_exceptions=True
    )
    if isinstance(sim, BaseException):
        out["errors"].append(f"sim: {sim}")
    else:
        out.update(sim, sim_ok=True)
    if isinstance(tags, BaseException):
        out["errors"].append(f"dda: {tags}")
    else:
        out.update(tags, dda_ok=True)
    return out


@app.post("/api/train")
async def start_train(req: TrainRequest):
    body = {
        "pins": [
            {"pin": A_PIN, "phase_deg": 0.0},
            {"pin": B_PIN, "phase_deg": PHASE_FOR_DIRECTION[req.direction]},
        ],
        "frequency_hz": req.frequency_hz,
        "duty": 0.5,
        "cycles": req.cycles,
    }
    r = await app.state.http.post("/di/pulse_train", json=body)
    if r.status_code != 200:
        return JSONResponse({"error": r.text}, status_code=502)
    log.info("train started: %.1f Hz %s cycles=%s", req.frequency_hz, req.direction, req.cycles)
    return r.json()


@app.post("/api/train/stop")
async def stop_train():
    r = await app.state.http.post("/di/pulse_train/stop")
    if r.status_code != 200:
        return JSONResponse({"error": r.text}, status_code=502)
    log.info("train stopped")
    return {"stopped": r.json()}


@app.post("/api/follow")
async def set_follow(req: FollowRequest):
    st = app.state.follow
    if req.enabled is not None:
        st["enabled"] = req.enabled
        if not req.enabled and st["driving"] is not None:
            # Don't leave a controller-commanded train running headless.
            await app.state.http.post("/di/pulse_train/stop")
            st["driving"] = None
        log.info("motor follow %s", "enabled" if req.enabled else "disabled")
    if req.frequency_hz is not None:
        st["frequency_hz"] = req.frequency_hz
        if st["driving"] is not None:
            await _start_train(st["driving"], req.frequency_hz)
    return dict(st)


@app.post("/api/home")
async def set_home(req: HomeRequest):
    r = await app.state.http.post(f"/di/{HOME_PIN}", json={"value": req.value})
    if r.status_code != 200:
        return JSONResponse({"error": r.text}, status_code=502)
    return {"di": r.json()}


@app.post("/api/home/pulse")
async def pulse_home():
    """One rising edge on the home pin (active-high config homes on the rise)."""
    r = await app.state.http.post(f"/di/{HOME_PIN}", json={"value": True})
    if r.status_code != 200:
        return JSONResponse({"error": r.text}, status_code=502)
    await asyncio.sleep(0.3)
    await app.state.http.post(f"/di/{HOME_PIN}", json={"value": False})
    log.info("home pulsed")
    return {"pulsed": True}


def run():
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(name)s %(message)s")
    uvicorn.run(app, host=BIND, port=PORT, log_level="warning")


if __name__ == "__main__":
    run()
