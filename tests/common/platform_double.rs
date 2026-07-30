//! `PlatformDouble` — an in-process tonic server implementing
//! `platform_iface.platformIface`, plus a quadrature gate model that injects
//! rising edges at an **exact** rate.
//!
//! # Why a test double instead of the shipped simulator
//!
//! Jarrod asked for pulses via the platform-interface simulator. That simulator
//! cannot carry this workload, for two independent reasons found in its source:
//!
//! * **It level-samples DI on a 50 ms timer**
//!   (`doover-platform-interface/src/doover_platform_interface/drivers/platform_iface_sim.py:93-100`
//!   — one task per pin, `await asyncio.sleep(0.05)` then `get_di_state`). At 15
//!   rising edges/s per sensor the waveform has transitions every 16.7 ms, so
//!   two thirds of them are invisible to a 50 ms sampler. The 90-degree gap that
//!   *is* the direction signal cannot survive at all.
//! * **Its `getDIEvents` speaks a different vocabulary from hardware**
//!   (`platform_iface_sim.py:302-312` emits `"rising"`/`"falling"`/`"both"`,
//!   where the Doovit driver emits `"DI_R"`/`"DI_F"`,
//!   `doovit_platform_iface.py:572`), and it *drains* the log on read.
//!
//! Extending the simulator would mean rewriting its sampling core inside another
//! repo that a different agent is working in. So this is a purpose-built gRPC
//! double instead: it speaks the real `platform_iface.proto` over a real tonic
//! server on a real socket, so the app under test uses the **real doover-rs
//! client, the real `startPulseCounter` stream and the real gRPC transport** —
//! only the edge source is synthetic, and synthetic is the point (the edges have
//! to be exact for the measurement to mean anything).
//!
//! # Fidelity to the real interface
//!
//! Two upstream behaviours are reproduced deliberately, because they change what
//! the client sees:
//!
//! 1. **`value` is never set on `pulseCounterResponse`.**
//!    `platform_iface_base.py:268-281` builds the message without it on every
//!    yield, so it is proto3-absent on every driver including real hardware. The
//!    double leaves it `None` too, so any code that trusted the level would fail
//!    here exactly as it fails on a Doovit.
//! 2. **The stream's first frame is not a pulse.** `platform_iface_base.py:268`
//!    yields `pulseCounterResponse(response_header=…, di=di)` with no `dt_secs`
//!    at subscribe time. doover-rs drops frames with `dt_secs <= 0`
//!    (`docker/platform.rs:850-852`), so that frame is correctly not counted —
//!    the double emits it so the test exercises that path.

use std::collections::HashMap;
use std::net::SocketAddr;
use std::sync::atomic::{AtomicI64, AtomicU64, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use doover_proto::platform_iface as pb;
use pb::platform_iface_server::{PlatformIface, PlatformIfaceServer};
use tokio::sync::mpsc;
use tokio_stream::wrappers::{ReceiverStream, TcpListenerStream};
use tonic::{Request, Response, Status};

type RpcResult<T> = std::result::Result<Response<T>, Status>;

/// Forward quadrature walk `00 -> 01 -> 11 -> 10`: index = phase, exactly one
/// bit changes between neighbours (including the 3 -> 0 wrap). Forward is index
/// INCREASING, which makes **B rise first** — the positive sense the decoder is
/// built around.
const GRAY: [(bool, bool); 4] = [(false, false), (false, true), (true, true), (true, false)];

/// One `startPulseCounter` subscriber: which pin, which edge filter, and the
/// queue its stream drains.
struct Subscriber {
    pin: i32,
    edge: String,
    tx: mpsc::Sender<Result<pb::PulseCounterResponse, Status>>,
}

impl Subscriber {
    /// The edge filter the platform applies server-side.
    fn wants(&self, level: bool) -> bool {
        match self.edge.as_str() {
            "both" => true,
            "rising" => level,
            "falling" => !level,
            _ => level,
        }
    }
}

/// The `doovit_fw` 1.9.1 `check_interrupts` sweep: a 50 ms loop that drains every
/// confirmed edge from the PIO debouncer and broadcasts each one individually.
///
/// The critical property, and the reason this is not the old coalescing model:
/// *"each event's real time is back-computed from its leading-edge tick, so the
/// 50ms cadence doesn't drift the timestamp (only delays the record/broadcast)"*
/// (`dio.py:269-271`). There is **no majority vote** and **no coalescing** — the
/// `round(num_risen/(num_fallen+num_risen))` that used to destroy rising edges is
/// gone, edge detection having moved into PIO1 SM0-3. So the model here delays
/// delivery and batches it, but preserves every edge, its order, and its
/// PIO-measured `dt_secs`.
pub struct FirmwareSweep {
    pub sweep_ms: u64,
    pending: Vec<(i32, bool, pb::PulseCounterResponse)>,
}

#[derive(Default)]
pub struct PlatformDoubleState {
    di_levels: Mutex<HashMap<i32, bool>>,
    do_levels: Mutex<HashMap<i32, bool>>,
    /// `irq_edge` per pin as the app configured it — recorded so the test can
    /// assert the app really asked the firmware for rising-only.
    pub di_irq_edge: Mutex<HashMap<i32, String>>,
    pub di_debounce_ms: Mutex<HashMap<i32, i32>>,
    subscribers: Mutex<Vec<Subscriber>>,
    /// Last RISING edge time per pin, for the PIO-style `dt_secs` (full tooth
    /// period in rising-only mode).
    last_rise: Mutex<HashMap<i32, Instant>>,
    /// When set, injected edges are held and released in `sweep_ms` batches, the
    /// way `doovit_fw` 1.9.1's `check_interrupts` sweep does. `None` = release
    /// immediately (a lossless transport, for measuring the app in isolation).
    pub sweep: Mutex<Option<FirmwareSweep>>,
    /// The platform's DI event log, keyed by pin, with a GLOBAL id sequence.
    events: Mutex<HashMap<i32, Vec<pb::EventDetail>>>,
    next_event_id: AtomicI64,

    /// Ground-truth edges injected (both polarities, all pins).
    pub injected: AtomicU64,
    /// Ground-truth RISING edges injected — the number callbacks must match.
    pub injected_rising: AtomicU64,
    /// Frames actually pushed onto a subscriber queue.
    pub streamed: AtomicU64,
    /// Frames a full queue forced us to drop. Must be 0; a non-zero value means
    /// the double, not the app, lost the pulse.
    pub queue_drops: AtomicU64,
}

impl PlatformDoubleState {
    fn now_ms() -> i64 {
        SystemTime::now().duration_since(UNIX_EPOCH).map(|d| d.as_millis() as i64).unwrap_or(0)
    }

    /// Inject one DI transition. A no-op if the level is unchanged — only a
    /// genuine change is an edge.
    ///
    /// This is the CM4 boundary: it appends to the event log, computes the
    /// per-pin `dt_secs`, and pushes to every subscriber whose edge filter
    /// matches. No coalescing, no re-timestamping: a **lossless transport**, so
    /// what the soak measures is the app, not a model of the firmware.
    pub fn set_di_level(&self, pin: i32, level: bool) {
        {
            let mut levels = self.di_levels.lock().unwrap();
            if levels.get(&pin).copied().unwrap_or(false) == level {
                return;
            }
            levels.insert(pin, level);
        }
        self.injected.fetch_add(1, Ordering::Relaxed);
        if level {
            self.injected_rising.fetch_add(1, Ordering::Relaxed);
        }

        let id = self.next_event_id.fetch_add(1, Ordering::Relaxed);
        self.events.lock().unwrap().entry(pin).or_default().push(pb::EventDetail {
            event_id: id as i32,
            event: if level { "DI_R" } else { "DI_F" }.to_string(),
            pin,
            value: if level { "1" } else { "0" }.to_string(),
            time: Self::now_ms(),
            cm4_online: Some(true),
        });

        let now = Instant::now();
        // `dt_secs` in rising-only mode is the interval since the SAME edge on
        // this pin -- one full tooth cycle -- and the firmware measures it in the
        // PIO, so it is exact regardless of when the sweep gets round to
        // delivering it. Model that: compute it from the INJECTION times, never
        // from delivery times. 0.0 means "the firmware had none", which it
        // reports on the first edge of a pin and after a dropped transition
        // (dio.py:300-304).
        let dt_secs = if level {
            let mut last = self.last_rise.lock().unwrap();
            let dt = last.get(&pin).map(|prev| now.duration_since(*prev).as_secs_f32());
            last.insert(pin, now);
            dt.filter(|d| *d > 0.0).unwrap_or(0.0)
        } else {
            0.0
        };

        let frame = pb::PulseCounterResponse {
            response_header: Some(ok_header()),
            di: Some(pin),
            // Deliberately absent -- see the module docs (upstream bug 1).
            value: None,
            dt_secs: Some(dt_secs),
        };

        // Firmware sweep model: hold the event until the next sweep boundary.
        // Delivery is delayed and batched; ORDER and dt_secs are preserved,
        // because 1.9.1 emits every confirmed edge with a back-computed
        // timestamp instead of majority-voting a window into one event.
        if let Some(sweep) = self.sweep.lock().unwrap().as_mut() {
            sweep.pending.push((pin, level, frame));
            return;
        }
        self.deliver(pin, level, frame);
    }

    /// Push one frame to every matching subscriber.
    fn deliver(&self, pin: i32, level: bool, frame: pb::PulseCounterResponse) {
        for sub in self.subscribers.lock().unwrap().iter() {
            if sub.pin != pin || !sub.wants(level) {
                continue;
            }
            match sub.tx.try_send(Ok(frame.clone())) {
                Ok(()) => {
                    self.streamed.fetch_add(1, Ordering::Relaxed);
                }
                Err(_) => {
                    self.queue_drops.fetch_add(1, Ordering::Relaxed);
                }
            }
        }
    }

    /// Enable the `doovit_fw` 1.9.1 sweep model: events are buffered and released
    /// in `sweep_ms` batches, in injection order.
    pub fn enable_firmware_sweep(&self, sweep_ms: u64) {
        *self.sweep.lock().unwrap() = Some(FirmwareSweep { sweep_ms, pending: Vec::new() });
    }

    /// Release everything the sweep is holding, in order. Call this on the sweep
    /// cadence (see `run_firmware_sweep`).
    pub fn flush_sweep(&self) {
        let pending = {
            let mut guard = self.sweep.lock().unwrap();
            match guard.as_mut() {
                Some(sweep) => std::mem::take(&mut sweep.pending),
                None => return,
            }
        };
        for (pin, level, frame) in pending {
            self.deliver(pin, level, frame);
        }
    }

    /// The configured sweep period, if the firmware model is enabled.
    pub fn sweep_period(&self) -> Option<Duration> {
        self.sweep.lock().unwrap().as_ref().map(|s| Duration::from_millis(s.sweep_ms))
    }

    /// Forget the injection counters, so asserting the starting phase does not
    /// read as gate movement.
    pub fn reset_counters(&self) {
        self.injected.store(0, Ordering::Relaxed);
        self.injected_rising.store(0, Ordering::Relaxed);
        self.streamed.store(0, Ordering::Relaxed);
    }

    pub fn subscriber_count(&self) -> usize {
        self.subscribers.lock().unwrap().len()
    }
}

fn ok_header() -> pb::ResponseHeader {
    pb::ResponseHeader { success: true, response_code: Some(0), message: Some("ok".into()) }
}

pub struct PlatformDouble(pub Arc<PlatformDoubleState>);

#[tonic::async_trait]
impl PlatformIface for PlatformDouble {
    type startPulseCounterStream = ReceiverStream<Result<pb::PulseCounterResponse, Status>>;

    async fn test_comms(
        &self,
        r: Request<pb::TestCommsRequest>,
    ) -> RpcResult<pb::TestCommsResponse> {
        Ok(Response::new(pb::TestCommsResponse {
            response_header: Some(ok_header()),
            response: r.into_inner().message,
        }))
    }

    /// Register a subscriber and immediately send the header-only frame the real
    /// interface opens every stream with (no `dt_secs`, no `value`).
    async fn start_pulse_counter(
        &self,
        r: Request<pb::PulseCounterRequest>,
    ) -> RpcResult<Self::startPulseCounterStream> {
        let req = r.into_inner();
        // Generously sized so a slow consumer shows up as latency in the
        // measurement rather than as a drop the app never caused.
        let (tx, rx) = mpsc::channel(65536);
        let _ = tx
            .send(Ok(pb::PulseCounterResponse {
                response_header: Some(ok_header()),
                di: Some(req.di),
                value: None,
                dt_secs: None,
            }))
            .await;
        self.0.subscribers.lock().unwrap().push(Subscriber {
            pin: req.di,
            edge: if req.edge.is_empty() { "rising".into() } else { req.edge },
            tx,
        });
        Ok(Response::new(ReceiverStream::new(rx)))
    }

    async fn get_di(&self, r: Request<pb::GetDiRequest>) -> RpcResult<pb::GetDiResponse> {
        let levels = self.0.di_levels.lock().unwrap();
        Ok(Response::new(pb::GetDiResponse {
            response_header: Some(ok_header()),
            di: r.into_inner().di.iter().map(|p| levels.get(p).copied().unwrap_or(false)).collect(),
        }))
    }

    async fn get_do(&self, r: Request<pb::GetDoRequest>) -> RpcResult<pb::GetDoResponse> {
        let levels = self.0.do_levels.lock().unwrap();
        Ok(Response::new(pb::GetDoResponse {
            response_header: Some(ok_header()),
            r#do: r
                .into_inner()
                .r#do
                .iter()
                .map(|p| levels.get(p).copied().unwrap_or(false))
                .collect(),
        }))
    }

    async fn set_do(&self, r: Request<pb::SetDoRequest>) -> RpcResult<pb::SetDoResponse> {
        let req = r.into_inner();
        let mut levels = self.0.do_levels.lock().unwrap();
        for (i, pin) in req.r#do.iter().enumerate() {
            let v = req.value.get(i).or_else(|| req.value.first()).copied().unwrap_or(false);
            levels.insert(*pin, v);
        }
        Ok(Response::new(pb::SetDoResponse {
            response_header: Some(ok_header()),
            r#do: req.r#do.iter().map(|p| levels.get(p).copied().unwrap_or(false)).collect(),
        }))
    }

    async fn set_di_config(
        &self,
        r: Request<pb::SetDiConfigRequest>,
    ) -> RpcResult<pb::SetDiConfigResponse> {
        let req = r.into_inner();
        if let Some(edge) = &req.irq_edge {
            self.0.di_irq_edge.lock().unwrap().insert(req.pin, edge.clone());
        }
        if let Some(ms) = req.debounce_ms {
            self.0.di_debounce_ms.lock().unwrap().insert(req.pin, ms);
        }
        Ok(Response::new(pb::SetDiConfigResponse {
            response_header: Some(ok_header()),
            config: Some(pb::DiConfig {
                pin: req.pin,
                pnp_mode: req.pnp_mode.unwrap_or(true),
                irq_edge: self
                    .0
                    .di_irq_edge
                    .lock()
                    .unwrap()
                    .get(&req.pin)
                    .cloned()
                    .unwrap_or_default(),
                debounce_ms: self
                    .0
                    .di_debounce_ms
                    .lock()
                    .unwrap()
                    .get(&req.pin)
                    .copied()
                    .unwrap_or(0),
                wake_on_event: req.wake_on_event.unwrap_or(false),
            }),
        }))
    }

    async fn get_di_config(
        &self,
        r: Request<pb::GetDiConfigRequest>,
    ) -> RpcResult<pb::GetDiConfigResponse> {
        let pin = r.into_inner().pin;
        Ok(Response::new(pb::GetDiConfigResponse {
            response_header: Some(ok_header()),
            config: Some(pb::DiConfig {
                pin,
                pnp_mode: true,
                irq_edge: self.0.di_irq_edge.lock().unwrap().get(&pin).cloned().unwrap_or_default(),
                debounce_ms: self.0.di_debounce_ms.lock().unwrap().get(&pin).copied().unwrap_or(0),
                wake_on_event: false,
            }),
        }))
    }

    /// Re-serves the whole retained log for the pin on every call and ignores
    /// `events_from`, exactly like the platform does — the client owns its
    /// cursor (`testing/LEARNINGS.md` §4).
    async fn get_di_events(
        &self,
        r: Request<pb::GetDiEventsRequest>,
    ) -> RpcResult<pb::GetDiEventsResponse> {
        let req = r.into_inner();
        let events = self.0.events.lock().unwrap();
        let all = events.get(&req.pin).cloned().unwrap_or_default();
        let keep: Vec<pb::EventDetail> = all
            .into_iter()
            .filter(|e| match e.event.as_str() {
                "DI_R" => req.rising,
                "DI_F" => req.falling,
                _ => req.include_system_events,
            })
            .collect();
        Ok(Response::new(pb::GetDiEventsResponse {
            response_header: Some(ok_header()),
            events_synced: Some(true),
            events: keep,
        }))
    }

    async fn get_ai(&self, _r: Request<pb::GetAiRequest>) -> RpcResult<pb::GetAiResponse> {
        Err(Status::unimplemented("get_ai"))
    }
    async fn schedule_do(
        &self,
        _r: Request<pb::ScheduleDoRequest>,
    ) -> RpcResult<pb::ScheduleDoResponse> {
        Err(Status::unimplemented("schedule_do"))
    }
    async fn get_ao(&self, _r: Request<pb::GetAoRequest>) -> RpcResult<pb::GetAoResponse> {
        Err(Status::unimplemented("get_ao"))
    }
    async fn set_ao(&self, _r: Request<pb::SetAoRequest>) -> RpcResult<pb::SetAoResponse> {
        Err(Status::unimplemented("set_ao"))
    }
    async fn schedule_ao(
        &self,
        _r: Request<pb::ScheduleAoRequest>,
    ) -> RpcResult<pb::ScheduleAoResponse> {
        Err(Status::unimplemented("schedule_ao"))
    }
    async fn get_value(&self, _r: Request<pb::GetValueRequest>) -> RpcResult<pb::GetValueResponse> {
        Err(Status::unimplemented("get_value"))
    }
    async fn set_value(&self, _r: Request<pb::SetValueRequest>) -> RpcResult<pb::SetValueResponse> {
        Err(Status::unimplemented("set_value"))
    }
    async fn get_events(
        &self,
        _r: Request<pb::GetEventsRequest>,
    ) -> RpcResult<pb::GetEventsResponse> {
        Err(Status::unimplemented("get_events"))
    }
    async fn get_system_status(
        &self,
        _r: Request<pb::GetSystemStatusRequest>,
    ) -> RpcResult<pb::GetSystemStatusResponse> {
        Err(Status::unimplemented("get_system_status"))
    }
    async fn get_input_voltage(
        &self,
        _r: Request<pb::GetInputVoltageRequest>,
    ) -> RpcResult<pb::GetInputVoltageResponse> {
        Err(Status::unimplemented("get_input_voltage"))
    }
    async fn get_system_power(
        &self,
        _r: Request<pb::GetSystemPowerRequest>,
    ) -> RpcResult<pb::GetSystemPowerResponse> {
        Err(Status::unimplemented("get_system_power"))
    }
    async fn get_temperature(
        &self,
        _r: Request<pb::GetTemperatureRequest>,
    ) -> RpcResult<pb::GetTemperatureResponse> {
        Err(Status::unimplemented("get_temperature"))
    }
    async fn get_io_table(
        &self,
        _r: Request<pb::GetIoTableRequest>,
    ) -> RpcResult<pb::GetIoTableResponse> {
        Err(Status::unimplemented("get_io_table"))
    }
    async fn sync_rtc_time(
        &self,
        _r: Request<pb::SyncRtcTimeRequest>,
    ) -> RpcResult<pb::SyncRtcTimeResponse> {
        Err(Status::unimplemented("sync_rtc_time"))
    }
    async fn get_location(
        &self,
        _r: Request<pb::GetLocationRequest>,
    ) -> RpcResult<pb::GetLocationResponse> {
        Err(Status::unimplemented("get_location"))
    }
    async fn get_shutdown_immunity(
        &self,
        _r: Request<pb::GetShutdownImmunityRequest>,
    ) -> RpcResult<pb::GetShutdownImmunityResponse> {
        Err(Status::unimplemented("get_shutdown_immunity"))
    }
    async fn set_shutdown_immunity(
        &self,
        _r: Request<pb::SetShutdownImmunityRequest>,
    ) -> RpcResult<pb::SetShutdownImmunityResponse> {
        Err(Status::unimplemented("set_shutdown_immunity"))
    }
    async fn schedule_startup(
        &self,
        _r: Request<pb::ScheduleStartupRequest>,
    ) -> RpcResult<pb::ScheduleStartupResponse> {
        Err(Status::unimplemented("schedule_startup"))
    }
    async fn schedule_shutdown(
        &self,
        _r: Request<pb::ScheduleShutdownRequest>,
    ) -> RpcResult<pb::ScheduleShutdownResponse> {
        Err(Status::unimplemented("schedule_shutdown"))
    }
    async fn reboot(&self, _r: Request<pb::RebootRequest>) -> RpcResult<pb::RebootResponse> {
        Err(Status::unimplemented("reboot"))
    }
    async fn shutdown(&self, _r: Request<pb::ShutdownRequest>) -> RpcResult<pb::ShutdownResponse> {
        Err(Status::unimplemented("shutdown"))
    }
    async fn get_wake_on_voltage(
        &self,
        _r: Request<pb::GetWakeOnVoltageRequest>,
    ) -> RpcResult<pb::GetWakeOnVoltageResponse> {
        Err(Status::unimplemented("get_wake_on_voltage"))
    }
    async fn set_wake_on_voltage(
        &self,
        _r: Request<pb::SetWakeOnVoltageRequest>,
    ) -> RpcResult<pb::SetWakeOnVoltageResponse> {
        Err(Status::unimplemented("set_wake_on_voltage"))
    }
    async fn get_wake_reason(
        &self,
        _r: Request<pb::GetWakeReasonRequest>,
    ) -> RpcResult<pb::GetWakeReasonResponse> {
        Err(Status::unimplemented("get_wake_reason"))
    }
    async fn get_sleep_log(
        &self,
        _r: Request<pb::GetSleepLogRequest>,
    ) -> RpcResult<pb::GetSleepLogResponse> {
        Err(Status::unimplemented("get_sleep_log"))
    }
    async fn get_sleep_log_interval(
        &self,
        _r: Request<pb::GetSleepLogIntervalRequest>,
    ) -> RpcResult<pb::GetSleepLogIntervalResponse> {
        Err(Status::unimplemented("get_sleep_log_interval"))
    }
    async fn set_sleep_log_interval(
        &self,
        _r: Request<pb::SetSleepLogIntervalRequest>,
    ) -> RpcResult<pb::SetSleepLogIntervalResponse> {
        Err(Status::unimplemented("set_sleep_log_interval"))
    }
    async fn load_firmware(
        &self,
        _r: Request<pb::LoadFirmwareRequest>,
    ) -> RpcResult<pb::LoadFirmwareResponse> {
        Err(Status::unimplemented("load_firmware"))
    }
    async fn load_bootloader(
        &self,
        _r: Request<pb::LoadBootloaderRequest>,
    ) -> RpcResult<pb::LoadBootloaderResponse> {
        Err(Status::unimplemented("load_bootloader"))
    }
    async fn get_firmware_version(
        &self,
        _r: Request<pb::GetFirmwareVersionRequest>,
    ) -> RpcResult<pb::GetFirmwareVersionResponse> {
        Err(Status::unimplemented("get_firmware_version"))
    }
}

/// Drive the firmware sweep on its cadence until the returned handle is aborted.
pub fn run_firmware_sweep(state: Arc<PlatformDoubleState>) -> tokio::task::AbortHandle {
    let period = state.sweep_period().expect("firmware sweep must be enabled first");
    let task = tokio::spawn(async move {
        loop {
            // Sleep first, like `check_interrupts` (no sweep at t=0).
            tokio::time::sleep(period).await;
            state.flush_sweep();
        }
    });
    task.abort_handle()
}

/// Start the double on an ephemeral port; returns its shared state and URI.
pub async fn spawn_platform_double() -> (Arc<PlatformDoubleState>, String) {
    let state = Arc::new(PlatformDoubleState::default());
    state.next_event_id.store(1, Ordering::Relaxed);
    let listener =
        tokio::net::TcpListener::bind("127.0.0.1:0").await.expect("bind platform double");
    let addr: SocketAddr = listener.local_addr().expect("local addr");
    let service = PlatformIfaceServer::new(PlatformDouble(state.clone()));
    tokio::spawn(async move {
        let _ = tonic::transport::Server::builder()
            .add_service(service)
            .serve_with_incoming(TcpListenerStream::new(listener))
            .await;
    });
    (state, format!("http://{addr}"))
}

// ---------------------------------------------------------------------------
// The gate model
// ---------------------------------------------------------------------------

/// A toothed target walked one quarter cycle at a time, driving two DI pins.
///
/// Rate convention — `rising_hz_per_sensor` counts **rising edges on ONE pin per
/// second**, nothing else. At the specified 15.0:
///
/// | quantity | value |
/// |---|---|
/// | rising edges/s on one pin | 15 |
/// | rising edges/s combined (callbacks) | 30 |
/// | full tooth cycles/s | 15 |
/// | all edges/s on the wire (both polarities, both pins) | 60 |
/// | A-rise -> B-rise spacing (90 deg) | 16.67 ms |
/// | gap between successive edges of any kind | 16.67 ms |
pub struct QuadratureGate {
    state: Arc<PlatformDoubleState>,
    a_pin: i32,
    b_pin: i32,
    rising_hz_per_sensor: f64,
    phase: usize,
    /// +1 forward (B leads), -1 reverse.
    pub direction: i64,
    /// Signed 2x rising-edge count — the ground truth the decoder must match.
    pub true_position: i64,
    /// Signed quarter cycles travelled: real physical position at 4x.
    pub true_quarter_phase: i64,
    pub rising_emitted: u64,
    pub edges_emitted: u64,
    /// Injection times of every edge, for the pacing-accuracy report.
    pub edge_times: Vec<Instant>,
}

impl QuadratureGate {
    pub fn new(
        state: Arc<PlatformDoubleState>,
        a_pin: i32,
        b_pin: i32,
        rising_hz_per_sensor: f64,
    ) -> Self {
        Self {
            state,
            a_pin,
            b_pin,
            rising_hz_per_sensor,
            phase: 0,
            direction: 1,
            true_position: 0,
            true_quarter_phase: 0,
            rising_emitted: 0,
            edges_emitted: 0,
            edge_times: Vec::new(),
        }
    }

    /// One quarter cycle; also the 90-degree A->B spacing. 16.667 ms at 15 Hz.
    pub fn edge_interval(&self) -> Duration {
        Duration::from_secs_f64(1.0 / (4.0 * self.rising_hz_per_sensor))
    }

    #[allow(dead_code)]
    pub fn cycle_period(&self) -> Duration {
        Duration::from_secs_f64(1.0 / self.rising_hz_per_sensor)
    }

    /// Assert both starting levels without counting them as travel.
    pub fn seed(&self) {
        let (a, b) = GRAY[self.phase];
        self.state.set_di_level(self.a_pin, a);
        self.state.set_di_level(self.b_pin, b);
        self.state.reset_counters();
    }

    /// Advance exactly one quarter cycle. Channels alternate A, B, A, B in BOTH
    /// directions — reversal is modelled purely as flipping the sign of the phase
    /// walk, which is what makes it show up as a same-channel repeat.
    pub fn step(&mut self) {
        let (prev_a, _) = GRAY[self.phase];
        self.phase = (self.phase as i64 + self.direction).rem_euclid(4) as usize;
        let (new_a, new_b) = GRAY[self.phase];
        let level = if new_a != prev_a {
            self.state.set_di_level(self.a_pin, new_a);
            new_a
        } else {
            self.state.set_di_level(self.b_pin, new_b);
            new_b
        };
        self.true_quarter_phase += self.direction;
        self.edges_emitted += 1;
        if level {
            self.true_position += self.direction;
            self.rising_emitted += 1;
        }
        self.edge_times.push(Instant::now());
    }

    /// Run for `duration`, optionally reversing once at `reverse_at`.
    ///
    /// Paced against an ABSOLUTE schedule (`start + n * interval`) so a late
    /// wakeup cannot accumulate drift — a drifting injector would show up as app
    /// jitter and invalidate the whole measurement. Shortfalls are reported by
    /// [`achieved_rising_hz`](Self::achieved_rising_hz) instead of being hidden.
    pub async fn run_for(&mut self, duration: Duration, reverse_at: Option<Duration>) {
        let interval = self.edge_interval();
        let start = Instant::now();
        let mut n = 0u32;
        let mut reversed = false;
        loop {
            let offset = interval.mul_f64(n as f64);
            if offset >= duration {
                return;
            }
            let due = start + offset;
            let now = Instant::now();
            if due > now {
                tokio::time::sleep(due - now).await;
            }
            if let Some(at) = reverse_at {
                if !reversed && start.elapsed() >= at {
                    self.direction = -self.direction;
                    reversed = true;
                }
            }
            self.step();
            n += 1;
        }
    }

    /// Achieved rate over all edges of any polarity on either pin.
    pub fn achieved_hz(&self) -> f64 {
        if self.edge_times.len() < 2 {
            return 0.0;
        }
        let span = self.edge_times[self.edge_times.len() - 1]
            .duration_since(self.edge_times[0])
            .as_secs_f64();
        if span <= 0.0 {
            return 0.0;
        }
        (self.edge_times.len() - 1) as f64 / span
    }

    /// Achieved rising edges/s **per sensor** (a quarter of all edges).
    #[allow(dead_code)]
    pub fn achieved_rising_hz_per_sensor(&self) -> f64 {
        self.achieved_hz() / 4.0
    }

    /// Achieved rising edges/s **combined** across both sensors — the callback
    /// rate the app has to keep up with.
    #[allow(dead_code)]
    pub fn achieved_rising_hz_combined(&self) -> f64 {
        self.achieved_hz() / 2.0
    }

    /// Worst deviation of an injection interval from nominal, in ms — how good
    /// the *injector* was, so app-side jitter can be told apart from it.
    #[allow(dead_code)]
    pub fn max_injection_error_ms(&self) -> f64 {
        let nominal = self.edge_interval().as_secs_f64();
        self.edge_times
            .windows(2)
            .map(|w| (w[1].duration_since(w[0]).as_secs_f64() - nominal).abs() * 1000.0)
            .fold(0.0, f64::max)
    }

    /// Signed gap between the rising-edge count and true physical position, both
    /// in 2x counts. Zero on a one-way run; non-zero after a reversal because a
    /// channel's rising edge sits at a different physical position per
    /// direction. This is the irreducible quantiser error of rising-only sensing.
    #[allow(dead_code)]
    pub fn reversal_error_counts(&self) -> i64 {
        // Half-to-even, matching Python's round() in the reference model.
        let half = self.true_quarter_phase as f64 / 2.0;
        let rounded = if (half.fract().abs() - 0.5).abs() < f64::EPSILON {
            let down = half.trunc();
            if (down as i64) % 2 == 0 {
                down
            } else {
                down + half.signum()
            }
        } else {
            half.round()
        };
        self.true_position - rounded as i64
    }
}
