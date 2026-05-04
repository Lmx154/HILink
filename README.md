# hilink

`hilink` is a no_std-first UART protocol crate for hardware-in-the-loop
simulation and lightweight drone communication.

The first-class design goal is deterministic HIL correlation:

- a sensor frame belongs to Gazebo or simulator tick `N`
- the flight-controller estimator output also references tick `N`
- the motor command was generated after consuming tick `N`
- the simulator/backend can compare truth, estimate, and actuator output using
  the same simulation stamp

The crate is intended to run in firmware and host software. It has no required
dependencies.

For a compact field-by-field lookup table, see [PROTOCOL.md](PROTOCOL.md).

## Cargo Features

By default, the library builds as `no_std`.

```toml
[dependencies]
hilink = { path = "path/to/hilink", default-features = false }
```

For host/demo binaries, enable `std` explicitly:

```toml
[dependencies]
hilink = { path = "path/to/hilink", features = ["std"] }
```

The current `std` feature only gates the example/demo binary. The protocol
implementation itself uses `core`.

## Wire Format

Each UART frame is:

```text
COBS(Header + Payload + CRC16) + 0x00 delimiter
```

Details:

- byte order: little-endian for all integer and float fields
- frame delimiter: `0x00`
- byte stuffing: COBS
- CRC: CRC16-CCITT-FALSE over `Header + Payload`
- CRC storage: little-endian `u16`
- public Rust structs are `#[repr(C)]`, but the wire format is explicit
  serialization, not raw struct memory

The raw packet before COBS is:

```text
Header      12 bytes
Payload     payload_len bytes
CRC16       2 bytes
```

Use:

```rust
use hilink::WirePayload;

let raw_len = hilink::raw_frame_len(hilink::HilSensorFrame::WIRE_LEN);
let encoded_len = hilink::encoded_frame_len(hilink::HilSensorFrame::WIRE_LEN);
```

`encoded_frame_len` returns a conservative maximum buffer size including the
final `0x00` delimiter.

## Protocol Conventions

HIL messages use these frame, sign, and unit conventions:

- world frame: NED, with +X north, +Y east, and +Z down
- local position: `position_ned_m` is meters from the simulator's chosen local
  NED origin
- GPS position: `lat_deg` and `lon_deg` are WGS84 degrees, and `alt_msl_m` is
  altitude above mean sea level in meters
- body frame: +X forward, +Y right, and +Z down
- quaternion layout: `attitude_quat` is `[w, x, y, z]`
- quaternion meaning: `attitude_quat` is a unit quaternion that rotates vectors
  from body frame to NED frame
- gyro: `gyro_rps` is body angular rate in rad/s, positive by the right-hand
  rule about each body axis
- accel: `accel_mps2` is body-frame specific force in m/s^2, not inertial
  acceleration; a level stationary vehicle reports approximately
  `[0.0, 0.0, -9.81]`
- magnetometer: `mag_ut` is body-frame magnetic field in microtesla
- barometer: `baro_altitude_m` is pressure-derived altitude above mean sea
  level in meters using the simulator/backend's configured atmosphere and
  reference

Actuator commands use normalized motor units:

- `motor_cmd[i]` is a normalized unsigned command for motor `i`
- `0` means stopped/off
- `65535` means maximum command
- the FC must saturate commands to `0..=65535` before sending
- the host/simulator maps normalized commands to rotor velocity, thrust, PWM, or
  another backend-specific actuator model
- if `response_flags::MOTORS_VALID` is clear, the host must not apply
  `motor_cmd`; for first bring-up it should hold the previous valid command or
  command zero according to the test's configured safety policy
- floating-point NaN or infinity values are invalid on the wire; senders should
  clear the matching validity flag or command flag instead of sending non-finite
  sensor, estimator, or actuator values
- raw DShot is reserved for bench commands; normal HIL actuator output should
  remain normalized and be mapped internally to `0` or `48..=2047`

## Header

Every message starts with this header:

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Header {
    pub version: u8,
    pub msg_type: u8,
    pub flags: u8,
    pub reserved: u8,
    pub seq: u16,
    pub send_time_ms: u32,
    pub payload_len: u16,
}
```

Header fields:

- `version`: protocol version, currently `1`
- `msg_type`: numeric `MsgType`
- `flags`: packet-level flags
- `reserved`: must be sent as `0`
- `seq`: sender-side sequence number, monotonic modulo `u16`
- `send_time_ms`: sender-local transport timestamp in milliseconds
- `payload_len`: serialized payload length in bytes

Header flags:

```rust
pub mod header_flags {
    pub const REQUEST_ACK: u8 = 1 << 0;
    pub const RETRANSMISSION: u8 = 1 << 1;
    pub const MORE_FRAGMENTS: u8 = 1 << 2;
    pub const URGENT_CONTROL: u8 = 1 << 3;
}
```

Timing rules:

- `Header::send_time_ms` is when the packet was sent by the sender
- `Header::send_time_ms` is only for transport diagnostics and latency
  estimates
- sender-local clock synchronization is not assumed
- wraparound of `send_time_ms` is expected
- estimator and control timing must not use `send_time_ms`
- `SimStamp` is the simulated instant that the payload belongs to

Do not overload one field to mean both.

## Simulation Stamp

HIL data uses a shared simulation stamp:

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct SimStamp {
    pub sim_tick: u64,
    pub sim_time_us: u64,
}
```

Use both fields:

- `sim_tick`: exact simulator step index for deterministic matching
- `sim_time_us`: simulated time for plots, logs, and interpolation

The flight controller should treat `HilSensorFrame.stamp` as the authoritative
sensor sample time. When it emits estimator or actuator output for that input,
it should copy the same stamp into its response.

## Message IDs

```rust
#[repr(u8)]
pub enum MsgType {
    Ping = 1,
    Pong = 2,
    Ack = 3,
    Nack = 4,
    Heartbeat = 5,

    HilSensorFrame = 10,
    HilResponseFrame = 11,
    HilReady = 12,

    Imu = 20,
    Mag = 21,
    Baro = 22,
    Gps = 23,
    Battery = 24,
    SystemState = 25,
    MotorState = 26,
    EstimatorState = 27,
    RadioStatus = 28,
    TelemetrySnapshot = 29,

    Arm = 40,
    Disarm = 41,
    ControlWaypoint = 42,
    CvWaypoint = 43,
    TofWaypoint = 44,
    MissionWaypoint = 45,
    Rtl = 46,

    BenchEnable = 60,
    BenchDisable = 61,
    MotorTest = 62,
    MotorSweep = 63,
    MotorStop = 64,
    DshotCommand = 65,
    ActuatorStatusRequest = 66,
    ActuatorStatus = 67,

    LoRaHeartbeat = 80,
    LoRaFlightSnapshot = 81,
    LoRaNavSnapshot = 82,
    LoRaGpsSnapshot = 83,
    LoRaPowerSnapshot = 84,
    LoRaEvent = 85,
    LoRaFaults = 86,
    LoRaLinkStatus = 87,

    LoRaCommand = 100,
    LoRaCommandAck = 101,
    LoRaSetProfile = 102,
    LoRaRequestSnapshot = 103,
}
```

Primary HIL runtime messages:

- `HilSensorFrame`: simulator/backend to flight controller
- `HilResponseFrame`: flight controller to simulator/backend
- `HilReady`: flight controller to simulator/backend, sent when firmware has
  toggled HIL mode on and the simulator can begin or resume stepping
- `Heartbeat`: liveness and current FC state
- `Ack` / `Nack`: command acknowledgements

Command/control messages:

- `Arm`
- `Disarm`
- `ControlWaypoint`
- `CvWaypoint`
- `TofWaypoint`
- `MissionWaypoint`
- `Rtl`
- `BenchEnable`
- `BenchDisable`
- `MotorTest`
- `MotorSweep`
- `MotorStop`
- `DshotCommand`
- `ActuatorStatusRequest`

Bench telemetry messages:

- `ActuatorStatus`

LoRa flight radio messages:

- `LoRaFlightSnapshot`: compact 10 Hz flight state
- `LoRaGpsSnapshot`: compact GPS state, usually 1-2 Hz
- `LoRaLinkStatus`: radio/link health, usually 1 Hz
- `LoRaEvent` / `LoRaFaults`: immediate event and fault reporting
- `LoRaCommand` / `LoRaCommandAck`: compact command envelope and ACK
- `LoRaHeartbeat`, `LoRaNavSnapshot`, `LoRaPowerSnapshot`,
  `LoRaSetProfile`, and `LoRaRequestSnapshot`: lower-rate support messages

## Bench Actuator Commands

Bench actuator commands are separate from both arming state and normal HIL motor
output:

```text
Arm/Disarm        -> safety state
Bench commands    -> direct motor validation
HIL motor_cmd      -> simulator/control-loop output
```

`BenchEnable` enters explicit bench-test mode and should be acked or nacked. It
uses `bench::ENABLE_MAGIC` (`0x4D4F544F`, "MOTO") and an auto-expiring
`timeout_ms`. Firmware may require `Arm` before accepting `BenchEnable`, or may
accept bench mode before arm while holding all motor output at zero until armed.
When the timeout expires, all motors return to zero.

`MotorTest` is the primary bring-up command. It selects motors by mask
(`bit0=M1`, `bit1=M2`, `bit2=M3`, `bit3=M4`) and supports these modes:

- `motor_test_mode::STOP` (`0`)
- `motor_test_mode::RAW_DSHOT` (`1`)
- `motor_test_mode::NORMALIZED` (`2`)

Safety policy for `MotorTest`:

- reject if disarmed
- reject if bench mode is disabled
- reject DShot special commands `1..=47`; use `DshotCommand` for those
- clamp throttle to `0` or `48..=2047`, or nack out-of-range input
- limit `duration_ms` to `bench::MAX_TEST_DURATION_MS` (`5000`)
- command timeout always returns all motors to zero

`MotorSweep` allows firmware-driven repeated validation. If multiple motor bits
are set, firmware should sweep one motor at a time unless a later protocol
revision adds an explicit simultaneous flag.

`MotorStop` is a zero-length urgent bench stop. It immediately zeros all motors,
is valid even when bench mode is disabled, and should be acked. It does not
disarm the flight controller.

`DshotCommand` is reserved for ESC special commands such as beep, direction, or
save settings. Early firmware may reject all commands except known-safe beep
commands until actuator control is stable.

`ActuatorStatusRequest` is a zero-length request. `ActuatorStatus` reports what
the firmware believes it is outputting, including `armed`, `bench_enabled`,
`active_motor_mask`, `mode`, four `commanded_dshot` values, command age, bench
timeout, and `actuator_flags`.

Recommended bench smoke test:

```text
1. Arm
2. BenchEnable timeout 30000 ms
3. MotorTest M1 raw DShot 150 for 1000 ms
4. MotorTest M2 raw DShot 150 for 1000 ms
5. MotorTest M3 raw DShot 150 for 1000 ms
6. MotorTest M4 raw DShot 150 for 1000 ms
7. MotorTest all motors raw DShot 120 for 1000 ms
8. MotorTest all motors raw DShot 200 for 1000 ms
9. MotorStop
10. Disarm
```

Recommended fault checks:

- `MotorTest M1 250 for 5000 ms`, stop sending commands, verify timeout zero
- `MotorTest M1 250`, then `MotorStop`, verify immediate zero
- `MotorTest M1 250`, then `Disarm`, verify immediate zero
- try `MotorTest` while disarmed and expect `Nack`
- try raw DShot `47` as throttle and expect `Nack`
- try raw DShot `2048` and expect `Nack` or clamp with `CLAMPED`

## Sequence and Acknowledgement Policy

Each sender maintains its own `Header::seq` counter:

- increment `seq` once for every outbound packet from that sender
- wrap from `65535` to `0`
- compare sequence numbers modulo `u16`; do not treat wraparound as a protocol
  fault by itself
- duplicates of command messages may be ignored after the command has already
  been handled
- streamed HIL traffic is matched by `SimStamp`, not by `seq`

`AckPayload` and `NackPayload` refer to both sequence number and message type:

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct AckPayload {
    pub acked_seq: u16,
    pub acked_msg_type: u8,
    pub status: u8,
}

#[repr(C)]
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct NackPayload {
    pub rejected_seq: u16,
    pub rejected_msg_type: u8,
    pub reason: u8,
}
```

First-version reliability policy:

- `Ack` / `Nack` are for control and command messages such as `Arm`, `Disarm`,
  `Rtl`, waypoint commands, bench commands, and optional `HilReady`
  confirmation
- `HilSensorFrame` and `HilResponseFrame` are not retransmitted
- HIL sensor/response frames are not acked in the normal runtime loop
- a retransmitted command should set `header_flags::RETRANSMISSION`
- if a command needs positive confirmation, set `header_flags::REQUEST_ACK`
- duplicate HIL frames with an already-processed `sim_tick` are dropped
- out-of-order HIL frames are protocol errors during the deterministic bring-up
  loop

## HIL Ready Signal

`HilReadyPayload` is a zero-length payload:

```rust
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct HilReadyPayload;
```

Direction: flight controller to simulator/backend.

Meaning: HIL mode has been toggled on inside the FC firmware, and the simulator
is clear to start sending HIL sensor frames.

The message type is the signal. Use `Header::seq` and `Header::send_time_ms` for
transport correlation. If the simulator requires confirmation, it can respond
with `Ack`.

Example:

```rust
use hilink::{HilReadyPayload, WirePayload};

fn send_hil_ready(seq: u16, send_time_ms: u32) -> hilink::Result<()> {
    let ready = HilReadyPayload;
    let mut raw = [0u8; hilink::raw_frame_len(HilReadyPayload::WIRE_LEN)];
    let mut encoded = [0u8; hilink::encoded_frame_len(HilReadyPayload::WIRE_LEN)];

    let n = hilink::encode_packet(&ready, seq, send_time_ms, &mut raw, &mut encoded)?;
    uart_write(&encoded[..n]);
    Ok(())
}

fn uart_write(_bytes: &[u8]) {
    // write to UART, USB serial, or another byte transport
}
```

## HIL Sensor Frame

`HilSensorFrame` is the recommended first input message for deterministic HIL.
It bundles the sensor data for one simulator step.

Direction: simulator/backend to flight controller.

Serialized payload length: `115` bytes.

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilSensorFrame {
    pub stamp: SimStamp,
    pub valid_flags: u32,

    pub accel_mps2: [f32; 3],
    pub gyro_rps: [f32; 3],
    pub mag_ut: [f32; 3],

    pub pressure_pa: f32,
    pub baro_altitude_m: f32,
    pub temperature_c: f32,

    pub lat_deg: f64,
    pub lon_deg: f64,
    pub alt_msl_m: f32,
    pub vel_ned_mps: [f32; 3],
    pub sats: u8,
    pub fix_type: u8,
    pub reserved0: [u8; 3],

    pub battery_voltage_v: f32,
    pub rssi_dbm: i16,
    pub snr_db_x100: i16,
    pub loss_pct_x100: u16,
}
```

Validity flags:

```rust
pub mod valid {
    pub const ACCEL: u32 = 1 << 0;
    pub const GYRO: u32 = 1 << 1;
    pub const MAG: u32 = 1 << 2;
    pub const BARO: u32 = 1 << 3;
    pub const GPS: u32 = 1 << 4;
    pub const BATTERY: u32 = 1 << 5;
    pub const RADIO: u32 = 1 << 6;
}
```

Use `valid_flags` because sensors may update at different rates:

- IMU can be valid every simulator tick
- GPS may be valid every 20th tick
- barometer, magnetometer, battery, and radio may update slower

## HIL Response Frame

`HilResponseFrame` is the recommended first response message from the flight
controller.

Direction: flight controller to simulator/backend.

Serialized payload length: `72` bytes.

```rust
#[repr(C)]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct HilResponseFrame {
    pub stamp: SimStamp,
    pub system_state: u8,
    pub reserved0: [u8; 3],
    pub flags: u32,

    pub position_ned_m: [f32; 3],
    pub velocity_ned_mps: [f32; 3],
    pub attitude_quat: [f32; 4],

    pub motor_cmd: [u16; 4],
}
```

Response flags:

```rust
pub mod response_flags {
    pub const ARMED: u32 = 1 << 0;
    pub const FAILSAFE: u32 = 1 << 1;
    pub const ESTIMATOR_VALID: u32 = 1 << 2;
    pub const MOTORS_VALID: u32 = 1 << 3;
}
```

The FC should copy the input `HilSensorFrame.stamp` into the response. This
means the simulator can directly match:

- simulator truth at tick `N`
- sensor frame at tick `N`
- estimator output for tick `N`
- actuator output for tick `N`

`motor_cmd` follows the normalized actuator convention from
[Protocol Conventions](#protocol-conventions): `0` is off, `65535` is maximum,
and the simulator/backend owns the mapping from normalized command to the
physical actuator model.

## Other Payloads

Messages with `SimStamp`:

- `ImuPayload`: `40` bytes
- `MagPayload`: `28` bytes
- `BaroPayload`: `28` bytes
- `GpsPayload`: `52` bytes
- `EstimatorStatePayload`: `80` bytes
- `MotorStatePayload`: `24` bytes
- `HeartbeatPayload`: `24` bytes
- `SystemStatePayload`: `28` bytes
- `TelemetrySnapshotPayload`: `74` bytes

Status payloads without `SimStamp`:

- `ActuatorStatusPayload`: `20` bytes
- `LoRaHeartbeatPayload`: `12` bytes
- `LoRaFlightSnapshotPayload`: `22` bytes
- `LoRaNavSnapshotPayload`: `18` bytes
- `LoRaGpsSnapshotPayload`: `22` bytes
- `LoRaPowerSnapshotPayload`: `12` bytes
- `LoRaEventPayload`: `15` bytes
- `LoRaFaultsPayload`: `14` bytes
- `LoRaLinkStatusPayload`: `18` bytes

Command payloads:

- `PingPayload`: `0` bytes
- `PongPayload`: `0` bytes
- `HilReadyPayload`: `0` bytes
- `ArmPayload`: `0` bytes
- `DisarmPayload`: `0` bytes
- `RtlPayload`: `0` bytes
- `BenchEnablePayload`: `8` bytes
- `BenchDisablePayload`: `0` bytes
- `MotorTestPayload`: `12` bytes
- `MotorSweepPayload`: `16` bytes
- `MotorStopPayload`: `0` bytes
- `DshotCommandPayload`: `4` bytes
- `ActuatorStatusRequestPayload`: `0` bytes
- `LoRaCommandPayload`: `16` bytes
- `LoRaCommandAckPayload`: `12` bytes
- `LoRaSetProfilePayload`: `8` bytes
- `LoRaRequestSnapshotPayload`: `4` bytes
- `AckPayload`: `4` bytes
- `NackPayload`: `4` bytes
- `ControlWaypointPayload`: `40` bytes
- `MissionWaypointPayload`: `40` bytes
- `CvWaypointPayload`: `32` bytes
- `TofWaypointPayload`: `28` bytes

Waypoint messages include `ref_stamp: SimStamp`. This does not require the FC to
wait until that tick. It records the simulation context in which the command was
generated.

## LoRa Flight Radio Profile

The LoRa dialect is optimized around small semantic packets rather than raw
sensor or estimator frames. A complete HILink packet still carries the normal
12-byte HILink header, 2-byte CRC, and COBS delimiter, so the LoRa payloads stay
small: normally 12-28 bytes.

Core telemetry:

```text
LoRaFlightSnapshotPayload, 22 bytes
time_ms: u32
state: u8
mode: u8
flags: u16
altitude_dm: i32
vertical_velocity_cms: i16
accel_mag_cms2: u16
battery_mv: u16
pyro_or_actuator_flags: u16
fault_summary: u16
```

GPS is separate and lower-rate:

```text
LoRaGpsSnapshotPayload, 22 bytes
time_ms: u32
lat_e7: i32
lon_e7: i32
alt_msl_dm: i32
ground_speed_cms: u16
heading_cdeg: u16
sats: u8
fix_type: u8
```

Link health:

```text
LoRaLinkStatusPayload, 18 bytes
time_ms: u32
uplink_rssi_dbm: i8
uplink_snr_x4: i8
downlink_rssi_dbm: i8
downlink_snr_x4: i8
rx_packets_delta: u16
tx_packets_delta: u16
lost_packets_delta: u16
active_profile: u8
telemetry_rate_hz: u8
reserved: u16
```

Events, faults, and commands:

```text
LoRaEventPayload, 15 bytes
time_ms: u32
event_id: u16
severity: u8
arg0: i32
arg1: i32

LoRaFaultsPayload, 14 bytes
time_ms: u32
active_faults: u32
latched_faults: u32
inhibit_flags: u16

LoRaCommandPayload, 16 bytes
command_id: u16
command_seq: u16
expires_ms: u16
flags: u16
arg0: i32
arg1: i32

LoRaCommandAckPayload, 12 bytes
command_id: u16
command_seq: u16
status: u8
reason: u8
state: u8
reserved: u8
detail: i32
```

Command IDs live in `lora_command_id` and include `ARM`, `DISARM`, `ABORT`,
`MOTOR_STOP`, `SET_MODE`, `SET_TELEMETRY_RATE`, `SET_RADIO_PROFILE`,
`REQUEST_SNAPSHOT`, `REQUEST_GPS`, `REQUEST_FAULTS`,
`ENTER_RECOVERY_BEACON`, and `PING`.

ACK statuses live in `lora_command_status`: `ACCEPTED`, `REJECTED`,
`DENIED_STATE`, `DENIED_SAFETY`, `INVALID_ARG`, `EXPIRED`,
`DUPLICATE_ACCEPTED`, `DUPLICATE_REJECTED`, and `BUSY`.

Recommended scheduling:

- SF7/500 kHz: 10 Hz `LoRaFlightSnapshot`, 1-2 Hz `LoRaGpsSnapshot`, 1 Hz
  `LoRaLinkStatus`, immediate events/faults/command ACKs
- SF8/500 kHz: 5-10 Hz flight snapshots, 1 Hz GPS, 1 Hz link status, commands
  preempt telemetry
- SF8/250 kHz fallback: 5 Hz flight snapshots, 0.5-1 Hz GPS, 1 Hz link status
  or folded heartbeat
- Recovery beacon: 1 Hz or lower, flight and GPS alternating, debug disabled

## Encoding Packets

Use `encode_packet` when the payload type determines the message ID:

```rust
use hilink::{HilSensorFrame, SimStamp, WirePayload, valid};

fn send_sensor_frame(seq: u16, send_time_ms: u32) -> hilink::Result<()> {
    let frame = HilSensorFrame {
        stamp: SimStamp {
            sim_tick: 15_234,
            sim_time_us: 76_170_000,
        },
        valid_flags: valid::ACCEL | valid::GYRO | valid::GPS,
        accel_mps2: [0.0, 0.0, -9.81],
        gyro_rps: [0.0, 0.0, 0.0],
        mag_ut: [0.0, 0.0, 0.0],
        pressure_pa: 101_325.0,
        baro_altitude_m: 0.0,
        temperature_c: 25.0,
        lat_deg: 30.2672,
        lon_deg: -97.7431,
        alt_msl_m: 171.0,
        vel_ned_mps: [0.0, 0.0, 0.0],
        sats: 12,
        fix_type: 3,
        reserved0: [0; 3],
        battery_voltage_v: 16.2,
        rssi_dbm: -48,
        snr_db_x100: 1_150,
        loss_pct_x100: 0,
    };

    let mut raw = [0u8; hilink::raw_frame_len(HilSensorFrame::WIRE_LEN)];
    let mut encoded = [0u8; hilink::encoded_frame_len(HilSensorFrame::WIRE_LEN)];

    let n = hilink::encode_packet(&frame, seq, send_time_ms, &mut raw, &mut encoded)?;

    uart_write(&encoded[..n]);
    Ok(())
}

fn uart_write(_bytes: &[u8]) {
    // write to UART, USB serial, or another byte transport
}
```

Use `encode_packet_raw` only when you need to set header fields manually.

## Decoding Packets

Feed one full COBS-delimited frame into `decode_packet`, including the final
`0x00` delimiter.

```rust
use hilink::{HilSensorFrame, MsgType, WirePayload};

fn handle_frame(incoming_frame: &[u8]) -> hilink::Result<()> {
    let mut raw = [0u8; hilink::raw_frame_len(HilSensorFrame::WIRE_LEN)];
    let packet = hilink::decode_packet(incoming_frame, &mut raw)?;

    match packet.header.message_type()? {
        MsgType::HilSensorFrame => {
            let sensors = hilink::decode_payload::<HilSensorFrame>(&packet)?;
            handle_hil_sensors(sensors);
        }
        MsgType::HilReady => {
            let _ready = hilink::decode_payload::<hilink::HilReadyPayload>(&packet)?;
            handle_hil_ready();
        }
        _ => {
            handle_other_packet(packet.header.msg_type);
        }
    }

    Ok(())
}

fn handle_hil_sensors(_sensors: hilink::HilSensorFrame) {}
fn handle_hil_ready() {}
fn handle_other_packet(_msg_type: u8) {}
```

For UART streams, accumulate bytes until `0x00`, then pass that complete frame
to `decode_packet`.

## Recommended Bring-up Loop

Simulator/backend:

1. Wait for `HilReady` from the FC firmware.
2. Step the simulator once.
3. Read simulator tick and simulated time.
4. Build `HilSensorFrame { stamp: SimStamp { sim_tick, sim_time_us }, ... }`.
5. Send the sensor frame over UART.
6. Wait for exactly one matching `HilResponseFrame`.
7. Verify the response stamp matches the input stamp.
8. Log simulator truth, sensor input, FC estimate, and FC motor output for tick
   `N`.
9. Apply `motor_cmd` to the simulator bridge.
10. Repeat.

Flight controller:

1. Toggle HIL mode in firmware.
2. Send `HilReady`.
3. Receive `HilSensorFrame`.
4. Treat `HilSensorFrame.stamp` as the authoritative sensor sample time.
5. Run estimator/control for that input.
6. Send `HilResponseFrame` with the same stamp.

## HIL Step Failure Policy

The first-version HIL loop is lockstep:

- the simulator/backend allows exactly one outstanding `HilSensorFrame`
- the simulator/backend must not send tick `N + 1` until tick `N` has either
  produced a matching response or timed out
- the FC should process at most one HIL sensor frame at a time
- if the FC receives a new `HilSensorFrame` while one is already in flight, it
  should ignore it or send `Nack`; the simulator/backend should treat either
  outcome as a protocol error during bring-up
- a `HilResponseFrame` whose `stamp` does not exactly match the outstanding
  input `stamp` is a protocol error
- a stale response for an older `sim_tick` is dropped and counted as a protocol
  fault
- a duplicate response for the current `sim_tick` is dropped after the first
  valid response has been applied
- a duplicate sensor frame for an already-processed `sim_tick` is dropped by the
  FC

Timeout policy is chosen by the host test configuration:

- if no response arrives before the timeout, flag a HIL fault for that tick
- the host should either hold the last valid motor command or command zero
- the timeout result must be logged with the missed `SimStamp`
- the simulator should not silently advance and later apply a stale motor
  command to a newer tick

## Error Handling

The crate returns `hilink::Error`:

- `BufferTooSmall`: caller-provided scratch/output buffer is too small
- `PayloadTooLarge`: payload cannot fit in `Header::payload_len`
- `HeaderTooShort`: decoded raw frame is shorter than a header
- `PayloadLenMismatch`: header length does not match the payload/body length
- `BadDelimiter`: encoded frame does not end in `0x00`
- `CobsDecodeZero`: invalid zero byte inside COBS data
- `CobsDecodeOverrun`: invalid COBS code overruns input
- `CrcMismatch`: CRC16 validation failed
- `UnknownMsgType`: `msg_type` is not recognized
- `InvalidPayloadLength`: payload decoder got the wrong byte length

## Testing

Run library tests:

```bash
cargo test --lib
```

Check the firmware/no_std path:

```bash
cargo check --no-default-features --lib
```

Check the demo binary:

```bash
cargo check --features std --bin hilink-demo
```

## License

MIT
