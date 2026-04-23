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
- `flags`: packet-level flags, reserved for future use
- `reserved`: must be sent as `0`
- `seq`: sender-side sequence number
- `send_time_ms`: sender local transport timestamp in milliseconds
- `payload_len`: serialized payload length in bytes

Important timing rule:

- `Header::send_time_ms` is when the packet was sent by the sender
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

Command payloads:

- `PingPayload`: `0` bytes
- `PongPayload`: `0` bytes
- `HilReadyPayload`: `0` bytes
- `ArmPayload`: `0` bytes
- `DisarmPayload`: `0` bytes
- `RtlPayload`: `0` bytes
- `AckPayload`: `4` bytes
- `NackPayload`: `4` bytes
- `ControlWaypointPayload`: `40` bytes
- `MissionWaypointPayload`: `40` bytes
- `CvWaypointPayload`: `32` bytes
- `TofWaypointPayload`: `28` bytes

Waypoint messages include `ref_stamp: SimStamp`. This does not require the FC to
wait until that tick. It records the simulation context in which the command was
generated.

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
6. Wait for `HilResponseFrame`.
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
