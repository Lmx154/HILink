# hilink Protocol Map

All numeric fields are little-endian. Normal HILink UART packet framing is
`COBS(Header + Payload + CRC16) + 0x00`.

Normal HILink is the semantic API spoken by FC, GCS, host tools, and
simulators. RF dialects are transport internals owned by radio bridge firmware:

```text
GCS <-> GS radio module        normal HILink
GS radio <-> vehicle radio     compact RF dialect
vehicle radio <-> FC           normal HILink
```

FC and GCS endpoints must not emit or consume RF dialect messages directly.

## Header

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | version | u8 | 1 |
| 1 | msg_type | u8 | 1 |
| 2 | flags | u8 | 1 |
| 3 | reserved | u8 | 1 |
| 4 | seq | u16 | 2 |
| 6 | send_time_ms | u32 | 4 |
| 10 | payload_len | u16 | 2 |

Header length: 12 bytes.

## Message IDs

| ID | Message | Payload | Direction |
| ---: | --- | --- | --- |
| 1 | Ping | PingPayload | either |
| 2 | Pong | PongPayload | either |
| 3 | Ack | AckPayload | either |
| 4 | Nack | NackPayload | either |
| 5 | Heartbeat | HeartbeatPayload | FC -> host |
| 10 | HilSensorFrame | HilSensorFrame | host -> FC |
| 11 | HilResponseFrame | HilResponseFrame | FC -> host |
| 12 | HilReady | HilReadyPayload | FC -> host |
| 20 | Imu | ImuPayload | host -> FC |
| 21 | Mag | MagPayload | host -> FC |
| 22 | Baro | BaroPayload | host -> FC |
| 23 | Gps | GpsPayload | host -> FC |
| 24 | Battery | reserved | host -> FC |
| 25 | SystemState | SystemStatePayload | FC -> host |
| 26 | MotorState | MotorStatePayload | FC -> host |
| 27 | EstimatorState | EstimatorStatePayload | FC -> host |
| 28 | RadioStatus | reserved | host -> FC |
| 29 | TelemetrySnapshot | TelemetrySnapshotPayload | FC -> host |
| 40 | Arm | ArmPayload | host -> FC |
| 41 | Disarm | DisarmPayload | host -> FC |
| 42 | ControlWaypoint | ControlWaypointPayload | host -> FC |
| 43 | CvWaypoint | CvWaypointPayload | host -> FC |
| 44 | TofWaypoint | TofWaypointPayload | host -> FC |
| 45 | MissionWaypoint | MissionWaypointPayload | host -> FC |
| 46 | Rtl | RtlPayload | host -> FC |
| 60 | BenchEnable | BenchEnablePayload | host -> FC |
| 61 | BenchDisable | BenchDisablePayload | host -> FC |
| 62 | MotorTest | MotorTestPayload | host -> FC |
| 63 | MotorSweep | MotorSweepPayload | host -> FC |
| 64 | MotorStop | MotorStopPayload | host -> FC |
| 65 | DshotCommand | DshotCommandPayload | host -> FC |
| 66 | ActuatorStatusRequest | ActuatorStatusRequestPayload | host -> FC |
| 67 | ActuatorStatus | ActuatorStatusPayload | FC -> host |

Normal HILink messages are the semantic API spoken by FC, GCS, host tools, and
simulators. RF dialect messages are not valid `MsgType` values.

## Payload Sizes

| Payload | Bytes |
| --- | ---: |
| PingPayload | 0 |
| PongPayload | 0 |
| HilReadyPayload | 0 |
| ArmPayload | 0 |
| DisarmPayload | 0 |
| RtlPayload | 0 |
| BenchDisablePayload | 0 |
| MotorStopPayload | 0 |
| ActuatorStatusRequestPayload | 0 |
| AckPayload | 4 |
| NackPayload | 4 |
| DshotCommandPayload | 4 |
| BenchEnablePayload | 8 |
| MotorTestPayload | 12 |
| MotorSweepPayload | 16 |
| ActuatorStatusPayload | 20 |
| HeartbeatPayload | 24 |
| MotorStatePayload | 24 |
| BaroPayload | 28 |
| MagPayload | 28 |
| TofWaypointPayload | 28 |
| CvWaypointPayload | 32 |
| ControlWaypointPayload | 40 |
| ImuPayload | 40 |
| MissionWaypointPayload | 40 |
| GpsPayload | 52 |
| HilResponseFrame | 72 |
| TelemetrySnapshotPayload | 74 |
| EstimatorStatePayload | 80 |
| HilSensorFrame | 115 |

## Shared Blocks

### SimStamp, 16 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | sim_tick | u64 | 8 |
| 8 | sim_time_us | u64 | 8 |

## Runtime Payloads

### HilSensorFrame, 115 bytes

| Field | Type |
| --- | --- |
| stamp | SimStamp |
| valid_flags | u32 |
| accel_mps2 | [f32; 3] |
| gyro_rps | [f32; 3] |
| mag_ut | [f32; 3] |
| pressure_pa | f32 |
| baro_altitude_m | f32 |
| temperature_c | f32 |
| lat_deg | f64 |
| lon_deg | f64 |
| alt_msl_m | f32 |
| vel_ned_mps | [f32; 3] |
| sats | u8 |
| fix_type | u8 |
| reserved0 | [u8; 3] |
| battery_voltage_v | f32 |
| rssi_dbm | i16 |
| snr_db_x100 | i16 |
| loss_pct_x100 | u16 |

### HilResponseFrame, 72 bytes

| Field | Type |
| --- | --- |
| stamp | SimStamp |
| system_state | u8 |
| reserved0 | [u8; 3] |
| flags | u32 |
| position_ned_m | [f32; 3] |
| velocity_ned_mps | [f32; 3] |
| attitude_quat | [f32; 4] |
| motor_cmd | [u16; 4] |

### Telemetry and State Payloads

| Payload | Fields |
| --- | --- |
| HeartbeatPayload | stamp: SimStamp, system_state: u8, reserved0: [u8; 3], flags: u32 |
| SystemStatePayload | stamp: SimStamp, system_state: u8, reserved0: [u8; 3], flags: u32, battery_voltage_v: f32 |
| MotorStatePayload | stamp: SimStamp, motor_cmd: [u16; 4] |
| EstimatorStatePayload | stamp: SimStamp, position_ned_m: [f32; 3], velocity_ned_mps: [f32; 3], attitude_quat: [f32; 4], gyro_bias: [f32; 3], accel_bias: [f32; 3] |
| TelemetrySnapshotPayload | stamp: SimStamp, system_state: u8, reserved0: [u8; 3], flags: u32, position_ned_m: [f32; 3], velocity_ned_mps: [f32; 3], attitude_quat: [f32; 4], battery_voltage_v: f32, rssi_dbm: i16, snr_db_x100: i16, loss_pct_x100: u16 |

### Sensor Component Payloads

| Payload | Fields |
| --- | --- |
| ImuPayload | stamp: SimStamp, accel_mps2: [f32; 3], gyro_rps: [f32; 3] |
| MagPayload | stamp: SimStamp, mag_ut: [f32; 3] |
| BaroPayload | stamp: SimStamp, pressure_pa: f32, altitude_m: f32, temperature_c: f32 |
| GpsPayload | stamp: SimStamp, lat_deg: f64, lon_deg: f64, alt_msl_m: f32, vel_ned_mps: [f32; 3], sats: u8, fix_type: u8, reserved: [u8; 2] |

## RF Dialect Payloads

RF dialect messages are compact semantic packets intended for the flight radio
link. They are not normal HILink messages and do not use the normal HILink
12-byte header or COBS delimiter.

The compact RF frame is:

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | rf_msg_type | u8 | 1 |
| 1 | payload | bytes | payload length |
| 1 + payload_len | crc16 | u16 | 2 |

CRC is CRC16-CCITT-FALSE over `rf_msg_type + payload`, stored little-endian.
In Rust, compact RF helpers are available under `hilink::rf`, and LoRa payloads
and constants are available under `hilink::rf::lora`.

Pure bridge helpers are also available under `hilink::rf`:

| Helper | Purpose |
| --- | --- |
| `telemetry_to_lora_flight` | Compress `TelemetrySnapshotPayload` into `LoRaFlightSnapshotPayload` |
| `gps_to_lora_gps` | Compress `GpsPayload` into `LoRaGpsSnapshotPayload` |
| `normal_command_to_lora` | Convert supported normal HILink commands into `LoRaCommandPayload` and correlation metadata |
| `lora_command_to_normal` | Convert supported RF commands back into normal command identities |
| `ack_to_lora_command_ack` | Convert normal `Ack`/`Nack` into RF command ACK using correlation metadata |
| `lora_command_ack_to_normal` | Convert RF command ACK back into normal `Ack`/`Nack` using correlation metadata |

V1 active RF message types:

| RF ID | Message | Payload |
| ---: | --- | --- |
| 1 | LoRaFlightSnapshot | LoRaFlightSnapshotPayload |
| 2 | LoRaGpsSnapshot | LoRaGpsSnapshotPayload |
| 3 | LoRaEvent | LoRaEventPayload |
| 4 | LoRaFaults | LoRaFaultsPayload |
| 5 | LoRaLinkStatus | LoRaLinkStatusPayload |
| 16 | LoRaCommand | LoRaCommandPayload |
| 17 | LoRaCommandAck | LoRaCommandAckPayload |
| 18 | LoRaSetProfile | LoRaSetProfilePayload |
| 19 | LoRaRequestSnapshot | LoRaRequestSnapshotPayload |

Use the existing high-bandwidth HIL and component messages for USB,
simulation, tests, and bench work. Radio bridge firmware translates normal
HILink messages to and from this RF dialect.

RF payload sizes:

| Payload | Bytes |
| --- | ---: |
| LoRaRequestSnapshotPayload | 4 |
| LoRaSetProfilePayload | 8 |
| LoRaCommandAckPayload | 12 |
| LoRaFaultsPayload | 14 |
| LoRaEventPayload | 15 |
| LoRaCommandPayload | 16 |
| LoRaLinkStatusPayload | 18 |
| LoRaFlightSnapshotPayload | 22 |
| LoRaGpsSnapshotPayload | 22 |

### LoRaFlightSnapshotPayload, 22 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | time_ms | u32 | 4 |
| 4 | state | u8 | 1 |
| 5 | mode | u8 | 1 |
| 6 | flags | u16 | 2 |
| 8 | altitude_dm | i32 | 4 |
| 12 | vertical_velocity_cms | i16 | 2 |
| 14 | accel_mag_cms2 | u16 | 2 |
| 16 | battery_mv | u16 | 2 |
| 18 | pyro_or_actuator_flags | u16 | 2 |
| 20 | fault_summary | u16 | 2 |

### LoRaGpsSnapshotPayload, 22 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | time_ms | u32 | 4 |
| 4 | lat_e7 | i32 | 4 |
| 8 | lon_e7 | i32 | 4 |
| 12 | alt_msl_dm | i32 | 4 |
| 16 | ground_speed_cms | u16 | 2 |
| 18 | heading_cdeg | u16 | 2 |
| 20 | sats | u8 | 1 |
| 21 | fix_type | u8 | 1 |

### LoRaLinkStatusPayload, 18 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | time_ms | u32 | 4 |
| 4 | uplink_rssi_dbm | i8 | 1 |
| 5 | uplink_snr_x4 | i8 | 1 |
| 6 | downlink_rssi_dbm | i8 | 1 |
| 7 | downlink_snr_x4 | i8 | 1 |
| 8 | rx_packets_delta | u16 | 2 |
| 10 | tx_packets_delta | u16 | 2 |
| 12 | lost_packets_delta | u16 | 2 |
| 14 | active_profile | u8 | 1 |
| 15 | telemetry_rate_hz | u8 | 1 |
| 16 | reserved | u16 | 2 |

### LoRaEventPayload, 15 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | time_ms | u32 | 4 |
| 4 | event_id | u16 | 2 |
| 6 | severity | u8 | 1 |
| 7 | arg0 | i32 | 4 |
| 11 | arg1 | i32 | 4 |

### LoRaFaultsPayload, 14 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | time_ms | u32 | 4 |
| 4 | active_faults | u32 | 4 |
| 8 | latched_faults | u32 | 4 |
| 12 | inhibit_flags | u16 | 2 |

### LoRaCommandPayload, 16 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | command_id | u16 | 2 |
| 2 | command_seq | u16 | 2 |
| 4 | expires_ms | u16 | 2 |
| 6 | flags | u16 | 2 |
| 8 | arg0 | i32 | 4 |
| 12 | arg1 | i32 | 4 |

### LoRaCommandAckPayload, 12 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | command_id | u16 | 2 |
| 2 | command_seq | u16 | 2 |
| 4 | status | u8 | 1 |
| 5 | reason | u8 | 1 |
| 6 | state | u8 | 1 |
| 7 | reserved | u8 | 1 |
| 8 | detail | i32 | 4 |

Other LoRa payloads:

| Payload | Fields |
| --- | --- |
| LoRaSetProfilePayload | command_seq: u16, profile: u8, telemetry_rate_hz: u8, gps_rate_hz: u8, link_status_rate_hz: u8, flags: u16 |
| LoRaRequestSnapshotPayload | command_seq: u16, request_flags: u16 |

Recommended scheduling:

| Profile | Schedule |
| --- | --- |
| SF7/500 kHz | 10 Hz flight snapshots, 1-2 Hz GPS, 1 Hz link status, immediate events/faults/ACKs |
| SF8/500 kHz | 5-10 Hz flight snapshots, 1 Hz GPS, 1 Hz link status, commands preempt telemetry |
| SF8/250 kHz fallback | 5 Hz flight snapshots, 0.5-1 Hz GPS, 1 Hz link status or folded heartbeat |
| Recovery beacon | 1 Hz or lower, flight and GPS alternating, debug disabled |

LoRa command IDs:

| Name | Value |
| --- | ---: |
| lora_command_id::ARM | 1 |
| lora_command_id::DISARM | 2 |
| lora_command_id::ABORT | 3 |
| lora_command_id::MOTOR_STOP | 4 |
| lora_command_id::SET_MODE | 5 |
| lora_command_id::SET_TELEMETRY_RATE | 6 |
| lora_command_id::SET_RADIO_PROFILE | 7 |
| lora_command_id::REQUEST_SNAPSHOT | 8 |
| lora_command_id::REQUEST_GPS | 9 |
| lora_command_id::REQUEST_FAULTS | 10 |
| lora_command_id::ENTER_RECOVERY_BEACON | 11 |
| lora_command_id::PING | 12 |

LoRa command statuses:

| Name | Value |
| --- | ---: |
| lora_command_status::ACCEPTED | 0 |
| lora_command_status::REJECTED | 1 |
| lora_command_status::DENIED_STATE | 2 |
| lora_command_status::DENIED_SAFETY | 3 |
| lora_command_status::INVALID_ARG | 4 |
| lora_command_status::EXPIRED | 5 |
| lora_command_status::DUPLICATE_ACCEPTED | 6 |
| lora_command_status::DUPLICATE_REJECTED | 7 |
| lora_command_status::BUSY | 8 |

### LoRa Field Constants

State values:

| Name | Value |
| --- | ---: |
| lora_state::UNKNOWN | 0 |
| lora_state::BOOT | 1 |
| lora_state::STANDBY | 2 |
| lora_state::ARMED | 3 |
| lora_state::ASCENT | 4 |
| lora_state::COAST | 5 |
| lora_state::DESCENT | 6 |
| lora_state::LANDED | 7 |
| lora_state::ABORT | 8 |
| lora_state::FAILSAFE | 9 |

Mode values:

| Name | Value |
| --- | ---: |
| lora_mode::UNKNOWN | 0 |
| lora_mode::MANUAL | 1 |
| lora_mode::AUTO | 2 |
| lora_mode::GUIDED | 3 |
| lora_mode::HIL | 4 |
| lora_mode::RECOVERY | 5 |

Flight flags:

| Name | Bit |
| --- | ---: |
| lora_flags::ARMED | 0 |
| lora_flags::FAILSAFE | 1 |
| lora_flags::GPS_VALID | 2 |
| lora_flags::ESTIMATOR_VALID | 3 |
| lora_flags::BATTERY_LOW | 4 |
| lora_flags::PYRO_SAFE | 5 |
| lora_flags::RADIO_DEGRADED | 6 |
| lora_flags::RECOVERY_ACTIVE | 7 |

Fault bits used by `LoRaFaultsPayload.active_faults`,
`LoRaFaultsPayload.latched_faults`, and the low 16-bit `fault_summary`:

| Name | Bit |
| --- | ---: |
| lora_fault::IMU | 0 |
| lora_fault::MAG | 1 |
| lora_fault::BARO | 2 |
| lora_fault::GPS | 3 |
| lora_fault::ESTIMATOR | 4 |
| lora_fault::BATTERY | 5 |
| lora_fault::RADIO | 6 |
| lora_fault::STORAGE | 7 |
| lora_fault::ACTUATOR | 8 |
| lora_fault::PYRO | 9 |
| lora_fault::SAFETY_INHIBIT | 10 |

Pyro/actuator flags:

| Name | Bit |
| --- | ---: |
| lora_pyro_actuator_flags::MOTOR_OUTPUT_ACTIVE | 0 |
| lora_pyro_actuator_flags::MOTOR_OUTPUT_CLAMPED | 1 |
| lora_pyro_actuator_flags::PYRO_CONTINUITY_1 | 2 |
| lora_pyro_actuator_flags::PYRO_CONTINUITY_2 | 3 |
| lora_pyro_actuator_flags::PYRO_FIRED_1 | 4 |
| lora_pyro_actuator_flags::PYRO_FIRED_2 | 5 |
| lora_pyro_actuator_flags::PYRO_INHIBITED | 6 |

Event IDs:

| Name | Value |
| --- | ---: |
| lora_event_id::BOOT | 1 |
| lora_event_id::STATE_CHANGE | 2 |
| lora_event_id::MODE_CHANGE | 3 |
| lora_event_id::ARM_ACCEPTED | 4 |
| lora_event_id::DISARMED | 5 |
| lora_event_id::ABORT_TRIGGERED | 6 |
| lora_event_id::GPS_FIX_CHANGED | 7 |
| lora_event_id::RADIO_PROFILE_CHANGED | 8 |
| lora_event_id::FAULT_ASSERTED | 9 |
| lora_event_id::FAULT_CLEARED | 10 |
| lora_event_id::PYRO_FIRED | 11 |
| lora_event_id::RECOVERY_BEACON_ENTERED | 12 |

Event severities:

| Name | Value |
| --- | ---: |
| lora_event_severity::INFO | 0 |
| lora_event_severity::WARNING | 1 |
| lora_event_severity::ERROR | 2 |
| lora_event_severity::CRITICAL | 3 |

Command flags:

| Name | Bit |
| --- | ---: |
| lora_command_flags::URGENT | 0 |
| lora_command_flags::REQUIRE_ARMED | 1 |
| lora_command_flags::ALLOW_WHILE_FAILSAFE | 2 |
| lora_command_flags::QUEUE_IF_BUSY | 3 |

Command ACK reasons:

| Name | Value |
| --- | ---: |
| lora_command_reason::NONE | 0 |
| lora_command_reason::BAD_STATE | 1 |
| lora_command_reason::SAFETY_INHIBIT | 2 |
| lora_command_reason::AUTH_REQUIRED | 3 |
| lora_command_reason::UNSUPPORTED | 4 |
| lora_command_reason::BAD_ARGUMENT | 5 |
| lora_command_reason::EXPIRED | 6 |
| lora_command_reason::DUPLICATE | 7 |
| lora_command_reason::RADIO_BUSY | 8 |

Profile flags:

| Name | Bit |
| --- | ---: |
| lora_profile_flags::TEMPORARY | 0 |
| lora_profile_flags::SAVE_DEFAULT | 1 |
| lora_profile_flags::ENTER_RECOVERY_RX_WINDOWS | 2 |

### LoRa Scaling And Scheduling

Scaling rules:

| Field | Rule |
| --- | --- |
| time_ms | sender-local monotonic milliseconds, wrap allowed |
| altitude_dm | AGL decimeters, `i32::MIN` invalid, otherwise saturate to `i32::MIN + 1..=i32::MAX` |
| alt_msl_dm | MSL decimeters, `i32::MIN` invalid |
| vertical_velocity_cms | centimeters/second, positive upward, saturating `i16` |
| lat_e7/lon_e7 | WGS84 degrees times `1e7`, `i32::MIN` invalid |
| heading_cdeg | centidegrees clockwise from true north, `u16::MAX` invalid |
| RSSI | signed dBm, clamp to `-127..=20` |
| SNR | quarter-dB units, clamp to `-80..=80` |

GPS is invalid when `lora_flags::GPS_VALID` is clear, `fix_type == 0`, or
lat/lon use `lora_scaling::LAT_LON_INVALID_E7`.

`LoRaLinkStatus` is owned by radio bridge firmware. The FC should not emit RF
dialect link-status frames. If link health is needed by the GCS over normal
HILink, the GS radio module translates RF link metadata into a normal HILink
status/reporting message. The implementation must document which component owns
uplink and downlink RSSI/SNR.

Use `LoRaCommand` inside the RF dialect for safety and mission commands. GCS and
FC endpoints still use normal HILink commands such as `Arm`, `Disarm`, `Rtl`,
`MotorStop`, `Ping`, `Ack`, and `Nack`. Use `LoRaSetProfile` and
`LoRaRequestSnapshot` inside the RF dialect for common radio-management
commands. `command_seq` is per-GS radio and monotonic modulo `u16`. Radio bridge
firmware maintains the correlation between normal HILink `Header::seq` and RF
`command_seq`. Receivers keep 16 command results or 30 seconds of duplicate
cache and return duplicate ACK statuses with the original result.

ACK timeout defaults:

| Profile | Timeout | Retries |
| --- | ---: | ---: |
| SF7/500 kHz | 250 ms | 3 |
| SF8/500 kHz | 400 ms | 3 |
| SF8/250 kHz fallback | 750 ms | 4 |
| Recovery beacon | 2000 ms | 2 |

Bridge helpers return `UnsupportedTranslation` when no V1 mapping is defined
for a message, and `CorrelationMismatch` when a normal ACK/NACK does not match
the RF command correlation metadata.

Scheduler priorities:

| Priority | Traffic |
| ---: | --- |
| P0 | abort, motor stop, disarm |
| P1 | command ACK/NACK |
| P2 | faults and events |
| P3 | flight snapshot |
| P4 | GPS and link status |
| P5 | requested debug |

Commands preempt telemetry. Stale telemetry is droppable; stale commands are
not. A flight snapshot is stale once a newer flight snapshot is queued.

### Waypoint Payloads

| Payload | Fields |
| --- | --- |
| ControlWaypointPayload | ref_stamp: SimStamp, lat_deg: f64, lon_deg: f64, alt_msl_m: f32, yaw_deg: f32 |
| MissionWaypointPayload | ref_stamp: SimStamp, lat_deg: f64, lon_deg: f64, alt_msl_m: f32, yaw_deg: f32 |
| CvWaypointPayload | ref_stamp: SimStamp, dir_body: [f32; 3], confidence: f32 |
| TofWaypointPayload | ref_stamp: SimStamp, distance_m: f32, bearing_deg: f32, elevation_deg: f32 |

## Bench Payloads

### BenchEnablePayload, 8 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | magic | u32 | 4 |
| 4 | timeout_ms | u16 | 2 |
| 6 | reserved0 | [u8; 2] | 2 |

`magic = 0x4D4F544F` ("MOTO").

### MotorTestPayload, 12 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | motor_mask | u8 | 1 |
| 1 | mode | u8 | 1 |
| 2 | reserved0 | [u8; 2] | 2 |
| 4 | value | u16 | 2 |
| 6 | duration_ms | u16 | 2 |
| 8 | ramp_ms | u16 | 2 |
| 10 | reserved1 | u16 | 2 |

### MotorSweepPayload, 16 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | motor_mask | u8 | 1 |
| 1 | mode | u8 | 1 |
| 2 | reserved0 | [u8; 2] | 2 |
| 4 | start_value | u16 | 2 |
| 6 | end_value | u16 | 2 |
| 8 | step_value | u16 | 2 |
| 10 | step_duration_ms | u16 | 2 |
| 12 | zero_between_ms | u16 | 2 |
| 14 | repeat_count | u8 | 1 |
| 15 | reserved1 | u8 | 1 |

### DshotCommandPayload, 4 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | motor_mask | u8 | 1 |
| 1 | command | u8 | 1 |
| 2 | repeat_count | u8 | 1 |
| 3 | reserved0 | u8 | 1 |

### ActuatorStatusPayload, 20 bytes

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | armed | u8 | 1 |
| 1 | bench_enabled | u8 | 1 |
| 2 | active_motor_mask | u8 | 1 |
| 3 | mode | u8 | 1 |
| 4 | commanded_dshot | [u16; 4] | 8 |
| 12 | last_command_age_ms | u16 | 2 |
| 14 | bench_timeout_ms | u16 | 2 |
| 16 | flags | u32 | 4 |

## Bench Constants

| Name | Value |
| --- | ---: |
| bench::MOTOR_MASK_M1 | 0x01 |
| bench::MOTOR_MASK_M2 | 0x02 |
| bench::MOTOR_MASK_M3 | 0x04 |
| bench::MOTOR_MASK_M4 | 0x08 |
| bench::MOTOR_MASK_ALL | 0x0F |
| bench::MAX_TEST_DURATION_MS | 5000 |
| motor_test_mode::STOP | 0 |
| motor_test_mode::RAW_DSHOT | 1 |
| motor_test_mode::NORMALIZED | 2 |

## Actuator Flags

| Flag | Bit |
| --- | ---: |
| OUTPUT_ACTIVE | 0 |
| COMMAND_TIMEOUT | 1 |
| CLAMPED | 2 |
| REJECTED_WHILE_DISARMED | 3 |
| BENCH_MODE_ENABLED | 4 |

## Command Acks

`AckPayload`, 4 bytes:

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | acked_seq | u16 | 2 |
| 2 | acked_msg_type | u8 | 1 |
| 3 | status | u8 | 1 |

`NackPayload`, 4 bytes:

| Offset | Field | Type | Bytes |
| ---: | --- | --- | ---: |
| 0 | rejected_seq | u16 | 2 |
| 2 | rejected_msg_type | u8 | 1 |
| 3 | reason | u8 | 1 |
