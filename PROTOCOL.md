# hilink Protocol Map

All numeric fields are little-endian. Packet framing is
`COBS(Header + Payload + CRC16) + 0x00`.

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
