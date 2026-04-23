use hilink::WirePayload;

fn main() {
    println!(
        "hilink protocol v{}: header={}B hil_sensor={}B hil_response={}B",
        hilink::PROTOCOL_VERSION,
        hilink::HEADER_LEN,
        hilink::HilSensorFrame::WIRE_LEN,
        hilink::HilResponseFrame::WIRE_LEN
    );
}
