import Foundation
@main
struct SensorTests {
    static func main() {
        let model = Controller(preview: true)
        model.connected = true
        var clock = Date()
        model.now = { clock }
        func reading(raw: Int=2048, magnet: Int=32, error: Int=0, age: Int=20) {
            model.receive("@STP {\"v\":2,\"ack\":0,\"error\":0,\"fault\":0,\"hz\":0,\"target\":0,\"dir\":0,\"want_dir\":0,\"run\":0,\"ppr\":200,\"phase\":\"READY\",\"max_hz\":2400,\"angle_raw\":\(raw),\"magnet_status\":\(magnet),\"sensor_error\":\(error),\"sensor_age_ms\":\(age)}")
        }
        reading(); assert(model.measuredAngle == 180 && model.sensorHealthy)
        reading(raw: 4095); assert(abs(model.measuredAngle! - 359.912109375)<0.00001)
        reading(magnet: 0); assert(model.measuredAngle == nil && model.sensorNotice == "No magnet detected")
        reading(magnet: 40); assert(model.measuredAngle == nil && model.sensorNotice.contains("strong"))
        reading(magnet: 48); assert(model.measuredAngle == 180 && model.sensorNotice.contains("Live angle"))
        reading(magnet: 16); assert(model.measuredAngle == nil) // Low field without magnet detection is unavailable.
        reading(error: 2); assert(model.measuredAngle == nil && model.sensorNotice.contains("Multiplexer"))
        reading(error: 3); assert(model.measuredAngle == nil && model.sensorNotice.contains("channel 0"))
        reading(age: 501); assert(model.measuredAngle == nil && model.sensorNotice.contains("stale"))
        reading(); clock.addTimeInterval(0.7); assert(model.measuredAngle == nil)
        reading(raw: 5000); assert(model.measuredAngle == nil)
        model.connected = false; assert(model.measuredAngle == nil)
        print("Sensor UI checks passed: angle conversion, magnet diagnostics, stale and invalid data")
    }
}
