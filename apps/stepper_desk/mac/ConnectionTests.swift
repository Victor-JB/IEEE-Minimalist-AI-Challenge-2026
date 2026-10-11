// Read-only integration check against the deployed Q: never arms or starts.
import Foundation
import Darwin
@main
struct ConnectionTests {
    static func main() {
        signal(SIGPIPE, SIG_IGN)
        let models = (0..<3).map { Controller(axis: $0) }
        models.forEach { $0.connect() }
        let deadline = Date().addingTimeInterval(8)
        while Date() < deadline {
            RunLoop.main.run(until: Date().addingTimeInterval(0.05))
            if models.allSatisfy({ $0.connected }) { break }
        }
        guard models.allSatisfy({ $0.connected }) else {
            models.forEach { print("Axis \($0.axis+1): \($0.notice)"); $0.shutdown() }
            exit(1)
        }
        let until = Date().addingTimeInterval(3)
        while Date() < until { RunLoop.main.run(until: Date().addingTimeInterval(0.05)) }
        var passed = true
        for model in models {
            let okay = model.connected && !model.armed && !model.held && !model.running && model.telemetry?.fault == 0 && model.telemetry?.hz == 0
            print("Motor \(model.axis+1): connected=\(model.connected), stopped=\(!model.running), fault=\(model.telemetry?.fault ?? -999), disarmed=\(!model.armed)")
            if CommandLine.arguments.contains("--sensors") {
                let responding = model.sensorFresh && model.telemetry?.sensor_error == 0
                print("Sensor \(model.axis+1): raw=\(model.telemetry?.angle_raw ?? -1), magnet=\(model.telemetry?.magnet_status ?? -1), error=\(model.telemetry?.sensor_error ?? -1), age=\(model.telemetry?.sensor_age_ms ?? -1) ms; \(model.sensorNotice)")
                passed = passed && responding
            }
            passed = passed && okay
            model.shutdown()
        }
        exit(passed ? 0 : 1)
    }
}
