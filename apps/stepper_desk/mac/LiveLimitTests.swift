// Read-only hardware verification: syncs guard calibration, never arms or starts.
import Foundation
import Darwin
@main struct LiveLimitTests {
    static func main() {
        signal(SIGPIPE,SIG_IGN)
        let settings=ArmSettings(defaults:UserDefaults(suiteName:"dev.tongtong.stepperdeskq"))
        guard settings.joints.allSatisfy(\.calibrated) else { print("Saved calibration missing"); exit(1) }
        let motors=(0..<3).map { Controller(axis:$0) }
        for motor in motors {
            let joint=ArmMapping.jointForMotor[motor.axis]
            motor.limitConfiguration={ LimitConfiguration(settings.joints[joint]) }
            motor.onCommand={ command,_ in
                precondition(!command.hasPrefix("CURVE") && !command.hasPrefix("HOLD"),"Read-only test must not start motion")
            }
            motor.connect()
        }
        let deadline=Date().addingTimeInterval(12)
        while Date()<deadline {
            RunLoop.main.run(until:Date().addingTimeInterval(0.05))
            if motors.allSatisfy({ $0.connected && $0.limitReady }) { break }
        }
        var passed=true
        for motor in motors {
            let okay=motor.connected && motor.limitReady && !motor.running && !motor.armed && !motor.held && motor.telemetry?.hz==0 && motor.telemetry?.fault==0
            let joint=ArmMapping.jointForMotor[motor.axis]
            print("\(ArmMapping.names[joint]) / motor \(motor.axis+1): verified=\(okay), state=\(motor.telemetry?.limit_state ?? -1), angle=\(Double(motor.telemetry?.joint_angle_mdeg ?? 0)/1000)°, limits=\(motor.telemetry?.limit_min_mdeg ?? 0)…\(motor.telemetry?.limit_max_mdeg ?? 0) mdeg, forward sign=\(motor.telemetry?.limit_forward_sign ?? 0), stopped=\(!motor.running), disarmed=\(!motor.armed)")
            print(motor.limitNotice)
            if !okay { print(motor.notice) }
            passed = passed && okay
            motor.shutdown()
        }
        exit(passed ? 0 : 1)
    }
}
