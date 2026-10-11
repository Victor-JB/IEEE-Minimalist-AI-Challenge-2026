import Foundation
import Combine

// Hardware motor channels and mux channels are independent: the first two
// motors are swapped, while sensors remain in shoulder/elbow/head order.
enum ArmMapping {
    static let motorForJoint = [1, 0, 2]
    static let jointForMotor = [1, 0, 2]
    static let names = ["Shoulder", "Elbow", "Head"]
    static let connections = ["Base → first segment", "First → second segment", "Second segment → head"]
    static let microsteps = [2, 16, 16]
    // Relative joint angles in the calibrated convention, not physical motor numbers.
    static let limits: [ClosedRange<Double>] = [-34...38, -72...79, -80...87]
    static func limitLabel(_ joint: Int) -> String {
        String(format: "Range %+.0f° to %+.0f°", limits[joint].lowerBound, limits[joint].upperBound)
    }
    static func outsideLimits(_ angle: Double?, joint: Int) -> Bool {
        guard let angle, angle.isFinite else { return false }
        return !limits[joint].contains(angle)
    }
    static func installHardwareDefaults(_ defaults: UserDefaults) {
        let migration = "q.arm.hardware.2026-10-11"
        guard !defaults.bool(forKey: migration) else { return }
        for axis in 0..<3 { defaults.set(microsteps[axis], forKey: "q.axis\(axis).microsteps") }
        defaults.set(true, forKey: migration)
    }
}

struct JointCalibration: Codable, Equatable {
    var zero = 0.0
    var reversed = false
    var calibrated = false
    func angle(raw: Double?) -> Double? {
        guard calibrated, zero.isFinite, let raw, raw.isFinite else { return nil }
        return ArmGeometry.wrap((raw - zero) * (reversed ? -1 : 1))
    }
    mutating func capture(raw: Double, knownAngle: Double) {
        guard raw.isFinite, knownAngle.isFinite else { return }
        zero = ArmGeometry.wrap(raw - knownAngle * (reversed ? -1 : 1))
        calibrated = true
    }
}

enum ArmGeometry {
    static func wrap(_ value: Double) -> Double {
        guard value.isFinite else { return 0 }
        let remainder = (value + 180).truncatingRemainder(dividingBy: 360)
        return (remainder < 0 ? remainder + 360 : remainder) - 180
    }

}

final class ArmSettings: ObservableObject {
    @Published var joints: [JointCalibration] { didSet { save() } }
    private let defaults: UserDefaults?
    private struct Saved: Codable { var joints: [JointCalibration] }
    init(defaults: UserDefaults? = .standard) {
        self.defaults = defaults
        let saved = defaults?.data(forKey: "q.arm.geometry.v1").flatMap { try? JSONDecoder().decode(Saved.self, from: $0) }
        if let saved, saved.joints.count == 3,
           saved.joints.allSatisfy({ $0.zero.isFinite }) {
            joints = saved.joints
        } else {
            joints = Array(repeating: JointCalibration(), count: 3)
        }
    }
    private func save() {
        guard joints.count == 3, joints.allSatisfy({ $0.zero.isFinite }),
              let data = try? JSONEncoder().encode(Saved(joints: joints)) else { return }
        defaults?.set(data, forKey: "q.arm.geometry.v1")
    }
}

// Confirmed on this wiring: Forward increases the raw sensor angle for all three
// joints. With the saved calibration this is shoulder +, elbow −, head −.
struct LimitConfiguration: Equatable {
    let zeroUdeg: Int
    let reversed: Int
    let forwardSign: Int
    init?(_ calibration: JointCalibration) {
        guard calibration.calibrated, calibration.zero.isFinite else { return nil }
        zeroUdeg = Int((ArmGeometry.wrap(calibration.zero) * 1_000_000).rounded())
        reversed = calibration.reversed ? 1 : 0
        forwardSign = calibration.reversed ? -1 : 1
    }
    var command: String { "LIMITS \(zeroUdeg) \(reversed) \(forwardSign)" }
    func matches(_ state: BoardStatus?, motor: Int) -> Bool {
        let bounds=ArmMapping.limits[ArmMapping.jointForMotor[motor]]
        return state?.limit_revision == 1 && state?.limit_configured == 1 &&
            state?.limit_zero_udeg == zeroUdeg && state?.limit_reversed == reversed &&
            state?.limit_forward_sign == forwardSign &&
            state?.limit_min_mdeg == Int(bounds.lowerBound * 1000) &&
            state?.limit_max_mdeg == Int(bounds.upperBound * 1000)
    }
    func permits(state: Int?, direction: Int) -> Bool {
        let sign=direction == 0 ? forwardSign : -forwardSign
        return state == 0 || (state == 3 && sign > 0) || (state == 4 && sign < 0)
    }
}
