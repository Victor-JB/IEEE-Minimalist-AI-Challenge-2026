import Foundation
@main
struct ArmTests {
    static func near(_ a: Double, _ b: Double) { assert(abs(a-b)<0.00001, "\(a) != \(b)") }
    static func main() {
        assert(ArmMapping.motorForJoint == [1,0,2])
        for joint in 0..<3 { assert(ArmMapping.jointForMotor[ArmMapping.motorForJoint[joint]] == joint) }
        assert(ArmMapping.limits == [-34.0...38.0, -72.0...79.0, -80.0...87.0])
        for index in 0..<3 {
            let limit = ArmMapping.limits[index]
            assert(!ArmMapping.outsideLimits(limit.lowerBound, joint:index))
            assert(!ArmMapping.outsideLimits(limit.upperBound, joint:index))
            assert(ArmMapping.outsideLimits(limit.lowerBound - 0.01, joint:index))
            assert(ArmMapping.outsideLimits(limit.upperBound + 0.01, joint:index))
            assert(!ArmMapping.outsideLimits(nil, joint:index))
        }
        let motors = (0..<3).map { Controller(axis:$0,preview:true) }
        assert(motors.map(\.microsteps) == [2,16,16])
        assert(motors.map(\.ppr) == [400,3200,3200])
        assert(motors.map(\.maxRPM) == [360,45,45])
        assert(motors.allSatisfy { $0.rampError == nil && !$0.armed && !$0.held })
        var joint = JointCalibration()
        assert(joint.angle(raw:10)==nil)
        joint.capture(raw:359,knownAngle:0)
        near(joint.angle(raw:1)!,2)
        joint.reversed=true; joint.capture(raw:90,knownAngle:30)
        near(joint.angle(raw:100)!,20)
        assert(joint.angle(raw:nil)==nil && joint.angle(raw:.nan)==nil)
        let suite="stepper-arm-test-"+UUID().uuidString
        let defaults=UserDefaults(suiteName:suite)!
        defer { defaults.removePersistentDomain(forName:suite) }
        defaults.set(1,forKey:"q.axis0.microsteps")
        ArmMapping.installHardwareDefaults(defaults)
        assert((0..<3).map { defaults.integer(forKey:"q.axis\($0).microsteps") } == [2,16,16])
        defaults.set(4,forKey:"q.axis1.microsteps")
        ArmMapping.installHardwareDefaults(defaults)
        assert(defaults.integer(forKey:"q.axis1.microsteps")==4) // Later user choices survive launches.
        let settings=ArmSettings(defaults:defaults)
        settings.joints[0].capture(raw:120,knownAngle:45)
        let restored=ArmSettings(defaults:defaults)
        near(restored.joints[0].angle(raw:120)!,45)
        print("PASS: motor/sensor mapping, microstep defaults and migration, calibration, angle wrapping, missing data, saved calibration")
    }
}
