import Foundation
@main struct LimitTests {
    static func main() {
        for joint in 0..<3 {
            let axis=ArmMapping.motorForJoint[joint]
            let model=Controller(axis:axis,preview:true)
            var calibration=JointCalibration(zero: [22.5,53.349609375,-30.322265625][joint],reversed:joint != 0,calibrated:true)
            model.limitConfiguration={ LimitConfiguration(calibration) }
            let config=LimitConfiguration(calibration)!
            assert(config.forwardSign == (joint == 0 ? 1 : -1))
            var commands=[String]()
            model.onCommand={ commands.append($0); _ = $1 }
            var clock=Date(); model.now={clock}; model.connected=true
            func status(_ condition: Int, ready: Bool=true, phase: String="READY", run:Int=0, ack:Int=0) {
                let bounds=ArmMapping.limits[joint]
                model.receive("@STP {\"v\":2,\"ack\":\(ack),\"error\":0,\"fault\":0,\"hz\":0,\"target\":0,\"dir\":0,\"want_dir\":0,\"run\":\(run),\"ppr\":\(model.ppr),\"phase\":\"\(phase)\",\"max_hz\":2400,\"limit_revision\":1,\"limit_state\":\(condition),\"limit_configured\":\(ready ? 1 : 0),\"limit_zero_udeg\":\(config.zeroUdeg),\"limit_reversed\":\(config.reversed),\"limit_forward_sign\":\(config.forwardSign),\"limit_min_mdeg\":\(Int(bounds.lowerBound*1000)),\"limit_max_mdeg\":\(Int(bounds.upperBound*1000))}")
            }
            assert(!model.canArm)
            status(1,ready:false)
            assert(!model.canArm && commands.last==config.command)
            status(0); assert(model.canArm)
            model.toggleArmed(); assert(model.canHold(direction:0) && model.canHold(direction:1))
            status(4)
            let inward=config.forwardSign>0 ? 1 : 0
            assert(model.canHold(direction:inward) && !model.canHold(direction:1-inward))
            assert(!model.beginHold(direction:1-inward))
            assert(model.beginHold(direction:inward))
            status(0,phase:"RAMPING",run:1,ack:1)
            status(3,phase:"LIMIT_STOP")
            assert(!model.armed && !model.held)
            status(2); assert(!model.canArm)
            status(0); model.toggleArmed(); assert(model.beginHold(direction:0))
            status(0,phase:"RAMPING",run:1,ack:2)
            calibration.zero += 1 // A stale MCU calibration cannot keep a held move alive.
            model.heartbeat(); assert(!model.held && !model.armed && commands.contains("STOP"))
            clock.addTimeInterval(1); assert(!model.limitReady)
        }
        print("PASS: board calibration handshake, directional limit interlocks, inward recovery, limit stop disarm, sensor loss and stale calibration")
    }
}
