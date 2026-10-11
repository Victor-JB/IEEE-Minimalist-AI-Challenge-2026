import Foundation
@main struct PowerTests {
    static func main() {
        let model=Controller(preview:true)
        model.connected=true
        var commands=[String]()
        model.onCommand={ commands.append($0); _ = $1 }
        func status(_ awake: Int, _ generation: Int, _ ready: Int, enabled: Int=1, run: Int=0, ack: Int=0) {
            model.receive("@STP {\"v\":2,\"ack\":\(ack),\"error\":0,\"fault\":0,\"hz\":0,\"target\":0,\"dir\":0,\"want_dir\":0,\"run\":\(run),\"ppr\":400,\"phase\":\"READY\",\"max_hz\":2400,\"drivers_awake\":\(awake),\"power_generation\":\(generation),\"power_ready\":\(ready),\"power_enabled\":\(enabled)}")
        }
        status(0,1,0)
        assert(model.powerSupported && model.canArm && !model.beginHold(direction:0))
        model.toggleArmed(); assert(model.armed && commands.isEmpty) // Arm leaves asleep.
        status(0,1,0); assert(model.armed)
        assert(model.beginHold(direction:0))
        assert(commands.last!.hasPrefix("CURVE ") && commands.last!.hasSuffix(" 1"))
        status(1,1,0,run:1,ack:1) // Automatic wake must not disarm during settling.
        assert(model.armed && model.held)
        status(1,1,1,run:1)
        model.endHold(); status(0,1,0)
        assert(!model.held && model.armed && model.canArm)
        model.setMotionEnabled(false)
        assert(commands.last=="POWER 0 1" && !model.armed)
        status(0,2,0,enabled:0)
        assert(!model.canArm && !model.beginHold(direction:0))
        model.setMotionEnabled(true); assert(commands.last=="POWER 1 2")
        status(0,3,0); assert(model.canArm && !model.armed && !model.driversAwake)
        model.toggleArmed(); assert(model.beginHold(direction:0))
        status(1,3,1,run:1,ack:2)
        status(0,4,0,enabled:0) // Lock from a different axis cancels hold.
        assert(!model.held && !model.armed && !model.canArm)
        print("PASS: arm stays asleep, auto-wake preserves hold, release sleeps, explicit lock and re-enable never start motion")
    }
}
