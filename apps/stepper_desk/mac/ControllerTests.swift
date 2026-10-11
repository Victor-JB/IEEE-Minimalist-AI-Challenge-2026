import Foundation
@main
struct ControllerTests {
    static func status(_ model: Controller, ack: UInt32 = 0, run: Int = 0, phase: String = "READY", error: Int = 0, v: Int = 2) {
        model.receive("@STP {\"v\":\(v),\"ack\":\(ack),\"error\":\(error),\"fault\":0,\"hz\":\(run == 1 ? 500 : 0),\"target\":1000,\"dir\":0,\"want_dir\":0,\"run\":\(run),\"ppr\":200,\"phase\":\"\(phase)\",\"max_hz\":2400}")
    }
    static func main() {
        let model = Controller(preview: true)
        var clock = Date()
        model.now = { clock }
        var commands: [(String, Bool)] = []
        model.onCommand = { commands.append(($0, $1)) }
        assert(model.microsteps == 2)
        model.microsteps = 1
        assert(model.ppr == 200 && model.maxRPM == 720 && model.minRPM == 6)
        model.rpm = 60; assert(model.requestedHz == 200)
        model.microsteps = 16; model.rpm = 30
        assert(model.ppr == 3200 && model.maxRPM == 45 && model.requestedHz == 1600)
        model.microsteps = 1; model.rpm = 300
        assert(model.effectiveRamp[1].rpm == 150 && model.rampError == nil)
        model.vref = "0.50"; assert(model.currentLimit == 1)
        model.vref = "nan"; assert(model.currentLimit == nil)
        model.vref = "-1"; assert(model.currentLimit == nil)
        model.connected = true; status(model)
        assert(!model.armed && !model.beginHold(direction: 0) && !model.beginHold(direction: 1) && commands.isEmpty)
        model.toggleArmed()
        assert(model.armed && commands.isEmpty) // Arming never sends motion commands.
        assert(!model.beginHold(direction: 2))
        assert(model.beginHold(direction: 0))
        assert(model.held && commands.last!.0 == "CURVE 1 0 200 3 0 0 60 500 3000 1000")
        let duringHold = commands.count
        assert(!model.beginHold(direction: 1) && !model.canHold(direction: 1))
        model.endHold(direction: 1)
        assert(model.held && model.direction == 0 && commands.count == duringHold)
        model.heartbeat(); assert(commands.last!.0 == "HOLD 1")
        // Release before acknowledgement cannot be restarted by a late reply.
        model.endHold(direction: 0)
        assert(!model.held && model.armed && model.pending == nil && commands.last!.0 == "STOP" && commands.last!.1)
        status(model, ack: 1, run: 1, phase: "ARMING")
        let count = commands.count
        model.heartbeat(); assert(commands.count == count && !model.held)
        assert(!model.beginHold(direction: 1)) // Wait for confirmed stop before reversing.
        status(model)
        assert(model.beginHold(direction: 1))
        assert(commands.last!.0 == "CURVE 2 1 200 3 0 0 60 500 3000 1000")
        status(model, ack: 2, run: 1, phase: "RAMPING")
        assert(model.commandedRPM == 150 && model.running)
        model.toggleArmed() // Disarming while running requests priority STOP.
        assert(!model.armed && !model.held && commands.last!.0 == "STOP" && commands.last!.1)
        status(model)
        model.toggleArmed(); assert(model.beginHold(direction: 0))
        status(model, ack: 3, run: 1, phase: "RAMPING")
        status(model, phase: "LINK_LOST")
        assert(!model.held && !model.armed && commands.last!.0 == "STOP")
        let afterStop = commands.count
        model.heartbeat(); assert(commands.count == afterStop)
        model.toggleArmed(); assert(model.beginHold(direction: 0))
        status(model, ack: 4, run: 1, phase: "RAMPING")
        clock.addTimeInterval(0.61); model.heartbeat()
        assert(!model.connected && !model.held && !model.armed && commands.last!.0 == "STOP")
        model.connected = true; status(model)
        model.ramp[1].seconds = 0.01
        let beforeInvalid = commands.count
        model.toggleArmed(); assert(!model.armed && !model.beginHold(direction: 0) && commands.count == beforeInvalid)
        model.ramp = Controller.defaultRamp
        model.toggleArmed()
        model.ramp[1].rpm = .nan
        assert(!model.beginHold(direction: 0) && !model.held && commands.count == beforeInvalid && model.rampError != nil)
        model.ramp = Controller.defaultRamp
        model.rpm = 100; assert(model.effectiveRamp[1].rpm == 100)
        model.rpm = 300
        assert(model.beginHold(direction: 0)); status(model, ack: 5, error: -22)
        assert(!model.held && !model.armed && commands.last!.0 == "STOP")
        model.toggleArmed(); model.disconnect()
        assert(!model.armed)
        model.connected = true; status(model, v: 1)
        assert(!model.connected && !model.held && !model.armed)
        print("PASS: arming interlock, independent directions, no overlap, disarm STOP, release before ack, renewals, loss of link, curves, RPM")
    }
}
