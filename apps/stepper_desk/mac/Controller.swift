import Foundation
import Combine

struct BoardStatus: Decodable {
    let v: Int
    let ack: UInt32
    let error: Int
    let fault: Int
    let hz: Int
    let target: Int
    let dir: Int
    let want_dir: Int
    let run: Int
    let ppr: Int
    let phase: String
    let max_hz: Int
    let angle_raw: Int?
    let magnet_status: Int?
    let sensor_error: Int?
    let sensor_age_ms: Int?
    let limit_state: Int?
    let limit_configured: Int?
    let limit_zero_udeg: Int?
    let limit_reversed: Int?
    let limit_forward_sign: Int?
    let joint_angle_mdeg: Int?
    let limit_min_mdeg: Int?
    let limit_max_mdeg: Int?
    let limit_revision: Int?
}

struct RampPoint: Identifiable, Codable, Equatable {
    var id = UUID()
    var seconds: Double
    var rpm: Double
}

final class Controller: ObservableObject {
    @Published var ports = SSHLink.ports()
    @Published var port = ""
    @Published var connected = false
    @Published var connecting = false
    @Published var telemetry: BoardStatus?
    @Published var rpm = 300.0
    @Published private(set) var direction = 0
    @Published private(set) var armed = false
    @Published var fullSteps = 200
    @Published var microsteps = 1
    @Published var ramp = Controller.defaultRamp
    @Published private(set) var held = false
    @Published var vref = ""
    @Published var notice = "Connect Arduino Q over USB to begin."
    @Published var problem = false
    @Published var pending: UInt32?
    @Published var stopping = false
    private let link: SSHLink
    let axis: Int
    private var keyPrefix: String { "q.axis\(axis)." }
    private var timer: Timer?
    private var connectedAt = Date.distantPast
    private var lastStatus = Date.distantPast
    private var pendingAt = Date.distantPast
    private var sequence: UInt32 = 0
    private var session: UInt32?
    private var accepted = false
    private let preview: Bool
    // Injectable transport and clock for hold/release and failure-path tests.
    var onCommand: ((String, Bool) -> Void)?
    var now: () -> Date = Date.init
    var limitConfiguration: (() -> LimitConfiguration?)?
    private var lastLimitSync = Date.distantPast
    var limitReady: Bool {
        guard let provider=limitConfiguration else { return true } // Isolated controller tests/diagnostics.
        guard let config=provider(), config.matches(telemetry,motor:axis),
              now().timeIntervalSince(lastStatus)<=0.6, let state=telemetry?.limit_state else { return false }
        return state == 0 || state == 3 || state == 4
    }
    var limitNotice: String {
        guard connected else { return "Connect to synchronize joint limits." }
        guard let config=limitConfiguration?() else { return "Calibrate this joint before running; limit guard is locked." }
        guard telemetry?.limit_revision == 1 else { return "Joint-limit firmware update required; motion locked." }
        guard config.matches(telemetry,motor:axis) else { return "Synchronizing calibrated limits with the board…" }
        switch telemetry?.limit_state {
        case 0: return "Board limit guard active · sensor monitored."
        case 2: return "Joint sensor unavailable or stale · motion locked."
        case 3: return "Lower limit · hold \(config.forwardSign > 0 ? "Forward" : "Reverse") to move back into range."
        case 4: return "Upper limit · hold \(config.forwardSign < 0 ? "Forward" : "Reverse") to move back into range."
        default: return "Limit guard not ready · motion locked."
        }
    }
    private func synchronizeLimits() {
        guard connected, !held, !running, !armed, !stopping, let config=limitConfiguration?(),
              telemetry?.limit_revision == 1, !config.matches(telemetry,motor:axis),
              now().timeIntervalSince(lastLimitSync)>=0.5 else { return }
        lastLimitSync=now(); send(config.command)
    }

    static var defaultRamp: [RampPoint] {
        [RampPoint(seconds: 0, rpm: 0), RampPoint(seconds: 0.06, rpm: 150),
         RampPoint(seconds: 3, rpm: 300)]
    }
    var ppr: Int { max(1, min(fullSteps, 10000)) * microsteps }
    var maxHz: Int { telemetry?.max_hz ?? 2400 }
    var maxRPM: Double { Double(maxHz) * 60 / Double(ppr) }
    var minRPM: Double { 20 * 60 / Double(ppr) }
    var requestedHz: Int { rpm.isFinite ? Int((min(max(rpm, 0), maxRPM) * Double(ppr) / 60).rounded()) : 0 }
    var running: Bool { telemetry?.run == 1 }
    var canArm: Bool {
        connected && !held && !running && pending == nil && !stopping &&
        (telemetry?.fault ?? 0) == 0 && rampError == nil && limitReady
    }
    func canHold(direction requested: Int) -> Bool {
        guard armed, connected, !stopping, (telemetry?.fault ?? 0) == 0,
              requested == 0 || requested == 1 else { return false }
        if limitConfiguration != nil {
            guard limitReady, let config=limitConfiguration?(), config.permits(state:telemetry?.limit_state,direction:requested) else { return false }
        }
        return held ? direction == requested : (!running && pending == nil)
    }
    // Interior points are capped at the chosen target; the final point follows it.
    var effectiveRamp: [RampPoint] {
        ramp.enumerated().map { i, point in
            var p = point
            if i == ramp.count - 1 { p.rpm = rpm }
            else { p.rpm = min(p.rpm, rpm) }
            return p
        }
    }
    var rampError: String? {
        guard (1...10000).contains(fullSteps), [1,2,4,8,16,32].contains(microsteps), ppr <= 100000,
              rpm.isFinite, rpm >= minRPM, rpm <= maxRPM else { return "Enter valid motor settings and a target within the speed range." }
        guard (2...8).contains(ramp.count), ramp[0].seconds == 0, ramp[0].rpm == 0 else { return "Use 2–8 points, starting at 0 seconds and 0 RPM." }
        var previous = -20
        for (index, p) in effectiveRamp.enumerated() {
            guard p.seconds.isFinite, p.seconds >= 0, p.seconds <= 60,
                  p.rpm.isFinite, p.rpm >= 0, p.rpm <= maxRPM,
                  p.rpm == 0 || p.rpm >= minRPM else { return "Keep points within 0–60 seconds and the allowed speed range." }
            let ms = Int((p.seconds * 1000).rounded())
            guard index == 0 || ms - previous >= 20 else { return "Point times must increase by at least 0.02 seconds." }
            previous = ms
        }
        return nil
    }
    var sensorFresh: Bool {
        connected && now().timeIntervalSince(lastStatus) <= 0.6 &&
        (telemetry?.sensor_age_ms ?? Int.max) <= 500
    }
    var sensorHealthy: Bool {
        sensorFresh && telemetry?.sensor_error == 0 && ((telemetry?.magnet_status ?? 0) & 0x28) == 0x20 &&
        (0...4095).contains(telemetry?.angle_raw ?? -1)
    }
    var measuredAngle: Double? {
        guard sensorHealthy, let raw = telemetry?.angle_raw else { return nil }
        return Double(raw) * 360 / 4096
    }
    var sensorNotice: String {
        guard connected else { return "Connect to read sensor" }
        guard let error = telemetry?.sensor_error else { return "Sensor firmware update needed" }
        if error == 1 { return "Waiting for sensor" }
        if !sensorFresh { return "Sensor reading stale" }
        switch error {
        case 2: return "Multiplexer not responding (0x70)"
        case 3: return "AS5600 not responding on channel \(axis)"
        case 4: return "I²C bus unavailable"
        case 0: break
        default: return "Sensor read error \(error)"
        }
        let bits = telemetry?.magnet_status ?? 0
        if bits & 0x20 == 0 { return "No magnet detected" }
        if bits & 0x08 != 0 { return "Magnet too strong — increase gap" }
        if bits & 0x10 != 0 { return "Live angle · low magnetic field" }
        return "Magnet detected · live angle"
    }
    var commandedRPM: Double {
        guard let s = telemetry, s.ppr > 0 else { return 0 }
        return Double(s.hz) * 60 / Double(s.ppr)
    }
    var phase: String {
        guard connected else { return connecting ? "Connecting" : "Disconnected" }
        switch telemetry?.phase {
        case "RUNNING": return "Running"
        case "ARMING": return "Starting"
        case "RAMPING": return "Ramping"
        case "LIMIT_STOP": return "Joint limit · stopped"
        case "LINK_LOST": return "Hold expired · stopped"
        case "ERROR", "RX_ERROR": return "Stopped · device error"
        default: return "Stopped"
        }
    }
    var currentLimit: Double? {
        guard let v = Double(vref), v.isFinite, v >= 0, v <= 3.5 else { return nil }
        return v * 2
    }

    init(axis: Int = 0, preview: Bool = false) {
        self.preview = preview
        self.axis = axis
        self.link = SSHLink(axis: axis)
        microsteps = ArmMapping.microsteps[axis]
        if preview { ports = ["Arduino Q · USB"]; port = ports[0]; clampRPM(); return }
        let defaults = UserDefaults.standard
        ArmMapping.installHardwareDefaults(defaults)
        let savedSteps = defaults.integer(forKey: keyPrefix + "fullSteps")
        if (1...10000).contains(savedSteps) { fullSteps = savedSteps }
        let savedMicro = defaults.integer(forKey: keyPrefix + "microsteps")
        if [1, 2, 4, 8, 16, 32].contains(savedMicro) { microsteps = savedMicro }
        if let data = defaults.data(forKey: keyPrefix + "rampPoints"),
           let saved = try? JSONDecoder().decode([RampPoint].self, from: data), (2...8).contains(saved.count) { ramp = saved }
        port = ports.contains(defaults.string(forKey: keyPrefix + "port") ?? "") ? defaults.string(forKey: keyPrefix + "port")! : (ports.first ?? "")
        clampRPM()
        link.onLine = { [weak self] in self?.receive($0) }
        link.onError = { [weak self] message in
            guard let self else { return }
            self.armed = false; self.held = false; self.session = nil; self.accepted = false
            self.connected = false; self.connecting = false
            self.pending = nil; self.stopping = false
            self.problem = true; self.notice = message
        }
        let heartbeat = Timer(timeInterval: 0.1, repeats: true) { [weak self] _ in self?.heartbeat() }
        RunLoop.main.add(heartbeat, forMode: .common)
        timer = heartbeat
    }
    private func send(_ command: String, priority: Bool = false) {
        if let onCommand { onCommand(command, priority) }
        if !preview { link.send(command, priority: priority) }
    }
    func refreshPorts() {
        ports = SSHLink.ports()
        if !ports.contains(port) { port = ports.first ?? "" }
    }
    func clampRPM() {
        rpm = min(max(rpm.isFinite ? rpm : minRPM, minRPM), maxRPM)
        if !preview {
            UserDefaults.standard.set(fullSteps, forKey: keyPrefix + "fullSteps")
            UserDefaults.standard.set(microsteps, forKey: keyPrefix + "microsteps")
        }
    }
    func saveRamp() {
        if !preview, let data = try? JSONEncoder().encode(ramp) { UserDefaults.standard.set(data, forKey: keyPrefix + "rampPoints") }
    }
    func connect() {
        guard !preview, !port.isEmpty else { return }
        armed = false; held = false; session = nil; accepted = false
        connected = false; connecting = true; problem = false
        telemetry = nil; pending = nil; stopping = false
        lastLimitSync = .distantPast
        connectedAt = now(); notice = "Waiting for Stepper Desk firmware…"
        UserDefaults.standard.set(port, forKey: keyPrefix + "port")
        link.connect(port)
    }
    func disconnect() {
        armed = false; held = false; session = nil; accepted = false
        if !preview { link.close() }
        connected = false; connecting = false; pending = nil; stopping = false
        notice = "Disconnected. STEP pulses stopped; the motor can still hold torque."
        problem = false
    }
    func shutdown() { armed = false; held = false; session = nil; if !preview { link.close() } }

    func toggleArmed() {
        if armed { disarm() }
        else if canArm {
            armed = true; problem = false
            notice = "Armed. Hold Forward or Hold Reverse to run."
        }
    }
    func disarm() {
        armed = false
        if held || running || pending != nil { stop() }
        else { notice = "Disarmed. Press Arm before holding a direction button."; problem = false }
    }
    @discardableResult
    func beginHold(direction requested: Int) -> Bool {
        guard canHold(direction: requested), !held else { return false }
        if let error = rampError { notice = error; problem = true; return false }
        direction = requested
        sequence = sequence == UInt32.max ? 1 : sequence + 1
        session = sequence; held = true; accepted = false
        pending = sequence; pendingAt = now(); problem = false
        notice = "Hold \(direction == 0 ? "Forward" : "Reverse") to run. Release to stop without a ramp."
        let values = effectiveRamp.map {
            "\(Int(($0.seconds * 1000).rounded())) \(Int(($0.rpm * Double(ppr) / 60).rounded()))"
        }.joined(separator: " ")
        send("CURVE \(sequence) \(direction) \(ppr) \(ramp.count) \(values)")
        return true
    }
    func endHold(direction requested: Int? = nil) {
        if held && (requested == nil || requested == direction) { stop() }
    }
    func stop() {
        held = false; session = nil; accepted = false; pending = nil
        guard connected || connecting else { return }
        stopping = true
        send("STOP", priority: true)
        notice = "Stopping STEP pulses…"; problem = false
    }
    func receive(_ line: String) {
        guard connected || connecting, let prefix = line.range(of: "@STP "),
              let data = String(line[prefix.upperBound...]).data(using: .utf8),
              let state = try? JSONDecoder().decode(BoardStatus.self, from: data) else { return }
        guard state.v == 2 else {
            disconnect(); notice = "Update the board firmware for hold-to-run control."; problem = true; return
        }
        let first = !connected
        telemetry = state; connected = true; connecting = false; lastStatus = now()
        if first { armed = false; notice = "Connected. Press Arm, then hold a direction button."; problem = false; clampRPM() }
        if let wanted = pending, state.ack == wanted {
            pending = nil
            if state.error != 0 {
                disarm(); notice = "Device rejected the ramp (\(state.error))."; problem = true
            } else if held { accepted = true }
        }
        // A stop, lease expiry, or device failure requires a fresh press.
        if held && accepted && state.run == 0 { disarm() }
        if stopping && state.run == 0 {
            stopping = false; notice = armed ? "Armed. Hold either direction to run again." : "Disarmed. Press Arm to enable direction controls."; problem = false
        }
        if state.fault != 0 {
            disarm()
            notice = "Device error \(state.fault). Reset the board before restarting."; problem = true
        }
        if state.phase == "LIMIT_STOP" {
            armed = false; held = false; session = nil; accepted = false
            notice = limitNotice; problem = true
        }
        synchronizeLimits()
        if state.phase == "LINK_LOST" {
            armed = false
            notice = "Hold expired. Release, then arm again to restart."; problem = true
        }
    }
    func heartbeat() {
        if connecting && now().timeIntervalSince(connectedAt) > 7 {
            if !preview { link.close() }; connecting = false; problem = true
            notice = "No controller response. Check USB and the Stepper Desk Q firmware."
        }
        guard connected else { return }
        if now().timeIntervalSince(lastStatus) > 0.6 {
            disarm(); if !preview { link.close() }
            connected = false; pending = nil; stopping = false; problem = true
            notice = "Connection lost. The board stops after 0.3 seconds without hold renewals."
            return
        }
        if pending != nil && now().timeIntervalSince(pendingAt) > 0.5 {
            disarm(); notice = "Ramp was not acknowledged; stop requested."; problem = true
            return
        }
        if limitConfiguration != nil, armed && !limitReady { disarm(); notice=limitNotice; problem=true }
        synchronizeLimits()
        if armed && held, let session { send("HOLD \(session)") }
    }
}
