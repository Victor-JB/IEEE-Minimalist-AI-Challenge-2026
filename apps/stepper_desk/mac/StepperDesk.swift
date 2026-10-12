import SwiftUI
import AppKit

private let mint = Color(red: 0.43, green: 0.88, blue: 0.75)
private let canvas = Color(red: 0.055, green: 0.07, blue: 0.085)
private let panel = Color(red: 0.095, green: 0.115, blue: 0.135)
private let muted = Color(red: 0.58, green: 0.65, blue: 0.68)

struct Panel<Content: View>: View {
    @ViewBuilder var content: Content
    var body: some View {
        VStack(alignment: .leading, spacing: 18) { content }
            .padding(24).frame(maxWidth: .infinity, alignment: .leading)
            .background(panel, in: RoundedRectangle(cornerRadius: 18))
            .overlay(RoundedRectangle(cornerRadius: 18).stroke(.white.opacity(0.06)))
    }
}

struct DeskView: View {
    @ObservedObject var model: Controller
    @ObservedObject var sensor: Controller
    @State private var showingRamp = false
    var body: some View {
        ScrollView(.vertical) {
            VStack(alignment: .leading, spacing: 22) {
                header
                connection
                HStack(alignment: .top, spacing: 20) {
                    VStack(spacing: 20) { motion; angleFeedback; footerNote }
                        .frame(maxWidth: .infinity)
                    VStack(spacing: 20) { setup; torque }
                        .frame(width: 310)
                }
                HStack(spacing: 10) {
                    Image(systemName: model.problem ? "exclamationmark.circle.fill" : "info.circle")
                        .foregroundStyle(model.problem ? Color.orange : mint)
                    Text(model.notice).font(.system(size: 12)).foregroundStyle(model.problem ? Color.orange : muted)
                    Spacer()
                }.frame(minHeight: 30)
            }
            .padding(30)
        }
        .frame(minWidth: 930, minHeight: 630)
        .foregroundStyle(Color.white)
        .background(canvas).preferredColorScheme(.dark)
        .tint(mint)
        .onChange(of: model.fullSteps) { model.clampRPM() }
        .onChange(of: model.microsteps) { model.clampRPM() }
        .onReceive(NotificationCenter.default.publisher(for: NSApplication.willTerminateNotification)) { _ in model.shutdown() }
        .onReceive(NotificationCenter.default.publisher(for: NSApplication.didResignActiveNotification)) { _ in model.disarm() }
        .onReceive(NotificationCenter.default.publisher(for: NSWindow.didResignKeyNotification)) { _ in model.disarm() }
        .onReceive(NotificationCenter.default.publisher(for: NSWindow.willCloseNotification)) { _ in model.disarm() }
        .onDisappear { model.disarm() }
        .sheet(isPresented: $showingRamp) { RampEditor(model: model) }
    }

    private var header: some View {
        HStack(spacing: 14) {
            Image(systemName: "dial.medium.fill").font(.system(size: 34)).foregroundStyle(mint)
                .frame(width: 54, height: 54).background(mint.opacity(0.08), in: RoundedRectangle(cornerRadius: 15))
            VStack(alignment: .leading, spacing: 3) {
                Text("Motor \(model.axis + 1) · \(ArmMapping.names[ArmMapping.jointForMotor[model.axis]])").font(.system(size: 28, weight: .semibold, design: .rounded))
                Text("ARDUINO Q  /  DRV8825  /  USB CONTROL").font(.system(size: 10, weight: .medium, design: .monospaced)).tracking(1.3).foregroundStyle(muted)
            }
            Spacer()
            HStack(spacing: 7) {
                Circle().fill(model.connected ? mint : Color.gray).frame(width: 7, height: 7)
                Text(model.connected ? "CONNECTED" : "OFFLINE").font(.system(size: 10, weight: .semibold)).tracking(1)
            }.padding(.horizontal, 13).padding(.vertical, 9)
                .background(.white.opacity(0.045), in: Capsule())
        }
    }

    private var connection: some View {
        HStack(spacing: 14) {
            Image(systemName: "cable.connector").foregroundStyle(muted)
            Text("Arduino Q · USB or Tailscale").font(.system(size: 13, weight: .medium))
            Spacer()
            Text("STEP D\([3,5,9][model.axis]) · DIR D\([2,4,7][model.axis])").font(.system(size: 11, design: .monospaced)).foregroundStyle(muted)
            Button(model.connected || model.connecting ? "Disconnect" : "Connect") {
                if model.connected || model.connecting { model.disconnect() } else { model.connect() }
            }.buttonStyle(.bordered).controlSize(.large).disabled(model.port.isEmpty)
        }.padding(14).background(panel, in: RoundedRectangle(cornerRadius: 12))
    }

    private var motion: some View {
        Panel {
            HStack {
                Text("MOTION").font(.system(size: 11, weight: .semibold)).tracking(1.8).foregroundStyle(muted)
                Spacer()
                Text(model.phase).font(.system(size: 12, weight: .medium)).foregroundStyle(model.running ? mint : muted)
            }
            HStack(alignment: .firstTextBaseline, spacing: 9) {
                Text(model.commandedRPM, format: .number.precision(.fractionLength(1)))
                    .font(.system(size: 66, weight: .light, design: .rounded)).monospacedDigit()
                Text("RPM").font(.system(size: 16, weight: .medium)).foregroundStyle(muted)
                Spacer()
                VStack(alignment: .trailing, spacing: 4) {
                    Text("\(model.telemetry?.hz ?? 0)").font(.system(size: 23, weight: .medium, design: .monospaced))
                    Text("pulses / sec").font(.system(size: 11)).foregroundStyle(muted)
                }
            }
            Text(model.limitNotice).font(.system(size: 11)).foregroundStyle(model.limitReady ? muted : .orange)
            Text("Commanded speed · calculated, not measured").font(.system(size: 11)).foregroundStyle(muted)
                .padding(.top, -14)
            ProgressView(value: min(Double(model.telemetry?.hz ?? 0), Double(model.maxHz)), total: Double(model.maxHz))
                .tint(mint)
            Divider().overlay(.white.opacity(0.04))
            HStack {
                Text("Target speed").font(.system(size: 14, weight: .medium))
                Spacer()
                TextField("RPM", value: $model.rpm, format: .number.precision(.fractionLength(1)))
                    .textFieldStyle(.plain).padding(6).background(.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 5)).frame(width: 94).multilineTextAlignment(.trailing)
                    .accessibilityLabel("Target RPM").disabled(model.held || model.running)
                Text("RPM").font(.system(size: 12)).foregroundStyle(muted)
            }
            Slider(value: $model.rpm, in: model.minRPM...model.maxRPM)
                .accessibilityLabel("Target speed slider").disabled(model.held || model.running)
            HStack {
                Text(model.minRPM, format: .number.precision(.fractionLength(1)))
                Spacer()
                Text("\(Int(model.maxRPM)) RPM ceiling")
            }.font(.system(size: 10, design: .monospaced)).foregroundStyle(muted).padding(.top, -12)
            HStack {
                Text(model.armed ? "ARMED" : "DISARMED")
                    .font(.system(size: 11, weight: .semibold)).tracking(1)
                    .foregroundStyle(model.armed ? Color.orange : muted)
                Spacer()
                Text("Release before changing direction or speed.")
                    .font(.system(size: 11)).foregroundStyle(muted)
            }
            HStack(spacing: 12) {
                HoldToRun(title: "Hold Forward", activeTitle: "Forward · release to stop",
                          enabled: model.canHold(direction: 0), held: model.held && model.direction == 0,
                          begin: { model.beginHold(direction: 0) }, end: { model.endHold(direction: 0) })
                    .frame(maxWidth: .infinity).frame(height: 48)
                HoldToRun(title: "Hold Reverse", activeTitle: "Reverse · release to stop",
                          enabled: model.canHold(direction: 1), held: model.held && model.direction == 1,
                          begin: { model.beginHold(direction: 1) }, end: { model.endHold(direction: 1) })
                    .frame(maxWidth: .infinity).frame(height: 48)
                Button { model.toggleArmed() } label: {
                    Label(model.armed ? "Disarm" : "Arm", systemImage: model.armed ? "lock.open.fill" : "lock.fill")
                        .font(.system(size: 14, weight: .semibold)).frame(width: 109, height: 48)
                        .foregroundStyle(canvas)
                        .background((model.armed ? Color.orange : mint).opacity(model.armed || model.canArm ? 1 : 0.25), in: RoundedRectangle(cornerRadius: 9))
                }.buttonStyle(.plain)
                    .disabled(!model.armed && !model.canArm)
            }.padding(.top, 4)

        }
    }

    private var angleFeedback: some View {
        Panel {
            HStack {
                Text("ANGLE SENSOR").font(.system(size: 11, weight: .semibold)).tracking(1.8).foregroundStyle(muted)
                Spacer()
                Text("AS5600 · CH \(sensor.axis)").font(.system(size: 11, design: .monospaced)).foregroundStyle(muted)
            }
            HStack(alignment: .firstTextBaseline) {
                Text(sensor.measuredAngle.map { String(format: "%.2f°", $0) } ?? "—")
                    .font(.system(size: 44, weight: .light, design: .rounded)).monospacedDigit()
                Spacer()
                Text("Absolute angle · 0–360°").font(.system(size: 11)).foregroundStyle(muted)
            }
            Label(sensor.sensorNotice, systemImage: sensor.sensorHealthy ? "checkmark.circle.fill" : "exclamationmark.circle")
                .font(.system(size: 12)).foregroundStyle(sensor.sensorHealthy ? mint : .orange)
            Text("Live feedback only. The motor still follows your speed and ramp; position correction is not enabled.")
                .font(.system(size: 11)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
        }
    }

    private var setup: some View {
        Panel {
            Text("MOTOR SETUP").font(.system(size: 11, weight: .semibold)).tracking(1.8).foregroundStyle(muted)
            Text("Creality 42–40").font(.system(size: 18, weight: .semibold))
            HStack {
                Text("Full steps / rev").font(.system(size: 12))
                Spacer()
                TextField("Steps", value: $model.fullSteps, format: .number.grouping(.never))
                    .textFieldStyle(.plain).padding(6).background(.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 5)).frame(width: 72).multilineTextAlignment(.trailing)
                    .accessibilityLabel("Full steps per revolution")
            }.disabled(model.held || model.running)
            HStack {
                Text("Microstepping").font(.system(size: 12))
                Spacer()
                Picker("Microstepping", selection: $model.microsteps) {
                    ForEach([1, 2, 4, 8, 16, 32], id: \.self) { n in Text(n == 1 ? "Full step" : "1 / \(n)").tag(n) }
                }.labelsHidden().frame(width: 120).accessibilityLabel("Microstepping")
            }.disabled(model.held || model.running)
            Text("200 steps is a provisional setting. Match microstepping to the driver's MODE pins; this menu only changes the RPM calculation.")
                .font(.system(size: 11)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
            Divider()
            HStack {
                Text("Startup ramp").font(.system(size: 12))
                Spacer()
                Button("Edit curve…") { showingRamp = true }
                    .disabled(model.held || model.running)
            }
            RampPlot(points: model.effectiveRamp).frame(height: 80)
            Text("Default: 150 RPM in 0.06 s, then smoothly to your target. Points are capped at the target speed.")
                .font(.system(size: 11)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)

        }
    }

    private var torque: some View {
        Panel {
            HStack {
                Text("TORQUE / CURRENT").font(.system(size: 11, weight: .semibold)).tracking(1).foregroundStyle(muted)
                Spacer()
                Text("MANUAL").font(.system(size: 9, weight: .bold)).foregroundStyle(Color.orange)
            }
            Text("Set on the driver").font(.system(size: 18, weight: .semibold))
            Text("Each driver’s trimmer sets coil current. Torque adjustment is manual with this wiring.")
                .font(.system(size: 12)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
            HStack {
                Text("Measured VREF").font(.system(size: 12))
                Spacer()
                TextField("e.g. 0.50", text: $model.vref).textFieldStyle(.plain).padding(6).background(.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 5)).frame(width: 76)
                    .accessibilityLabel("Measured VREF")
                Text("V").foregroundStyle(muted)
            }
            if let amps = model.currentLimit {
                Text("\(amps, specifier: "%.2f") A current limit")
                    .font(.system(size: 15, weight: .medium, design: .monospaced)).foregroundStyle(mint)
            } else {
                Text("Enter your meter reading above").font(.system(size: 11)).foregroundStyle(muted)
            }
            Text("Calculator only · R100 resistors\nAll drivers sleep when all motors stop.")
                .font(.system(size: 10)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
        }
    }

    private var footerNote: some View {
        HStack(alignment: .top, spacing: 10) {
            Image(systemName: "button.programmable").foregroundStyle(mint)
            Text("Arm, then hold Forward or Reverse to run. Release or drag away to stop without a ramp. Leaving this window disarms the controls. If hold renewals cease, the board stops within about 0.3 seconds.")
                .font(.system(size: 11)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
        }.padding(.horizontal, 8).frame(maxWidth: .infinity, alignment: .leading)
    }
}

// AppKit retains the original mouse-down view for mouse-up, including releases
// outside the window. Dragging out cancels; re-entering cannot restart motion.
final class HoldButtonView: NSView {
    var enabled = false { didSet { needsDisplay = true } }
    var held = false { didSet { if !held { pressed = false }; needsDisplay = true } }
    var title = "Hold to run" { didSet { setAccessibilityLabel("\(title); release or drag away to stop"); needsDisplay = true } }
    var activeTitle = "Running · release to stop" { didSet { needsDisplay = true } }
    var begin: (() -> Bool)?
    var end: (() -> Void)?
    private var pressed = false
    override init(frame: NSRect) {
        super.init(frame: frame)
        setAccessibilityElement(true)
        setAccessibilityRole(.button)
        setAccessibilityLabel("Hold to run; release or drag away to stop")
        toolTip = "Keep the mouse button pressed to run the motor."
    }
    required init?(coder: NSCoder) { fatalError("init(coder:) has not been implemented") }
    override func mouseDown(with event: NSEvent) {
        guard enabled, !pressed else { return }
        pressed = begin?() ?? false
        needsDisplay = true
    }
    override func mouseDragged(with event: NSEvent) {
        if pressed && !bounds.contains(convert(event.locationInWindow, from: nil)) { release() }
    }
    override func mouseUp(with event: NSEvent) { release() }
    override func viewWillMove(toWindow newWindow: NSWindow?) {
        if newWindow == nil { release() }
        super.viewWillMove(toWindow: newWindow)
    }
    private func release() {
        if pressed { pressed = false; end?(); needsDisplay = true }
    }
    // A single accessibility click has no release signal, so cannot start motion.
    override func accessibilityPerformPress() -> Bool { false }
    override func draw(_ dirtyRect: NSRect) {
        let active = pressed && held
        NSColor(calibratedRed: 0.43, green: 0.88, blue: 0.75, alpha: enabled ? (active ? 0.7 : 1) : 0.25).setFill()
        NSBezierPath(roundedRect: bounds.insetBy(dx: 1, dy: 1), xRadius: 9, yRadius: 9).fill()
        let text = active ? activeTitle : title
        let attributes: [NSAttributedString.Key: Any] = [
            .font: NSFont.systemFont(ofSize: 14, weight: .semibold),
            .foregroundColor: NSColor(calibratedRed: 0.055, green: 0.07, blue: 0.085, alpha: 1)]
        let size = (text as NSString).size(withAttributes: attributes)
        (text as NSString).draw(at: NSPoint(x: (bounds.width-size.width)/2, y: (bounds.height-size.height)/2), withAttributes: attributes)
    }
}
struct HoldToRun: NSViewRepresentable {
    var title: String
    var activeTitle: String
    var enabled: Bool
    var held: Bool
    var begin: () -> Bool
    var end: () -> Void
    func makeNSView(context: Context) -> HoldButtonView { HoldButtonView(frame: .zero) }
    func updateNSView(_ view: HoldButtonView, context: Context) {
        view.title = title; view.activeTitle = activeTitle
        view.begin = begin; view.end = end; view.enabled = enabled; view.held = held
    }
    static func dismantleNSView(_ view: HoldButtonView, coordinator: ()) { view.end?() }
}

struct RampPlot: View {
    let points: [RampPoint]
    private var valid: [RampPoint] { points.filter { $0.seconds.isFinite && $0.rpm.isFinite && $0.seconds >= 0 && $0.rpm >= 0 } }
    var body: some View {
        VStack(spacing: 4) {
            Canvas { context, size in
                let values = valid
                let end = max(values.map(\.seconds).max() ?? 1, 0.02)
                let top = max(values.map(\.rpm).max() ?? 1, 1)
                func position(_ p: RampPoint) -> CGPoint {
                    CGPoint(x: 5 + p.seconds / end * (size.width-10), y: size.height-5-p.rpm/top*(size.height-10))
                }
                var grid = Path()
                for n in 0...3 {
                    let y = 5 + Double(n)/3*(size.height-10)
                    grid.move(to: CGPoint(x: 5, y: y)); grid.addLine(to: CGPoint(x: size.width-5, y: y))
                }
                context.stroke(grid, with: .color(.white.opacity(0.08)), lineWidth: 1)
                var path = Path()
                for (i, p) in values.enumerated() {
                    if i == 0 { path.move(to: position(p)) } else { path.addLine(to: position(p)) }
                }
                context.stroke(path, with: .color(mint), style: StrokeStyle(lineWidth: 2.5, lineCap: .round, lineJoin: .round))
                for p in values {
                    let q = position(p)
                    context.fill(Path(ellipseIn: CGRect(x: q.x-3, y: q.y-3, width: 6, height: 6)), with: .color(mint))
                }
            }
            HStack {
                Text("RPM ↑")
                Spacer()
                Text("Time → \(valid.last?.seconds ?? 0, specifier: "%.2f") s")
            }.font(.system(size: 10, design: .monospaced)).foregroundStyle(muted)
        }
    }
}

struct RampEditor: View {
    @ObservedObject var model: Controller
    @Environment(\.dismiss) private var dismiss
    @State private var original: [RampPoint]
    init(model: Controller) {
        self.model = model
        _original = State(initialValue: model.ramp)
    }
    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            Text("Startup ramp").font(.system(size: 24, weight: .semibold))
            Text("Add points to shape the speed over time. The motor follows straight lines between points, then stays at your target while held.")
                .font(.system(size: 12)).foregroundStyle(muted).fixedSize(horizontal: false, vertical: true)
            RampPlot(points: model.effectiveRamp).frame(height: 150)
            HStack {
                Text("Time (seconds)").frame(width: 150, alignment: .leading)
                Text("Speed (RPM)").frame(width: 150, alignment: .leading)
                Spacer()
            }.font(.system(size: 11, weight: .semibold)).foregroundStyle(muted)
            ForEach($model.ramp) { $point in
                let first = point.id == model.ramp.first?.id
                let last = point.id == model.ramp.last?.id
                HStack(spacing: 16) {
                    TextField("Seconds", value: $point.seconds, format: .number.precision(.fractionLength(2)))
                        .textFieldStyle(.plain).padding(6).background(.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 5)).frame(width: 134).disabled(first)
                    if last {
                        Text("\(model.rpm, specifier: "%.1f") · target").frame(width: 134, alignment: .leading)
                    } else {
                        TextField("RPM", value: $point.rpm, format: .number.precision(.fractionLength(1)))
                            .textFieldStyle(.plain).padding(6).background(.white.opacity(0.08), in: RoundedRectangle(cornerRadius: 5)).frame(width: 134).disabled(first)
                    }
                    Spacer()
                    if !first && !last {
                        Button { model.ramp.removeAll { $0.id == point.id } } label: { Image(systemName: "minus.circle") }
                            .buttonStyle(.plain).help("Remove this point")
                    } else { Text(first ? "Start" : "Finish").font(.system(size: 11)).foregroundStyle(muted) }
                }
            }
            HStack {
                Button("Add point") { addPoint() }.disabled(model.ramp.count >= 8 || !canAdd)
                Text("\(model.ramp.count) / 8 points").font(.system(size: 11)).foregroundStyle(muted)
                Spacer()
                Button("Restore default") { model.ramp = Controller.defaultRamp }
            }
            Text(model.rampError ?? "Interior speeds are capped at the target. Times must increase by at least 0.02 s; maximum duration is 60 s.")
                .font(.system(size: 11)).foregroundStyle(model.rampError == nil ? muted : .orange).fixedSize(horizontal: false, vertical: true)
            HStack {
                Button("Cancel") { model.ramp = original; dismiss() }.keyboardShortcut(.cancelAction)
                Spacer()
                Button("Save curve") { model.saveRamp(); dismiss() }
                    .buttonStyle(.borderedProminent).disabled(model.rampError != nil).keyboardShortcut(.defaultAction)
            }
        }.padding(28).frame(width: 530).foregroundStyle(Color.white).background(canvas).preferredColorScheme(.dark).tint(mint)
    }
    private var canAdd: Bool {
        guard model.ramp.count >= 2 else { return false }
        let a = model.ramp[model.ramp.count-2].seconds, b = model.ramp.last!.seconds
        return a.isFinite && b.isFinite && b-a >= 0.04
    }
    private func addPoint() {
        guard canAdd else { return }
        let values = model.effectiveRamp
        let a = values[values.count-2], b = values.last!
        model.ramp.insert(RampPoint(seconds: ((a.seconds+b.seconds)/2*1000).rounded()/1000, rpm: (a.rpm+b.rpm)/2), at: model.ramp.count-1)
    }
}

final class AppDelegate: NSObject, NSApplicationDelegate {
    func applicationShouldTerminateAfterLastWindowClosed(_ sender: NSApplication) -> Bool { true }
}

final class DeskModel: ObservableObject {
    let axes: [Controller]
    let arm: ArmSettings
    let poses: PoseController
    init(preview: Bool = false) {
        axes = (0..<3).map { Controller(axis: $0, preview: preview) }
        arm = ArmSettings(defaults: preview ? nil : .standard)
        poses = PoseController(settings: arm)
        for motor in axes {
            let joint=ArmMapping.jointForMotor[motor.axis]
            motor.limitConfiguration = { [weak arm] in
                guard let arm else { return nil }
                return LimitConfiguration(arm.joints[joint])
            }
        }
    }
    func disarmAll() { axes.forEach { $0.disarm() }; poses.stop() }
    func shutdown() { axes.forEach { $0.shutdown() }; poses.shutdown() }
}
struct AxisSummary: View {
    @ObservedObject var model: Controller
    @ObservedObject var sensor: Controller
    let selected: Bool
    let select: () -> Void
    var body: some View {
        Button(action: select) {
            HStack(spacing: 12) {
                Image(systemName: "gearshape.2.fill").foregroundStyle(selected ? mint : muted)
                VStack(alignment: .leading, spacing: 5) {
                    Text("\(ArmMapping.names[ArmMapping.jointForMotor[model.axis]]) · M\(model.axis+1)").font(.system(size: 15, weight: .semibold))
                    Text(sensor.measuredAngle.map { String(format: "Sensor %.1f° · ", $0) } .map { $0 + model.phase } ?? model.phase).font(.system(size: 11)).foregroundStyle(muted)
                }
                Spacer()
                Text("\(model.commandedRPM, specifier: "%.0f") RPM")
                    .font(.system(size: 16, design: .rounded)).monospacedDigit()
            }.padding(17).background(selected ? mint.opacity(0.09) : panel, in: RoundedRectangle(cornerRadius: 12))
                .overlay(RoundedRectangle(cornerRadius: 12).stroke(selected ? mint.opacity(0.55) : .clear))
        }.buttonStyle(.plain)
    }
}
struct DriverPowerView: View {
    @ObservedObject var model: Controller
    let disarmAll: () -> Void
    var body: some View {
        HStack(spacing: 12) {
            Text(!model.powerSupported ? "Connect selected joint · automatic sleep firmware required" :
                 (!model.powerAllowsMotion ? "Asleep and locked" : (model.driversAwake ? "Drivers awake · automatic sleep when stopped" : "Drivers asleep · automatic wake on movement")))
                .font(.system(size: 12)).foregroundStyle(muted)
            Spacer()
            Button("Enable movement") { disarmAll(); model.setMotionEnabled(true) }
                .disabled(!model.powerSupported || model.powerAllowsMotion)
            Button("Sleep & lock") { disarmAll(); model.setMotionEnabled(false) }
                .disabled(!model.powerSupported || !model.powerAllowsMotion)
        }.padding(.horizontal, 30).padding(.top, 12)
    }
}
struct QDashboard: View {
    @ObservedObject var desk: DeskModel
    @State private var selected = 1
    @State private var workspace = 0
    var body: some View {
        VStack(spacing: 0) {
            HStack {
                Text("STEPPER DESK Q").font(.system(size: 11, weight: .semibold)).tracking(2)
                Spacer()
                Text("Fixed base · three joints").font(.system(size: 11)).foregroundStyle(muted)
                Button("Connect all") { desk.axes.filter { !$0.connected && !$0.connecting }.forEach { $0.connect() } }.disabled(workspace == 2)
                Button("Disarm all") { desk.disarmAll() }
            }.padding(.horizontal, 30).padding(.top, 22)
            if workspace != 2 {
            HStack(spacing: 12) {
                ForEach(ArmMapping.motorForJoint, id: \.self) { axis in
                    AxisSummary(model: desk.axes[axis], sensor: desk.axes[ArmMapping.jointForMotor[axis]], selected: selected == axis) {
                        desk.disarmAll(); selected = axis
                    }
                }
            }.padding(.horizontal, 30).padding(.top, 18)
            DriverPowerView(model: desk.axes[selected], disarmAll: desk.disarmAll)
            }
            Picker("Workspace", selection: $workspace) {
                Text("Live arm").tag(0)
                Text("Motor settings & ramps").tag(1)
                Text("Poses & recordings").tag(2)
            }.pickerStyle(.segmented).frame(width: 520, height: 28).padding(.top, 18)
                .onChange(of: workspace) {
                    desk.disarmAll()
                    if workspace == 2 { desk.axes.forEach { $0.disconnect() } }
                    else { desk.poses.disconnect() }
                }
            if workspace == 2 {
                PoseWorkspace(model: desk.poses)
            } else if workspace == 0 {
                ArmView(shoulder: desk.axes[0], elbow: desk.axes[1], head: desk.axes[2], motor: desk.axes[selected], settings: desk.arm, disarmAll: desk.disarmAll)
            } else {
                DeskView(model: desk.axes[selected], sensor: desk.axes[ArmMapping.jointForMotor[selected]]).id(selected)
            }
        }.frame(minWidth: 1000, minHeight: 800).background(canvas).foregroundStyle(.white).preferredColorScheme(.dark).tint(mint)
            .onReceive(NotificationCenter.default.publisher(for: NSApplication.willTerminateNotification)) { _ in desk.shutdown() }
            .onReceive(NotificationCenter.default.publisher(for: NSApplication.didResignActiveNotification)) { _ in desk.disarmAll() }
            .onReceive(NotificationCenter.default.publisher(for: NSWindow.didResignKeyNotification)) { _ in desk.disarmAll() }
            .onReceive(NotificationCenter.default.publisher(for: NSWindow.willCloseNotification)) { _ in desk.disarmAll() }
    }
}
#if !STEPPER_UI_TEST
@main
#endif
struct StepperDeskApp: App {
    @NSApplicationDelegateAdaptor(AppDelegate.self) var delegate
    @StateObject private var desk = DeskModel()
    init() { signal(SIGPIPE, SIG_IGN) }
    var body: some Scene {
        Window("Stepper Desk Q", id: "main") { QDashboard(desk: desk) }
            .windowStyle(.hiddenTitleBar).defaultSize(width: 1120, height: 1000)
            .commands {
                CommandGroup(replacing: .newItem) {}
                CommandMenu("Motors") {
                    Button("Disarm all") { desk.disarmAll() }.keyboardShortcut(".", modifiers: [.command])
                    Button("Disarm all (Escape)") { desk.disarmAll() }.keyboardShortcut(.escape, modifiers: [])
                }
            }
    }
}
