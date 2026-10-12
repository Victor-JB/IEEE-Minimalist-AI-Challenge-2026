import SwiftUI
import AppKit

private let poseMint = Color(red: 0.43, green: 0.88, blue: 0.75)
private let poseMuted = Color(red: 0.60, green: 0.66, blue: 0.70)

struct PoseWorkspace: View {
    @ObservedObject var model: PoseController
    @State private var action = ""
    @State private var side = false
    @State private var plotJoint = 0

    private var busy: Bool { model.held || model.sample?.run == 1 }
    private var editingDisabled: Bool { busy || model.replaying }
    private var shownSample: ArmSample? {
        if model.replaying, !model.replay.isEmpty {
            return model.replay[min(model.replay.count - 1, max(0, Int(model.replayIndex)))]
        }
        return model.sample
    }
    private var plottedSamples: [ArmSample] {
        let source = model.replaying ? model.replay : model.history
        guard source.count > 1000 else { return source }
        // Keep both endpoints and bound per-render work when opening long recordings.
        return (0..<1000).map { source[$0 * (source.count - 1) / 999] }
    }

    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                header
                connection
                Text(model.notice).font(.system(size: 13)).foregroundStyle(poseMint)
                    .frame(maxWidth: .infinity, alignment: .leading).padding(14)
                    .background(poseMint.opacity(0.06), in: RoundedRectangle(cornerRadius: 10))
                    .accessibilityLabel("Arm status: \(model.notice)")
                HStack(alignment: .top, spacing: 20) {
                    VStack(spacing: 20) { preview; feedback; recordings }.frame(maxWidth: .infinity)
                    VStack(spacing: 20) { ratioVerification; poseTargets }.frame(width: 410)
                }
            }.padding(26)
        }
        .frame(minWidth: 1000, minHeight: 680)
        .foregroundStyle(.white).tint(poseMint).preferredColorScheme(.dark)
        .background(Color(red: 0.055, green: 0.07, blue: 0.085))
        .onReceive(NotificationCenter.default.publisher(for: NSApplication.didResignActiveNotification)) { _ in model.stop() }
        .onReceive(NotificationCenter.default.publisher(for: NSWindow.didResignKeyNotification)) { _ in model.stop() }
        .onReceive(NotificationCenter.default.publisher(for: NSWindow.willCloseNotification)) { _ in model.stop() }
        .onDisappear { model.stop() }
        .onExitCommand { model.stop() }
    }

    private var header: some View {
        HStack(spacing: 14) {
            Image(systemName: "point.3.connected.trianglepath.dotted")
                .font(.system(size: 30)).foregroundStyle(poseMint)
                .frame(width: 54, height: 54).background(poseMint.opacity(0.08), in: RoundedRectangle(cornerRadius: 15))
            VStack(alignment: .leading, spacing: 4) {
                Text("Poses & recordings").font(.system(size: 28, weight: .semibold, design: .rounded))
                Text("Measure the joints. Teach a pose. See every move.")
                    .font(.system(size: 12)).foregroundStyle(poseMuted)
            }
            Spacer()
            Label(model.replaying ? "VISUAL REPLAY" : "LIVE ARM", systemImage: model.replaying ? "play.rectangle" : "waveform.path.ecg")
                .font(.system(size: 10, weight: .semibold)).tracking(0.8)
                .padding(10).background(.white.opacity(0.05), in: Capsule())
        }
    }

    private var connection: some View {
        HStack(spacing: 14) {
            Circle().fill(model.connected ? poseMint : .gray).frame(width: 7, height: 7)
            VStack(alignment: .leading, spacing: 3) {
                Text(model.connecting ? "Connecting…" : model.connected ? "Arduino Q · whole arm connected" : "Arduino Q · USB or Tailscale")
                    .font(.system(size: 13, weight: .medium))
                Text(model.sample?.enabled == 0 ? "Movement locked · drivers asleep" : model.sample?.awake == 1 ? "Drivers awake" : "Drivers sleep between moves")
                    .font(.system(size: 11)).foregroundStyle(poseMuted)
            }
            Spacer()
            Button(model.sample?.enabled == 0 ? "Enable movement" : "Sleep & lock") {
                model.power(model.sample?.enabled == 0)
            }.disabled(!model.connected || model.replaying)
            Button(model.armed ? "Disarm" : "Arm") { model.toggleArm() }
                .tint(model.armed ? .orange : poseMint)
                .disabled(!model.armed && !model.canArm)
            Button(model.connected || model.connecting ? "Disconnect" : "Connect") {
                if model.connected || model.connecting { model.disconnect() } else { model.connect() }
            }.disabled(model.replaying && !model.connected)
        }.buttonStyle(.bordered).controlSize(.large)
            .padding(16).background(.white.opacity(0.04), in: RoundedRectangle(cornerRadius: 12))
    }

    private var preview: some View {
        Panel {
            HStack {
                sectionTitle(model.replaying ? "RECORDED ARM" : "MEASURED ARM")
                Spacer()
                Toggle("Side view", isOn: $side).toggleStyle(.switch).controlSize(.small)
            }
            ArmCADView(angles: model.displayedAngles, side: side)
                .frame(height: 330).clipShape(RoundedRectangle(cornerRadius: 12))
            HStack {
                ForEach(0..<3, id: \.self) { j in
                    VStack(alignment: .leading, spacing: 4) {
                        Text(ArmMapping.names[j]).font(.system(size: 11)).foregroundStyle(poseMuted)
                        Text(angleText(model.displayedAngles[j])).font(.system(size: 22, weight: .medium, design: .rounded)).monospacedDigit()
                    }.frame(maxWidth: .infinity, alignment: .leading)
                }
            }
            Text("Drag to orbit. The model shows measured angles; dragging it never moves the arm.")
                .font(.system(size: 11)).foregroundStyle(poseMuted)
        }
    }

    private var ratioVerification: some View {
        Panel {
            HStack {
                sectionTitle("1 · VERIFY JOINT MOTION")
                Spacer()
                Text("\(model.ratios.filter { $0 > 0 }.count) / 3")
                    .font(.system(size: 12, weight: .medium, design: .monospaced)).foregroundStyle(poseMint)
            }
            Button("Import verified measurements…") { model.chooseRatioImport() }.disabled(model.connected || model.connecting)
            Picker("Joint", selection: $model.joint) {
                ForEach(0..<3, id: \.self) { Text(ArmMapping.names[$0]).tag($0) }
            }.pickerStyle(.segmented).disabled(editingDisabled)
            Text("We measure STEP pulses per joint degree directly, so the gearbox ratio and motor step angle can remain unknown.")
                .font(.system(size: 12)).foregroundStyle(poseMuted).fixedSize(horizontal: false, vertical: true)
            VStack(alignment: .leading, spacing: 8) {
                instruction("1", "Arm, then hold a direction for 5–10° of travel.")
                instruction("2", "Release, let the reading settle, then save the trial.")
                instruction("3", "Repeat at least three times, including both directions. Measurements must agree within 5%.")
            }
            HStack(spacing: 10) {
                jogButton(direction: 0, title: "Hold Forward")
                jogButton(direction: 1, title: "Hold Reverse")
            }.frame(height: 46)
            Text("Slow measurement · 100 pulses/s · release to stop")
                .font(.system(size: 10)).foregroundStyle(poseMuted)
            if let sample = model.sample, sample.axes.count == 3 {
                let axis = sample.axes[ArmMapping.motorForJoint[model.joint]]
                HStack {
                    Text(axis.usable ? String(format: "Current %+.2f°", Double(axis.angle) / 1000) : "Waiting for fresh angle")
                    Spacer()
                    Text("\(axis.count) issued pulses")
                }.font(.system(size: 10, design: .monospaced)).foregroundStyle(poseMuted)
            }
            HStack {
                Button("Save trial") { model.saveTrial() }.disabled(!model.canSaveTrial || model.replaying)
                Spacer()
                Button("Clear trials") { model.resetRatio() }.disabled(!model.ready || editingDisabled || model.sample?.run != 0 || model.trials[model.joint].isEmpty)
            }.buttonStyle(.bordered)
            if model.trials[model.joint].isEmpty {
                Text("No measurements saved for this joint.").font(.system(size: 12)).foregroundStyle(poseMuted)
            } else {
                VStack(spacing: 8) {
                    ForEach(Array(model.trials[model.joint].enumerated()), id: \.element.id) { index, trial in
                        HStack {
                            Text("\(index + 1). \(trial.direction == 0 ? "Forward" : "Reverse")")
                            Spacer()
                            Text("\(trial.pulses) pulses · \(String(format: "%+.2f°", trial.degrees))")
                            Text(String(format: "%.2f", trial.ppd)).frame(width: 52, alignment: .trailing)
                        }.font(.system(size: 10, design: .monospaced))
                    }
                }
            }
            Divider()
            if model.ratios[model.joint] > 0 {
                Label(String(format: "Verified · %.3f pulses/°", model.ratios[model.joint]), systemImage: "checkmark.seal.fill")
                    .font(.system(size: 13, weight: .medium)).foregroundStyle(poseMint)
            } else if let candidate = model.candidate {
                Text(String(format: "Measurements agree · %.3f pulses/°", candidate)).font(.system(size: 12)).foregroundStyle(poseMint)
            } else {
                Text("More consistent measurements needed.").font(.system(size: 12)).foregroundStyle(poseMuted)
            }
            Text("After changing the gearing or microstep wiring, clear and reverify the affected joint before moving to poses.")
                .font(.system(size: 10)).foregroundStyle(poseMuted).fixedSize(horizontal: false, vertical: true)
            Button("Accept verified measurement") { model.acceptRatio() }
                .buttonStyle(.borderedProminent)
                .disabled(!model.ready || editingDisabled || model.sample?.run != 0 || model.candidate == nil)
        }
    }

    private func jogButton(direction: Int, title: String) -> some View {
        let key = "jog\(direction)"
        return HoldToRun(title: title, activeTitle: "Release to stop",
                         enabled: model.armed && model.ready && (!model.held || action == key),
                         held: model.held && action == key,
                         begin: {
                             let started = model.beginJog(direction)
                             if started { action = key }
                             return started
                         }, end: { if action == key { model.releaseHold() } })
            .frame(maxWidth: .infinity)
    }

    private var poseTargets: some View {
        Panel {
            sectionTitle("2 · TEACH & MOVE")
            Text(model.allVerified ? "All three joints verified." : "Verify all three joints before moving to a pose.")
                .font(.system(size: 12)).foregroundStyle(model.allVerified ? poseMint : poseMuted)
            HStack {
                Text("Target angles").font(.system(size: 13, weight: .medium))
                Spacer()
                Button("Use current") { model.useCurrent() }.disabled(editingDisabled || model.sample?.healthy != true)
            }
            ForEach(0..<3, id: \.self) { j in
                HStack {
                    VStack(alignment: .leading, spacing: 3) {
                        Text(ArmMapping.names[j]).font(.system(size: 12))
                        Text("\(Int(ArmMapping.limits[j].lowerBound))° to +\(Int(ArmMapping.limits[j].upperBound))°")
                            .font(.system(size: 10)).foregroundStyle(poseMuted)
                    }
                    Spacer()
                    TextField("Degrees", value: $model.targets[j], format: .number.precision(.fractionLength(2)))
                        .textFieldStyle(.roundedBorder).multilineTextAlignment(.trailing).frame(width: 80)
                        .accessibilityLabel("\(ArmMapping.names[j]) target angle")
                    Text("°").foregroundStyle(poseMuted)
                    Stepper("\(ArmMapping.names[j]) target", value: $model.targets[j], in: ArmMapping.limits[j], step: 0.5).labelsHidden()
                }.disabled(editingDisabled)
            }
            Text("First moves are limited to 5° per joint from the current position, with a 2° margin inside each joint limit.")
                .font(.system(size: 11)).foregroundStyle(poseMuted).fixedSize(horizontal: false, vertical: true)
            HoldToRun(title: "Hold to move to pose", activeTitle: "Moving · release to stop",
                      enabled: model.armed && model.canMovePose && (!model.held || action == "pose"),
                      held: model.held && action == "pose",
                      begin: { let started = model.beginPose(); if started { action = "pose" }; return started },
                      end: { if action == "pose" { model.releaseHold() } })
                .frame(height: 48)
            Text("\(model.armed ? "ARMED" : "DISARMED") · \(model.sample?.stateLabel ?? "Disconnected")")
                .font(.system(size: 10, weight: .medium)).foregroundStyle(model.armed ? .orange : poseMuted)
            Divider()
            HStack {
                TextField("Pose name", text: $model.poseName).textFieldStyle(.roundedBorder)
                Button("Save current pose") { model.teach() }
                    .disabled(!model.ready || model.sample?.run != 0 || model.poseName.trimmingCharacters(in: .whitespacesAndNewlines).isEmpty)
            }.disabled(editingDisabled)
            Text("Saving records the current measured position.").font(.system(size: 10)).foregroundStyle(poseMuted)
            ForEach(model.poses) { pose in
                HStack {
                    Button { model.selectPose(pose) } label: {
                        VStack(alignment: .leading, spacing: 3) {
                            Text(pose.name).font(.system(size: 12, weight: .medium))
                            Text(pose.angles.map { String(format: "%+.1f°", $0) }.joined(separator: " · "))
                                .font(.system(size: 10, design: .monospaced)).foregroundStyle(poseMuted)
                        }.frame(maxWidth: .infinity, alignment: .leading)
                    }.buttonStyle(.plain)
                    Button { model.deletePose(pose) } label: { Image(systemName: "trash") }
                        .buttonStyle(.borderless).accessibilityLabel("Delete pose \(pose.name)")
                }.padding(10).background(.white.opacity(0.035), in: RoundedRectangle(cornerRadius: 8))
                    .disabled(editingDisabled)
            }
        }
    }

    private var feedback: some View {
        Panel {
            HStack {
                sectionTitle("TARGET & MEASURED FEEDBACK")
                Spacer()
                Picker("Plot joint", selection: $plotJoint) {
                    ForEach(0..<3, id: \.self) { Text(ArmMapping.names[$0]).tag($0) }
                }.labelsHidden().frame(width: 135)
            }
            HStack(spacing: 16) {
                Label("Measured", systemImage: "circle.fill").foregroundStyle(poseMint)
                Label("Reference", systemImage: "minus").foregroundStyle(.orange)
                Spacer()
                if let s = shownSample, s.axes.count == 3 {
                    let axis = s.axes[ArmMapping.motorForJoint[plotJoint]]
                    Text(!axis.usable ? "No fresh angle" : [2, 4, 9].contains(s.state) ? String(format: "Error %+.2f°", (Double(axis.angle) - Double(axis.reference)) / 1000) : "No pose reference")
                        .monospacedDigit().foregroundStyle(poseMuted)
                }
            }.font(.system(size: 10))
            PoseFeedbackPlot(samples: plottedSamples, joint: plotJoint, errorOnly: false)
                .frame(height: 135)
            HStack {
                Text("Tracking error").font(.system(size: 10)).foregroundStyle(poseMuted)
                Spacer()
                Text(model.replaying ? "Entire recording" : "Recent received samples")
                    .font(.system(size: 10)).foregroundStyle(poseMuted)
            }
            PoseFeedbackPlot(samples: plottedSamples, joint: plotJoint, errorOnly: true)
                .frame(height: 65)
            Text("Reference and tracking error appear during pose moves, after arrival and at tracking stops. Error is measured minus planned angle, not torque.")
                .font(.system(size: 10)).foregroundStyle(poseMuted)
        }
    }

    private var recordings: some View {
        Panel {
            HStack {
                sectionTitle("RECORD & REVIEW")
                Spacer()
                if model.recording { Label("Recording", systemImage: "record.circle.fill").foregroundStyle(.red).font(.system(size: 11)) }
            }
            HStack {
                Button(model.recording ? "Finish recording" : "Start recording") {
                    if model.recording { model.endRecording() } else { model.startRecording() }
                }.disabled(!model.recording && (!model.connected || model.replaying))
                Button("Show file") { model.showRecordings() }.disabled(model.recordingPath.isEmpty)
                Spacer()
                Button("Open recording…") { model.loadReplay() }.disabled(busy || model.recording)
            }.buttonStyle(.bordered)
            Text("Records sensor feedback, planned angles, issued pulses and commands for later analysis.")
                .font(.system(size: 11)).foregroundStyle(poseMuted)
            if model.replaying {
                Divider()
                HStack {
                    Label("Visual replay · movement disabled", systemImage: "play.rectangle")
                        .font(.system(size: 12, weight: .medium)).foregroundStyle(poseMint)
                    Spacer()
                    Button("Return to live") { model.closeReplay() }
                }
                Slider(value: $model.replayIndex, in: 0...Double(max(1, model.replay.count - 1)), step: 1)
                    .accessibilityLabel("Recorded sample")
                Text("Sample \(min(model.replay.count, Int(model.replayIndex) + 1)) of \(model.replay.count)")
                    .font(.system(size: 10, design: .monospaced)).foregroundStyle(poseMuted)
            }
        }
    }

    private func sectionTitle(_ text: String) -> some View {
        Text(text).font(.system(size: 10, weight: .semibold)).tracking(1.2).foregroundStyle(poseMuted)
    }
    private func instruction(_ number: String, _ text: String) -> some View {
        HStack(alignment: .top, spacing: 9) {
            Text(number).font(.system(size: 10, weight: .semibold)).foregroundStyle(poseMint)
                .frame(width: 18, height: 18).background(poseMint.opacity(0.12), in: Circle())
            Text(text).font(.system(size: 11)).foregroundStyle(poseMuted).fixedSize(horizontal: false, vertical: true)
        }
    }
    private func angleText(_ angle: Double?) -> String {
        guard let angle, angle.isFinite else { return "—" }
        return String(format: "%+.1f°", angle)
    }
}

private struct PoseFeedbackPlot: View {
    let samples: [ArmSample]
    let joint: Int
    let errorOnly: Bool

    private var series: [(Double?, Double?)] {
        // Bound drawing work for long recordings while preserving breaks in unavailable data.
        let strideBy = max(1, Int(ceil(Double(samples.count) / 600)))
        let motor = ArmMapping.motorForJoint[joint]
        return stride(from: 0, to: samples.count, by: strideBy).map { index in
            let sample = samples[index]
            guard sample.axes.indices.contains(motor), sample.axes[motor].usable,
                  sample.axes[motor].reference != Int(Int32.min) else { return (nil, nil) }
            let axis = sample.axes[motor]
            let measured = Double(axis.angle) / 1000, reference = Double(axis.reference) / 1000
            let hasReference = sample.id > 0 && [2, 4, 9].contains(sample.state)
            return errorOnly ? (hasReference ? measured - reference : nil, nil) : (measured, hasReference ? reference : nil)
        }
    }

    var body: some View {
        let values = series
        let flattened = values.flatMap { [$0.0, $0.1].compactMap { $0 } }
        let low = min(flattened.min() ?? -1, errorOnly ? 0 : flattened.min() ?? -1)
        let high = max(flattened.max() ?? 1, errorOnly ? 0 : flattened.max() ?? 1)
        let padding = max(0.25, (high - low) * 0.12)
        let bottom = low - padding, top = high + padding
        GeometryReader { geometry in
            ZStack(alignment: .topLeading) {
                RoundedRectangle(cornerRadius: 8).fill(.black.opacity(0.18))
                Path { path in
                    for fraction in [0.25, 0.5, 0.75] {
                        let y = geometry.size.height * fraction
                        path.move(to: CGPoint(x: 0, y: y)); path.addLine(to: CGPoint(x: geometry.size.width, y: y))
                    }
                }.stroke(.white.opacity(0.055), lineWidth: 1)
                if errorOnly {
                    let zero = geometry.size.height * CGFloat(top / (top - bottom))
                    Path { path in path.move(to: CGPoint(x: 0, y: zero)); path.addLine(to: CGPoint(x: geometry.size.width, y: zero)) }
                        .stroke(.white.opacity(0.3), style: StrokeStyle(lineWidth: 1, dash: [3, 4]))
                }
                line(values: values.map { $0.0 }, size: geometry.size, bottom: bottom, top: top)
                    .stroke(poseMint, lineWidth: 1.6)
                if !errorOnly {
                    line(values: values.map { $0.1 }, size: geometry.size, bottom: bottom, top: top)
                        .stroke(.orange, style: StrokeStyle(lineWidth: 1.2, dash: [5, 3]))
                }
                VStack(alignment: .leading) {
                    Text(String(format: "%+.1f°", top)); Spacer(); Text(String(format: "%+.1f°", bottom))
                }.font(.system(size: 9, design: .monospaced)).foregroundStyle(poseMuted).padding(5)
                if flattened.isEmpty {
                    Text(errorOnly ? "Tracking error appears during pose moves" : "Waiting for angle feedback").font(.system(size: 11)).foregroundStyle(poseMuted)
                        .frame(maxWidth: .infinity, maxHeight: .infinity)
                }
            }
        }.accessibilityLabel(errorOnly ? "Measured minus reference angle plot" : "Measured and reference joint angle plot")
    }

    private func line(values: [Double?], size: CGSize, bottom: Double, top: Double) -> Path {
        Path { path in
            var connected = false
            for (index, value) in values.enumerated() {
                guard let value, value.isFinite else { connected = false; continue }
                let point = CGPoint(x: size.width * Double(index) / Double(max(1, values.count - 1)),
                                    y: size.height * (top - value) / (top - bottom))
                if connected { path.addLine(to: point) } else { path.move(to: point) }
                connected = true
            }
        }
    }
}
