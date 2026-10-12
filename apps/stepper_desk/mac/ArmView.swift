import SwiftUI

private let jointColors: [Color] = [Color(red: 0.43, green: 0.88, blue: 0.75), Color(red: 0.50, green: 0.69, blue: 1), Color(red: 1, green: 0.73, blue: 0.43)]

struct ArmView: View {
    @ObservedObject var shoulder: Controller
    @ObservedObject var elbow: Controller
    @ObservedObject var head: Controller
    @ObservedObject var motor: Controller
    @ObservedObject var settings: ArmSettings
    let disarmAll: () -> Void
    @State private var showingCalibration = false
    @State private var sideView = false
    private var sensors: [Controller] { [shoulder, elbow, head] }
    private var angles: [Double?] { (0..<3).map { settings.joints[$0].angle(raw: sensors[$0].measuredAngle) } }
    private var calibrated: Bool { settings.joints.allSatisfy(\.calibrated) }
    var body: some View {
        ScrollView {
            VStack(alignment: .leading, spacing: 20) {
                HStack(alignment: .top) {
                    VStack(alignment: .leading, spacing: 5) {
                        Text("Your arm, live").font(.system(size: 28, weight: .semibold, design: .rounded))
                        Text("Fusion ARM v29 · three measured joints · fixed base").font(.system(size: 12)).foregroundStyle(.secondary)
                    }
                    Spacer()
                    Button("Calibrate angles…") { disarmAll(); showingCalibration = true }.controlSize(.large)
                }
                HStack(alignment: .top, spacing: 20) {
                    Panel {
                        HStack {
                            Label(angles.allSatisfy { $0 != nil } ? "LIVE POSE" : "POSE INCOMPLETE", systemImage: "viewfinder")
                                .font(.system(size: 11, weight: .semibold)).foregroundStyle(jointColors[0])
                            Spacer()
                            Picker("Camera", selection: $sideView) { Text("3D").tag(false); Text("Side").tag(true) }.pickerStyle(.segmented).frame(width: 110, height: 24)
                        }
                        Group {
                            if ProcessInfo.processInfo.environment["STEPPER_PREVIEW"] == "1" {
                                Image(nsImage: previewImage).resizable().scaledToFit()
                            } else { ArmCADView(angles: angles, side: sideView) }
                        }.frame(height: 350).clipShape(RoundedRectangle(cornerRadius: 12))
                            .accessibilityLabel("Arm side view. " + (0..<3).map { "\(ArmMapping.names[$0]): \(angles[$0].map { String(format: "%.1f degrees", $0) } ?? "unavailable")" }.joined(separator: ". "))
                        Text(calibrated ? "Solid parts follow the sensors. Faded parts use a reference pose. Drag to orbit; scroll to zoom." : "Set each joint’s known angle in Calibrate. Faded CAD parts show a reference pose until calibrated; they are not measurements.")
                            .font(.system(size: 11)).foregroundStyle(.secondary).fixedSize(horizontal: false, vertical: true)
                    }
                    VStack(spacing: 12) {
                        ForEach(0..<3) { joint in jointCard(joint) }
                    }.frame(width: 290)
                }
                compactControls
                Text("Measured joint limits stop outward motion; hold the opposite direction to move back. RPM is motor shaft speed.")
                    .font(.system(size: 11)).foregroundStyle(.secondary)
            }.padding(30)
        }
        .sheet(isPresented: $showingCalibration) {
            ArmCalibrationView(shoulder: shoulder, elbow: elbow, head: head, settings: settings)
        }
        .onDisappear { disarmAll() }
    }
    private var previewImage: NSImage {
        let model=ArmCADScene(); model.update(angles:angles); model.setCamera(side:sideView)
        return model.snapshot(size:CGSize(width:1100,height:700))
    }
    private func jointCard(_ joint: Int) -> some View {
        VStack(alignment: .leading, spacing: 8) {
            HStack {
                Text("\(joint + 1)  \(ArmMapping.names[joint])").font(.system(size: 15, weight: .semibold))
                Spacer()
                Text(angles[joint].map { String(format: "%+.1f°", $0) } ?? "—")
                    .font(.system(size: 22, weight: .medium, design: .rounded)).monospacedDigit()
            }.foregroundStyle(jointColors[joint])
            Text(ArmMapping.connections[joint]).font(.system(size: 11))
            Text(ArmMapping.limitLabel(joint)).font(.system(size: 11, weight: .medium, design: .monospaced)).foregroundStyle(.secondary)
            if ArmMapping.outsideLimits(angles[joint], joint: joint) {
                Label("Outside configured range", systemImage: "exclamationmark.triangle")
                    .font(.system(size: 10)).foregroundStyle(.orange)
            }
            Text("Motor \(ArmMapping.motorForJoint[joint] + 1) · sensor \(joint + 1) / CH \(joint)")
                .font(.system(size: 10, design: .monospaced)).foregroundStyle(.secondary)
            Text(sensors[joint].sensorHealthy ? (settings.joints[joint].calibrated ? sensors[joint].sensorNotice : "Live sensor · set joint zero") : sensors[joint].sensorNotice)
                .font(.system(size: 10)).foregroundStyle(sensors[joint].sensorHealthy ? Color.gray : .orange)
            if let raw = sensors[joint].measuredAngle {
                Text("Sensor \(raw, specifier: "%.1f")°").font(.system(size: 10)).foregroundStyle(.secondary)
            }
        }.padding(16).frame(maxWidth: .infinity, alignment: .leading)
            .background(.white.opacity(0.04), in: RoundedRectangle(cornerRadius: 14))
    }
    private var compactControls: some View {
        Panel {
            HStack {
                Text("\(ArmMapping.names[ArmMapping.jointForMotor[motor.axis]]) · Motor \(motor.axis + 1)")
                    .font(.system(size: 18, weight: .semibold))
                Text("1/\(motor.microsteps) step").font(.system(size: 11)).foregroundStyle(.secondary)
                Spacer()
                Text("\(motor.commandedRPM, specifier: "%.1f") RPM · \(motor.phase)").font(.system(size: 12)).monospacedDigit()
            }
            HStack(spacing: 14) {
                Text("Target RPM").font(.system(size: 12))
                TextField("Target RPM", value: $motor.rpm, format: .number.precision(.fractionLength(1)))
                    .frame(width: 75).disabled(motor.held || motor.running)
                Slider(value: $motor.rpm, in: motor.minRPM...motor.maxRPM).disabled(motor.held || motor.running)
                    .accessibilityLabel("Target motor RPM")
                Text("Max \(motor.maxRPM, specifier: "%.0f")").font(.system(size: 11)).foregroundStyle(.secondary)
            }
            HStack(spacing: 12) {
                HoldToRun(title: "Hold Forward", activeTitle: "Forward · release to stop", enabled: motor.canHold(direction: 0), held: motor.held && motor.direction == 0, begin: { motor.beginHold(direction: 0) }, end: { motor.endHold(direction: 0) }).frame(height: 46)
                HoldToRun(title: "Hold Reverse", activeTitle: "Reverse · release to stop", enabled: motor.canHold(direction: 1), held: motor.held && motor.direction == 1, begin: { motor.beginHold(direction: 1) }, end: { motor.endHold(direction: 1) }).frame(height: 46)
                Button(motor.armed ? "Disarm" : "Arm") { motor.toggleArmed() }
                    .buttonStyle(.borderedProminent).controlSize(.large).frame(width: 100)
                    .disabled(!motor.armed && !motor.canArm)
            }
            Text(ArmMapping.limitLabel(ArmMapping.jointForMotor[motor.axis]) + " · Automatic stops on the board.")
                .font(.system(size: 11)).foregroundStyle(.secondary)
            Text(motor.limitNotice).font(.system(size: 11)).foregroundStyle(motor.limitReady ? Color.secondary : .orange)
            Text(motor.notice).font(.system(size: 11)).foregroundStyle(motor.problem ? Color.orange : .secondary)
        }
    }
}

struct ArmCalibrationView: View {
    @ObservedObject var shoulder: Controller
    @ObservedObject var elbow: Controller
    @ObservedObject var head: Controller
    @ObservedObject var settings: ArmSettings
    @Environment(\.dismiss) private var dismiss
    @State private var known = [0.0,0.0,0.0]
    private var sensors: [Controller] { [shoulder,elbow,head] }
    private var stopped: Bool { sensors.allSatisfy { !$0.armed && !$0.running && !$0.held && !$0.stopping && $0.pending == nil } }
    var body: some View {
        VStack(alignment: .leading, spacing: 18) {
            Text("Align the arm view").font(.system(size: 25, weight: .semibold, design: .rounded))
            Text("Shoulder 0° points straight up. Elbow and head 0° continue straight from the previous segment. Positive angles bend toward the front, matching the CAD branch (clockwise in Side view).")
                .font(.system(size: 12)).foregroundStyle(.secondary).fixedSize(horizontal:false,vertical:true)
            Text("For each joint, enter its current physical angle, then click Set angle. Reverse a sensor if its angle moves the wrong way. Recalibrate after reversing. This only changes the drawing.")
                .font(.system(size: 12)).fixedSize(horizontal:false,vertical:true)
            ForEach(0..<3) { joint in
                VStack(alignment: .leading, spacing: 10) {
                    HStack {
                        Text(ArmMapping.names[joint]).font(.headline).foregroundStyle(jointColors[joint])
                        Text("Motor \(ArmMapping.motorForJoint[joint]+1) · sensor \(joint+1)").font(.caption).foregroundStyle(.secondary)
                        Spacer()
                        Text(settings.joints[joint].calibrated ? "Calibrated" : "Needs calibration").font(.caption)
                    }
                    HStack {
                        Text(sensors[joint].measuredAngle.map { String(format:"Sensor %.1f°", $0) } ?? "Sensor unavailable").font(.caption).frame(width:135,alignment:.leading)
                        TextField("Known joint angle",value:$known[joint],format:.number.precision(.fractionLength(1))).frame(width:65).accessibilityLabel("\(ArmMapping.names[joint]) known joint angle")
                        Text("°")
                        Button("Set angle") {
                            if stopped, let raw = sensors[joint].measuredAngle, known[joint].isFinite, (-180...180).contains(known[joint]) {
                                settings.joints[joint].capture(raw:raw,knownAngle:known[joint])
                            }
                        }.disabled(!stopped || sensors[joint].measuredAngle == nil || !known[joint].isFinite || !(-180...180).contains(known[joint]))
                        Spacer()
                        Toggle("Reverse sensor",isOn:Binding(get:{settings.joints[joint].reversed},set:{settings.joints[joint].reversed=$0;settings.joints[joint].calibrated=false})).toggleStyle(.checkbox)
                    }
                }.padding(14).background(.white.opacity(0.04),in:RoundedRectangle(cornerRadius:12))
            }
            Divider()
            Text("CAD geometry · ARM v29").font(.headline)
            Text("First and second links: 18.5 cm each. Wrist to head-tilt pivot: 4.5 cm. Meshes and joint pivots come from full-model-animations. The base is fixed; the separate head-tilt servo stays at zero.")
                .font(.caption).foregroundStyle(.secondary).fixedSize(horizontal:false,vertical:true)
            HStack {
                Text("Calibration saves on this Mac.").font(.caption).foregroundStyle(.secondary)
                Spacer()
                Button("Done") { dismiss() }.keyboardShortcut(.defaultAction)
            }
        }.padding(28).frame(width:650).foregroundStyle(.white).background(Color(red:0.055,green:0.07,blue:0.085)).preferredColorScheme(.dark)
    }
}
