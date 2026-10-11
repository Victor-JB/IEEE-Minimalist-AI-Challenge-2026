import Foundation
import Combine
import AppKit
import UniformTypeIdentifiers

struct ArmAxisSample: Codable {
    var angle: Int, reference: Int, target: Int, count: Int, direction: Int, age: Int, magnet: Int, sensor_error: Int
    var usable: Bool { angle != Int(Int32.min) && age >= 0 && age <= 100 && sensor_error == 0 && magnet & 0x28 == 0x20 }
}
struct ArmSample: Codable {
    var v: Int, error: Int, owner: Int, state: Int, id: Int, ms: Int, run: Int, awake: Int, enabled: Int, generation: Int, configured: Int, verified: Int, duration: Int, elapsed: Int
    var axes: [ArmAxisSample]
    var configuration_echo: [Int]? = nil
    var healthy: Bool { axes.count == 3 && axes.allSatisfy { $0.usable } }
    var angles: [Double] { ArmMapping.motorForJoint.map { Double(axes[$0].angle)/1000 } }
    var references: [Double] { ArmMapping.motorForJoint.map { Double(axes[$0].reference)/1000 } }
    var stateLabel: String {
        let names = ["Idle", "Preparing", "Moving", "Measuring", "Reached pose", "Released", "Hold expired", "Sensor unavailable", "Joint limit", "Tracking error", "Timing fault", "Measurement travel reached", "Settling with power"]
        return names.indices.contains(state) ? names[state] : "Unknown state"
    }
}
struct RatioTrial: Codable, Identifiable {
    var id = UUID()
    var direction: Int, pulses: Int, degrees: Double
    var ppd: Double { Double(pulses)/abs(degrees) }
    static func make(direction: Int, pulses: Int, degrees: Double, forward: Int) -> RatioTrial? {
        guard (0...1).contains(direction), pulses >= 50, pulses <= 3600, degrees.isFinite, abs(degrees) >= 5, abs(degrees) <= 13,
              degrees * Double(direction == 0 ? forward : -forward) > 0 else { return nil }
        let trial = RatioTrial(direction: direction, pulses: pulses, degrees: degrees)
        return (1...10000).contains(trial.ppd) ? trial : nil
    }
    static func verified(_ trials: [RatioTrial], forward: Int? = nil) -> Double? {
        guard trials.count >= 3, Set(trials.map(\.direction)) == Set([0,1]),
              trials.allSatisfy({ $0.pulses >= 50 && $0.pulses <= 3600 && $0.degrees.isFinite && abs($0.degrees) >= 5 && abs($0.degrees) <= 13 && (forward == nil || $0.degrees * Double($0.direction == 0 ? forward! : -forward!) > 0) && $0.ppd.isFinite && (1...10000).contains($0.ppd) }) else { return nil }
        let sorted = trials.map(\.ppd).sorted(), median = (sorted[(sorted.count-1)/2] + sorted[sorted.count/2])/2
        guard (sorted.last! - sorted.first!)/median <= 0.05 else { return nil }
        return median
    }
}
struct TaughtPose: Codable, Identifiable {
    var id = UUID()
    var name: String
    var angles: [Double]
    var fingerprint: String
}
struct PoseRecording: Codable {
    var type: String
    var version = 1
    var time: Double
    var sample: ArmSample?
    var text: String?
    var fingerprint: String?
    var ratios: [Double]?
}

final class PoseController: ObservableObject {
    @Published var connected = false
    @Published var connecting = false
    @Published var armed = false
    @Published var held = false
    @Published var sample: ArmSample?
    @Published var notice = "Connect to measure joint ratios and record feedback."
    @Published var joint = 0
    @Published var trials: [[RatioTrial]] = [[],[],[]]
    @Published var ratios = [0.0,0.0,0.0]
    @Published var poses: [TaughtPose] = []
    @Published var targets = [0.0,0.0,0.0]
    @Published var poseName = "Taught pose"
    @Published var recording = false
    @Published var recordingPath = ""
    @Published var replay: [ArmSample] = []
    @Published var replayIndex = 0.0
    @Published var replaying = false
    @Published var history: [ArmSample] = []
    private let link = SSHLink(axis: -1)
    private let settings: ArmSettings
    private let defaults: UserDefaults
    private var timer: Timer?
    private var lastReceived = Date.distantPast
    private var configuredFingerprint = ""
    private var pendingConfigurations: [[Int]] = [[-1]]
    private var id = 0
    private var beganTrial: (joint: Int, direction: Int, angle: Double, id: Int)?
    private var releasedAt = Date.distantPast
    private var file: FileHandle?
    private var recordStart = Date()
    private var savedFingerprint = ""
    private var lastState = -1
    private var needsInitialTargets = true
    private struct Saved: Codable { var fingerprint: String; var trials: [[RatioTrial]]; var ratios: [Double]; var poses: [TaughtPose] }
    init(settings: ArmSettings, defaults: UserDefaults = .standard) {
        self.settings=settings; self.defaults=defaults
        if let data=defaults.data(forKey: "q.arm.poses.v1"), let saved=try? JSONDecoder().decode(Saved.self,from:data), saved.trials.count==3, saved.ratios.count==3 {
            savedFingerprint=saved.fingerprint; trials=saved.trials; poses=saved.poses
            ratios=(0..<3).map { j in
                guard saved.fingerprint == fingerprint, saved.ratios[j].isFinite, saved.ratios[j] > 0,
                      let c=LimitConfiguration(settings.joints[j]),
                      let checked=RatioTrial.verified(saved.trials[j],forward:c.forwardSign),
                      abs(checked-saved.ratios[j]) < 0.000001 else { return 0 }
                return checked
            }
        }
        link.onLine = { [weak self] in self?.receive($0) }
        link.onError = { [weak self] error in self?.fail(error) }
        timer=Timer.scheduledTimer(withTimeInterval:0.1,repeats:true) { [weak self] _ in self?.tick() }
    }
    var fingerprint: String {
        settings.joints.enumerated().map { i,c in
            let motor=ArmMapping.motorForJoint[i]
            let micro=defaults.integer(forKey:"q.axis\(motor).microsteps")
            let full=defaults.integer(forKey:"q.axis\(motor).fullSteps")
            return "\(i):\(c.zero):\(c.reversed):\(c.calibrated):\(micro):\(full)"
        }.joined(separator:"|")
    }
    var ready: Bool { connected && Date().timeIntervalSince(lastReceived)<0.5 && sample?.healthy == true && sample?.configured == 7 && sample?.enabled == 1 && configuredFingerprint == fingerprint && pendingConfigurations.isEmpty && !replaying }
    var canArm: Bool { ready && sample?.run == 0 }
    var allVerified: Bool { ratios.allSatisfy { $0.isFinite && $0>0 } && savedFingerprint == fingerprint && sample?.verified == 7 }
    var canMovePose: Bool {
        guard ready, allVerified, targets.count==3, let current=sample?.angles else { return false }
        return (0..<3).allSatisfy { j in
            let range=ArmMapping.limits[j]
            return current[j].isFinite && range.contains(current[j]) && targets[j].isFinite &&
                targets[j]>=range.lowerBound+2 && targets[j]<=range.upperBound-2 && abs(targets[j]-current[j])<=5
        }
    }
    var displayedAngles: [Double?] {
        let current = replaying && !replay.isEmpty ? replay[min(replay.count-1,max(0,Int(replayIndex)))] : sample
        guard let current, current.healthy else { return [nil,nil,nil] }
        return current.angles.map(Optional.some)
    }
    var candidate: Double? { RatioTrial.verified(trials[joint],forward:LimitConfiguration(settings.joints[joint])?.forwardSign) }
    var canSaveTrial: Bool { !held && sample?.run == 0 && Date().timeIntervalSince(releasedAt)>0.3 && ready && beganTrial != nil }
    func connect() {
        guard !connected && !connecting else { return }
        invalidateIfChanged(); connecting=true; configuredFingerprint=""; sample=nil; needsInitialTargets=true; id=0; beganTrial=nil; history=[]; replaying=false
        notice="Connecting to the whole arm…"; link.connect("")
    }
    func disconnect() { stop(); link.close(); connected=false;connecting=false; sample=nil; endRecording() }
    func shutdown() { disconnect(); timer?.invalidate() }
    func stop() {
        if held { releasedAt=Date() }
        held=false;armed=false
        if connected { send("STOP",priority:true) }
    }
    func releaseHold() { if held { stop() } }
    func toggleArm() { if armed { stop() } else if canArm { armed=true; notice="Armed · hold a button to move." } }
    func power(_ enabled: Bool) { stop(); if let s=sample { send("POWER \(enabled ? 1 : 0) \(s.generation)",priority:true) } }
    func invalidateIfChanged() {
        if savedFingerprint != fingerprint {
            beganTrial=nil;releasedAt = .distantPast
            ratios=[0,0,0]; trials=[[],[],[]]; savedFingerprint=fingerprint; persist()
        }
    }
    private func configure() {
        guard connected else { return }
        configuredFingerprint=fingerprint
        pendingConfigurations=[[-1]]
        var commands: [[Int]] = []
        for j in 0..<3 {
            guard let c=LimitConfiguration(settings.joints[j]) else { notice="Calibrate all angle sensors in Live arm first.";return }
            let value = ratios[j].isFinite ? max(0,min(10000000,Int((ratios[j]*1000).rounded()))) : 0
            commands.append([ArmMapping.motorForJoint[j],c.zeroUdeg,c.reversed,c.forwardSign,value,value>0 ? 1 : 0])
        }
        pendingConfigurations=commands
        for values in commands { send("CONFIG " + values.map(String.init).joined(separator:" ")) }
    }
    func beginJog(_ direction: Int) -> Bool {
        guard armed,ready,!held,let s=sample, s.run==0, let c=LimitConfiguration(settings.joints[joint]) else { return false }
        let a=s.angles[joint],sign=direction==0 ? c.forwardSign : -c.forwardSign,range=ArmMapping.limits[joint]
        guard (a>range.lowerBound || sign>0) && (a<range.upperBound || sign<0) else { notice="Choose the direction back into the joint range.";return false }
        id+=1;beganTrial=(joint,direction,a,id);held=true;releasedAt = .distantFuture
        send("JOG \(id) \(ArmMapping.motorForJoint[joint]) \(direction) \(s.generation)")
        notice="Measuring at 100 pulses/s · release after 5–10° of travel.";return true
    }
    func beginPose() -> Bool {
        guard armed,canMovePose,!held,let s=sample,s.run==0 else { return false }
        id+=1;beganTrial=nil;held=true
        let motorTargets=ArmMapping.jointForMotor.map { Int((targets[$0]*1000).rounded()) }
        send("POSE \(id) \(motorTargets.map(String.init).joined(separator:" ")) \(s.generation)")
        notice="Following a smooth coordinated move · release to stop.";return true
    }
    func saveTrial() {
        guard canSaveTrial,let t=beganTrial,let s=sample,s.id==t.id,
              [5,11].contains(s.state),let c=LimitConfiguration(settings.joints[t.joint]),
              let trial=RatioTrial.make(direction:t.direction,pulses:s.axes[ArmMapping.motorForJoint[t.joint]].count,degrees:s.angles[t.joint]-t.angle,forward:c.forwardSign) else {
            notice="Trial rejected. Use 5–10° of clean travel, release, and wait for the sensor to settle.";return
        }
        trials[t.joint].append(trial);ratios[t.joint]=0;beganTrial=nil;persist();configure();notice="Saved measurement. Collect at least three, including both directions."
    }
    struct ImportedRatios: Decodable {
        struct Trial: Decodable { var direction: Int; var pulses: Int; var degrees: Double }
        var version: Int; var fingerprint: String; var trials: [String:[Trial]]
    }
    func importRatios(_ data: Data) throws {
        guard !connected && !connecting && !held else { throw NSError(domain:"RatioImport",code:1,userInfo:[NSLocalizedDescriptionKey:"Disconnect pose control before importing."]) }
        let imported=try JSONDecoder().decode(ImportedRatios.self,from:data)
        guard imported.version == 1 && imported.fingerprint == fingerprint else { throw NSError(domain:"RatioImport",code:2,userInfo:[NSLocalizedDescriptionKey:"Calibration settings do not match."]) }
        var checkedTrials: [[RatioTrial]]=[]; var checkedRatios: [Double]=[]
        for (j,name) in ["shoulder","elbow","head"].enumerated() {
            guard let rows=imported.trials[name],let c=LimitConfiguration(settings.joints[j]) else { throw NSError(domain:"RatioImport",code:3) }
            let values=rows.compactMap { RatioTrial.make(direction:$0.direction,pulses:$0.pulses,degrees:$0.degrees,forward:c.forwardSign) }
            guard values.count == rows.count,let ratio=RatioTrial.verified(values,forward:c.forwardSign) else { throw NSError(domain:"RatioImport",code:4,userInfo:[NSLocalizedDescriptionKey:"Trials are invalid or disagree by more than 5%."]) }
            checkedTrials.append(values);checkedRatios.append(ratio)
        }
        trials=checkedTrials;ratios=checkedRatios;savedFingerprint=fingerprint;persist()
        notice="Imported verified measurements for all three joints."
    }
    func chooseRatioImport() {
        let panel=NSOpenPanel();panel.allowedContentTypes=[.json];panel.allowsMultipleSelection=false
        guard panel.runModal() == .OK,let url=panel.url else { return }
        do { try importRatios(Data(contentsOf:url)) } catch { notice="Import rejected: \(error.localizedDescription)" }
    }
    func acceptRatio() { guard ready,!held,sample?.run==0,let value=candidate else { return };stop();ratios[joint]=value;persist();configure();notice="Verified \(String(format:"%.3f",value)) pulses per joint degree." }
    func resetRatio() { guard ready,!held,sample?.run==0 else { return };stop();trials[joint]=[];ratios[joint]=0;persist();configure() }
    func useCurrent() { if let s=sample,s.healthy { targets=s.angles } }
    func teach() { guard ready,let s=sample,s.run==0 else{return};poses.append(TaughtPose(name:poseName,angles:s.angles,fingerprint:fingerprint));persist();notice="Pose saved; saving never moves the arm." }
    func selectPose(_ pose: TaughtPose) { stop();guard pose.fingerprint==fingerprint,pose.angles.count==3 else { notice="This pose uses an older calibration. Teach it again.";return };targets=pose.angles }
    func deletePose(_ pose: TaughtPose) { poses.removeAll{$0.id==pose.id};persist() }
    private func persist(){if let data=try? JSONEncoder().encode(Saved(fingerprint:savedFingerprint,trials:trials,ratios:ratios,poses:poses)){defaults.set(data,forKey:"q.arm.poses.v1")}}
    private func send(_ text:String,priority:Bool=false){append(PoseRecording(type:"command",time:Date().timeIntervalSince(recordStart),text:text));link.send(text,priority:priority)}
    private func tick(){
        if connected && configuredFingerprint != fingerprint { stop();invalidateIfChanged();configure();notice="Settings changed; ratios must be verified again." }
        if held && !ready { stop();notice="Feedback is unavailable; movement stopped." }
        else if held { send("HOLD \(id)") }
        if connected && Date().timeIntervalSince(lastReceived)>0.6 { fail("Feedback timed out. Reconnect to continue.");link.close() }
    }
    private func fail(_ text:String){beganTrial=nil;held=false;armed=false;connected=false;connecting=false;notice=text;endRecording()}
    private func receive(_ line:String){
        guard line.hasPrefix("@ARM "), let data=String(line.dropFirst(5)).data(using:.utf8), let next=try? JSONDecoder().decode(ArmSample.self,from:data),next.v==1,next.axes.count==3 else{return}
        if next.error == 0, let echo=next.configuration_echo { pendingConfigurations.removeAll { $0 == echo } }
        let previous=sample;sample=next;lastReceived=Date();connecting=false
        let first = !connected; connected=true
        append(PoseRecording(type:"sample",time:Date().timeIntervalSince(recordStart),sample:next))
        history.append(next);if history.count>300{history.removeFirst(history.count-300)}
        if next.healthy && needsInitialTargets { useCurrent();needsInitialTargets=false }
        if first { configure();notice="Connected · drivers stay asleep until a held move." }
        if next.error != 0 { stop();beganTrial=nil;notice="Command rejected (\(next.error)). Check limits, calibration and connection.";return }
        if let previous, previous.generation != next.generation { held=false;armed=false;beganTrial=nil }
        if next.enabled==0 { held=false;armed=false }
        if next.id==id && next.run==0 && next.state>=4 {
            if held { releasedAt=Date();held=false;armed=false }
            if next.state != lastState { notice=next.stateLabel }
        }
        if (6...10).contains(next.state) { beganTrial=nil }
        lastState=next.state
    }
    func startRecording(){
        guard connected,!recording else{return}
        do {
            let dir=try FileManager.default.url(for:.applicationSupportDirectory,in:.userDomainMask,appropriateFor:nil,create:true).appendingPathComponent("Stepper Desk Q/Recordings",isDirectory:true)
            try FileManager.default.createDirectory(at:dir,withIntermediateDirectories:true)
            let url=dir.appendingPathComponent("arm-\(Int(Date().timeIntervalSince1970))-\(UUID().uuidString.prefix(6)).jsonl")
            guard FileManager.default.createFile(atPath:url.path,contents:nil) else { throw CocoaError(.fileWriteUnknown) }
            file=try FileHandle(forWritingTo:url);recordingPath=url.path;recordStart=Date();recording=true
            append(PoseRecording(type:"metadata",time:0,text:"motor-order arrays; angle/reference/target in millidegrees; counts are issued STEP edges; no torque measurement; transport sampling ~10 Hz",fingerprint:fingerprint,ratios:ratios))
        }catch{notice="Cannot record: \(error.localizedDescription)"}
    }
    func endRecording(){try? file?.close();file=nil;recording=false}
    private func append(_ event:PoseRecording){guard let file else{return};do{var data=try JSONEncoder().encode(event);data.append(10);try file.write(contentsOf:data)}catch{endRecording();notice="Recording stopped: \(error.localizedDescription)"}}
    func showRecordings(){if !recordingPath.isEmpty{NSWorkspace.shared.activateFileViewerSelecting([URL(fileURLWithPath:recordingPath)])}}
    func loadReplay(){
        stop();let panel=NSOpenPanel();panel.allowedContentTypes=[UTType(filenameExtension:"jsonl") ?? .data,.json,.plainText];panel.allowsMultipleSelection=false
        if !recordingPath.isEmpty { panel.directoryURL=URL(fileURLWithPath:recordingPath).deletingLastPathComponent() }
        else { panel.directoryURL=FileManager.default.urls(for:.applicationSupportDirectory,in:.userDomainMask).first?.appendingPathComponent("Stepper Desk Q/Recordings") }
        guard panel.runModal() == .OK,let url=panel.url else{return}
        do{let data=try Data(contentsOf:url);replay=try Self.decodeReplay(data);replayIndex=0;replaying=true;notice="Visual replay only · motor commands are disabled."}catch{notice="Cannot open recording: \(error.localizedDescription)"}
    }
    static func decodeReplay(_ data:Data)throws->[ArmSample]{
        guard data.count<=50_000_000 else{throw CocoaError(.fileReadTooLarge)}
        var result:[ArmSample]=[]
        for line in data.split(separator:10){let event=try JSONDecoder().decode(PoseRecording.self,from:Data(line));guard event.version==1 else{throw CocoaError(.fileReadCorruptFile)}
            if event.type=="sample",let sample=event.sample,sample.v==1,sample.axes.count==3{result.append(sample)}}
        guard !result.isEmpty else{throw CocoaError(.fileReadCorruptFile)};return result
    }
    func closeReplay(){stop();replaying=false;replay=[];notice=connected ? "Live feedback · disarmed." : "Connect to continue commissioning."}
}
