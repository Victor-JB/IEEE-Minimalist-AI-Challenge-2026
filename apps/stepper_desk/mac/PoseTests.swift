import Foundation
@main struct PoseTests {
    static func main() throws {
        assert(RatioTrial.make(direction:0,pulses:800,degrees:8,forward:1)?.ppd == 100)
        assert(RatioTrial.make(direction:0,pulses:800,degrees:-8,forward:1) == nil)
        assert(RatioTrial.make(direction:1,pulses:800,degrees:-8,forward:1) != nil)
        assert(RatioTrial.make(direction:0,pulses:40,degrees:6,forward:1) == nil)
        assert(RatioTrial.make(direction:0,pulses:500,degrees:4.9,forward:1) == nil)
        assert(RatioTrial.make(direction:0,pulses:500,degrees:.nan,forward:1) == nil)
        let good=[RatioTrial(direction:0,pulses:800,degrees:8),RatioTrial(direction:1,pulses:810,degrees:-8),RatioTrial(direction:0,pulses:795,degrees:8)]
        assert(RatioTrial.verified(good) == 100)
        assert(RatioTrial.verified(Array(good.prefix(2))) == nil)
        assert(RatioTrial.verified(good.map{RatioTrial(direction:0,pulses:$0.pulses,degrees:abs($0.degrees))}) == nil)
        assert(RatioTrial.verified(good+[RatioTrial(direction:1,pulses:1000,degrees:-8)]) == nil)
        let even=good+[RatioTrial(direction:1,pulses:804,degrees:-8)]
        assert(RatioTrial.verified(even,forward:1) == 100.25)
        assert(RatioTrial.verified(good,forward:-1) == nil)
        assert(RatioTrial.make(direction:0,pulses:3601,degrees:8,forward:1) == nil)
        let axis=ArmAxisSample(angle:1000,reference:2000,target:3000,count:12,direction:0,age:20,magnet:0x30,sensor_error:0)
        assert(axis.usable) // weak but detected field remains usable
        var failed=axis;failed.age=101;assert(!failed.usable)
        failed=axis;failed.magnet=0x28;assert(!failed.usable)
        let sample=ArmSample(v:1,error:0,owner:1,state:0,id:0,ms:10,run:0,awake:0,enabled:1,generation:1,configured:7,verified:0,duration:0,elapsed:0,axes:[axis,axis,axis])
        let encoder=JSONEncoder()
        var recording=try encoder.encode(PoseRecording(type:"metadata",time:0,text:"test"));recording.append(10)
        recording.append(try encoder.encode(PoseRecording(type:"command",time:1,text:"JOG 1 0 0 1")));recording.append(10)
        recording.append(try encoder.encode(PoseRecording(type:"sample",time:2,sample:sample)));recording.append(10)
        let replay=try PoseController.decodeReplay(recording);assert(replay.count==1 && replay[0].axes[0].count==12)
        do {_ = try PoseController.decodeReplay(Data("not json".utf8));assert(false)}catch{}
        let suite="pose-test-\(UUID())",defaults=UserDefaults(suiteName:suite)!
        defer{defaults.removePersistentDomain(forName:suite)}
        let settings=ArmSettings(defaults:defaults);let controller=PoseController(settings:settings,defaults:defaults)
        assert(!controller.beginJog(0) && !controller.beginPose() && !controller.canArm)
        controller.invalidateIfChanged();controller.ratios=[100,100,100]
        settings.joints[0].zero=1;controller.invalidateIfChanged();assert(controller.ratios==[0,0,0])
        for j in 0..<3 { settings.joints[j].calibrated=true }
        var imported: [String:Any] = ["version":1,"fingerprint":controller.fingerprint,"trials":Dictionary(uniqueKeysWithValues:["shoulder","elbow","head"].enumerated().map { j,name in
            let sign=LimitConfiguration(settings.joints[j])!.forwardSign
            return (name,good.map { ["direction":$0.direction,"pulses":$0.pulses,"degrees":abs($0.degrees)*Double($0.direction == 0 ? sign : -sign)] })
        })]
        controller.poses=[TaughtPose(name:"Keep",angles:[0,0,0],fingerprint:controller.fingerprint)]
        try controller.importRatios(JSONSerialization.data(withJSONObject:imported))
        assert(controller.ratios == [100,100,100] && controller.poses.count == 1)
        imported["fingerprint"]="wrong"
        do { try controller.importRatios(JSONSerialization.data(withJSONObject:imported));assert(false) } catch {}
        assert(controller.ratios == [100,100,100])
        let restored=PoseController(settings:settings,defaults:defaults)
        assert(restored.ratios == [100,100,100] && restored.poses.count == 1)
        restored.shutdown()
        var saved=try JSONSerialization.jsonObject(with:defaults.data(forKey:"q.arm.poses.v1")!) as! [String:Any]
        saved["ratios"]=[999,999,999]
        defaults.set(try JSONSerialization.data(withJSONObject:saved),forKey:"q.arm.poses.v1")
        let tampered=PoseController(settings:settings,defaults:defaults)
        assert(tampered.ratios == [0,0,0]);tampered.shutdown()
        controller.shutdown()
        print("Pose verification and recording tests passed")
    }
}
