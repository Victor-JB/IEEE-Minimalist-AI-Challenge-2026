import AppKit
import SwiftUI
@main
struct Preview {
    @MainActor static func main() {
        let _ = NSApplication.shared
        NSApp.appearance=NSAppearance(named:.darkAqua)
        let desk = DeskModel(preview:true)
        let mode = CommandLine.arguments.count > 2 ? CommandLine.arguments[2] : "offline"
        if mode != "offline" {
            let timestamp=Date()
            for axis in 0..<3 {
                let model=desk.axes[axis]
                model.connected=true; model.now={timestamp}
                model.receive("@STP {\"v\":2,\"ack\":0,\"error\":0,\"fault\":0,\"hz\":0,\"target\":0,\"dir\":0,\"want_dir\":0,\"run\":0,\"ppr\":\(model.ppr),\"phase\":\"READY\",\"max_hz\":2400,\"angle_raw\":\([355,980,3897][axis]),\"magnet_status\":\(axis == 1 ? 48 : 32),\"sensor_error\":\(mode == "partial" && axis == 1 ? 3 : 0),\"sensor_age_ms\":20}")
                desk.arm.joints[axis].capture(raw:Double([355,980,3897][axis])*360/4096,knownAngle:[-35,85,-50][axis])
                model.notice="Connected. Press Arm, then hold a direction button."
            }
        }
        let root: AnyView = mode == "calibration" ? AnyView(ArmCalibrationView(shoulder:desk.axes[0],elbow:desk.axes[1],head:desk.axes[2],settings:desk.arm)) : AnyView(QDashboard(desk:desk))
        let view=NSHostingView(rootView:root)
        view.frame = mode == "calibration" ? NSRect(x:0,y:0,width:706,height:650) : NSRect(x:0,y:0,width:1120,height:1000)
        view.appearance=NSAppearance(named:.darkAqua)
        view.layoutSubtreeIfNeeded()
        let bitmap=view.bitmapImageRepForCachingDisplay(in:view.bounds)!
        view.cacheDisplay(in:view.bounds,to:bitmap)
        try! bitmap.representation(using:.png,properties:[:])!.write(to:URL(fileURLWithPath:CommandLine.arguments[1]))
    }
}
