import AppKit
import SceneKit
@main
struct CADTests {
    static func near(_ a: CGFloat,_ b: CGFloat) { assert(abs(a-b)<0.0001,"\(a) != \(b)") }
    static func main() {
        let model=ArmCADScene()
        assert(model.loadError==nil,model.loadError ?? "")
        assert(model.joints.allSatisfy { !$0.childNodes.isEmpty })
        model.update(angles:[0,0,0])
        let elbow=model.joints[1].convertPosition(SCNVector3Zero,to:nil)
        let wrist=model.joints[2].convertPosition(SCNVector3Zero,to:nil)
        near(elbow.x,0); near(elbow.y,21.5); near(wrist.y,40)
        model.update(angles:[90,-90,0])
        let bent=model.joints[2].convertPosition(SCNVector3Zero,to:nil)
        near(bent.x,18.5); near(bent.y,21.5)
        near(model.joints[2].childNodes.last!.opacity,1)
        model.update(angles:[20,nil,10])
        near(model.joints[0].childNodes.last!.opacity,1)
        near(model.joints[1].childNodes.last!.opacity,0.2)
        near(model.joints[2].childNodes.last!.opacity,0.2)
        model.update(angles:[nil,0,0])
        near(model.joints[0].childNodes.last!.opacity,0.2)
        let missing=ArmCADScene(resources:URL(fileURLWithPath:"/tmp/no-stepper-cad-assets"))
        assert(missing.loadError != nil)
        print("PASS: CAD assets, 18.5 cm pivots, cumulative joint transforms, positive direction, downstream invalidation, missing resources")
    }
}
