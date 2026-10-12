import SwiftUI
import SceneKit
import simd

struct CADManifest: Decodable {
    struct Part: Decodable {
        let joint: Int
        let file: String
        let position: [Float]
        let quaternion: [Float] // MuJoCo w, x, y, z
        let color: [CGFloat]
    }
    let source: String
    let model: String
    let lengths: [Double]
    let parts: [Part]
}

final class ArmCADScene {
    let scene = SCNScene()
    let camera = SCNNode()
    let joints = (0..<3).map { _ in SCNNode() }
    private var visuals: [[SCNNode]] = [[],[],[]]
    private(set) var loadError: String?
    static var resourceURL: URL {
        if let path = ProcessInfo.processInfo.environment["STEPPER_CAD_RESOURCES"] { return URL(fileURLWithPath:path) }
        return Bundle.main.resourceURL!.appendingPathComponent("ArmCAD")
    }
    init(resources: URL = ArmCADScene.resourceURL) {
        scene.background.contents=NSColor(calibratedRed:0.075,green:0.09,blue:0.11,alpha:1)
        let rig=SCNNode(); rig.eulerAngles.z = .pi; rig.position.y=3
        scene.rootNode.addChildNode(rig)
        rig.addChildNode(joints[0]); joints[0].addChildNode(joints[1]); joints[1].addChildNode(joints[2])
        joints[1].position=SCNVector3(0,-18.5,0); joints[2].position=SCNVector3(0,-18.5,0)
        do {
            let manifest=try JSONDecoder().decode(CADManifest.self,from:Data(contentsOf:resources.appendingPathComponent("model.json")))
            var cache: [String:SCNGeometry]=[:]
            for part in manifest.parts {
                guard (0..<3).contains(part.joint), part.position.count==3, part.quaternion.count==4, part.color.count==4 else { continue }
                let geometry: SCNGeometry
                if let saved=cache[part.file] { geometry=saved.copy() as! SCNGeometry }
                else {
                    let loaded=try Self.mesh(url:resources.appendingPathComponent(part.file))
                    cache[part.file]=loaded; geometry=loaded.copy() as! SCNGeometry
                }
                let material=SCNMaterial()
                material.diffuse.contents=NSColor(calibratedRed:part.color[0],green:part.color[1],blue:part.color[2],alpha:1)
                material.roughness.contents=0.62
                material.lightingModel = .physicallyBased; material.isDoubleSided=true
                geometry.materials=[material]
                let node=SCNNode(geometry:geometry)
                node.position=SCNVector3(part.position[0],part.position[1],part.position[2])
                let q=part.quaternion
                node.simdOrientation=simd_normalize(simd_quatf(ix:q[1],iy:q[2],iz:q[3],r:q[0]))
                joints[part.joint].addChildNode(node); visuals[part.joint].append(node)
            }
        } catch { loadError="CAD model unavailable: \(error.localizedDescription)" }
        let pedestal=SCNNode(geometry:SCNBox(width:10,height:3,length:9,chamferRadius:0.6))
        pedestal.position.y=0.5; pedestal.geometry?.firstMaterial?.diffuse.contents=NSColor(calibratedWhite:0.34,alpha:1)
        scene.rootNode.addChildNode(pedestal)
        let ambient=SCNNode(); ambient.light=SCNLight(); ambient.light?.type = .ambient
        ambient.light?.intensity=500; scene.rootNode.addChildNode(ambient)
        for (position,intensity) in [(SCNVector3(30,60,45),CGFloat(1200)),(SCNVector3(-30,25,-35),CGFloat(750))] {
            let light=SCNNode(); light.light=SCNLight(); light.light?.type = .omni
            light.light?.intensity=intensity; light.position=position; scene.rootNode.addChildNode(light)
        }
        camera.camera=SCNCamera(); camera.camera?.usesOrthographicProjection=true
        camera.camera?.orthographicScale=30; camera.camera?.zNear=0.1; camera.camera?.zFar=300
        scene.rootNode.addChildNode(camera); setCamera(side:false)
    }
    func setCamera(side: Bool) {
        camera.position=side ? SCNVector3(0,22,90) : SCNVector3(48,30,85)
        camera.look(at:SCNVector3(0,22,0))
    }
    func update(angles: [Double?]) {
        SCNTransaction.begin(); SCNTransaction.animationDuration=0
        var validPrefix=true
        for index in 0..<3 {
            let angle=index<angles.count ? angles[index] : nil
            validPrefix = validPrefix && angle != nil && (angle?.isFinite ?? false)
            // Unavailable downstream parts are a faded reference, never stale live data.
            let displayed=validPrefix ? angle! : [-20.0,75.0,-40.0][index]
            joints[index].eulerAngles.z = CGFloat(-displayed * .pi / 180)
            for node in visuals[index] { node.opacity=validPrefix ? 1 : 0.20 }
        }
        SCNTransaction.commit()
    }
    func snapshot(size: CGSize) -> NSImage {
        let renderer=SCNRenderer(device:nil,options:nil)
        renderer.scene=scene; renderer.pointOfView=camera
        return renderer.snapshot(atTime:0,with:size,antialiasingMode:.multisampling4X)
    }
    private static func mesh(url: URL) throws -> SCNGeometry {
        let data=try Data(contentsOf:url)
        guard data.count>=84 else { throw CocoaError(.fileReadCorruptFile) }
        let count=data.withUnsafeBytes { Int(UInt32(littleEndian:$0.loadUnaligned(fromByteOffset:80,as:UInt32.self))) }
        guard count>0, count<1_000_000, data.count==84+count*50 else { throw CocoaError(.fileReadCorruptFile) }
        var vertices=[SCNVector3](), normals=[SCNVector3](), indices=[UInt32]()
        vertices.reserveCapacity(count*3); normals.reserveCapacity(count*3); indices.reserveCapacity(count*3)
        data.withUnsafeBytes { bytes in
            func number(_ offset: Int) -> Float { Float(bitPattern:UInt32(littleEndian:bytes.loadUnaligned(fromByteOffset:offset,as:UInt32.self))) }
            for triangle in 0..<count {
                let offset=84+triangle*50
                let normal=SCNVector3(number(offset),number(offset+4),number(offset+8))
                for vertex in 0..<3 {
                    let start=offset+12+vertex*12
                    vertices.append(SCNVector3(number(start)*0.1,number(start+4)*0.1,number(start+8)*0.1)) // mm → cm
                    normals.append(normal); indices.append(UInt32(indices.count))
                }
            }
        }
        return SCNGeometry(sources:[SCNGeometrySource(vertices:vertices),SCNGeometrySource(normals:normals)],elements:[SCNGeometryElement(indices:indices,primitiveType:.triangles)])
    }
}

struct ArmCADView: NSViewRepresentable {
    let angles: [Double?]
    let side: Bool
    final class Coordinator { let model=ArmCADScene(); var side=false }
    func makeCoordinator() -> Coordinator { Coordinator() }
    func makeNSView(context: Context) -> SCNView {
        let view=SCNView()
        view.scene=context.coordinator.model.scene; view.pointOfView=context.coordinator.model.camera
        view.allowsCameraControl=true; view.antialiasingMode = .multisampling4X
        view.defaultCameraController.target=SCNVector3(0,22,0)
        view.defaultCameraController.inertiaEnabled=false
        view.setAccessibilityLabel("Fusion ARM v29. Drag to orbit the view; this never moves a motor.")
        return view
    }
    func updateNSView(_ view: SCNView, context: Context) {
        context.coordinator.model.update(angles:angles)
        if side != context.coordinator.side {
            context.coordinator.model.setCamera(side:side)
            view.pointOfView=context.coordinator.model.camera; context.coordinator.side=side
        }
    }
}
