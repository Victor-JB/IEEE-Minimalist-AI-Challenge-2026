import SwiftUI
import AppKit
@main
struct UITests {
    @MainActor static func main() {
        let _ = NSApplication.shared
        let button = HoldButtonView(frame: NSRect(x: 0, y: 0, width: 350, height: 48))
        var starts = 0, stops = 0
        button.begin = { starts += 1; button.held = true; return true }
        button.end = { stops += 1; button.held = false }
        func event(_ type: NSEvent.EventType, _ x: CGFloat = 10, _ y: CGFloat = 10) -> NSEvent {
            NSEvent.mouseEvent(with: type, location: NSPoint(x:x,y:y), modifierFlags: [], timestamp: 0,
                               windowNumber: 0, context: nil, eventNumber: 0, clickCount: 1, pressure: 1)!
        }
        button.mouseDown(with: event(.leftMouseDown)); assert(starts == 0)
        button.enabled = true
        button.mouseDown(with: event(.leftMouseDown)); assert(starts == 1)
        button.mouseDragged(with: event(.leftMouseDragged, 1000, 1000)); assert(stops == 1)
        button.mouseDragged(with: event(.leftMouseDragged)); button.mouseUp(with: event(.leftMouseUp))
        assert(starts == 1 && stops == 1)
        button.mouseDown(with: event(.leftMouseDown)); button.mouseUp(with: event(.leftMouseUp, 1000, 1000))
        assert(starts == 2 && stops == 2)
        assert(!button.accessibilityPerformPress())
        print("PASS: native mouse release outside, drag-away cancellation, no restart on re-entry, disabled control")
    }
}
