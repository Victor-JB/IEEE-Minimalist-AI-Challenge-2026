import Foundation
import Darwin

/// Each motor gets its own authenticated SSH session and Bridge ownership token.
final class SSHLink {
    var onLine: ((String) -> Void)?
    var onError: ((String) -> Void)?
    private let axis: Int
    private let queue = DispatchQueue(label: "dev.tongtong.stepperq.ssh")
    private var process: Process?
    private var input: Pipe?
    private var output: Pipe?
    private var errors: Pipe?
    private var reader: DispatchSourceTimer?
    private var incoming = Data(), outgoing = Data(), errorText = Data()
    private var generation = 0
    init(axis: Int) { self.axis = axis }
    static func ports() -> [String] { ["Arduino Q · USB"] }

    func connect(_ unused: String) {
        queue.async { [self] in
            closeInternal()
            // Re-establish the USB forward after reconnecting the cable.
            let adb = Process()
            let paths = ["/opt/homebrew/bin/adb", "/usr/local/bin/adb"]
            guard let path = paths.first(where: { FileManager.default.isExecutableFile(atPath: $0) }) else {
                fail("Install Android Platform Tools with Homebrew, then reconnect."); return
            }
            adb.executableURL = URL(fileURLWithPath: path)
            adb.arguments = ["-s", "2709356903", "forward", "tcp:2222", "tcp:22"]
            adb.standardOutput = FileHandle.nullDevice; adb.standardError = FileHandle.nullDevice
            do { try adb.run() } catch { fail("Cannot start the Arduino Q USB connection."); return }
            // Polling below watches the SSH connection; ADB forward normally completes instantly.
            let deadline = Date().addingTimeInterval(2)
            while adb.isRunning && Date() < deadline { Thread.sleep(forTimeInterval: 0.02) }
            if adb.isRunning { adb.terminate(); fail("USB setup timed out. Reconnect Arduino Q."); return }
            guard adb.terminationStatus == 0 else { fail("Arduino Q not found. Check its USB cable."); return }
            let task = Process(), pin = Pipe(), pout = Pipe(), perr = Pipe()
            task.executableURL = URL(fileURLWithPath: "/usr/bin/ssh")
            // Board login currently takes ~3.4 s. A 1 s SSH keepalive killed
            // healthy logins before the gateway started. Motor leases stay 300 ms.
            task.arguments = ["-T", "-p", "2222", "-o", "BatchMode=yes", "-o", "ConnectTimeout=3",
                              "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=2", "arduino@localhost",
                              "/home/arduino/stepper-desk-q/.venv/bin/python -u /home/arduino/stepper-desk-q/python/gateway.py --axis \(axis)"]
            task.standardInput = pin; task.standardOutput = pout; task.standardError = perr
            do { try task.run() } catch { fail("Cannot open SSH to Arduino Q: \(error.localizedDescription)"); return }
            process = task; input = pin; output = pout; errors = perr
            for fd in [pin.fileHandleForWriting.fileDescriptor, pout.fileHandleForReading.fileDescriptor, perr.fileHandleForReading.fileDescriptor] {
                _ = fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_NONBLOCK)
            }
            outgoing = Data("HELLO\n".utf8)
            let timer = DispatchSource.makeTimerSource(queue: queue)
            timer.schedule(deadline: .now(), repeating: .milliseconds(10))
            timer.setEventHandler { [weak self] in self?.poll() }
            reader = timer; timer.resume()
        }
    }
    func send(_ command: String, priority: Bool = false) {
        queue.async {
            guard self.input != nil else { return }
            if priority { self.outgoing.removeAll() }
            if self.outgoing.count > 2048 { self.fail("Command queue full; disconnected."); return }
            self.outgoing.append(Data(((priority ? "\n" : "") + command + "\n").utf8))
            self.poll()
        }
    }
    func close() { queue.sync { closeInternal() } }
    private func closeInternal() {
        generation += 1
        reader?.cancel(); reader = nil
        if let pipe = input {
            let bytes = Array("\nSTOP\n".utf8)
            _ = bytes.withUnsafeBytes { Darwin.write(pipe.fileHandleForWriting.fileDescriptor, $0.baseAddress!, $0.count) }
            try? pipe.fileHandleForWriting.close()
        }
        // EOF lets the remote gateway stop its owned motor before exiting.
        // Terminate SSH after a grace period if the remote side is unreachable.
        if let task = process {
            DispatchQueue.global().asyncAfter(deadline: .now()+0.4) { if task.isRunning { task.terminate() } }
        }
        input = nil; output = nil; errors = nil; process = nil
        incoming.removeAll(); outgoing.removeAll(); errorText.removeAll()
    }
    private func fail(_ message: String) {
        closeInternal()
        let callback = onError, version = generation
        DispatchQueue.main.async { [weak self] in
            guard let self, self.queue.sync(execute: { self.generation == version }) else { return }
            callback?(message)
        }
    }
    private func poll() {
        guard let input, let output, let errors else { return }
        if !outgoing.isEmpty {
            let n = outgoing.withUnsafeBytes { Darwin.write(input.fileHandleForWriting.fileDescriptor, $0.baseAddress!, $0.count) }
            if n > 0 { outgoing.removeFirst(n) }
            else if n < 0 && errno != EAGAIN && errno != EINTR { fail("Connection lost; motor watchdog will stop motion."); return }
        }
        var buffer = [UInt8](repeating: 0, count: 4096)
        let e = Darwin.read(errors.fileHandleForReading.fileDescriptor, &buffer, buffer.count)
        if e > 0 { errorText.append(contentsOf: buffer.prefix(e)); if errorText.count > 8192 { errorText.removeFirst(errorText.count-8192) } }
        for _ in 0..<8 {
            let n = Darwin.read(output.fileHandleForReading.fileDescriptor, &buffer, buffer.count)
            if n > 0 {
                incoming.append(contentsOf: buffer.prefix(n))
                while let end = incoming.firstIndex(of: 10) {
                    let line = String(decoding: incoming[..<end], as: UTF8.self)
                    incoming.removeSubrange(...end)
                    if line.hasPrefix("@ERR ") { fail(String(line.dropFirst(5))); return }
                    let callback = onLine, version = generation
                    DispatchQueue.main.async { [weak self] in
                        guard let self, self.queue.sync(execute: { self.generation == version }) else { return }
                        callback?(line)
                    }
                }
                if incoming.count > 8192 { fail("Invalid telemetry stream."); return }
            } else {
                if n == 0 || (n < 0 && errno != EAGAIN && errno != EINTR) {
                    let detail = String(decoding: errorText, as: UTF8.self).trimmingCharacters(in: .whitespacesAndNewlines)
                    fail(detail.isEmpty ? "Arduino Q disconnected. Reconnect to continue." : String(detail.prefix(350)))
                }
                break
            }
        }
    }
}
