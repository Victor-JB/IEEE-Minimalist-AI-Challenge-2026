#!/bin/zsh
set -eu
MAC_DIR="$(cd "$(dirname "$0")" && pwd)"
APP_DIR="$MAC_DIR/Stepper Desk Q.app"
mkdir -p "$APP_DIR/Contents/MacOS" "$APP_DIR/Contents/Resources" /tmp/stepper-swift-cache
xcrun swiftc -swift-version 5 -O -parse-as-library -target "$(uname -m)-apple-macosx14.0" \
  -module-cache-path /tmp/stepper-swift-cache \
  "$MAC_DIR/SSHLink.swift" "$MAC_DIR/Controller.swift" "$MAC_DIR/ArmModel.swift" "$MAC_DIR/ArmCAD.swift" "$MAC_DIR/ArmView.swift" "$MAC_DIR/PoseController.swift" "$MAC_DIR/PoseView.swift" "$MAC_DIR/StepperDesk.swift" \
  -framework SwiftUI -framework AppKit -framework SceneKit -o "$APP_DIR/Contents/MacOS/StepperDeskQ"
cat > "$APP_DIR/Contents/Info.plist" <<'PLIST'
<?xml version="1.0" encoding="UTF-8"?>
<!DOCTYPE plist PUBLIC "-//Apple//DTD PLIST 1.0//EN" "http://www.apple.com/DTDs/PropertyList-1.0.dtd">
<plist version="1.0"><dict>
<key>CFBundleExecutable</key><string>StepperDeskQ</string>
<key>CFBundleIdentifier</key><string>dev.tongtong.stepperdeskq</string>
<key>CFBundleName</key><string>Stepper Desk Q</string>
<key>CFBundleDisplayName</key><string>Stepper Desk Q</string>
<key>CFBundlePackageType</key><string>APPL</string>
<key>CFBundleShortVersionString</key><string>1.4</string>
<key>CFBundleVersion</key><string>6</string>
<key>LSMinimumSystemVersion</key><string>14.0</string>
<key>NSHighResolutionCapable</key><true/>
</dict></plist>
PLIST
cp -R "$MAC_DIR/Resources/ArmCAD" "$APP_DIR/Contents/Resources/"
codesign --force --sign - "$APP_DIR"
printf 'Built %s\n' "$APP_DIR"
