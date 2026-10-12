#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEMP="$(mktemp -d /tmp/stepper-q-tests.XXXXXX)"
trap 'rm -rf "$TEMP"' EXIT
# Firmware + arm API tests live with the module now.
bash "$ROOT/../../arm/scripts/test.sh"
python3 -m unittest discover -s "$ROOT/tests" -p 'test_*.py'
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/ControllerTests.swift" -o "$TEMP/controller"
"$TEMP/controller"
xcrun swiftc -swift-version 5 -D STEPPER_UI_TEST -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/ArmCAD.swift" "$ROOT/mac/ArmView.swift" "$ROOT/mac/PoseController.swift" "$ROOT/mac/PoseView.swift" "$ROOT/mac/StepperDesk.swift" "$ROOT/mac/UITests.swift" -framework SwiftUI -framework AppKit -framework SceneKit -o "$TEMP/ui"
"$TEMP/ui"

xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/SensorTests.swift" -o "$TEMP/sensor-ui"
"$TEMP/sensor-ui"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/ArmTests.swift" -o "$TEMP/arm"
"$TEMP/arm"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/ArmCAD.swift" "$ROOT/mac/CADTests.swift" -framework SwiftUI -framework AppKit -framework SceneKit -o "$TEMP/cad"
STEPPER_CAD_RESOURCES="$ROOT/mac/Resources/ArmCAD" "$TEMP/cad"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/LimitTests.swift" -o "$TEMP/limits-ui"
"$TEMP/limits-ui"

xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/PowerTests.swift" -o "$TEMP/power-ui"
"$TEMP/power-ui"

xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/PoseController.swift" "$ROOT/mac/PoseTests.swift" -o "$TEMP/pose"
"$TEMP/pose"

