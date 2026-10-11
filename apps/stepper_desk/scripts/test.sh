#!/bin/bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")/.." && pwd)"
TEMP="$(mktemp -d /tmp/stepper-q-tests.XXXXXX)"
trap 'rm -rf "$TEMP"' EXIT
c++ -std=c++17 "$ROOT/tests/motion_test.cpp" -o "$TEMP/motion"
"$TEMP/motion"
c++ -std=c++17 "$ROOT/tests/sensors_test.cpp" -o "$TEMP/sensors"
"$TEMP/sensors"
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
c++ -std=c++17 "$ROOT/tests/limits_test.cpp" -o "$TEMP/limits"
"$TEMP/limits"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/LimitTests.swift" -o "$TEMP/limits-ui"
"$TEMP/limits-ui"

c++ -std=c++17 "$ROOT/tests/power_test.cpp" -o "$TEMP/power"
"$TEMP/power"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/PowerTests.swift" -o "$TEMP/power-ui"
"$TEMP/power-ui"

c++ -std=c++17 "$ROOT/tests/trajectory_test.cpp" -o "$TEMP/trajectory"
"$TEMP/trajectory"
xcrun swiftc -swift-version 5 -parse-as-library -module-cache-path /tmp/stepper-swift-cache "$ROOT/mac/SSHLink.swift" "$ROOT/mac/Controller.swift" "$ROOT/mac/ArmModel.swift" "$ROOT/mac/PoseController.swift" "$ROOT/mac/PoseTests.swift" -o "$TEMP/pose"
"$TEMP/pose"

c++ -std=c++17 "$ROOT/tests/counted_jog_test.cpp" -o "$TEMP/counted-jog"
"$TEMP/counted-jog"
c++ -std=c++17 -O1 "$ROOT/tests/servo_test.cpp" -o "$TEMP/servo"
"$TEMP/servo"
