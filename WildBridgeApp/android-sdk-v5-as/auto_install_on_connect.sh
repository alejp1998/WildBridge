#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
VARIANT="current"
BUILD=false
CHECK_ONLY=false

for arg in "$@"; do
  case "$arg" in
    --build)
      BUILD=true
      ;;
    --check)
      CHECK_ONLY=true
      ;;
    current)
      VARIANT="current"
      ;;
    demoBiomass|demo_biomass)
      VARIANT="demoBiomass"
      ;;
    *)
      echo "Usage: $0 [current|demoBiomass|demo_biomass] [--build] [--check]" >&2
      exit 1
      ;;
  esac
done

if [[ "$VARIANT" == "demoBiomass" ]]; then
  PACKAGE_NAME="com.dji.sampleV5.aircraft.demo_biomass"
else
  PACKAGE_NAME="com.dji.sampleV5.aircraft"
fi
TASK_NAME="assemble${VARIANT^}Debug"
APK_PATH="$ROOT_DIR/../android-sdk-v5-sample/build/outputs/apk/$VARIANT/debug/sample-${VARIANT}Debug.apk"
LAUNCH_ACTIVITY="dji.sampleV5.aircraft.DJIAircraftMainActivity"

use_android_sdk() {
  export ANDROID_HOME="$1"
  export ANDROID_SDK_ROOT="$1"
  export PATH="$1/platform-tools:$PATH"
}

ensure_android_sdk_configured() {
  if [[ -n "${ANDROID_HOME:-}" && -d "$ANDROID_HOME" ]]; then
    use_android_sdk "$ANDROID_HOME"
    return
  fi
  if [[ -n "${ANDROID_SDK_ROOT:-}" && -d "$ANDROID_SDK_ROOT" ]]; then
    use_android_sdk "$ANDROID_SDK_ROOT"
    return
  fi
  if [[ -f "$ROOT_DIR/local.properties" ]] && grep -q '^sdk\.dir=' "$ROOT_DIR/local.properties"; then
    sdk_dir="$(grep -m1 '^sdk\.dir=' "$ROOT_DIR/local.properties" | cut -d= -f2- | tr -d '\r')"
    if [[ -d "$sdk_dir" ]]; then
      use_android_sdk "$sdk_dir"
      return
    fi

    echo "Android SDK location in $ROOT_DIR/local.properties does not exist: $sdk_dir" >&2
    echo "Set sdk.dir to the actual SDK path, or export ANDROID_HOME=/path/to/Android/Sdk." >&2
    exit 1
  fi

  echo "Android SDK location not configured." >&2
  echo "Create $ROOT_DIR/local.properties with a valid sdk.dir, for example:" >&2
  echo "  sdk.dir=$HOME/Android/Sdk" >&2
  echo "Or export ANDROID_HOME=/path/to/Android/Sdk before running this script." >&2
  echo "Template: $ROOT_DIR/local.properties.example" >&2
  exit 1
}

ensure_java_compiler_configured() {
  if [[ -n "${JAVA_HOME:-}" && -x "$JAVA_HOME/bin/javac" ]]; then
    return
  fi
  if command -v javac >/dev/null 2>&1; then
    return
  fi

  local_jdk="$HOME/.jdks/temurin-21"
  if [[ -x "$local_jdk/bin/javac" ]]; then
    export JAVA_HOME="$local_jdk"
    export PATH="$JAVA_HOME/bin:$PATH"
    return
  fi

  echo "Java compiler not found. Install a JDK or set JAVA_HOME to a JDK directory." >&2
  echo "Current Java runtime: $(command -v java || echo not found)" >&2
  exit 1
}

build_selected_variant() {
  ensure_android_sdk_configured
  ensure_java_compiler_configured
  "$ROOT_DIR/gradlew" -p "$ROOT_DIR" ":sample:$TASK_NAME" --warning-mode summary
}

if [[ "$CHECK_ONLY" == true ]]; then
  echo "Variant: $VARIANT"
  echo "Package: $PACKAGE_NAME"
  echo "APK: $APK_PATH"
  if [[ -f "$APK_PATH" ]]; then
    echo "APK status: found"
  else
    echo "APK status: missing"
  fi
  exit 0
fi

if [[ "$BUILD" == true ]]; then
  build_selected_variant
fi

if [[ ! -f "$APK_PATH" ]]; then
  echo "APK not found, building $VARIANT first: $APK_PATH" >&2
  build_selected_variant
fi

if [[ ! -f "$APK_PATH" ]]; then
  APK_PATH="$(find "$ROOT_DIR/../android-sdk-v5-sample/build/outputs/apk/$VARIANT" -type f -name "sample-${VARIANT}Debug.apk" -print -quit 2>/dev/null || true)"
fi

if [[ -z "$APK_PATH" || ! -f "$APK_PATH" ]]; then
  echo "APK not found after build for variant: $VARIANT" >&2
  echo "Expected under: $ROOT_DIR/../android-sdk-v5-sample/build/outputs/apk/$VARIANT" >&2
  exit 1
fi

ensure_android_sdk_configured

echo "Waiting for a DJI RC / Android device over ADB..."
adb wait-for-device

serial="$(adb get-serialno 2>/dev/null || true)"
model="$(adb shell getprop ro.product.model 2>/dev/null | tr -d '\r' || true)"
product="$(adb shell getprop ro.product.product.name 2>/dev/null | tr -d '\r' || true)"

echo "Device detected: serial=${serial:-unknown} model=${model:-unknown} product=${product:-unknown}"
echo "Installing $VARIANT: $APK_PATH"
adb install -r "$APK_PATH"

echo "Launching $PACKAGE_NAME"
adb shell am start -n "$PACKAGE_NAME/$LAUNCH_ACTIVITY" >/dev/null || \
  adb shell monkey -p "$PACKAGE_NAME" -c android.intent.category.LAUNCHER 1 >/dev/null

echo "Done. PID/control profile is selected inside the app from DJI ProductKey.KeyProductType."
