#!/usr/bin/env bash
set -euo pipefail

APP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$APP_DIR"

JAVA_CMD=""
JAVAC_CMD=""

if [[ -n "${JAVA_HOME:-}" ]]; then
    [[ -x "$JAVA_HOME/bin/java" ]] && JAVA_CMD="$JAVA_HOME/bin/java"
    [[ -x "$JAVA_HOME/bin/javac" ]] && JAVAC_CMD="$JAVA_HOME/bin/javac"
fi

[[ -z "$JAVA_CMD" ]] && JAVA_CMD="$(command -v java || true)"
[[ -z "$JAVAC_CMD" ]] && JAVAC_CMD="$(command -v javac || true)"

if [[ -z "$JAVA_CMD" || -z "$JAVAC_CMD" ]]; then
    echo "[ERROR] Missing java/javac."
    echo "[HINT] Install JDK 17 or newer, then set JAVA_HOME or add java and javac to PATH."
    exit 1
fi

JAVA2D_MODE="${AGV_JAVA2D_MODE:-safe}"
JAVA2D_OPTS=()
case "$JAVA2D_MODE" in
    safe)
        JAVA2D_OPTS=(-Dsun.java2d.opengl=false -Dsun.java2d.xrender=true -Dsun.java2d.pmoffscreen=false)
        ;;
    software)
        JAVA2D_OPTS=(-Dsun.java2d.opengl=false -Dsun.java2d.xrender=false -Dsun.java2d.pmoffscreen=false)
        ;;
    opengl)
        JAVA2D_OPTS=(-Dsun.java2d.opengl=True -Dsun.java2d.xrender=false -Dsun.java2d.pmoffscreen=false)
        ;;
    default)
        JAVA2D_OPTS=()
        ;;
    *)
        echo "[ERROR] Invalid AGV_JAVA2D_MODE=$JAVA2D_MODE. Use safe, software, opengl, or default."
        exit 1
        ;;
esac

PYTHON_CMD=""
if [[ -x ".venv/bin/python" ]]; then
    PYTHON_CMD=".venv/bin/python"
elif command -v python3 >/dev/null 2>&1; then
    PYTHON_CMD="python3"
elif command -v python >/dev/null 2>&1; then
    PYTHON_CMD="python"
fi

if [[ -n "$PYTHON_CMD" ]]; then
    if ! "$PYTHON_CMD" -c "import pygame" >/dev/null 2>&1; then
        echo "[WARN] pygame is not installed for $PYTHON_CMD. PS5 controller script will fail until you install it."
        echo "[HINT] Ubuntu: python3 -m venv .venv && . .venv/bin/activate && python -m pip install -r scripts/requirements.txt"
    fi
    export AGV_PYTHON="$PYTHON_CMD"
else
    echo "[WARN] Python was not found. GUI can start, but PS5 controller script will not."
fi

mkdir -p build/classes uart_temp
find src -name '*.java' -print > build/sources.txt

"$JAVAC_CMD" --release 17 -encoding UTF-8 \
    -cp "lib/jSerialComm-2.10.4.jar" \
    -d build/classes \
    @build/sources.txt

"$JAVA_CMD" "${JAVA2D_OPTS[@]}" \
    -Djava.io.tmpdir=uart_temp \
    --enable-native-access=ALL-UNNAMED \
    -Djava.library.path=lib \
    -cp "build/classes:lib/jSerialComm-2.10.4.jar" \
    app.App
