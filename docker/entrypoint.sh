#!/bin/bash
set -e

CARLA_DIST_DIR="/carla/PythonAPI/carla/dist"
CARLA_INSTALL_MARKER="/tmp/.carla_python_installed"

install_carla_python() {
    # Find the .egg file for Python 3.8
    EGG_FILE=$(find "$CARLA_DIST_DIR" -name "carla-*py3.8*.egg" 2>/dev/null | head -n 1)

    if [ -z "$EGG_FILE" ]; then
        echo "[entrypoint] WARNING: No carla egg file for Python 3.8 found in $CARLA_DIST_DIR"
        echo "[entrypoint] Skipping CARLA Python installation."
        return
    fi

    echo "[entrypoint] Found egg: $EGG_FILE"

    EXTRACT_DIR="/tmp/carla_egg_extracted"
    rm -rf "$EXTRACT_DIR"
    mkdir -p "$EXTRACT_DIR"

    # Extract the egg (it's a zip archive)
    unzip -q "$EGG_FILE" -d "$EXTRACT_DIR"

    # Write a minimal setup.py
    CARLA_VERSION=$(basename "$EGG_FILE" | sed 's/carla-\([0-9.]*\)-.*/\1/')
    cat > "$EXTRACT_DIR/setup.py" <<EOF
from distutils.core import setup
setup(
    name='carla',
    version='${CARLA_VERSION}',
    packages=['carla'],
)
EOF

    # Install with pip3.8 in editable mode (no --rm so reinstall is safe)
    pip3.8 install -e "$EXTRACT_DIR" --quiet

    echo "[entrypoint] CARLA Python $CARLA_VERSION installed for Python 3.8."
    touch "$CARLA_INSTALL_MARKER"
}

# Only install if not already done in this container session
if [ ! -f "$CARLA_INSTALL_MARKER" ]; then
    if [ -d "$CARLA_DIST_DIR" ]; then
        install_carla_python
    else
        echo "[entrypoint] CARLA dist directory not found ($CARLA_DIST_DIR). Is /carla mounted?"
    fi
fi

exec "$@"
