#!/bin/bash
set -e

# === Configuration ===
VENV_PATH="/root/ros2_driver_venv"

echo "🧹 Removing old environment (if any)..."
rm -rf "$VENV_PATH"

echo "🐍 Creating new Python virtual environment..."
python3 -m venv "$VENV_PATH"
source "$VENV_PATH/bin/activate"

echo "⬇️ Installing required dependencies..."
pip install --upgrade pip
pip install \
    numpy \
    pyyaml \
    jinja2 \
    typeguard \
    "roslibpy==1.6.0" \
    autobahn \
    twisted \
    cryptography \
    service-identity \
    pyopenssl \
    attrs \
    typing-extensions \
    txaio \
    zope-interface

echo "✅ Verifying core modules..."
python3 - <<'PY'
import roslibpy.actionlib, numpy, yaml, jinja2, typeguard
print("✅ Environment setup successful — all modules loaded.")
PY

echo "📦 To activate later, run:"
echo "    source $VENV_PATH/bin/activate"
