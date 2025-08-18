#!/usr/bin/env bash
set -euo pipefail
IFS=$'\n\t'

echo "? Updating system packages..."
sudo apt update && sudo apt full-upgrade -y

echo "? Installing system dependencies..."
sudo apt install -y \
  python3 python3-venv python3-pip \
  python3-opencv python3-picamera2 \
  libatlas-base-dev libcap-dev \
  libcamera-apps libcamera-dev libcamera-tools \
  python3-libcamera

# Create virtual environment if it doesn't exist
if [[ ! -d RunClub ]]; then
  echo "? Creating Python virtual environment 'RunClub'..."
  python3 -m venv RunClub --system-site-packages
fi

# Activate the virtual environment
echo "?? Activating virtual environment and upgrading pip..."
source RunClub/bin/activate
pip install --upgrade pip

# No packages to install since requirements.txt is empty
#echo "? No Python packages to install (requirements.txt is empty)."

echo "? Setup complete. Run 'source RunClub/bin/activate' to activate the environment."
