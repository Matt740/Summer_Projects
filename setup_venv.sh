#!/bin/bash

# Function to check if a package is installed
check_and_install() {
  PACKAGE=$1
  if dpkg -s "$PACKAGE" &> /dev/null; then
    echo "$PACKAGE is already installed."
  else
    echo "Installing $PACKAGE..."
    sudo apt install -y "$PACKAGE"
  fi
}

# Update package index
echo "Updating package list..."
sudo apt update

# Check and install system dependencies
check_and_install python3-venv
check_and_install libcap-dev
check_and_install libcamera-apps
check_and_install libcamera-dev
check_and_install libcamera-tools
check_and_install python3-libcamera  # if available
check_and_install libcamera

# Create virtual environment if it doesn't exist
if [ -d "myenv" ]; then
  echo "Virtual environment 'myenv' already exists."
else
  echo "Creating virtual environment 'myenv'..."
  python3 -m venv myenv
fi

# Print activation instructions
echo "To activate the virtual environment, run:"
echo "  source myenv/bin/activate"

# Install Python dependencies
echo "Installing/upgrading pip and Python packages from requirements.txt..."
myenv/bin/pip install --upgrade pip

if [ -f "requirements.txt" ]; then
  myenv/bin/pip install -r requirements.txt
else
  echo "No requirements.txt found — skipping package install."
fi

echo "✅ Setup complete."
