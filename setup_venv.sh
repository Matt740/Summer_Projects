#!/bin/bash

echo "? Updating system packages..."
sudo apt update && sudo apt upgrade -y

echo "? Installing system dependencies..."
sudo apt install -y \
    python3 \
    python3-venv \
    python3-pip \
    python3-opencv \
    python3-picamera2 \
    libatlas-base-dev

# Create the virtual environment if it doesn't already exist
if [ ! -d "myenv" ]; then
    echo "? Creating Python virtual environment..."
    python3 -m venv myenv
fi

# Activate the virtual environment
echo "? Activating virtual environment..."
source myenv/bin/activate

echo "? Upgrading pip..."
python3 -m ensurepip --upgrade
pip install --upgrade pip

echo "? Installing Python packages (excluding opencv-python and picamera2)..."
# Filter out incompatible packages before installing
grep -v -E '^(opencv-python|picamera2)' requirements.txt > filtered_requirements.txt
pip install -r filtered_requirements.txt
rm filtered_requirements.txt

echo "? Setup complete."
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
check_and_install python3
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
