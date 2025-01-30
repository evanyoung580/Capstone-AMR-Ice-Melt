# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Update package lists
echo "Updating package lists..."
sudo apt update -y

# Install Git
if command_exists git; then
    echo "Git is already installed."
else
    echo "Installing Git..."
    sudo apt install git -y
fi

# Install qterminal
if command_exists qterminal; then
    echo "qterminal is already installed."
else
    echo "Installing qterminal..."
    sudo apt install qterminal -y
fi

# Install Python3 and pip
if command_exists python3; then
    echo "Python3 is already installed."
else
    echo "Installing Python3..."
    sudo apt install python3 -y
fi

if command_exists pip3; then
    echo "pip3 is already installed."
else
    echo "Installing pip3..."
    sudo apt install python3-pip -y
fi

# Install C++ compiler (g++)
if command_exists g++; then
    echo "C++ compiler (g++) is already installed."
else
    echo "Installing C++ compiler (g++)..."
    sudo apt install g++ -y
fi

# Install CMake
if command_exists cmake; then
    echo "CMake is already installed."
else
    echo "Installing CMake..."
    sudo apt install cmake -y
fi

# Install YAML libraries for C++
if [ -f /usr/include/yaml-cpp/yaml.h ]; then
    echo "YAML library for C++ is already installed."
else
    echo "Installing YAML library for C++..."
    sudo apt install libyaml-cpp-dev -y
fi

# Install YAML libraries for Python
if python3 -c "import yaml" >/dev/null 2>&1; then
    echo "YAML library for Python is already installed."
else
    echo "Installing YAML library for Python..."
    pip3 install pyyaml
fi

# Install JSON library for C++
if [ -f /usr/include/nlohmann/json.hpp ]; then
    echo "JSON library for C++ is already installed."
else
    echo "installing JSON library for C++..."
    sudo apt install nlohmann-json3-dev

fi

# Install ZMQ library for C++
if [ -f /usr/include/zmq.h ]; then
    echo "ZMQ library for C++ is already installed."
else
    echo "Installing ZMQ library for C++..."
    sudo apt install libzmq3-dev -y
fi

# Install ZMQ library for Python
if python3 -c "import zmq" >/dev/null 2>&1; then
    echo "ZMQ library for Python is already installed."
else
    echo "Installing ZMQ library for Python..."
    pip install pyzmq --break-system-packages
fi

# Install OpenCV with contrib modules for Python
if python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -q "contrib" >/dev/null 2>&1; then
    echo "OpenCV with contrib modules is already installed."
else
    echo "Installing OpenCV with contrib modules for Python..."
    pip install opencv-contrib-python --break-system-packages
fi

echo "All required dependencies have been checked and installed if necessary."

echo "Setup complete."
