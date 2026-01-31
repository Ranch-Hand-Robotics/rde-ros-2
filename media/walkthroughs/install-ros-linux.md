# Installing ROS 2 on Linux

To use the Robot Developer Extensions, you need to have ROS 2 installed on your system.

## Installation via apt (Recommended for Linux)

ROS 2 can be installed using the official apt repositories from ros.org.

### Step 1: Set up sources

```bash
sudo apt update && sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl gnupg lsb-release
```

### Step 2: Add the ROS 2 GPG key

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

### Step 3: Add the repository

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(source /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Step 4: Install ROS 2

```bash
sudo apt update
sudo apt install -y ros-kilted-desktop
```

Replace `kilted` with your desired ROS 2 distribution (e.g., `humble`, `iron`, `jazzy`, `kilted`).

### Step 5: Source the setup script

Add the following to your `~/.bashrc`:

```bash
source /opt/ros/kilted/setup.bash
```

## Verify Installation

After installation, verify that ROS 2 is properly installed:

```bash
ros2 --help
```

## Additional Resources

* [Official ROS 2 Installation Guide](https://docs.ros.org/en/kilted/Installation.html)
* [ROS 2 Documentation](https://docs.ros.org/en/kilted/index.html)

Once ROS 2 is installed, the extension will automatically detect it when you open a ROS 2 workspace!
