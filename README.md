# Klipper-ROS 2 Simulation Bridge

[![ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble-blue)](https://docs.ros.org/en/humble/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04-orange)](https://ubuntu.com/)
[![License](https://img.shields.io/badge/License-MIT-green.svg)](LICENSE)

A complete 3D printer simulation environment that bridges a simulated Klipper instance to ROS 2 Humble, enabling real-time data streaming without any physical hardware.

## 🎯 Features

- **Virtual 3D Printer Simulation**: Complete Klipper environment running on virtual MCU
- **Real-time ROS 2 Integration**: Stream toolhead position and temperature data to ROS 2 topics
- **Hardware-Free Development**: Perfect for developing and testing robotics applications that interact with 3D printers
- **Web Interface**: Full Mainsail dashboard for printer control and monitoring

## 📁 Project Structure

```
klipper_ros_simulation_ws/
├── klipper_config/
│   └── printer.cfg              # Validated Klipper configuration for simulation
└── src/
    └── klipper_ros2_bridge/     # ROS 2 package containing the bridge node
        ├── package.xml
        ├── setup.py
        └── klipper_ros2_bridge/
            └── klipper_bridge.py
```

## 🔧 Prerequisites

- **Operating System**: Ubuntu 22.04 LTS
- **ROS 2**: Humble Hawksbill (installed and sourced)
- **Klipper Stack**: Klipper, Moonraker, and Mainsail (recommend installation via [KIAUH](https://github.com/dw-0/kiauh))
- **Build Tools**: 
  ```bash
  sudo apt update
  sudo apt install git python3-pip build-essential
  ```

## 🚀 Installation

### Part 1: Configure the Klipper Environment

#### 1. Install Klipper Stack with KIAUH

```bash
# Clone and run KIAUH
git clone https://github.com/dw-0/kiauh.git
cd kiauh
./kiauh.sh
```

In KIAUH:
1. Select `1) [Install]`
2. Install `1) [Klipper]`, `2) [Moonraker]`, and `3) [Mainsail]`

#### 2. Build Virtual MCU Firmware

Configure Klipper for simulation with virtual sensors:

```bash
cd ~/klipper
make menuconfig
```

In the configuration menu:
- Set **Micro-controller Architecture** to `(Linux process)`
- Enable `[*] Enable extra low-level configuration options` (press Spacebar)
- Enable `[*] Host/MCU Temperature sensors`
- Press `Q` to quit, then `Y` to save

Build and install the firmware:

```bash
make
sudo make flash
```

#### 3. Deploy Klipper Configuration

Clone this repository and set up the configuration:

```bash
# Clone the repository
git clone <your-repository-url> klipper_ros_simulation_ws
cd klipper_ros_simulation_ws

# Ensure config directory exists
mkdir -p ~/printer_data/config

# Copy the printer configuration
cp ./klipper_config/printer.cfg ~/printer_data/config/
```

#### 4. Start Klipper Services

```bash
sudo systemctl start klipper-mcu
sudo systemctl start klipper
sudo systemctl start moonraker
sudo systemctl start nginx
```

✅ **Verification**: Navigate to `http://127.0.0.1` in your browser to see the Mainsail dashboard.

### Part 2: Build the ROS 2 Bridge

#### 1. Install Dependencies

```bash
# Install Python WebSocket library
pip install websockets

# Install ROS 2 dependencies
cd klipper_ros_simulation_ws
rosdep install -i --from-path src --rosdistro humble -y
```

#### 2. Build the Workspace

```bash
colcon build
```

## 🎮 Usage

### Running the Simulation

You'll need two terminals. Source the workspace in each:

**Terminal 1 - Launch the Bridge:**
```bash
source install/setup.bash
ros2 run klipper_ros2_bridge klipper_bridge
```

You should see: `Bridge started and connected to Moonraker's WebSocket`

**Terminal 2 - Monitor ROS 2 Topics:**
```bash
source install/setup.bash

# List available topics
ros2 topic list

# Monitor toolhead position in real-time
ros2 topic echo /klipper/toolhead/position
```

### Controlling the Virtual Printer

1. Open your browser and go to `http://127.0.0.1`
2. Use the Mainsail interface to:
   - Click "Home" to home all axes
   - Use axis controls to move the virtual toolhead
   - Monitor temperatures and other printer states

3. Watch real-time data streaming in Terminal 2 as you control the printer!

## 📊 Available ROS 2 Topics

| Topic | Message Type | Description |
|-------|--------------|-------------|
| `/klipper/toolhead/position` | `geometry_msgs/Point` | Real-time toolhead X, Y, Z coordinates |
| `/klipper/temperatures` | `sensor_msgs/Temperature` | Hotend and bed temperatures |

## 🛠️ Development

This simulation environment is perfect for:

- **Robotics Research**: Develop algorithms that need to interact with 3D printer state
- **Path Planning**: Test toolpath optimization without hardware
- **Multi-Robot Systems**: Integrate 3D printers into larger robotic workflows
- **Educational Projects**: Learn ROS 2 and 3D printer control safely

## 🔧 Troubleshooting

### Common Issues

**Bridge won't connect to Moonraker:**
- Ensure all Klipper services are running: `sudo systemctl status klipper moonraker`
- Check Moonraker is accessible: `curl http://127.0.0.1:7125/printer/info`

**No data on ROS topics:**
- Verify the bridge node is running without errors
- Check topic list: `ros2 topic list`
- Ensure workspace is properly sourced

**Mainsail shows "Printer not ready":**
- Restart Klipper services in order:
  ```bash
  sudo systemctl restart klipper-mcu
  sudo systemctl restart klipper
  sudo systemctl restart moonraker
  ```

### Logs and Debugging

- **Klipper logs**: `sudo journalctl -u klipper -f`
- **Moonraker logs**: `sudo journalctl -u moonraker -f`
- **Bridge logs**: Check terminal output where bridge is running

## 🤝 Contributing

Contributions are welcome! Please feel free to submit a Pull Request. For major changes, please open an issue first to discuss what you would like to change.

## 📝 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## 🙏 Acknowledgments

- [Klipper](https://github.com/Klipper3d/klipper) - The amazing 3D printer firmware
- [KIAUH](https://github.com/dw-0/kiauh) - Klipper Installation And Update Helper
- [ROS 2](https://ros.org/) - Robot Operating System 2

---

**Happy Printing! 🖨️🤖**
