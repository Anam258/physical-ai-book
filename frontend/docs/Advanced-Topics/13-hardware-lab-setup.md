![Hero Diagram: Hardware Lab Setup](../../static/img/hardware-lab.jpg)

:::info Learning Objectives
* Design a comprehensive humanoid robotics laboratory setup
* Configure safety systems and operational protocols
* Integrate multiple robotic platforms and simulation environments
* Establish networking and computing infrastructure for robotics research
:::

# Hardware Lab Setup

A well-designed humanoid robotics laboratory requires careful planning of physical infrastructure, safety systems, computing resources, and operational protocols. This setup must support both simulation and real-robot development while ensuring researcher safety and equipment protection.

## Laboratory Layout and Infrastructure

A humanoid robotics lab should be designed with distinct areas for different activities:

- **Robot workspace**: Open area for robot operation and testing
- **Computing cluster**: High-performance computing for simulation and AI
- **Development stations**: Workbenches for robot maintenance and assembly
- **Safety zones**: Protected areas for researchers during robot operation

### Physical Infrastructure Requirements

```yaml
# Laboratory infrastructure checklist
flooring:
  - non-slip surface
  - static dissipative (ESD-safe)
  - load-bearing capacity: >200 lbs/sq ft

power:
  - redundant power supplies
  - uninterruptible power supply (UPS)
  - dedicated circuits for robots
  - emergency power-off switches

networking:
  - high-speed ethernet (1 Gbps minimum)
  - WiFi 6 for wireless devices
  - dedicated ROS 2 network domain
  - network switches with QoS support

lighting:
  - adjustable LED lighting
  - minimal heat generation
  - color temperature: 4000K-5000K
  - emergency lighting backup
```

:::tip Key Concept
Safety is paramount in humanoid robotics labs. All installations must include emergency stop systems, safety barriers, and proper ventilation for equipment cooling.
:::

## Safety Systems and Protocols

Humanoid robots require comprehensive safety infrastructure:

### Emergency Systems

```python
# Safety monitoring system
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

class SafetyMonitor(Node):
    def __init__(self):
        super().__init__('safety_monitor')

        # Safety zones monitoring
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/safety_lidar',
            self.lidar_callback,
            10
        )

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            10
        )

        # Safety timer
        self.safety_timer = self.create_timer(0.1, self.check_safety)

    def lidar_callback(self, msg):
        """Monitor safety zones around robot"""
        self.safety_zones = self.analyze_lidar_data(msg)

    def check_safety(self):
        """Check if safety conditions are met"""
        if not self.are_safety_conditions_met():
            self.trigger_emergency_stop()

    def are_safety_conditions_met(self):
        """Evaluate safety conditions"""
        # Check if people are in danger zone
        if self.safety_zones['danger_zone'] > 0:
            return False

        # Check robot operational limits
        if self.is_robot_unstable():
            return False

        return True
```

### Physical Safety Infrastructure

- **Light curtains**: Photoelectric barriers around robot workspace
- **Safety mats**: Pressure-sensitive floor mats for zone detection
- **Emergency stops**: Multiple easily accessible E-Stop buttons
- **Safety barriers**: Physical barriers during high-risk operations

## Computing Infrastructure

Humanoid robotics requires significant computational resources:

### High-Performance Computing Cluster

```bash
# Example HPC configuration for robotics
computing_nodes:
  - node_name: "simulation-server"
    gpu: "NVIDIA RTX A6000 (48GB)"
    cpu: "AMD EPYC 7742 (64 cores)"
    ram: "256GB DDR4"
    storage: "2TB NVMe + 10TB HDD"

  - node_name: "ai-training"
    gpu: "NVIDIA A100 (80GB) x 4"
    cpu: "Intel Xeon Gold 6248R (40 cores)"
    ram: "512GB DDR4"
    storage: "4TB NVMe + 20TB HDD"

  - node_name: "robot-control"
    gpu: "NVIDIA RTX 4080 (16GB)"
    cpu: "Intel i9-13900K (24 cores)"
    ram: "64GB DDR4"
    storage: "1TB NVMe"
```

### Network Configuration

```yaml
# ROS 2 network configuration
network_settings:
  domain_id: 42  # Dedicated ROS 2 domain
  qos_profiles:
    sensor_data:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 5
    control_commands:
      reliability: reliable
      durability: transient_local
      history: keep_last
      depth: 1

# Network security
security:
  - VPN access for remote operations
  - Firewall rules for ROS 2 traffic
  - Network segmentation for safety
```

## Robot Integration and Testing

The lab setup must support multiple robot platforms:

### Robot Bringup System

```python
# Multi-robot bringup system
import subprocess
import time
from pathlib import Path

class RobotBringup:
    def __init__(self, robot_name, config_path):
        self.robot_name = robot_name
        self.config_path = Path(config_path)

    def bringup_robot(self):
        """Bring up robot with safety checks"""
        # 1. Check safety systems
        if not self.check_safety_systems():
            raise RuntimeError("Safety systems not ready")

        # 2. Launch robot drivers
        self.launch_drivers()

        # 3. Initialize ROS 2 nodes
        self.initialize_ros_nodes()

        # 4. Run system checks
        self.run_system_checks()

    def launch_drivers(self):
        """Launch robot-specific drivers"""
        driver_launch_file = self.config_path / f"{self.robot_name}_drivers.launch.py"
        subprocess.run(["ros2", "launch", str(driver_launch_file)])

    def run_system_checks(self):
        """Run comprehensive system checks"""
        checks = [
            self.check_joint_states,
            self.check_sensors,
            self.check_controllers,
            self.check_communication
        ]

        for check in checks:
            if not check():
                raise RuntimeError(f"System check failed: {check.__name__}")

    def check_joint_states(self):
        """Verify joint states are publishing"""
        # Implementation to check joint state publisher
        return True
```

## Operational Protocols

### Daily Operations Checklist

```markdown
## Pre-Operation Checklist

- [ ] Safety systems tested and operational
- [ ] Robot battery level >80%
- [ ] Computing systems operational
- [ ] Network connectivity verified
- [ ] Emergency procedures reviewed
- [ ] Workspace clear of obstacles

## Post-Operation Checklist

- [ ] Robot safely powered down
- [ ] Workspace cleaned and organized
- [ ] Data backed up
- [ ] Equipment secured
- [ ] Logbook updated
```

## Hands-on Lab

Set up a humanoid robotics lab with:
1. Physical infrastructure planning
2. Safety system installation and testing
3. Computing cluster configuration
4. Network setup for ROS 2 operations
5. Operational protocols implementation

## Self-Assessment

1. What are the key safety considerations for humanoid robotics labs?
2. How should computing resources be allocated for different robotics tasks?
3. What network configurations are essential for robotics operations?