![Hero Diagram: URDF for Humanoids](../../static/img/ros2-network3.jpg)

:::info Learning Objectives
* Understand the Unified Robot Description Format (URDF) for humanoid robots
* Create complex robot models with multiple joints and links
* Implement kinematic chains for bipedal locomotion
* Validate and visualize URDF models in RViz and Gazebo
:::

# ROS 2 Fundamentals 3: URDF for Humanoids

The Unified Robot Description Format (URDF) is an XML-based format used to describe robot models in ROS. For humanoid robots, URDF becomes particularly important as it defines the complex kinematic structure with multiple degrees of freedom required for bipedal locomotion and dexterous manipulation.

## Understanding URDF Structure

URDF describes robots in terms of **links** and *joints*. Links represent rigid bodies, while joints define the kinematic and dynamic relationships between links. A humanoid robot typically includes:

- **Links**: Head, torso, arms (upper/lower), hands, legs (upper/lower), feet
- **Joints**: Revolute (rotational), prismatic (linear), fixed connections

### Basic URDF Components

```xml
<?xml version="1.0"?>
<robot name="humanoid_robot">
  <!-- Links define rigid bodies -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.1 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
    </inertial>
  </link>

  <!-- Joints define connections between links -->
  <joint name="joint_name" type="revolute">
    <parent link="base_link"/>
    <child link="child_link"/>
    <origin xyz="0.1 0 0" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="100" velocity="1"/>
  </joint>
</robot>
```

:::tip Key Concept
URDF defines the kinematic structure of robots, which is essential for forward and inverse kinematics calculations needed for humanoid locomotion and manipulation tasks.
:::

## Humanoid-Specific Considerations

Humanoid robots require special attention to balance and locomotion. The center of mass must be carefully calculated and maintained within the support polygon during walking. URDF models must include:

- Accurate inertial properties for each link
- Proper joint limits to prevent self-collision
- Correct mass distribution for stable locomotion

### Multi-Body Structure for Humanoids

```xml
<!-- Example humanoid torso -->
<link name="torso">
  <visual>
    <geometry>
      <cylinder radius="0.08" length="0.3"/>
    </geometry>
    <material name="gray">
      <color rgba="0.5 0.5 0.5 1.0"/>
    </material>
  </visual>
</link>

<!-- Hip joint for leg attachment -->
<joint name="left_hip_joint" type="revolute">
  <parent link="torso"/>
  <child link="left_thigh"/>
  <origin xyz="0 -0.1 -0.15" rpy="0 0 0"/>
  <axis xyz="0 0 1"/>
  <limit lower="-1.57" upper="1.57" effort="200" velocity="2"/>
</joint>
```

## Validation and Visualization

URDF models should be validated for kinematic correctness and visualized before simulation:

```bash
# Check URDF for errors
check_urdf my_humanoid.urdf

# Visualize in RViz
ros2 run rviz2 rviz2

# Launch in Gazebo
ros2 launch gazebo_ros gazebo.launch.py
```

## Hands-on Lab

Create a simplified humanoid model with:
1. A torso and head
2. Two arms with shoulder, elbow, and wrist joints
3. Two legs with hip, knee, and ankle joints
4. Proper inertial properties for each link
5. Visualize your model in RViz

## Self-Assessment

1. What are the main components of a URDF file?
2. Why is accurate inertial information important for humanoid robots?
3. How do joints define the kinematic structure of a robot?