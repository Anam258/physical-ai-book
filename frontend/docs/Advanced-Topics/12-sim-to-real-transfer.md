
# Sim-to-Real Transfer

:::info Learning Objectives
* Understand the challenges and techniques for sim-to-real transfer
* Implement domain randomization to improve transferability
* Apply system identification methods to bridge the sim-to-real gap
* Evaluate and validate robot behaviors across simulation and real platforms
:::
![Hero Diagram: Sim-to-Real Transfer](../../static/img/sim-to-real.jpg)

Sim-to-real transfer is the process of developing and training robotic systems in simulation environments and successfully deploying them on physical robots. This approach significantly reduces development time and costs while ensuring safety during algorithm development. However, the "reality gap" between simulation and the real world presents significant challenges.

## Understanding the Reality Gap

The reality gap encompasses differences between simulated and real environments that can cause algorithms trained in simulation to fail when deployed on physical robots:

- **Visual differences**: Lighting, textures, and sensor noise
- **Physical differences**: Friction, compliance, and dynamics
- **Temporal differences**: Timing and latency variations
- **Sensor differences**: Noise, resolution, and accuracy variations

### Key Components of the Reality Gap

```python
# Simulation parameters
SIMULATION_PARAMS = {
    'gravity': 9.81,  # Perfectly known
    'friction': 0.5,  # Constant value
    'sensor_noise': 0.0,  # Noise-free sensors
    'timing': 0.0  # Perfect timing
}

# Real-world parameters (variable and uncertain)
REAL_WORLD_PARAMS = {
    'gravity': 9.81 + random.uniform(-0.05, 0.05),
    'friction': [0.3, 0.7],  # Range of possible values
    'sensor_noise': measured_noise_profile,
    'timing': measured_latency_distribution
}
```

:::tip Key Concept
Domain randomization is a key technique for reducing the sim-to-real gap by training models across a wide range of randomized simulation conditions, making them robust to variations encountered in the real world.
:::

## Domain Randomization Techniques

Domain randomization systematically varies simulation parameters to improve transferability:

### Visual Domain Randomization

```python
class VisualRandomizer:
    def __init__(self):
        self.lighting_range = (50, 200)  # Light intensity range
        self.texture_set = ['wood', 'metal', 'fabric', 'concrete']
        self.color_range = (0.1, 0.9)    # Color variation

    def randomize_visuals(self, scene):
        """Randomize visual properties in the scene"""
        # Randomize lighting
        light_intensity = random.uniform(*self.lighting_range)
        scene.set_light_intensity(light_intensity)

        # Randomize textures
        random_texture = random.choice(self.texture_set)
        scene.apply_texture(random_texture)

        # Randomize colors
        random_color = [random.uniform(*self.color_range) for _ in range(3)]
        scene.set_object_colors(random_color)
```

### Physical Domain Randomization

```python
class PhysicsRandomizer:
    def __init__(self):
        self.mass_variance = 0.1  # ±10% mass variation
        self.friction_range = (0.3, 0.8)
        self.damping_range = (0.01, 0.1)

    def randomize_physics(self, robot_model):
        """Randomize physical properties of the robot"""
        # Randomize link masses
        for link in robot_model.links:
            original_mass = link.mass
            randomized_mass = original_mass * random.uniform(
                1 - self.mass_variance,
                1 + self.mass_variance
            )
            link.mass = randomized_mass

        # Randomize friction coefficients
        for joint in robot_model.joints:
            joint.friction = random.uniform(*self.friction_range)
```

## System Identification for Model Refinement

System identification helps bridge the sim-to-real gap by identifying real-world parameters:

```python
import numpy as np
from scipy.optimize import minimize

class SystemIdentifier:
    def __init__(self, simulation_model):
        self.sim_model = simulation_model
        self.real_data = []

    def collect_real_data(self, robot, trajectory):
        """Collect real-world data for system identification"""
        states = []
        actions = []

        for action in trajectory:
            robot.execute_action(action)
            state = robot.get_state()
            states.append(state)
            actions.append(action)

        return states, actions

    def identify_parameters(self, real_states, real_actions):
        """Identify simulation parameters that match real behavior"""
        def parameter_error(params):
            # Set simulation parameters
            self.sim_model.set_params(params)

            # Simulate the same trajectory
            sim_states = self.sim_model.simulate(real_actions)

            # Calculate error between real and simulated states
            error = np.mean([
                np.linalg.norm(real - sim)
                for real, sim in zip(real_states, sim_states)
            ])
            return error

        # Optimize parameters to minimize error
        result = minimize(parameter_error,
                         x0=self.sim_model.get_default_params(),
                         method='BFGS')

        return result.x
```

## Validation and Evaluation Methods

Validating sim-to-real transfer requires systematic evaluation:

### Performance Metrics

```python
class TransferEvaluator:
    def __init__(self):
        self.metrics = {
            'success_rate': 0,
            'task_completion_time': float('inf'),
            'energy_efficiency': 0,
            'safety_violations': 0
        }

    def evaluate_transfer(self, sim_policy, real_robot):
        """Evaluate policy transfer performance"""
        results = []

        for task in self.task_set:
            # Execute task in simulation
            sim_result = self.execute_in_simulation(sim_policy, task)

            # Execute task on real robot
            real_result = self.execute_on_robot(sim_policy, task)

            # Compare results
            transfer_quality = self.compare_results(sim_result, real_result)
            results.append(transfer_quality)

        return self.aggregate_results(results)
```

## Advanced Transfer Techniques

### Domain Adaptation

```python
# Using domain adaptation networks
import torch
import torch.nn as nn

class DomainAdaptationNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.feature_extractor = nn.Sequential(
            nn.Linear(768, 512),  # Vision features
            nn.ReLU(),
            nn.Linear(512, 256),
            nn.ReLU()
        )

        self.sim_classifier = nn.Linear(256, 2)  # Sim vs Real
        self.task_classifier = nn.Linear(256, 10)  # Task classes

    def forward(self, x):
        features = self.feature_extractor(x)
        sim_pred = self.sim_classifier(features)
        task_pred = self.task_classifier(features)
        return features, sim_pred, task_pred
```

### Fine-Tuning Strategies

```python
def fine_tune_for_real(robot, sim_policy, real_episodes=10):
    """Fine-tune simulation-trained policy on real robot"""
    policy = copy.deepcopy(sim_policy)

    for episode in range(real_episodes):
        # Collect small amount of real data
        real_data = collect_real_experience(robot, policy)

        # Update policy with real data
        policy.update(real_data)

        # Evaluate updated policy
        success_rate = evaluate_policy(robot, policy)

        if success_rate > 0.9:  # Sufficient performance
            break

    return policy
```

## Hands-on Lab

Implement sim-to-real transfer techniques with:
1. Domain randomization in simulation
2. System identification for parameter estimation
3. Policy evaluation across simulation and real platforms
4. Fine-tuning strategies for improved transfer
5. Performance comparison and analysis

## Self-Assessment

1. What is the "reality gap" and why does it occur?
2. How does domain randomization help with sim-to-real transfer?
3. What are the key challenges in validating sim-to-real transfer?