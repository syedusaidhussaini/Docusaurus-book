# Chapter 6: Simulation-to-Reality Transfer

## Learning Objectives

After completing this chapter, you will be able to:
- Identify and quantify the reality gap in robotics simulation
- Implement domain randomization techniques
- Apply system identification methods to improve simulation accuracy
- Validate control algorithms across simulation and reality

## 6.1 Understanding the Reality Gap

The reality gap refers to the differences between simulated and real-world robot behavior. These differences can significantly impact the performance of algorithms developed in simulation when deployed to physical robots.

### Major Sources of the Reality Gap:
- **Modeling errors**: Inaccuracies in robot dynamics, kinematics, and physical properties
- **Sensor noise**: Differences in sensor characteristics and noise patterns
- **Actuator dynamics**: Delays, friction, and non-linearities in real actuators
- **Environmental factors**: Unmodeled external forces, surface properties, and disturbances
- **Computational delays**: Processing time and communication latencies

## 6.2 Quantifying Simulation Accuracy

To achieve successful transfer, we must quantify how closely our simulation matches reality:

### Physics Parameter Comparison

| Parameter | Simulation | Reality | Error (%) |
|-----------|------------|---------|-----------|
| Robot Mass | 25.0 kg | 24.8 kg | 0.8% |
| Leg Length | 0.85 m | 0.847 m | 0.4% |
| Max Torque (Hip) | 80 Nm | 78 Nm | 2.5% |
| IMU Noise (std dev) | 0.001 | 0.0012 | 20% |

### Performance Metrics for Transfer

```python
# simulation_reality_comparison.py
import numpy as np
from scipy.spatial.distance import euclidean
from sklearn.metrics import mean_squared_error

class SimulationRealityComparator:
    def __init__(self):
        self.simulation_data = []
        self.reality_data = []
        self.metrics = {}

    def add_data_pair(self, sim_data, real_data):
        """Add a pair of simulation and reality data points"""
        self.simulation_data.append(sim_data)
        self.reality_data.append(real_data)

    def calculate_performance_similarity(self):
        """Calculate similarity between simulation and reality performance"""
        if len(self.simulation_data) == 0 or len(self.reality_data) == 0:
            return 0.0

        # Convert to numpy arrays for easier computation
        sim_array = np.array(self.simulation_data)
        real_array = np.array(self.reality_data)

        # Calculate correlation coefficient
        correlation = np.corrcoef(sim_array.flatten(), real_array.flatten())[0, 1]

        # Calculate normalized root mean square error
        rmse = np.sqrt(mean_squared_error(sim_array, real_array))
        range_real = np.max(real_array) - np.min(real_array)
        nrmse = rmse / range_real if range_real != 0 else 0

        # Calculate similarity percentage (100% - NRMSE*100)
        similarity = max(0, (1 - nrmse) * 100)

        self.metrics['correlation'] = correlation
        self.metrics['nrmse'] = nrmse
        self.metrics['similarity'] = similarity

        return similarity

    def calculate_trajectory_similarity(self, sim_trajectory, real_trajectory):
        """Calculate similarity between two trajectories"""
        if len(sim_trajectory) != len(real_trajectory):
            # Interpolate to same length
            max_len = max(len(sim_trajectory), len(real_trajectory))
            sim_interp = np.interp(
                np.linspace(0, 1, max_len),
                np.linspace(0, 1, len(sim_trajectory)),
                sim_trajectory
            )
            real_interp = np.interp(
                np.linspace(0, 1, max_len),
                np.linspace(0, 1, len(real_trajectory)),
                real_trajectory
            )
        else:
            sim_interp = np.array(sim_trajectory)
            real_interp = np.array(real_trajectory)

        # Calculate Dynamic Time Warping distance
        dtw_distance = self.dtw_distance(sim_interp, real_interp)

        # Convert to similarity percentage
        max_possible_distance = np.sum(np.abs(sim_interp)) + np.sum(np.abs(real_interp))
        similarity = (1 - dtw_distance / max_possible_distance) * 100 if max_possible_distance > 0 else 100

        return similarity

    def dtw_distance(self, seq1, seq2):
        """Calculate Dynamic Time Warping distance between two sequences"""
        n, m = len(seq1), len(seq2)
        dtw_matrix = np.zeros((n + 1, m + 1))
        dtw_matrix[0, 1:] = np.inf
        dtw_matrix[1:, 0] = np.inf

        for i in range(1, n + 1):
            for j in range(1, m + 1):
                cost = abs(seq1[i-1] - seq2[j-1])
                dtw_matrix[i, j] = cost + min(
                    dtw_matrix[i-1, j],    # insertion
                    dtw_matrix[i, j-1],    # deletion
                    dtw_matrix[i-1, j-1]   # match
                )

        return dtw_matrix[n, m]
```

## 6.3 Domain Randomization Techniques

Domain randomization is a powerful technique to improve simulation-to-reality transfer by training algorithms across a wide range of randomized parameters:

### Physics Parameter Randomization

```python
# domain_randomization.py
import numpy as np
import random

class DomainRandomizer:
    def __init__(self):
        # Define parameter ranges for randomization
        self.param_ranges = {
            'robot_mass': (0.8, 1.2),  # Multiplier for base mass
            'link_length': (0.95, 1.05),  # Multiplier for link lengths
            'friction_coeff': (0.1, 1.0),  # Range for friction coefficients
            'restitution': (0.0, 0.3),  # Range for restitution coefficients
            'imu_noise_std': (0.0005, 0.002),  # Range for IMU noise
            'motor_delay': (0.001, 0.01),  # Range for motor delays (seconds)
            'gravity': (9.5, 9.9),  # Range for gravity (m/s^2)
        }

    def randomize_parameters(self):
        """Generate randomized parameters for simulation"""
        randomized_params = {}

        for param, (min_val, max_val) in self.param_ranges.items():
            randomized_params[param] = random.uniform(min_val, max_val)

        return randomized_params

    def apply_randomization(self, gazebo_model):
        """Apply randomization to Gazebo model parameters"""
        params = self.randomize_parameters()

        # Apply mass randomization
        for link in gazebo_model.links:
            original_mass = link.inertial.mass
            link.inertial.mass *= params['robot_mass']

        # Apply friction randomization
        for collision in gazebo_model.collisions:
            collision.surface.friction.ode.mu = params['friction_coeff']
            collision.surface.friction.ode.mu2 = params['friction_coeff']

        # Apply restitution randomization
        for collision in gazebo_model.collisions:
            collision.surface.bounce.restitution_coefficient = params['restitution']

        # Apply gravity randomization
        gazebo_model.world.physics.gravity = [0, 0, -params['gravity']]

        return params

    def systematic_randomization(self, episode_num, total_episodes):
        """Apply systematic randomization based on training progress"""
        # Increase randomization range as training progresses
        progress = episode_num / total_episodes
        randomization_strength = 0.1 + 0.9 * progress  # Start low, increase over time

        randomized_params = {}
        for param, (min_val, max_val) in self.param_ranges.items():
            # Calculate centered randomization
            center = (min_val + max_val) / 2
            range_width = (max_val - min_val) / 2
            variation = range_width * randomization_strength

            randomized_params[param] = random.uniform(
                center - variation,
                center + variation
            )
            # Ensure bounds are respected
            randomized_params[param] = max(min_val, min(max_val, randomized_params[param]))

        return randomized_params
```

## 6.4 System Identification for Model Improvement

System identification helps us create more accurate simulation models by fitting parameters to real-world data:

### Joint-Level System Identification

```python
# system_identification.py
import numpy as np
from scipy.optimize import minimize
from scipy import signal

class JointSystemIdentifier:
    def __init__(self, joint_name):
        self.joint_name = joint_name
        self.model_params = {}
        self.identification_data = []

    def collect_data(self, input_torques, measured_positions, measured_velocities):
        """Collect input-output data for system identification"""
        self.identification_data.append({
            'input_torques': np.array(input_torques),
            'measured_positions': np.array(measured_positions),
            'measured_velocities': np.array(measured_velocities)
        })

    def dynamic_model(self, params, torque, position, velocity):
        """
        Dynamic model: I*ddtheta + B*dtheta + K*theta = tau
        Where:
        - I: Inertia
        - B: Damping/friction coefficient
        - K: Stiffness coefficient
        - tau: Applied torque
        """
        I, B, K = params
        ddtheta = (torque - B * velocity - K * position) / I
        return ddtheta

    def objective_function(self, params, data):
        """Objective function to minimize during parameter fitting"""
        total_error = 0
        dt = 0.001  # Integration time step

        for sample in data:
            torque = sample['input_torques']
            true_pos = sample['measured_positions']
            true_vel = sample['measured_velocities']

            # Simulate the model with current parameters
            est_pos = np.zeros_like(true_pos)
            est_vel = np.zeros_like(true_vel)

            # Initialize with first measured values
            est_pos[0] = true_pos[0]
            est_vel[0] = true_vel[0]

            # Integrate the model forward
            for i in range(1, len(torque)):
                ddtheta = self.dynamic_model(params, torque[i-1], est_pos[i-1], est_vel[i-1])
                est_vel[i] = est_vel[i-1] + ddtheta * dt
                est_pos[i] = est_pos[i-1] + est_vel[i] * dt

            # Calculate error
            pos_error = np.mean((est_pos - true_pos) ** 2)
            vel_error = np.mean((est_vel - true_vel) ** 2)
            total_error += pos_error + vel_error

        return total_error

    def identify_parameters(self):
        """Identify dynamic parameters using collected data"""
        # Initial guess for [I, B, K]
        initial_params = [0.1, 0.01, 0.1]  # [Inertia, Damping, Stiffness]

        # Optimize parameters
        result = minimize(
            self.objective_function,
            initial_params,
            args=(self.identification_data),
            method='L-BFGS-B',
            options={'ftol': 1e-9, 'gtol': 1e-9}
        )

        self.model_params = {
            'inertia': result.x[0],
            'damping': result.x[1],
            'stiffness': result.x[2],
            'optimization_success': result.success,
            'cost': result.fun
        }

        return self.model_params

    def update_simulation_model(self, gazebo_model):
        """Update Gazebo model with identified parameters"""
        # This would modify the URDF or SDF parameters in the simulation
        print(f"Updating {self.joint_name} with identified parameters:")
        print(f"  Inertia: {self.model_params['inertia']}")
        print(f"  Damping: {self.model_params['damping']}")
        print(f"  Stiffness: {self.model_params['stiffness']}")

        # In practice, this would modify the Gazebo model's inertial properties
        # and joint dynamics parameters
```

## 6.5 Control Algorithm Validation Framework

Create a framework to validate control algorithms across simulation and reality:

```python
# validation_framework.py
import numpy as np
import matplotlib.pyplot as plt
from datetime import datetime

class TransferValidationFramework:
    def __init__(self):
        self.simulation_results = []
        self.reality_results = []
        self.validation_metrics = {}
        self.test_scenarios = []

    def add_test_scenario(self, name, description, success_criteria):
        """Add a test scenario for validation"""
        self.test_scenarios.append({
            'name': name,
            'description': description,
            'success_criteria': success_criteria,
            'sim_results': [],
            'real_results': []
        })

    def run_simulation_test(self, scenario_name, controller, environment_params=None):
        """Run a test in simulation"""
        scenario = next((s for s in self.test_scenarios if s['name'] == scenario_name), None)
        if not scenario:
            raise ValueError(f"Scenario {scenario_name} not found")

        # Apply environment parameters to simulation
        if environment_params:
            self.apply_environment_params(environment_params)

        # Run the controller in simulation
        results = self.execute_controller(controller, 'simulation')

        scenario['sim_results'].append({
            'timestamp': datetime.now(),
            'results': results,
            'environment_params': environment_params
        })

        return results

    def run_reality_test(self, scenario_name, controller):
        """Run a test on the real robot"""
        scenario = next((s for s in self.test_scenarios if s['name'] == scenario_name), None)
        if not scenario:
            raise ValueError(f"Scenario {scenario_name} not found")

        # Run the controller on the real robot
        results = self.execute_controller(controller, 'reality')

        scenario['real_results'].append({
            'timestamp': datetime.now(),
            'results': results
        })

        return results

    def execute_controller(self, controller, environment):
        """Execute a controller in the specified environment"""
        # This would interface with either simulation or real robot
        # Return performance metrics and trajectory data
        pass

    def calculate_transfer_metrics(self, scenario_name):
        """Calculate transfer metrics for a specific scenario"""
        scenario = next((s for s in self.test_scenarios if s['name'] == scenario_name), None)
        if not scenario or not scenario['sim_results'] or not scenario['real_results']:
            return None

        sim_result = scenario['sim_results'][-1]['results']
        real_result = scenario['real_results'][-1]['results']

        # Calculate various transfer metrics
        metrics = {
            'performance_similarity': self.calculate_performance_similarity(sim_result, real_result),
            'trajectory_similarity': self.calculate_trajectory_similarity(sim_result['trajectory'], real_result['trajectory']),
            'success_rate': self.calculate_success_rate(sim_result, real_result, scenario['success_criteria']),
            'transfer_gap': self.calculate_transfer_gap(sim_result, real_result)
        }

        return metrics

    def calculate_performance_similarity(self, sim_result, real_result):
        """Calculate overall performance similarity"""
        # Compare key performance indicators
        sim_performance = sim_result.get('performance_metrics', {})
        real_performance = real_result.get('performance_metrics', {})

        similarities = []
        for key in sim_performance.keys():
            if key in real_performance:
                sim_val = sim_performance[key]
                real_val = real_performance[key]
                if real_val != 0:
                    similarity = 1 - abs(sim_val - real_val) / abs(real_val)
                    similarities.append(max(0, similarity))  # Ensure non-negative

        return np.mean(similarities) if similarities else 0.0

    def generate_validation_report(self, scenario_name):
        """Generate a comprehensive validation report"""
        scenario = next((s for s in self.test_scenarios if s['name'] == scenario_name), None)
        if not scenario:
            return "Scenario not found"

        metrics = self.calculate_transfer_metrics(scenario_name)
        if not metrics:
            return "No validation data available"

        report = f"""
        Validation Report: {scenario['name']}
        ========================================

        Description: {scenario['description']}

        Transfer Metrics:
        - Performance Similarity: {metrics['performance_similarity']:.2%}
        - Trajectory Similarity: {metrics['trajectory_similarity']:.2%}
        - Success Rate: {metrics['success_rate']:.2%}
        - Transfer Gap: {metrics['transfer_gap']:.4f}

        Success Criteria: {scenario['success_criteria']}

        Recommendation: {'SUCCESS' if metrics['performance_similarity'] > 0.8 else 'REQUIRES IMPROVEMENT'}
        """

        return report

    def visualize_transfer_results(self, scenario_name):
        """Create visualizations comparing simulation and reality results"""
        scenario = next((s for s in self.test_scenarios if s['name'] == scenario_name), None)
        if not scenario:
            return

        if not scenario['sim_results'] or not scenario['real_results']:
            print("No results to visualize")
            return

        sim_result = scenario['sim_results'][-1]['results']
        real_result = scenario['real_results'][-1]['results']

        fig, axes = plt.subplots(2, 2, figsize=(15, 10))

        # Plot 1: Trajectory comparison
        axes[0, 0].plot(sim_result['trajectory'][:, 0], sim_result['trajectory'][:, 1], label='Simulation', linewidth=2)
        axes[0, 0].plot(real_result['trajectory'][:, 0], real_result['trajectory'][:, 1], label='Reality', linewidth=2)
        axes[0, 0].set_title('Trajectory Comparison')
        axes[0, 0].set_xlabel('X Position')
        axes[0, 0].set_ylabel('Y Position')
        axes[0, 0].legend()
        axes[0, 0].grid(True)

        # Plot 2: Performance over time
        time_sim = np.arange(len(sim_result['performance']))
        time_real = np.arange(len(real_result['performance']))
        axes[0, 1].plot(time_sim, sim_result['performance'], label='Simulation', linewidth=2)
        axes[0, 1].plot(time_real, real_result['performance'], label='Reality', linewidth=2)
        axes[0, 1].set_title('Performance Over Time')
        axes[0, 1].set_xlabel('Time Step')
        axes[0, 1].set_ylabel('Performance')
        axes[0, 1].legend()
        axes[0, 1].grid(True)

        # Plot 3: Control effort comparison
        axes[1, 0].plot(sim_result['control_effort'], label='Simulation', linewidth=2)
        axes[1, 0].plot(real_result['control_effort'], label='Reality', linewidth=2)
        axes[1, 0].set_title('Control Effort Comparison')
        axes[1, 0].set_xlabel('Time Step')
        axes[1, 0].set_ylabel('Control Effort')
        axes[1, 0].legend()
        axes[1, 0].grid(True)

        # Plot 4: Error analysis
        error = np.abs(np.array(sim_result['performance'])[:min(len(sim_result['performance']), len(real_result['performance']))] -
                      np.array(real_result['performance'])[:min(len(sim_result['performance']), len(real_result['performance']))])
        axes[1, 1].plot(error, linewidth=2, color='red')
        axes[1, 1].set_title('Performance Error Over Time')
        axes[1, 1].set_xlabel('Time Step')
        axes[1, 1].set_ylabel('Absolute Error')
        axes[1, 1].grid(True)

        plt.tight_layout()
        plt.show()
```

## 6.6 Best Practices for Simulation-to-Reality Transfer

### 1. Gradual Complexity Increase
Start with simple tasks in simulation and gradually increase complexity before moving to reality.

### 2. Sensor Noise Modeling
Accurately model sensor noise, delays, and limitations in simulation to match real sensors.

### 3. Actuator Dynamics
Include realistic actuator dynamics, including delays, friction, and saturation limits.

### 4. Environmental Randomization
Use domain randomization to train controllers that are robust to environmental variations.

### 5. Validation Framework
Implement systematic validation procedures to measure transfer performance.

### 6. Iterative Improvement
Continuously update simulation models based on real-world data to reduce the reality gap.

## Exercises

1. Implement a domain randomization system for a simple humanoid walking task
2. Perform system identification on a simulated joint and compare with real robot data
3. Create a validation framework to test a balance controller across simulation and reality
4. Design an adaptive control system that adjusts parameters based on simulation-to-reality performance

## Summary

This chapter covered essential techniques for achieving successful simulation-to-reality transfer in humanoid robotics. You've learned how to quantify the reality gap, implement domain randomization, perform system identification, and validate control algorithms across simulation and reality. These techniques are crucial for developing robust humanoid robots that can operate effectively in the real world.