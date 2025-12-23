# Isaac Sim Setup Code Examples

## Basic Isaac Sim Environment Setup

```python
import omni
from omni.isaac.kit import SimulationApp

# Initialize Isaac Sim application
config = {
    'headless': False,  # Set to True for headless execution
    'enable_cameras': True,
    'carb_settings_path': './ carb/settings/path.json'
}

simulation_app = SimulationApp(config)

# Import required Isaac Sim modules
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.prims import create_prim

# Create the world instance
world = World(stage_units_in_meters=1.0)

# Add ground plane
from omni.isaac.core.objects import GroundPlane
ground_plane = world.scene.add(GroundPlane("/World/defaultGroundPlane"))

# Add lighting
from omni.isaac.core.utils.prims import create_prim
create_prim("/World/Light", "DistantLight", position=[0, 0, 5], rotation=[0, 0, 0])

# Add camera
from omni.isaac.sensor import Camera
camera = Camera(
    prim_path="/World/Camera",
    position=[2, 2, 2],
    look_at=[0, 0, 0]
)

print("Isaac Sim environment initialized successfully!")
```

## Humanoid Robot Loading Example

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.nucleus import get_assets_root_path
from omni.isaac.core.utils.stage import add_reference_to_stage
import numpy as np

# Initialize world
world = World(stage_units_in_meters=1.0)

# Add ground plane and lighting
from omni.isaac.core.objects import GroundPlane
world.scene.add(GroundPlane("/World/defaultGroundPlane"))

# Load a humanoid robot (example with ATRIAS robot from Isaac Sim assets)
assets_root_path = get_assets_root_path()
if assets_root_path is None:
    print("Could not find Isaac Sim assets. Please enable Isaac Sim Content in Extension Manager.")
else:
    # Add humanoid robot to the scene
    # Note: Replace with actual humanoid robot asset path
    humanoid_asset_path = assets_root_path + "/Isaac/Robots/Humanoid/humanoid_instanceable.usd"
    add_reference_to_stage(
        usd_path=humanoid_asset_path,
        prim_path="/World/Humanoid"
    )

    # Get reference to the humanoid robot in the simulation
    from omni.isaac.core.articulations import Articulation
    humanoid = world.scene.add(
        Articulation(
            prim_path="/World/Humanoid",
            name="humanoid",
        )
    )

print("Humanoid robot loaded successfully!")
```

## Sensor Configuration Example

```python
import omni
from omni.isaac.core import World
from omni.isaac.sensor import Camera
from omni.isaac.core.sensors import ImuSensor
import numpy as np

# Initialize world
world = World(stage_units_in_meters=1.0)

# Add ground plane
from omni.isaac.core.objects import GroundPlane
ground_plane = world.scene.add(GroundPlane("/World/defaultGroundPlane"))

# Add RGB Camera
rgb_camera = Camera(
    prim_path="/World/RGB_Camera",
    position=[1.5, 0, 1.5],
    frequency=30,  # 30 Hz
    resolution=(640, 480)
)

# Add Depth Camera
depth_camera = Camera(
    prim_path="/World/Depth_Camera",
    position=[1.5, 0.1, 1.5],  # Slightly offset from RGB
    frequency=30,
    resolution=(640, 480)
)
# Enable depth data
depth_camera.add_ground_truth_to_frame()

# Add IMU Sensor
imu_sensor = ImuSensor(
    prim_path="/World/Imu_Sensor",
    position=[0, 0, 1.0],  # At approximate CoM height
    frequency=100  # 100 Hz
)

# Reset the world to initialize sensors
world.reset()

# Example of capturing sensor data
for i in range(100):
    world.step(render=True)

    if i % 10 == 0:  # Log every 10 steps
        # Get RGB image
        rgb_image = rgb_camera.get_rgb()
        print(f"RGB Image shape: {rgb_image.shape}")

        # Get depth image
        depth_image = depth_camera.get_depth()
        print(f"Depth Image shape: {depth_image.shape}")

        # Get IMU data
        imu_data = imu_sensor.get_measured_value()
        print(f"IMU Data: {imu_data}")
```

## Physics Configuration Example

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.stage import add_reference_to_stage
from pxr import PhysxSchema, UsdPhysics
import carb

# Initialize world with specific physics parameters
world = World(
    stage_units_in_meters=1.0,
    rendering_dt=1.0/60.0,  # 60 FPS rendering
    physics_dt=1.0/240.0,   # 240 FPS physics (4x real-time)
)

# Access the physics scene directly to configure parameters
scene = world.scene
physics_scene_path = scene.get_physics_scene_prim().GetPath().AsString()
physics_scene = UsdPhysics.Scene.Get(world.stage, physics_scene_path)

# Configure physics scene properties
PhysxSchema.PhysxSceneAPI.Apply(world.stage, physics_scene_path)
physx_scene_api = PhysxSchema.PhysxSceneAPI.Get(world.stage, physics_scene_path)

# Set solver parameters
physx_scene_api.GetSolverTypeAttr().Set("TGS")  # Use TGS solver
physx_scene_api.GetMaxPositionIterationsAttr().Set(4)
physx_scene_api.GetMaxVelocityIterationsAttr().Set(16)

# Set bounce threshold
physx_scene_api.GetBounceThresholdAttr().Set(0.5)

# Set friction correlation mode
physx_scene_api.GetFrictionCorrelationModelAttr().Set("patch")

# Add a simple object to test physics
from omni.isaac.core.objects import DynamicCuboid
cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/Cube",
        name="cube",
        position=[0, 0, 2.0],
        size=0.1,
        mass=0.5
    )
)

print("Physics scene configured with custom parameters!")
```

## Domain Randomization Example

```python
import omni
from omni.isaac.core import World
from omni.isaac.core.utils.prims import create_prim
from omni.isaac.core.materials import OmniPBR
import numpy as np
import random

class DomainRandomizer:
    def __init__(self, world):
        self.world = world
        self.light_prim = None
        self.materials = []

    def randomize_lighting(self):
        """Randomize lighting conditions in the scene"""
        # Create or get light
        if not self.light_prim:
            from omni.isaac.core.utils.prims import create_prim
            self.light_prim = create_prim(
                prim_path="/World/randomized_light",
                prim_type="DistantLight",
                position=[0, 0, 10],
                rotation=[0, 0, 0]
            )

        # Randomize light properties
        light_intensity = random.uniform(500, 5000)  # Random intensity
        light_color = [
            random.uniform(0.8, 1.0),  # R
            random.uniform(0.8, 1.0),  # G
            random.uniform(0.9, 1.0)   # B (slightly blue)
        ]

        # Apply randomization
        self.light_prim.GetAttribute("inputs:intensity").Set(light_intensity)
        self.light_prim.GetAttribute("inputs:color").Set(light_color)

    def randomize_materials(self):
        """Randomize materials in the scene"""
        # Create a random OmniPBR material
        material_name = f"random_material_{random.randint(1000, 9999)}"
        material_path = f"/World/Materials/{material_name}"

        material = OmniPBR(
            prim_path=material_path,
            name=material_name,
            albedo_value=np.array([
                random.uniform(0.2, 1.0),
                random.uniform(0.2, 1.0),
                random.uniform(0.2, 1.0),
                1.0  # Alpha
            ]),
            roughness_value=random.uniform(0.1, 0.9),
            metallic_value=random.uniform(0.0, 0.2),
            specular_value=random.uniform(0.1, 0.9)
        )

        self.materials.append(material)
        return material

    def randomize_environment(self):
        """Apply all domain randomizations"""
        self.randomize_lighting()
        self.randomize_materials()

        print("Domain randomization applied!")

# Usage example
world = World(stage_units_in_meters=1.0)
domain_randomizer = DomainRandomizer(world)

# Add basic scene elements
from omni.isaac.core.objects import GroundPlane
ground_plane = world.scene.add(GroundPlane("/World/defaultGroundPlane"))

# Apply domain randomization
domain_randomizer.randomize_environment()
```