```mermaid
graph TB
    A[Isaac Sim: Photorealistic Simulation] --> B[Synthetic Data Generation]
    B --> C[Perception Model Training]
    C --> D[Isaac ROS: Hardware-Accelerated VSLAM]
    D --> E[Localization & Mapping]
    E --> F[Nav2: Humanoid Navigation]
    F --> G[Complete AI Robot Brain]

    style A fill:#e1f5fe
    style D fill:#f3e5f5
    style F fill:#e8f5e8
    style G fill:#fff3e0
```

## Perception → Localization → Navigation Flow

This diagram illustrates the core architecture of the AI Robot Brain:

1. **Isaac Sim** provides photorealistic simulation environment
2. **Synthetic Data Generation** creates training datasets with domain randomization
3. **Perception Models** are trained on synthetic data for transfer to real world
4. **Isaac ROS** provides hardware-accelerated VSLAM capabilities
5. **Localization & Mapping** creates environmental understanding
6. **Nav2** handles humanoid-specific navigation and path planning
7. **Complete AI Robot Brain** integrates all components for intelligent behavior