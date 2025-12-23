# Physical AI & Humanoid Robotics - Docusaurus Documentation

This repository contains educational modules for Physical AI & Humanoid Robotics, implemented as a Docusaurus-based documentation site. The content covers ROS 2 middleware, digital twin simulation, and simulation-to-reality transfer techniques.

## Modules Structure

### Module 1: ROS 2 Middleware for Humanoid Control
- **Directory**: `modules/001-ros2-middleware/`
- **Constitution**: Core principles for distributed control architecture
- **Plan**: Implementation strategy and technical specifications
- **Chapters**:
  - Chapter 1: Introduction to ROS 2
  - Chapter 2: Distributed Control Patterns
  - Chapter 3: Advanced ROS 2 Patterns

### Module 2: Digital Twin Simulation for Humanoid Robotics
- **Directory**: `modules/002-digital-twin/`
- **Constitution**: Principles for simulation-to-reality continuity
- **Plan**: Simulation environment and visualization strategy
- **Chapters**:
  - Chapter 4: Gazebo Simulation
  - Chapter 5: Unity for Human-Robot Interaction
  - Chapter 6: Simulation-to-Reality Transfer

### Module 3: The AI-Robot Brain (NVIDIA Isaac™)
- **Directory**: `modules/003-isaac-ai-brain/`
- **Constitution**: Principles for Isaac Sim, ROS integration, and navigation
- **Plan**: AI brain architecture and implementation strategy
- **Chapters**:
  - Chapter 7: Isaac Sim for Photorealistic Simulation
  - Chapter 8: Synthetic Data Generation for Perception Models
  - Chapter 9: Isaac ROS and Nav2 Integration

### Module 4: Vision-Language-Action (VLA)
- **Directory**: `modules/004-vla/`
- **Constitution**: Principles for vision-language-action integration and multimodal perception
- **Plan**: Cognitive planning and action execution implementation strategy
- **Chapters**:
  - Chapter 10: Voice Input and Natural Language Processing
  - Chapter 11: Vision Perception and Scene Understanding
  - Chapter 12: Cognitive Planning and Action Execution

## Getting Started

1. Install dependencies:
```bash
yarn install
```

2. Start the development server:
```bash
yarn start
```

3. The documentation will be available at http://localhost:3000

## Key Features

- **Modular Design**: Each module is self-contained with its own constitution and plan
- **Educational Focus**: Content designed for AI engineers and robotics students
- **Simulation-to-Reality**: Emphasis on techniques that bridge simulation and real-world robotics
- **ROS 2 Integration**: Comprehensive coverage of ROS 2 middleware for humanoid control

## Architecture

The educational content follows the Physical AI & Humanoid Robotics constitution with:
- Embodied intelligence first approach
- Accuracy via official documentation
- Simulation-to-reality continuity
- Modular, standalone chapters
- Real-world physics fidelity
- Distributed control architecture

## Project Structure

```
ui-docusaurus/
├── modules/                    # Educational modules
│   ├── 001-ros2-middleware/    # Module 1: ROS 2 concepts
│   │   ├── constitution.md     # Module-specific principles
│   │   ├── plan.md             # Implementation strategy
│   │   ├── chapter-1-introduction-to-ros2/
│   │   ├── chapter-2-distributed-control/
│   │   └── chapter-3-advanced-ros2-patterns/
│   ├── 002-digital-twin/       # Module 2: Simulation concepts
│   │   ├── constitution.md     # Module-specific principles
│   │   ├── plan.md             # Implementation strategy
│   │   ├── chapter-4-gazebo-simulation/
│   │   ├── chapter-5-unity-hri/
│   │   └── chapter-6-simulation-to-reality/
│   └── 003-isaac-ai-brain/     # Module 3: AI Robot Brain concepts
│       ├── constitution.md     # Module-specific principles
│       ├── plan.md             # Implementation strategy
│       ├── chapter-7-isaac-sim-photorealistic-simulation/
│       ├── chapter-8-synthetic-data-generation/
│       └── chapter-9-isaac-ros-vslam-nav2/
├── 004-vla/                    # Module 4: Vision-Language-Action concepts
│   ├── constitution.md         # Module-specific principles
│   ├── plan.md                 # Implementation strategy
│   ├── chapter-10-voice-input-natural-language-processing/
│   ├── chapter-11-vision-perception-scene-understanding/
│   └── chapter-12-cognitive-planning-action-execution/
├── docs/                       # General documentation
├── src/                        # Custom components
├── static/                     # Static assets
├── docusaurus.config.js        # Docusaurus configuration
└── sidebars.js                 # Navigation configuration
```

## Contributing

Each module follows the same structure:
1. `constitution.md` - Module-specific principles
2. `plan.md` - Implementation strategy
3. Chapter directories with educational content
4. Assets and examples

## Installation

```bash
yarn
```

## Local Development

```bash
yarn start
```

This command starts a local development server and opens up a browser window. Most changes are reflected live without having to restart the server.

## Build

```bash
yarn build
```

This command generates static content into the `build` directory and can be served using any static contents hosting service.

## Deployment

Using SSH:

```bash
USE_SSH=true yarn deploy
```

Not using SSH:

```bash
GIT_USER=<Your GitHub username> yarn deploy
```

If you are using GitHub pages for hosting, this command is a convenient way to build the website and push to the `gh-pages` branch.
"# Docusaurus-book" 
