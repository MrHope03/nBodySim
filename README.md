# N-Body Gravitational Simulation

A high-performance 3D gravitational n-body simulation with real-time visualization, implementing the Barnes-Hut algorithm for efficient force calculations.

## Features

### Physics Engine
- **Barnes-Hut Algorithm**: O(N log N) force calculations using octree spatial partitioning
- **Realistic Orbital Mechanics**: Proper gravitational force calculations with center-of-mass dynamics
- **Stable Integration**: Velocity Verlet-style integration for stable long-term simulations

### Visualization Options

#### 1. Matplotlib Renderer (Original - `main.py`)
- 3D matplotlib visualization with interactive controls
- Zoom slider for dynamic view scaling
- 2D projection mode option
- **Note**: Can be slow for real-time simulation with many particles

#### 2. Pygame Renderer (New - `main_pygame.py`) ⭐ **Recommended**
- **High-performance** hardware-accelerated rendering
- **Interactive 3D camera** with mouse controls
- **Real-time 60+ FPS** performance
- **Smooth orbital trails** with fading effects
- **Perspective projection** with proper depth handling

### Visual Effects
- **Orbital Trails**: Configurable fading trails showing body trajectories
- **Dynamic Sizing**: Body size scales with mass and distance
- **Realistic Colors**: Astronomical color palette for different body types
- **Glow Effects**: Larger bodies have subtle glow effects

## Quick Start

### Using Pygame Renderer (Recommended)
```bash
# Activate virtual environment
source .venv/bin/activate

# Run high-performance simulation
python main_pygame.py
```

### Using Matplotlib Renderer (Original)
```bash
# Activate virtual environment  
source .venv/bin/activate

# Run original simulation
python main.py
```

## Controls (Pygame Version)

- **Mouse Drag**: Rotate camera around the system
- **Mouse Wheel**: Zoom in/out
- **R Key**: Reset camera to default position
- **ESC**: Exit simulation

## Configuration

### Simulation Parameters
Edit the parameters in `main_pygame.py`:

```python
SIMULATION_SIZE = 1000      # Simulation space size
NUMBER_OF_PLANETS = 8       # Number of planets to generate
TIMESTEPS = 2000           # Maximum simulation steps
```

### Body Properties
- **Sun**: Central massive body (mass: 100,000 units)
- **Planets**: Randomly generated with realistic orbital velocities
- **Trail Length**: Configurable per body type

## Technical Details

### Architecture
- `n_body_system_pygame.py`: Core physics engine with Pygame integration
- `pygame_renderer.py`: High-performance 3D rendering system
- `main_pygame.py`: Simulation setup and main loop

### Performance Optimizations
- **Octree Spatial Partitioning**: Reduces force calculations from O(N²) to O(N log N)
- **Hardware Acceleration**: Pygame with OpenGL backend
- **Efficient Trail Rendering**: Alpha-blended trail segments
- **Frustum Culling**: Only render visible objects

### Barnes-Hut Algorithm
The simulation uses the Barnes-Hut algorithm for efficient gravitational force calculations:
1. **Spatial Subdivision**: 3D space divided into octree nodes
2. **Mass Aggregation**: Distant groups treated as single massive points
3. **Adaptive Precision**: Configurable theta parameter for accuracy vs. performance

## Dependencies

```
pygame>=2.6.0
numpy>=1.22.0
matplotlib>=3.5.0  # For original renderer only
```

## Development

### Branch Structure
- `main`: Original matplotlib implementation
- `pygame-renderer`: High-performance Pygame implementation

### Testing
```bash
# Test headless performance
python test_pygame_headless.py
```

## Performance Comparison

| Renderer | FPS | Interactivity | Visual Quality | Stability |
|----------|-----|---------------|----------------|-----------|
| Matplotlib | 1-5 | Limited | Good | Can hang |
| Pygame | 30-60+ | Excellent | Excellent | Very stable |

## Future Enhancements

- [ ] Collision detection and merging
- [ ] Configurable physics constants (G, time step)
- [ ] Save/load simulation states
- [ ] Multiple star systems
- [ ] Particle effects for collisions
- [ ] VR/AR visualization support

## License

Open source - feel free to modify and distribute.

---

**Tip**: For the best experience, use the Pygame renderer (`main_pygame.py`) which provides smooth real-time visualization with interactive controls!