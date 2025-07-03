#!/usr/bin/env python3
"""
N-Body Simulation with Pygame Renderer
High-performance 3D visualization of gravitational n-body systems
"""

from n_body_system_pygame import NBodySystem, Sun, Planet
import numpy as np
import time

def main():
    # Simulation parameters
    SIMULATION_SIZE = 1000
    NUMBER_OF_PLANETS = 8  # Increased for better demo
    TIMESTEPS = 2000  # More timesteps since it's faster now
    
    print("Initializing N-Body Simulation with Pygame...")
    print(f"Simulation size: {SIMULATION_SIZE}")
    print(f"Number of planets: {NUMBER_OF_PLANETS}")
    print(f"Target timesteps: {TIMESTEPS}")
    print("\nControls:")
    print("- Mouse drag: Rotate camera")
    print("- Mouse wheel: Zoom in/out")
    print("- R: Reset camera")
    print("- ESC: Exit simulation")
    print("\nStarting simulation...\n")

    # Create the system with Pygame renderer
    system = NBodySystem(size=SIMULATION_SIZE, renderer_width=1400, renderer_height=1000)

    # Add a central sun
    sun = Sun(system, mass=100000)  # Massive central star

    # Add planets with calculated orbital velocities
    planets_created = 0
    attempts = 0
    max_attempts = NUMBER_OF_PLANETS * 3
    
    while planets_created < NUMBER_OF_PLANETS and attempts < max_attempts:
        attempts += 1
        
        # Generate random position
        # Use different orbital distances for variety
        orbital_distance = np.random.uniform(150, 800)  # Varied orbital distances
        angle = np.random.uniform(0, 2 * np.pi)
        inclination = np.random.uniform(-0.3, 0.3)  # Small inclination for more realistic orbits
        
        # Position in orbital plane with slight inclination
        pos = np.array([
            orbital_distance * np.cos(angle),
            orbital_distance * np.sin(angle) * np.sin(inclination),
            orbital_distance * np.sin(angle) * np.cos(inclination)
        ])
        
        distance_to_sun = np.linalg.norm(pos)
        
        if distance_to_sun > 50:  # Minimum distance from sun
            # Calculate orbital velocity for stable orbit
            orbital_velocity_mag = np.sqrt(sun.mass / distance_to_sun) * 0.8  # Slightly elliptical
            
            # Velocity perpendicular to position (for circular-ish orbit)
            # Cross product with z-axis gives perpendicular vector in xy plane
            vel_direction = np.array([-pos[1], pos[0], 0])
            vel_direction = vel_direction / np.linalg.norm(vel_direction)
            
            # Add some randomness for elliptical orbits
            vel_direction += np.random.normal(0, 0.1, 3)
            vel_direction = vel_direction / np.linalg.norm(vel_direction)
            
            vel = vel_direction * orbital_velocity_mag
            
            # Create planet with varied mass
            planet_mass = np.random.uniform(5, 25)
            planet = Planet(system, mass=planet_mass, position=pos, velocity=vel)
            
            planets_created += 1
            print(f"Created planet {planets_created}: mass={planet_mass:.1f}, distance={distance_to_sun:.1f}")

    print(f"\nSimulation setup complete! Created {planets_created} planets.")
    print("Starting real-time simulation...")

    # Simulation loop
    timestep = 0
    start_time = time.time()
    last_fps_time = start_time
    fps_counter = 0
    
    try:
        while system.is_running() and timestep < TIMESTEPS:
            timestep += 1
            
            # Physics update
            system.calculate_all_body_interactions()
            system.update_all()
            
            # Render frame
            system.draw_all()
            system.present_frame(timestep, TIMESTEPS)
            
            # Performance monitoring
            fps_counter += 1
            current_time = time.time()
            
            # Print progress every 100 frames
            if timestep % 100 == 0:
                elapsed = current_time - start_time
                fps = fps_counter / (current_time - last_fps_time) if current_time > last_fps_time else 0
                print(f"Timestep {timestep}/{TIMESTEPS} - FPS: {fps:.1f} - Elapsed: {elapsed:.1f}s")
                last_fps_time = current_time
                fps_counter = 0
    
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user.")
    
    finally:
        # Cleanup
        total_time = time.time() - start_time
        print(f"\nSimulation completed!")
        print(f"Total timesteps: {timestep}")
        print(f"Total time: {total_time:.2f} seconds")
        print(f"Average FPS: {timestep/total_time:.1f}")
        
        system.cleanup()
        print("Resources cleaned up. Goodbye!")

if __name__ == "__main__":
    main()
