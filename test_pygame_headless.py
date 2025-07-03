#!/usr/bin/env python3
"""
Test script for Pygame N-Body simulation in headless mode
This will test the physics and rendering pipeline without requiring a display
"""

import os
# Set SDL to use dummy video driver for headless testing
os.environ['SDL_VIDEODRIVER'] = 'dummy'

from n_body_system_pygame import NBodySystem, Sun, Planet
import numpy as np
import time

def test_headless_simulation():
    print("Testing N-Body Simulation with Pygame (Headless Mode)")
    print("=" * 50)
    
    # Simulation parameters
    SIMULATION_SIZE = 500
    NUMBER_OF_PLANETS = 5
    TEST_TIMESTEPS = 50  # Short test
    
    try:
        # Create the system
        print("Creating N-Body system...")
        system = NBodySystem(size=SIMULATION_SIZE, renderer_width=800, renderer_height=600)
        
        # Add sun
        print("Adding central sun...")
        sun = Sun(system, mass=50000)
        
        # Add planets
        print(f"Adding {NUMBER_OF_PLANETS} planets...")
        for i in range(NUMBER_OF_PLANETS):
            # Simple orbital setup
            distance = 100 + i * 50
            angle = i * (2 * np.pi / NUMBER_OF_PLANETS)
            
            pos = np.array([
                distance * np.cos(angle),
                distance * np.sin(angle),
                np.random.uniform(-20, 20)
            ])
            
            # Orbital velocity
            orbital_speed = np.sqrt(sun.mass / distance) * 0.7
            vel = np.array([
                -orbital_speed * np.sin(angle),
                orbital_speed * np.cos(angle),
                0
            ])
            
            planet = Planet(system, mass=10 + i * 2, position=pos, velocity=vel)
            print(f"  Planet {i+1}: distance={distance:.1f}, mass={planet.mass:.1f}")
        
        print(f"\nRunning {TEST_TIMESTEPS} timesteps...")
        start_time = time.time()
        
        # Simulation loop
        for timestep in range(TEST_TIMESTEPS):
            # Physics calculations
            system.calculate_all_body_interactions()
            system.update_all()
            
            # Render (to dummy display)
            system.draw_all()
            system.present_frame(timestep + 1, TEST_TIMESTEPS)
            
            if (timestep + 1) % 10 == 0:
                print(f"  Completed timestep {timestep + 1}/{TEST_TIMESTEPS}")
        
        elapsed_time = time.time() - start_time
        fps = TEST_TIMESTEPS / elapsed_time
        
        print(f"\nTest Results:")
        print(f"  Total timesteps: {TEST_TIMESTEPS}")
        print(f"  Elapsed time: {elapsed_time:.3f} seconds")
        print(f"  Average FPS: {fps:.1f}")
        print(f"  Performance: {'EXCELLENT' if fps > 100 else 'GOOD' if fps > 30 else 'NEEDS_IMPROVEMENT'}")
        
        # Test body positions
        print(f"\nFinal body positions:")
        for i, body in enumerate(system.bodies):
            pos = body.position
            print(f"  Body {i}: ({pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f})")
        
        system.cleanup()
        print("\n✅ Headless test completed successfully!")
        return True
        
    except Exception as e:
        print(f"\n❌ Test failed with error: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_headless_simulation()
    exit(0 if success else 1)
