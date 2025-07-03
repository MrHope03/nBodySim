#!/usr/bin/env python3
"""
Performance comparison between Matplotlib and Pygame renderers
"""

import time
import os
import numpy as np

# Set headless mode for testing
os.environ['SDL_VIDEODRIVER'] = 'dummy'

def test_matplotlib_performance():
    """Test the original matplotlib implementation"""
    print("Testing Matplotlib Renderer Performance...")
    
    try:
        from n_body_system import NBodySystem, Sun, Planet
        import matplotlib
        matplotlib.use('Agg')  # Use non-interactive backend
        
        # Create system
        system = NBodySystem(size=500, projection_2d=False)
        
        # Add bodies
        sun = Sun(system, mass=50000)
        for i in range(5):
            distance = 100 + i * 50
            angle = i * (2 * np.pi / 5)
            pos = np.array([distance * np.cos(angle), distance * np.sin(angle), 0])
            orbital_speed = np.sqrt(sun.mass / distance) * 0.7
            vel = np.array([-orbital_speed * np.sin(angle), orbital_speed * np.cos(angle), 0])
            Planet(system, mass=10, position=pos, velocity=vel)
        
        # Time the simulation
        timesteps = 20
        start_time = time.time()
        
        for step in range(timesteps):
            system.calculate_all_body_interactions()
            system.update_all()
            system.draw_all()  # This is where matplotlib is slow
        
        elapsed = time.time() - start_time
        fps = timesteps / elapsed
        
        return fps, elapsed, timesteps
        
    except Exception as e:
        print(f"Matplotlib test failed: {e}")
        return 0, 0, 0

def test_pygame_performance():
    """Test the new pygame implementation"""
    print("Testing Pygame Renderer Performance...")
    
    try:
        from n_body_system_pygame import NBodySystem, Sun, Planet
        
        # Create system
        system = NBodySystem(size=500, renderer_width=800, renderer_height=600)
        
        # Add bodies
        sun = Sun(system, mass=50000)
        for i in range(5):
            distance = 100 + i * 50
            angle = i * (2 * np.pi / 5)
            pos = np.array([distance * np.cos(angle), distance * np.sin(angle), 0])
            orbital_speed = np.sqrt(sun.mass / distance) * 0.7
            vel = np.array([-orbital_speed * np.sin(angle), orbital_speed * np.cos(angle), 0])
            Planet(system, mass=10, position=pos, velocity=vel)
        
        # Time the simulation
        timesteps = 20
        start_time = time.time()
        
        for step in range(timesteps):
            system.calculate_all_body_interactions()
            system.update_all()
            system.draw_all()
            system.present_frame(step + 1, timesteps)
        
        elapsed = time.time() - start_time
        fps = timesteps / elapsed
        
        system.cleanup()
        return fps, elapsed, timesteps
        
    except Exception as e:
        print(f"Pygame test failed: {e}")
        return 0, 0, 0

def main():
    print("N-Body Simulation Performance Comparison")
    print("=" * 50)
    
    # Test matplotlib
    print("\n1. Testing Matplotlib Implementation...")
    mpl_fps, mpl_time, mpl_steps = test_matplotlib_performance()
    
    # Test pygame
    print("\n2. Testing Pygame Implementation...")
    pg_fps, pg_time, pg_steps = test_pygame_performance()
    
    # Results
    print("\n" + "=" * 50)
    print("PERFORMANCE COMPARISON RESULTS")
    print("=" * 50)
    
    print(f"\nMatplotlib Renderer:")
    print(f"  Timesteps: {mpl_steps}")
    print(f"  Time: {mpl_time:.3f} seconds")
    print(f"  FPS: {mpl_fps:.1f}")
    
    print(f"\nPygame Renderer:")
    print(f"  Timesteps: {pg_steps}")
    print(f"  Time: {pg_time:.3f} seconds")
    print(f"  FPS: {pg_fps:.1f}")
    
    if mpl_fps > 0 and pg_fps > 0:
        improvement = pg_fps / mpl_fps
        print(f"\nPerformance Improvement:")
        print(f"  Pygame is {improvement:.1f}x faster than Matplotlib")
        print(f"  Time reduction: {((mpl_time - pg_time) / mpl_time * 100):.1f}%")
    
    print(f"\nRecommendation:")
    if pg_fps > mpl_fps * 2:
        print("  ✅ Use Pygame renderer for significantly better performance")
    elif pg_fps > mpl_fps:
        print("  ✅ Use Pygame renderer for better performance")
    else:
        print("  ⚠️  Both renderers have similar performance")
    
    print("\nNote: This test runs in headless mode. Real display performance")
    print("      with GPU acceleration would show even greater improvements.")

if __name__ == "__main__":
    main()
