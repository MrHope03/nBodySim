# binary_star_system.py

from n_body_system import NBodySystem, Sun, Planet
import matplotlib.pyplot as plt
import numpy as np

# Simulation parameters
SIMULATION_SIZE = 1000
NUMBER_OF_PLANETS = 50
TIMESTEPS = 500
PROJECT_TO_2D = False # Set to True for a 2D projection view

system = NBodySystem(size=SIMULATION_SIZE, projection_2d=PROJECT_TO_2D)

# Add bodies to the system
sun = Sun(system, mass=100000) # Make sun significantly more massive

# Add planets with initial positions and velocities
for _ in range(NUMBER_OF_PLANETS):
    # Random position within a certain range
    pos = (np.random.rand(3) - 0.5) * SIMULATION_SIZE * 0.8 # Spread planets out
    # Calculate initial velocity for a roughly circular orbit
    # v = sqrt(G*M/r) for circular orbit, G is implicitly 1 here
    distance_to_sun = np.linalg.norm(pos)
    if distance_to_sun > 10: # Avoid placing planets too close to the sun or origin
        orbital_velocity_mag = np.sqrt(sun.mass / distance_to_sun)
        # Create a velocity vector perpendicular to the position vector from the sun
        # Simple cross product with a random vector gives a perpendicular direction
        random_vec = np.random.rand(3) - 0.5
        vel_dir = np.cross(pos, random_vec)
        vel_dir /= np.linalg.norm(vel_dir) # Normalize direction
        vel = vel_dir * orbital_velocity_mag * 0.5 # Scale velocity to make orbits more elliptical/stable within the view

        planet = Planet(system, mass=np.random.rand() * 20 + 5, position=pos, velocity=vel) # Vary planet masses

# Set up initial plot
plt.show(block=False) # Show the plot non-blocking so the code can continue

# Simulation loop
i = 0
while plt.fignum_exists(system.fig.number):
    print(f"Timestep {i+1}/{TIMESTEPS}")
    # Calculate forces using the Octree (updates velocities directly)
    system.calculate_all_body_interactions()
    # Move bodies based on their updated velocities
    system.update_all()
    # Draw the current state
    system.draw_all()
