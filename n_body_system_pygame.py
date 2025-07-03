import itertools
import math
import numpy as np
from collections import deque
from pygame_renderer import PygameRenderer, COLORS

class OctreeNode:
    def __init__(self, center, size):
        self.center = np.array(center, dtype=float)
        self.size = size
        self.children = [None] * 8
        self.body = None
        self.total_mass = 0
        self.com = np.zeros(3)
        self.is_internal = False

    def insert(self, body):
        if self.body is None and not self.is_internal:
            self.body = body
            self.total_mass = body.mass
            self.com = body.position.copy()
        else:
            if not self.is_internal:
                self.subdivide()
                self._insert_body(self.body)
                self.body = None
                self.is_internal = True

            self._insert_body(body)

            self.total_mass += body.mass
            self.com = (self.com * (self.total_mass - body.mass) + body.position * body.mass) / self.total_mass

    def _insert_body(self, body):
        index = 0
        offset = self.size / 4
        new_center = self.center.copy()
        for i in range(3):
            if body.position[i] > self.center[i]:
                index |= 1 << i
                new_center[i] += offset
            else:
                new_center[i] -= offset

        if self.children[index] is None:
            self.children[index] = OctreeNode(new_center, self.size / 2)

        self.children[index].insert(body)

    def subdivide(self):
        half_size = self.size / 2
        quarter_size = self.size / 4
        for i in range(8):
            offset = [0, 0, 0]
            if i & 1: offset[0] = quarter_size
            else: offset[0] = -quarter_size
            if i & 2: offset[1] = quarter_size
            else: offset[1] = -quarter_size
            if i & 4: offset[2] = quarter_size
            else: offset[2] = -quarter_size
            new_center = self.center + offset
            self.children[i] = OctreeNode(new_center, half_size)

    def compute_force_on(self, target_body, theta=0.5, G=1.0):
        if self.body is target_body and not self.is_internal:
             return

        distance_vector = self.com - target_body.position
        distance = np.linalg.norm(distance_vector)

        if distance == 0:
             return

        if not self.is_internal or (self.size / distance) < theta:
            force_dir = distance_vector / distance
            force_mag = G * self.total_mass * target_body.mass / distance**2

            acceleration = force_dir * (force_mag / target_body.mass)
            target_body.velocity += acceleration
        else:
            for child in self.children:
                if child is not None:
                    child.compute_force_on(target_body, theta, G)


class NBodySystem:
    def __init__(self, size, renderer_width=1200, renderer_height=900):
        self.initial_size = size
        self.size = size
        self.bodies = []
        
        # Initialize Pygame renderer instead of matplotlib
        self.renderer = PygameRenderer(renderer_width, renderer_height, "N-Body Simulation - Pygame")
        
        # Set initial zoom based on simulation size
        self.renderer.zoom_factor = 800.0 / size  # Adjust this ratio as needed

    def add_body(self, body):
        self.bodies.append(body)

    def update_all(self):
        self.bodies.sort(key=lambda item: item.position[0])
        for body in self.bodies:
            body.move()

    def draw_all(self):
        """Draw all bodies using Pygame renderer"""
        self.renderer.clear_screen()
        
        for body in self.bodies:
            # Convert trail positions to list for pygame renderer
            trail_positions = list(body.position_history) if hasattr(body, 'position_history') else None
            
            self.renderer.draw_body(
                position=body.position,
                mass=body.mass,
                color=body.color_rgb,
                trail_positions=trail_positions
            )

    def calculate_all_body_interactions(self):
        root = OctreeNode(center=(0, 0, 0), size=self.initial_size * 2)
        for body in self.bodies:
             root.insert(body)

        for body in self.bodies:
            root.compute_force_on(body)
    
    def is_running(self):
        """Check if simulation should continue running"""
        return self.renderer.is_running()
    
    def present_frame(self, timestep, total_timesteps):
        """Present the current frame"""
        self.renderer.present(timestep, total_timesteps)
    
    def cleanup(self):
        """Clean up resources"""
        self.renderer.cleanup()


class NBodySystemBody:
    min_display_size = 10
    display_log_base = 1.3
    DEFAULT_TRAIL_LENGTH = 50

    def __init__(
        self,
        nbody_system,
        mass,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
        trail_length=DEFAULT_TRAIL_LENGTH,
        color_name='planet'
    ):
        self.nbody_system = nbody_system
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.display_size = max(
            math.log(self.mass, self.display_log_base),
            self.min_display_size,
        )
        
        # Set color
        self.color_name = color_name
        self.color_rgb = COLORS.get(color_name, COLORS['planet'])

        # History of positions for the trail
        self.position_history = deque(maxlen=trail_length)
        self.position_history.append(self.position.copy())

        self.nbody_system.add_body(self)

    def move(self):
        self.position += self.velocity
        self.position_history.append(self.position.copy())


class Sun(NBodySystemBody):
    def __init__(
        self,
        nbody_system,
        mass=10_000,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
        trail_length=5  # Minimal trail for sun
    ):
        super(Sun, self).__init__(
            nbody_system, mass, position, velocity, 
            trail_length=trail_length, color_name='sun'
        )


class Planet(NBodySystemBody):
    # Cycle through different planet colors
    planet_colors = itertools.cycle(['mercury', 'venus', 'earth', 'mars', 'jupiter', 
                                   'saturn', 'uranus', 'neptune'])

    def __init__(
        self,
        nbody_system,
        mass=10,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
        trail_length=NBodySystemBody.DEFAULT_TRAIL_LENGTH
    ):
        color_name = next(Planet.planet_colors)
        super(Planet, self).__init__(
            nbody_system, mass, position, velocity, 
            trail_length=trail_length, color_name=color_name
        )
