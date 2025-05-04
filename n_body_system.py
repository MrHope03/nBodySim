import itertools
import math
import matplotlib.pyplot as plt
import numpy as np

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
        for i in range(8):
            offset = [(1 if i & (1 << axis) else -1) * self.size / 4 for axis in range(3)]
            new_center = self.center + offset
            self.children[i] = OctreeNode(new_center, self.size / 2)

    def compute_force_on(self, target_body, theta=0.5, G=1.0):
        if self.body is target_body and not self.is_internal:
            return

        distance = np.linalg.norm(self.com - target_body.position)
        if distance == 0:
            return

        if not self.is_internal or (self.size / distance) < theta:
            force_dir = (self.com - target_body.position) / distance
            force_mag = G * self.total_mass * target_body.mass / distance**2
            target_body.velocity += force_dir * (force_mag / target_body.mass)
        else:
            for child in self.children:
                if child is not None:
                    child.compute_force_on(target_body, theta, G)

class SolarSystem:
    def __init__(self, size, projection_2d=False):
        self.size = size
        self.projection_2d = projection_2d
        self.bodies = []

        self.fig, self.ax = plt.subplots(
            1,
            1,
            subplot_kw={"projection": "3d"},
            figsize=(self.size / 50, self.size / 50),
        )
        if self.projection_2d:
            self.ax.view_init(10, 0)
        else:
            self.ax.view_init(0, 0)
        self.fig.tight_layout()

    def add_body(self, body):
        self.bodies.append(body)

    def update_all(self):
        # Sorting key still works with numpy array element access
        self.bodies.sort(key=lambda item: item.position[0])
        for body in self.bodies:
            body.move()
            body.draw()

    def draw_all(self):
        self.ax.set_xlim((-self.size / 2, self.size / 2))
        self.ax.set_ylim((-self.size / 2, self.size / 2))
        self.ax.set_zlim((-self.size / 2, self.size / 2))
        if self.projection_2d:
            self.ax.xaxis.set_ticklabels([])
            self.ax.yaxis.set_ticklabels([])
            self.ax.zaxis.set_ticklabels([])
        else:
            self.ax.axis(False)
        plt.pause(0.001)
        self.ax.clear()

    def calculate_all_body_interactions(self):
        root = OctreeNode(center=(0, 0, 0), size=self.size)
        for body in self.bodies:
            root.insert(body)
        for body in self.bodies:
            root.compute_force_on(body)

class SolarSystemBody:
    min_display_size = 10
    display_log_base = 1.3

    def __init__(
        self,
        solar_system,
        mass,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        self.solar_system = solar_system
        self.mass = mass
        # Store position and velocity as numpy arrays
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.display_size = max(
            math.log(self.mass, self.display_log_base),
            self.min_display_size,
        )
        self.colour = "black"

        self.solar_system.add_body(self)

    def move(self):
        # Use numpy array addition for position update
        self.position += self.velocity

    def draw(self):
        # Unpack numpy array for plotting
        self.solar_system.ax.plot(
            *self.position,
            marker="o",
            markersize=self.display_size + self.position[0] / 30, # Still using position[0] for z-sorting effect
            color=self.colour
        )
        if self.solar_system.projection_2d:
            # Draw projection on the bottom plane
            self.solar_system.ax.plot(
                self.position[0],
                self.position[1],
                -self.solar_system.size / 2,
                marker="o",
                markersize=self.display_size / 2,
                color=(.5, .5, .5),
            )

    def accelerate_due_to_gravity(self, other):
        # Calculate distance vector using numpy subtraction
        distance = other.position - self.position
        # Calculate distance magnitude using numpy norm
        distance_mag = np.linalg.norm(distance)

        # Avoid division by zero if bodies are at the same location
        if distance_mag == 0:
            return

        # Calculate force magnitude
        force_mag = self.mass * other.mass / (distance_mag ** 2)
        # Calculate force vector by normalizing distance and multiplying by magnitude
        force = (distance / distance_mag) * force_mag

        # Apply acceleration to both bodies
        # Acceleration = Force / Mass
        self_acceleration = force / self.mass
        other_acceleration = -force / other.mass # Force on the other body is opposite

        # Update velocities using numpy addition/subtraction
        self.velocity += self_acceleration
        other.velocity += other_acceleration


class Sun(SolarSystemBody):
    def __init__(
        self,
        solar_system,
        mass=10_000,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        super(Sun, self).__init__(solar_system, mass, position, velocity)
        self.colour = "yellow"

class Planet(SolarSystemBody):
    colours = itertools.cycle([(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 0)]) # Added more colors

    def __init__(
        self,
        solar_system,
        mass=10,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        super(Planet, self).__init__(solar_system, mass, position, velocity)
        self.colour = next(Planet.colours)
