import itertools
import math
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.widgets as widgets

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
    def __init__(self, size, projection_2d=False):
        self.initial_size = size
        self.size = size
        self.projection_2d = projection_2d
        self.bodies = []

        self.fig = plt.figure(figsize=(10, 10))
        self.ax = self.fig.add_subplot(
            111,
            projection="3d",
            position=[0.05, 0.20, 0.90, 0.75]
        )

        if self.projection_2d:
            self.ax.view_init(10, 0)
        else:
            self.ax.view_init(0, 0)

        self.setup_ui()

    def setup_ui(self):
        axcolor = 'lightgoldenrodyellow'
        slider_ax = self.fig.add_axes([0.25, 0.05, 0.50, 0.03], facecolor=axcolor)

        self.scale_slider = widgets.Slider(
            ax=slider_ax,
            label='View Scale',
            valmin=self.initial_size * 0.1,
            valmax=self.initial_size * 2.0,
            valinit=self.initial_size,
            valstep=self.initial_size / 100.0
        )

        self.scale_slider.on_changed(self.update_scale)

        self._slider_ax = slider_ax

    def update_scale(self, val):
        self.size = val

    def add_body(self, body):
        self.bodies.append(body)

    def update_all(self):
        self.bodies.sort(key=lambda item: item.position[0])
        for body in self.bodies:
            body.move()

    def draw_all(self):
        self.ax.clear()

        limit = self.size / 2
        self.ax.set_xlim((-limit, limit))
        self.ax.set_ylim((-limit, limit))
        self.ax.set_zlim((-limit, limit))

        if self.projection_2d:
            self.ax.xaxis.set_ticklabels([])
            self.ax.yaxis.set_ticklabels([])
            self.ax.zaxis.set_ticklabels([])
            self.ax.set_xlabel('X')
            self.ax.set_ylabel('Y')
            self.ax.set_zlabel('Z')
            self.ax.set_title("N-Body Simulation (2D Projection)")
        else:
            self.ax.axis(False)
            self.ax.set_title("N-Body Simulation (3D)")

        for body in self.bodies:
            body.draw()

        plt.pause(0.001)


    def calculate_all_body_interactions(self):
        root = OctreeNode(center=(0, 0, 0), size=self.initial_size * 2)
        for body in self.bodies:
             root.insert(body)

        for body in self.bodies:
            root.compute_force_on(body)


class NBodySystemBody:
    min_display_size = 10
    display_log_base = 1.3

    def __init__(
        self,
        nbody_system,
        mass,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        self.nbody_system = nbody_system
        self.mass = mass
        self.position = np.array(position, dtype=float)
        self.velocity = np.array(velocity, dtype=float)
        self.display_size = max(
            math.log(self.mass, self.display_log_base),
            self.min_display_size,
        )
        self.colour = "black"

        self.nbody_system.add_body(self)

    def move(self):
        self.position += self.velocity

    def draw(self):
        self.nbody_system.ax.plot(
            self.position[0],
            self.position[1],
            self.position[2],
            marker="o",
            markersize=self.display_size + (self.position[0] / (self.nbody_system.size / 30)),
            color=self.colour
        )
        if self.nbody_system.projection_2d:
            limit = self.nbody_system.size / 2
            self.nbody_system.ax.plot(
                self.position[0],
                self.position[1],
                -limit,
                marker="o",
                markersize=self.display_size / 2,
                color=(.5, .5, .5),
                alpha=0.6
            )

class Sun(NBodySystemBody):
    def __init__(
        self,
        nbody_system,
        mass=10_000,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        super(Sun, self).__init__(nbody_system, mass, position, velocity)
        self.colour = "yellow"

class Planet(NBodySystemBody):
    colours = itertools.cycle([(1, 0, 0), (0, 1, 0), (0, 0, 1), (0, 1, 1), (1, 0, 1), (1, 1, 0),
                               (0.5, 0.5, 0.5), (0.7, 0.3, 0), (0, 0.7, 0.3)])

    def __init__(
        self,
        nbody_system,
        mass=10,
        position=(0, 0, 0),
        velocity=(0, 0, 0),
    ):
        super(Planet, self).__init__(nbody_system, mass, position, velocity)
        self.colour = next(Planet.colours)
