# binary_star_system.py

from n_body_system import SolarSystem, Sun, Planet
from matplotlib.pyplot import fignum_exists
solar_system = SolarSystem(400, projection_2d=False)

suns = (
    Sun(solar_system, position=(40, 40, 40), velocity=(6, 0, 6)),
    Sun(solar_system, position=(-40, -40, 40), velocity=(-6, 0, -6)),
    Sun(solar_system, position=(-100, -50, 40), velocity=(-6, 0, -6)),
)

# planets = (
#     Planet(
#         solar_system,
#         10,
#         position=(100, 100, 0),
#         velocity=(0, 5.5, 5.5),
#     ),
#     Planet(
#         solar_system,
#         20,
#         position=(0, 0, 0),
#         velocity=(-11, 11, 0),
#     ),
# )

# Simple solar system
# sun = Sun(solar_system)

# planets = (
#     Planet(
#         solar_system,
#         position=(150, 50, 0),
#         velocity=(0, 5, 5),
#     ),planets = (
#     Planet(
#         solar_system,
#         10,
#         position=(100, 100, 0),
#         velocity=(0, 5.5, 5.5),
#     ),
#     Planet(
#         solar_system,
#         20,
#         position=(0, 0, 0),
#         velocity=(-11, 11, 0),
#     ),
# )

#     Planet(
#         solar_system,
#         mass=20,
#         position=(100, -50, 150),
#         velocity=(5, 0, 0)
#     )
# )

while fignum_exists(solar_system.fig.number):
    solar_system.calculate_all_body_interactions()
    solar_system.update_all()
    solar_system.draw_all()
