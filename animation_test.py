import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import rccar2  # Assuming rccar2 contains the necessary function

# Car parameters
lf = 0.047  # Distance from center of mass to front wheel
lr = 0.05   # Distance from center of mass to rear wheel
Lw = 0.03   # Half-axle distance from the center of mass to the wheels
car_length = 0.12  # Estimated car length
car_width = 2 * Lw  # Car width is twice the half-axle distance

# Wheel dimensions
wheel_length = 0.02  # Length of each wheel (along the direction of the car)
wheel_width = 0.01  # Width of each wheel (perpendicular to the direction of the car)

# Load data
sim_file_directory = 'curva_t_255_d_20'
data = rccar2.read_sim_file(sim_file_directory)

# Unpack your data
(tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
 xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data

fig, ax = plt.subplots(figsize=(10, 8))
ax.set_xlim(np.min(Xe) - 0.5, np.max(Xe) + 0.5)
ax.set_ylim(np.min(Ye) - 0.5, np.max(Ye) + 0.5)
ax.set_aspect('equal')  # Ensures that the scale is the same on both axes

# Initialize the car body as a rectangle
car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)
ax.add_patch(car_body)

# Initialize rectangles for each wheel
wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]
for wheel in wheels:
    ax.add_patch(wheel)

# Marker for the center of mass
center_of_mass, = ax.plot([], [], 'ko', markersize=3)  # Black marker for the center of mass

def init():
    car_body.set_xy((-0.1, -0.1))  # Initially place the car off-screen
    center_of_mass.set_data([], [])
    for wheel in wheels:
        wheel.set_xy((-0.1, -0.1))  # Move wheels offscreen initially
    return [car_body, center_of_mass] + wheels

def update(frame):
    # Compute car body position and orientation
    car_x = Xe[frame] - car_length / 2 * np.cos(psi[frame]) + car_width / 2 * np.sin(psi[frame])
    car_y = Ye[frame] - car_length / 2 * np.sin(psi[frame]) - car_width / 2 * np.cos(psi[frame])
    car_body.set_xy((car_x, car_y))
    car_body.angle = np.degrees(psi[frame])

    # Update the center of mass marker
    center_of_mass.set_data(Xe[frame], Ye[frame])

    # Update wheel positions and orientations
    wheel_positions = [
        (Xe[frame] + lf * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
         Ye[frame] + lf * np.sin(psi[frame]) + Lw * np.cos(psi[frame])),
        (Xe[frame] + lf * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
         Ye[frame] + lf * np.sin(psi[frame]) - Lw * np.cos(psi[frame])),
        (Xe[frame] - lr * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
         Ye[frame] - lr * np.sin(psi[frame]) + Lw * np.cos(psi[frame])),
        (Xe[frame] - lr * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
         Ye[frame] - lr * np.sin(psi[frame]) - Lw * np.cos(psi[frame]))
    ]

    for i, wheel in enumerate(wheels):
        wheel_center_x = wheel_positions[i][0]  - wheel_length / 2 * np.cos(psi[frame]) + wheel_width / 2 * np.sin(psi[frame])
        wheel_center_y = wheel_positions[i][1]  - wheel_length / 2 * np.sin(psi[frame]) - wheel_width / 2 * np.cos(psi[frame])
        # Set the wheel's lower-left corner so that the wheel rotates around its center
        wheel.set_xy((wheel_center_x, wheel_center_y))
        if i < 2:  # Front wheels with steering
            # Negate dv to correct the steering direction
            wheel.angle = np.degrees(psi[frame] - dv[frame])
        else:  # Rear wheels without steering
            wheel.angle = np.degrees(psi[frame])

    return [car_body, center_of_mass] + wheels

ani = FuncAnimation(fig, update, frames=len(tsimu), init_func=init, blit=True, interval=50)
plt.show()
