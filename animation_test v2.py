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



# Initialize the car body as a rectangle
car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)


# Initialize rectangles for each wheel
wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]

def run_animation(fps=30):
    # Load data
    sim_file_directory = 'curva_t_255_d_20'
    data = rccar2.read_sim_file(sim_file_directory)

    # Unpack data
    (tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
     xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data

    # Calculate velocity from derivatives
    velocity = np.sqrt(xpsimu**2 + ypsimu**2)

    fig = plt.figure(figsize=(15, 10))
    grid = fig.add_gridspec(3, 2)

    ax_car = fig.add_subplot(grid[0, 0])
    ax_sf = fig.add_subplot(grid[0, 1])
    ax_pos_deriv = fig.add_subplot(grid[1, 0])
    ax_psi = fig.add_subplot(grid[1, 1])
    ax_velocity = fig.add_subplot(grid[2, 0])
    ax_positions = fig.add_subplot(grid[2, 1])

    ax_car.set_xlim(np.min(Xe) - 0.5, np.max(Xe) + 0.5)
    ax_car.set_ylim(np.min(Ye) - 0.5, np.max(Ye) + 0.5)
    ax_car.set_aspect('equal')

    car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)
    ax_car.add_patch(car_body)
    wheels = [Rectangle((0, 0), wheel_length, wheel_width, color=col, alpha=0.7) for col in ['red', 'green', 'blue', 'purple']]
    for wheel in wheels:
        ax_car.add_patch(wheel)

    trails = [ax_car.plot([], [], color=col, linewidth=2)[0] for col in ['red', 'green', 'blue', 'purple']]
    trail_length = 50

    center_of_mass, = ax_car.plot([], [], 'ko', markersize=3)

    def init():
        car_body.set_xy((-0.1, -0.1))
        center_of_mass.set_data([], [])
        for trail in trails:
            trail.set_data([], [])
        return [car_body, center_of_mass] + wheels + trails

    def update(frame):
        car_x = Xe[frame] - car_length / 2 * np.cos(psi[frame]) + car_width / 2 * np.sin(psi[frame])
        car_y = Ye[frame] - car_length / 2 * np.sin(psi[frame]) - car_width / 2 * np.cos(psi[frame])
        car_body.set_xy((car_x, car_y))
        car_body.angle = np.degrees(psi[frame])
        center_of_mass.set_data(Xe[frame], Ye[frame])

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
            wheel_x, wheel_y = wheel_positions[i]
            wheel.set_xy((wheel_x - wheel_length / 2, wheel_y - wheel_width / 2))
            wheel.angle = np.degrees(psi[frame] - dv[frame]) if i < 2 else np.degrees(psi[frame])
            xdata, ydata = trails[i].get_data()
            xdata = np.append(xdata, wheel_x)
            ydata = np.append(ydata, wheel_y)
            if len(xdata) > trail_length:
                xdata = xdata[-trail_length:]
                ydata = ydata[-trail_length:]
            trails[i].set_data(xdata, ydata)

        # Update all data plots
        ax_sf.set_data(tsimu[:frame+1], s_f[:frame+1])
        ax_pos_deriv.set_data(tsimu[:frame+1], np.column_stack((xpsimu[:frame+1], ypsimu[:frame+1])))
        ax_psi.set_data(tsimu[:frame+1], psi[:frame+1])
        ax_velocity.set_data(tsimu[:frame+1], velocity[:frame+1])
        ax_positions.set_data(tsimu[:frame+1], np.column_stack((xsimu[:frame+1], ysimu[:frame+1])))

        return [car_body, center_of_mass] + wheels + trails + [lines_sf, lines_xpsimu, lines_ypsimu, lines_psi, lines_velocity, lines_xsimu, lines_ysimu]

    interval = 1000 / fps  # Interval in milliseconds
    ani = FuncAnimation(fig, update, frames=len(tsimu), init_func=init, blit=True, interval=interval)
    plt.show()

# Call this with desired FPS
run_animation(fps=30)