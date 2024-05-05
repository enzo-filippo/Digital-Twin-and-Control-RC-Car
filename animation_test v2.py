import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Rectangle
import rccar2  # Assuming rccar2 contains the necessary function

def run_animation( sim_file_directory, fps=30,):

    # Car parameters
    lf = 0.047  # Distance from center of mass to front wheel
    lr = 0.05   # Distance from center of mass to rear wheel
    Lw = 0.03   # Half-axle distance from the center of mass to the wheels
    car_length = lf+lr  # Estimated car length
    car_width = 2 * Lw  # Car width is twice the half-axle distance

    # Wheel dimensions
    wheel_length = 0.02  # Length of each wheel (along the direction of the car)
    wheel_width = 0.01  # Width of each wheel (perpendicular to the direction of the car)

    # Load data
    data = rccar2.read_sim_file(sim_file_directory)

    # Unpack your data
    (tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
    xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data



    # Initialize the car body as a rectangle
    car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)


    # Initialize rectangles for each wheel
    wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]
    
    # Load data
    data = rccar2.read_sim_file(sim_file_directory)

    # Unpack data
    (tsimu, xsimu, xpsimu, ysimu, ypsimu, psi, psip, Xe, Ye,
     xef1, yef1, xer1, yer1, xef2, yef2, xer2, yer2, tv, dv, s_f, s_r) = data

    fig, ax = plt.subplots(figsize=(10, 8))
    ax.set_xlim(np.min(Xe) - 0.5, np.max(Xe) + 0.5)
    ax.set_ylim(np.min(Ye) - 0.5, np.max(Ye) + 0.5)
    ax.set_aspect('equal')

    # Initialize the car body as a rectangle
    car_body = Rectangle((0, 0), car_length, car_width, angle=0, color='blue', alpha=0.7)
    ax.add_patch(car_body)

    # Initialize rectangles for each wheel
    wheels = [Rectangle((0, 0), wheel_length, wheel_width, color='red', alpha=0.7) for _ in range(4)]
    for wheel in wheels:
        ax.add_patch(wheel)

    # Trails for wheels
    trails = [ax.plot([], [], 'k:', linewidth=1.5)[0] for _ in range(4)]
    trail_length = 50  # Number of past positions to remember for the trail

    # Marker for the center of mass and time display
    center_of_mass, = ax.plot([], [], 'o', markersize=3)
    time_text = ax.text(0.05, 0.95, '', transform=ax.transAxes)

    def init():
        car_body.set_xy((-0.1, -0.1))
        center_of_mass.set_data([], [])
        for trail in trails:
            trail.set_data([], [])
        time_text.set_text('')
        return [car_body, center_of_mass] + wheels + trails + [time_text]

    def update(frame):
        # Update car body and center of mass
        car_x = Xe[frame] - car_length / 2 * np.cos(psi[frame]) + car_width / 2 * np.sin(psi[frame])
        car_y = Ye[frame] - car_length / 2 * np.sin(psi[frame]) - car_width / 2 * np.cos(psi[frame])
        car_body.set_xy((car_x, car_y))
        car_body.angle = np.degrees(psi[frame])
        center_of_mass.set_data(Xe[frame], Ye[frame])
        time_text.set_text(f'Time: {tsimu[frame]:.2f}s')

        # Update wheel positions and orientations
        wheel_positions = np.array([
            [Xe[frame] + lf * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
             Ye[frame] + lf * np.sin(psi[frame]) + Lw * np.cos(psi[frame])],
            [Xe[frame] + lf * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
             Ye[frame] + lf * np.sin(psi[frame]) - Lw * np.cos(psi[frame])],
            [Xe[frame] - lr * np.cos(psi[frame]) - Lw * np.sin(psi[frame]),
             Ye[frame] - lr * np.sin(psi[frame]) + Lw * np.cos(psi[frame])],
            [Xe[frame] - lr * np.cos(psi[frame]) + Lw * np.sin(psi[frame]),
             Ye[frame] - lr * np.sin(psi[frame]) - Lw * np.cos(psi[frame])]
        ])
        for i, wheel in enumerate(wheels):
            wheel_center_x = wheel_positions[i, 0] - wheel_length / 2 * np.cos(psi[frame]) + wheel_width / 2 * np.sin(psi[frame])
            wheel_center_y = wheel_positions[i, 1] - wheel_length / 2 * np.sin(psi[frame]) - wheel_width / 2 * np.cos(psi[frame])
            wheel.set_xy((wheel_center_x, wheel_center_y))
            wheel.angle = np.degrees(psi[frame] - dv[frame]) if i < 2 else np.degrees(psi[frame])

            # Update trails
            xdata, ydata = trails[i].get_data()
            xdata = np.append(xdata, wheel_center_x)
            ydata = np.append(ydata, wheel_center_y)
            if len(xdata) > trail_length:
                xdata = xdata[-trail_length:]
                ydata = ydata[-trail_length:]
            trails[i].set_data(xdata, ydata)

        return [car_body, center_of_mass] + wheels + trails + [time_text]

    interval = 1000 / fps  # Interval in milliseconds
    ani = FuncAnimation(fig, update, frames=len(tsimu), init_func=init, blit=True, interval=interval)
    plt.show()

# Call this with desired FPS
run_animation('curva_t_255_d_20', fps=10)