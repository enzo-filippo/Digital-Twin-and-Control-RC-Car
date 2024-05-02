import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation

def draw_car(ax, x, y):
    """Draws a simple car at position (x, y) on the given axes."""
    car_width = 1.0
    car_height = 0.4
    car_length = 2.0

    # Car body
    ax.plot_surface(np.array([[x - car_length / 2, x + car_length / 2],
                               [x - car_length / 2, x + car_length / 2]]),
                    np.array([[y - car_width / 2, y - car_width / 2],
                               [y + car_width / 2, y + car_width / 2]]),
                    np.array([[0, 0],
                               [0, 0]]),
                    alpha=0.6, color='blue')

    # Windows
    ax.plot_surface(np.array([[x - car_length / 4, x + car_length / 4],
                               [x - car_length / 4, x + car_length / 4]]),
                    np.array([[y - car_width / 4, y - car_width / 4],
                               [y + car_width / 4, y + car_width / 4]]),
                    np.array([[car_height, car_height],
                               [car_height, car_height]]),
                    alpha=0.4, color='lightblue')

    # Wheels
    wheel_radius = 0.2
    ax.plot_surface(np.array([[x - car_length / 2, x - car_length / 2],
                               [x - car_length / 2, x - car_length / 2]]),
                    np.array([[y - car_width / 2 - wheel_radius, y - car_width / 2 - wheel_radius],
                               [y - car_width / 2, y - car_width / 2]]),
                    np.array([[0, 0],
                               [-wheel_radius, wheel_radius]]),
                    alpha=0.8, color='black')

    ax.plot_surface(np.array([[x - car_length / 2, x - car_length / 2],
                               [x - car_length / 2, x - car_length / 2]]),
                    np.array([[y + car_width / 2 + wheel_radius, y + car_width / 2 + wheel_radius],
                               [y + car_width / 2, y + car_width / 2]]),
                    np.array([[0, 0],
                               [-wheel_radius, wheel_radius]]),
                    alpha=0.8, color='black')

    ax.plot_surface(np.array([[x + car_length / 2, x + car_length / 2],
                               [x + car_length / 2, x + car_length / 2]]),
                    np.array([[y - car_width / 2 - wheel_radius, y - car_width / 2 - wheel_radius],
                               [y - car_width / 2, y - car_width / 2]]),
                    np.array([[0, 0],
                               [-wheel_radius, wheel_radius]]),
                    alpha=0.8, color='black')

    ax.plot_surface(np.array([[x + car_length / 2, x + car_length / 2],
                               [x + car_length / 2, x + car_length / 2]]),
                    np.array([[y + car_width / 2 + wheel_radius, y + car_width / 2 + wheel_radius],
                               [y + car_width / 2, y + car_width / 2]]),
                    np.array([[0, 0],
                               [-wheel_radius, wheel_radius]]),
                    alpha=0.8, color='black')

def animate_car(x, y, vx, vy, t):
    """Animates a car moving along the given trajectory."""
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_title('3D Car Animation')

    def update(frame):
        """Updates the animation for each frame."""
        ax.clear()
        ax.set_xlim(np.min(x) - 2, np.max(x) + 2)
        ax.set_ylim(np.min(y) - 2, np.max(y) + 2)
        ax.set_zlim(0, 2)  # Assuming the car is always at ground level
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('3D Car Animation')

        draw_car(ax, x[frame], y[frame])

    # Creating animation
    ani = FuncAnimation(fig, update, frames=range(len(t)), blit=False, interval=20)

    plt.show()

