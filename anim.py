import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

def read_file(filename):
    data = np.loadtxt(filename)
    return data[:, 0], data[:, 7], data[:, 8], data[:, 9], data[:, 10], data[:, 11], data[:, 12]

def init():
    lines['cg'].set_data([], [])
    lines['fw'].set_data([], [])
    lines['rw'].set_data([], [])
    return lines.values()

def animate(i):
    lines['cg'].set_data(x[:i], y[:i])
    lines['fw'].set_data(xef[i], yef[i])
    lines['rw'].set_data(xer[i], yer[i])
    return lines.values()

t, x, y, xef, yef, xer, yer = read_file(os.path.join('results','curva_t_255_d_20','sim.dat'))

fig, ax = plt.subplots()
ax.set_xlim(np.min(x), np.max(x))  # Adjust x-axis limits as needed
ax.set_ylim(np.min(y), np.max(y))  # Adjust y-axis limits as needed
lines = {'cg': ax.plot([], [], 'k-', color='blue', label='CG')[0],
         'fw': ax.plot([], [], 'bo', color='red', label='FW')[0],
         'rw': ax.plot([], [], 'ro', color='green', label='RW')[0]}

# Create the animation
ani = FuncAnimation(fig, animate, frames=len(t), init_func=init, blit=True, interval = 10)
ax.legend()

ani.save(os.path.join('results','curva_t_255_d_20','anim.mp4'), fps=30, extra_args=['-vcodec', 'libx264'])
plt.show()
