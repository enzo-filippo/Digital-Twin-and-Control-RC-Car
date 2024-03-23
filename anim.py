import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import os

rastro = 50

def read_file(filename):
    data = np.loadtxt(filename)
    return data[:, 0], data[:, 7], data[:, 8], data[:, 9], data[:, 10], data[:, 11], data[:, 12]

def init():
    lines['cg'].set_data([], [])
    lines['fw'].set_data([], [])
    lines['rw'].set_data([], [])
    return lines.values()

def animate(i):
    lines['cg'].set_data(x[i], y[i])
    lines['fw'].set_data(xef[i], yef[i])
    lines['rw'].set_data(xer[i], yer[i])
    lines['cgrastro'].set_data(x[:i], y[:i])
    lines['fwrastro'].set_data(xef[i-rastro:i], yef[i-rastro:i])
    lines['rwrastro'].set_data(xer[i-rastro:i], yer[i-rastro:i])
    return lines.values()

t, x, y, xef, yef, xer, yer = read_file(os.path.join('results','curva_t_255_d_20','sim.dat'))

fig, ax = plt.subplots()
ax.set_xlim(np.min(np.concatenate((x,y)))-0.1, np.max(np.concatenate((x,y)))+0.1)  # Adjust x-axis limits as needed
ax.set_ylim(np.min(np.concatenate((x,y)))-0.1, np.max(np.concatenate((x,y)))+0.1)  # Adjust y-axis limits as needed
ax.set_aspect('equal')
lines = {'cg': ax.plot([], [], 'bo', color='blue', label='CG')[0],
         'fw': ax.plot([], [], 'ro', color='red', label='FW')[0],
         'rw': ax.plot([], [], 'go', color='green', label='RW')[0],
         'cgrastro': ax.plot([], [], 'b-', color='blue', label='CG')[0],
         'fwrastro': ax.plot([], [], 'r:', color='red')[0],
         'rwrastro': ax.plot([], [], 'g:', color='green')[0]}

# Create the animation
ani = FuncAnimation(fig, animate, frames=len(t), init_func=init, blit=True, interval = 10)
ax.legend()

ani.save(os.path.join('results','curva_t_255_d_20','anim.mp4'), fps=30, extra_args=['-vcodec', 'libx264'])
plt.show()
