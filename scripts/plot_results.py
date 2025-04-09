import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# Load the data
df = pd.read_csv("data/bicycle_model_log.csv")

# ========== 1. XY Trajectory ==========
plt.figure()
plt.plot(df["x"], df["y"], label="Bicycle Path", color='blue')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Bicycle Model XY Trajectory")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.savefig("data/bicycle_xy_trajectory.png")

# ========== 2. Time-Series Plots ==========
plt.figure(figsize=(10, 6))

plt.subplot(3, 1, 1)
plt.plot(df["step"], df["x"], label="x")
plt.ylabel("x [m]")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(df["step"], df["y"], label="y")
plt.ylabel("y [m]")
plt.grid(True)
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(df["step"], df["theta"], label="theta", color='orange')
plt.xlabel("Time Step")
plt.ylabel("theta [rad]")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig("data/bicycle_states.png")

# ========== 3. Animated Trajectory ==========
fig, ax = plt.subplots()
ax.set_title("Bicycle Model Animation")
ax.set_xlabel("x [m]")
ax.set_ylabel("y [m]")
ax.set_xlim(df["x"].min() - 1, df["x"].max() + 1)
ax.set_ylim(df["y"].min() - 1, df["y"].max() + 1)
ax.grid(True)

robot_dot, = ax.plot([], [], 'ro', label="Robot")
path_line, = ax.plot([], [], 'b-', alpha=0.6)

x_data, y_data = [], []

def update(frame):
    x_data.append(df["x"].iloc[frame])
    y_data.append(df["y"].iloc[frame])
    robot_dot.set_data([x_data[-1]], [y_data[-1]])
    path_line.set_data(x_data, y_data)
    return robot_dot, path_line

ani = animation.FuncAnimation(
    fig, update, frames=len(df), interval=100, blit=True
)

ani.save("data/bicycle_animation.gif", writer='pillow', fps=10)
print("✅ Plots and animation saved in: data/")
