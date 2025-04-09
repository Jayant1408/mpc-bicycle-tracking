import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import os

# Load the CSV
csv_path = "data/bicycle_model_log.csv"
df = pd.read_csv(csv_path)

# ========== 1. XY Plot ==========
plt.figure()
plt.plot(df["x"], df["y"], label="Robot Path", color='blue')
plt.xlabel("x [m]")
plt.ylabel("y [m]")
plt.title("Bicycle Model XY Trajectory")
plt.grid(True)
plt.axis("equal")
plt.legend()
plt.savefig("data/trajectory_xy.png")

# ========== 2. Time-Series State Plots ==========
plt.figure(figsize=(10, 8))

plt.subplot(4, 1, 1)
plt.plot(df["step"], df["x"], label="x")
plt.ylabel("x [m]")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 2)
plt.plot(df["step"], df["y"], label="y")
plt.ylabel("y [m]")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 3)
plt.plot(df["step"], df["theta"], label="theta")
plt.ylabel("theta [rad]")
plt.grid(True)
plt.legend()

plt.subplot(4, 1, 4)
plt.plot(df["step"], df["v"], label="v")
plt.ylabel("v [m/s]")
plt.xlabel("Time Step")
plt.grid(True)
plt.legend()

plt.tight_layout()
plt.savefig("data/trajectory_states.png")

# ========== 3. Animated Path ==========
fig, ax = plt.subplots()
ax.set_title("Bicycle Model XY Motion Animation")
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

# Save the animation
# ani.save("data/trajectory_animation.mp4", writer='ffmpeg', fps=10)
# Save as GIF instead of MP4
ani.save("data/trajectory_animation.gif", writer='pillow', fps=10)

print("All plots and animation saved to: data/")
