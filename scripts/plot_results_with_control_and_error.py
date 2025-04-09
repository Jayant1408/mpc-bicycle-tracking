import pandas as pd
import matplotlib.pyplot as plt

# Load data
df = pd.read_csv("data/bicycle_model_log.csv")

# === 1. Trajectory Plot ===
plt.figure()
plt.plot(df["x"], df["y"], label="Actual Trajectory")
plt.plot(df["x_ref"], df["y_ref"], '--', label="Reference Trajectory", color='red')
plt.xlabel("X [m]")
plt.ylabel("Y [m]")
plt.title("Trajectory Tracking")
plt.legend()
plt.grid()

# === 2. Control Inputs ===
plt.figure()
plt.subplot(2, 1, 1)
plt.plot(df["step"], df["a_cmd"])
plt.ylabel("Acceleration [m/s²]")
plt.title("Control Inputs")
plt.grid()

plt.subplot(2, 1, 2)
plt.plot(df["step"], df["delta_cmd"])
plt.ylabel("Steering Angle [rad]")
plt.xlabel("Step")
plt.grid()

# === 3. Tracking Errors ===
plt.figure()
plt.subplot(4, 1, 1)
plt.plot(df["step"], df["x_err"])
plt.ylabel("X error [m]")
plt.title("Tracking Errors")
plt.grid()

plt.subplot(4, 1, 2)
plt.plot(df["step"], df["y_err"])
plt.ylabel("Y error [m]")
plt.grid()

plt.subplot(4, 1, 3)
plt.plot(df["step"], df["theta_err"])
plt.ylabel("Theta error [rad]")
plt.grid()

plt.subplot(4, 1, 4)
plt.plot(df["step"], df["v_err"])
plt.ylabel("Velocity error [m/s]")
plt.xlabel("Step")
plt.grid()

plt.tight_layout()
plt.tight_layout()
plt.savefig("data/simulation_results.png", dpi=300)
print("✅ Plot saved to data/simulation_results.png")

