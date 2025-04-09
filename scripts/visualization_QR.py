import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Load the CSV data
df = pd.read_csv("/Users/jayantk/Github_Projects/MPC_Bicycle_Tracking/data/bicycle_model_log.csv")

# Create the plot layout
fig = plt.figure(figsize=(14, 10))
gs = gridspec.GridSpec(3, 2, height_ratios=[1, 1, 1])

# XY trajectory plot
ax0 = fig.add_subplot(gs[0, 0])
ax0.plot(df["x"], df["y"], label="Vehicle", color='blue')
ax0.plot(df["x_ref"], df["y_ref"], '--', label="Reference", color='red')
ax0.set_title("XY Trajectory")
ax0.set_xlabel("x [m]")
ax0.set_ylabel("y [m]")
ax0.axis("equal")
ax0.grid(True)
ax0.legend()

# Control inputs
ax1 = fig.add_subplot(gs[0, 1])
ax1.plot(df["step"], df["a_cmd"], label="Acceleration [a_cmd]")
ax1.plot(df["step"], df["delta_cmd"], label="Steering [delta_cmd]")
ax1.set_title("Control Inputs")
ax1.set_xlabel("Time Step")
ax1.grid(True)
ax1.legend()

# State tracking errors
ax2 = fig.add_subplot(gs[1, :])
ax2.plot(df["step"], df["x_err"], label="x error")
ax2.plot(df["step"], df["y_err"], label="y error")
ax2.plot(df["step"], df["theta_err"], label="theta error")
ax2.plot(df["step"], df["v_err"], label="v error")
ax2.set_title("Tracking Errors")
ax2.set_xlabel("Time Step")
ax2.grid(True)
ax2.legend()

# Heading and speed comparison
ax3 = fig.add_subplot(gs[2, 0])
ax3.plot(df["step"], df["theta"], label="theta")
ax3.plot(df["step"], df["theta_ref"], '--', label="theta_ref")
ax3.set_title("Heading Angle")
ax3.set_xlabel("Time Step")
ax3.grid(True)
ax3.legend()

ax4 = fig.add_subplot(gs[2, 1])
ax4.plot(df["step"], df["v"], label="v")
ax4.plot(df["step"], df["v_ref"], '--', label="v_ref")
ax4.set_title("Velocity")
ax4.set_xlabel("Time Step")
ax4.grid(True)
ax4.legend()

plt.tight_layout()
plt.suptitle("MPC Tracking and Control Analysis", fontsize=16, y=1.02)
plt.subplots_adjust(top=0.94)
save_path = "/Users/jayantk/Github_Projects/MPC_Bicycle_Tracking/data/mpc_summary.png"
plt.savefig(save_path)
plt.show()
save_path


