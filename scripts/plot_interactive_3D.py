import pandas as pd
import plotly.graph_objects as go
import os

# Load trajectory data
csv_path = "data/bicycle_model_log.csv"
df = pd.read_csv(csv_path)

# Setup figure
fig = go.Figure()

# Add car trajectory
fig.add_trace(go.Scatter3d(
    x=df["x"],
    y=df["y"],
    z=df["theta"],  # Use heading as height for visualization
    mode='lines+markers',
    marker=dict(size=3, color=df["step"], colorscale='Viridis'),
    line=dict(color='blue', width=2),
    name="Vehicle Trajectory"
))

# Add reference path
if "x_ref" in df.columns and "y_ref" in df.columns and "theta_ref" in df.columns:
    fig.add_trace(go.Scatter3d(
        x=df["x_ref"],
        y=df["y_ref"],
        z=df["theta_ref"],
        mode='lines',
        line=dict(color='red', dash='dash'),
        name="Reference Path"
    ))

# Layout tweaks
fig.update_layout(
    scene=dict(
        xaxis_title='X [m]',
        yaxis_title='Y [m]',
        zaxis_title='Theta [rad]',
        aspectratio=dict(x=1, y=1, z=0.5),
    ),
    title="3D Interactive Trajectory",
    margin=dict(l=0, r=0, b=0, t=40)
)

# Save HTML
output_path = "data/trajectory_3d_interactive.html"
fig.write_html(output_path)
print(f"✅ 3D interactive plot saved at: {output_path}")
