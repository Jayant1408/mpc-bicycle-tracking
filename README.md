# 🚗 MPC Bicycle Tracking Simulation

This project simulates trajectory tracking using **Model Predictive Control (MPC)** for a nonlinear **kinematic bicycle model**. It supports tracking various reference trajectories including:

- 🔁 Sine wave paths  
- 🔵 Circular trajectories  
- 🔀 Figure-8 loops  

Control inputs are optimized at each step using a dense QP formulation, and results are logged for visualization.

---

## 📂 Project Structure

```
MPC_Bicycle_Tracking/
├── CMakeLists.txt
├── config.yaml              # YAML-based simulation configuration
├── src/
│   ├── main.cpp             # Main simulation loop
│   ├── mpc.cpp/hpp          # MPC solver (dense QP)
│   ├── bicycle_model.cpp/hpp# Kinematic model
│   └── reference_generator.cpp/hpp
├── data/                    # CSV log output
├── scripts/                 # Python plots and animations
├── dockerfile               # Optional container setup
├── media/                   # Demo gifs & plots for README
```

---

## 🛠️ How to Build & Run

```bash
mkdir build && cd build
cmake ..
make -j
./bicycle_mpc_sim
```

---

## ⚙️ Configuration (`config.yaml`)

```yaml
simulation:
  dt: 0.1
  L: 2.5
  total_steps: 150
  prediction_horizon: 40

cost:
  Q: [800.0, 1000.0, 200.0, 100.0]
  R: [100.0, 5.0]

initial_state:
  x: 0.0
  y: 0.0
  theta: 0.0
  v: 1.0

trajectory_mode: figure8   # Options: sine, circle, figure8
```

---

## 📈 Visualizations

Use Python scripts in `scripts/` to plot:

- XY path tracking
- State and control input evolution
- Tracking error over time
- Animated and interactive visualizations

---

## 🎥 Demo

![Trajectory Animation](media/bicycle_animation.gif)

---

## 🧭 XY Path Tracking

![XY Trajectory](media/bicycle_xy_trajectory.png)

---

## 📊 MPC Summary Plot

![State & Error Summary](media/mpc_summary.png)

---

## 📋 License

This project is licensed under the [MIT License](LICENSE).

---

## 👤 Author

**Jayant Kumar**  
🔗 [GitHub](https://github.com/Jayant1408)
