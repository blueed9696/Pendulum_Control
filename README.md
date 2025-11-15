# Pendulum Control

A simulation and control project for an inverted pendulum system using PyBullet physics engine.

## Project Overview

This project implements a single inverted pendulum system with physics simulation and control algorithms. The pendulum consists of a massless rod with a ball at the end, pivoting from a fixed point.

## Project Structure

```
single_inverted_pendulum/
├── model.py          # System parameters and model definition
├── controller.py     # Control algorithms (PD control, LQR placeholder)
└── simulation.py     # PyBullet physics simulation
```

## Components

### Model (`model.py`)
Defines the physical parameters of the pendulum system:
- **rod_length**: Length of the pendulum rod (1.0 m)
- **ball_radius**: Radius of the ball at the end (0.05 m)
- **ball_mass**: Mass of the ball (1.0 kg)
- **rod_thickness**: Thickness of the rod (0.01 m)

### Controller (`controller.py`)
Implements control strategies:
- **PD Control**: Proportional-Derivative controller with tunable gains (Kp, Kd)
- **LQR**: Linear Quadratic Regulator (placeholder for future implementation)

### Simulation (`simulation.py`)
PyBullet-based physics simulation featuring:
- Gravity-based dynamics (g = 9.81 m/s²)
- Multi-body system with revolute joint
- Real-time joint state tracking
- Ball position calculation

## Dependencies

- `pybullet`: Physics simulation engine
- `sympy`: Symbolic mathematics (in model.py)

## Getting Started

1. Install dependencies:
   ```bash
   pip install pybullet sympy
   ```

2. Run the simulation:
   ```bash
   python single_inverted_pendulum/simulation.py
   ```

## Notes

- The simulation currently runs the free pendulum (no control torque applied)
- Initial pendulum angle is set to 170° from vertical
- Simulation timestep: 1/24000 seconds