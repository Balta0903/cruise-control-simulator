# cruise-control-simulator
# Cruise Control PID Simulation

## Overview

This project is a cruise control simulation for a vehicle using a PID-style controller. The goal of the simulation is to model how a vehicle changes velocity over time while trying to reach and maintain a target velocity.

The model includes several physical effects, including rolling resistance, aerodynamic drag, road incline, and motor force. The simulation compares the vehicle velocity against a desired target velocity and evaluates basic performance metrics such as overshoot and settling time.

This project was created as a small engineering application of concepts from dynamics and control systems.

## Author

Baltazar Villasol

## Project Goal

The purpose of this project is to apply basic control theory to a physical system. Instead of only working with transfer functions, this simulation uses a numerical model of a vehicle and applies a controller directly to the system.

The main goal is to observe how a PID-style controller affects the vehicle response as it attempts to reach a target velocity.

## System Description

The vehicle is modeled using Newton’s Second Law:

```text
F = ma
```

At each time step, the program calculates the forces acting on the vehicle and updates the velocity numerically.

The main forces included in the model are:

* Motor force
* Rolling resistance
* Aerodynamic drag
* Force due to road incline

The vehicle velocity is updated using a discrete time step.

## Controller Description

The program uses a PID-style controller made of three terms:

```text
Proportional = Kp * error
Integral     = Ki * accumulated error
Derivative   = Kd * rate of change of error
```

The error is calculated as:

```text
error = target velocity - current velocity
```

The controller output is added to a feedforward motor force. The feedforward force is based on the approximate force needed to maintain the target velocity on flat ground.

A scaling constant, `k`, is used to turn the controller on or off and to scale the total controller effect.

```python
motor_force = feedforward_force + k * (proportional + integral + derivative)
```

If `k = 0`, the controller is off and the system behaves mostly like an open-loop model.

If `k > 0`, the controller affects the motor force and attempts to correct the velocity error.

## Parameters

The simulation uses the following main parameters:

| Variable               | Description                                    |
| ---------------------- | ---------------------------------------------- |
| `mass`                 | Vehicle mass in kilograms                      |
| `initial_velocity`     | Starting velocity of the vehicle               |
| `drag_coefficient`     | Aerodynamic drag coefficient used in the model |
| `friction_coefficient` | Rolling friction coefficient                   |
| `g`                    | Acceleration due to gravity                    |
| `time_step`            | Time interval used for numerical simulation    |
| `total_time`           | Total simulation time                          |
| `theta`                | Road angle in radians                          |
| `target_velocity`      | Desired vehicle velocity                       |
| `kp`                   | Proportional controller gain                   |
| `ki`                   | Integral controller gain                       |
| `kd`                   | Derivative controller gain                     |
| `k`                    | Overall controller scaling constant            |

## Simulation Process

The program follows these steps:

1. Define the vehicle parameters and controller gains.
2. Initialize arrays for time, velocity, target velocity, error, and motor force.
3. Calculate the velocity error.
4. Calculate the proportional, integral, and derivative controller terms.
5. Calculate the motor force.
6. Update the vehicle velocity using the force balance equation.
7. Store the new velocity, error, motor force, and time.
8. Repeat the process for each time step.
9. Calculate overshoot and approximate settling time.
10. Plot the velocity, target velocity, and error.

## Outputs

The program prints:

* Overshoot
* Approximate settling time

The program also plots:

* Vehicle velocity vs. time
* Target velocity vs. time
* Error vs. time

## Performance Metrics

### Overshoot

Overshoot is calculated as:

```text
maximum velocity - target velocity
```

This shows how far the vehicle velocity goes above the desired target velocity.

### Settling Time

The settling time is approximated as the first time the error becomes smaller than 5% of the target velocity.

```text
|error| < 0.05 * target velocity
```

This is a simplified settling time calculation. A more complete version would check whether the velocity enters the 5% range and stays there for the rest of the simulation.

## Assumptions

This simulation makes several simplifying assumptions:

* The vehicle is modeled as a point mass.
* Drag is proportional to velocity squared.
* Rolling resistance is based on the normal force.
* The feedforward motor force is calculated for flat-ground motion.
* Road incline can be added as a disturbance using `theta`.
* The simulation uses a fixed time step.
* The controller acts directly on motor force.

## How to Run

Make sure Python is installed with the required libraries:

```bash
pip install numpy matplotlib
```

Then run the program:

```bash
python cruise_control_simulation.py
```

## Required Libraries

This project uses:

```python
math
numpy
matplotlib
```

## Example Use

The target velocity can be changed by modifying:

```python
target_velocity = 50
```

The road angle can be changed by modifying:

```python
theta = math.radians(0)
```

For example, to simulate a 5-degree incline:

```python
theta = math.radians(5)
```

The controller gains can be adjusted by modifying:

```python
kp = 35
ki = 0.01
kd = 0
k = 10
```

Changing these values allows the user to observe how the controller affects the vehicle response.

## Conclusion

This project demonstrates how control theory can be applied to a simple vehicle cruise control problem. By modeling the vehicle forces and applying a PID-style controller, the simulation shows how feedback can be used to reduce velocity error and help the vehicle reach a desired target speed.

