# Two-wheeld self-balancing robot

This project is a two-wheeled self-balancing robot (TWSBR).
The simulations are based in OpenModelica and MATLAB.
The controllers implemented are:

- Proportional-Integral-Derivative (PID)
- Linear Quadratic Regulator (LQR)
- Nonlinear Model Predictive Control (NMPC)

## Usage

### OpenModelica

Open Modelica is used for several experiments and the models of the system. The files are distributed as.

- `SegwaySystem.mo` is the document that holds all the components together and the one on which the experiments where made.
- `SegwayDynamics.mo` contains the dynamics of the system, you can change parameters of the model, initial statesand disturbances. As input, it accepts torque and as output the state systems.
- `LQRController.mo` is the controller, you can set the offline gains, max absolute torque of the motors. As input tou have the feedback, the setpoints and as output the torques.
- `NoiseMatrix.mo` is the component that add a drift noise to the model.

To run the code, load the four codes into open modelica and simulate `SegwaySystem.mo`.

For getting new LQR gains, use the matlab file `calculate_lqr_gains.m`, this gets the optimal k_gains to put basde

### MATLAB

MATLAB is used to validate the dynamics of the system.
At the same time, it is used to simulate the MPC controller for both stabilization and path following.
The MPC has been implemented as a pitch, yaw, and linear velocity controller.
It does not control the position as the desire is just to provide a path to follow but without time restrictions.

- The file `twsbr_dynamics.m` contains the function that describes the dynamics of the system.
- The file `twsbr_lqr.m` contains the simulation of the LQR controller. Just hit play.
- The file `twsbr_nmpc.m` contains the simulation of the NMPC controller with white noise and drift simulation. Hit play.
- The file `twsbr_nmpc_global.m` contains the simulation of the NMPC controller without noise or drift. It attempts to follow different paths (e.g. line, circle, sinusoidal, figure8).

Additionally, a simulation of the ground is added to check if the robot hits the ground or not.
See `ground_contact_event.m`.
