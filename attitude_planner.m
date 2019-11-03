function [rot, omega] = attitude_planner(desired_state, params)

% Input parameters
%
%   desired_state: The desired states are:
%   desired_state.pos = [x; y; z], 
%   desired_state.vel = [x_dot; y_dot; z_dot],
%   desired_state.rot = [phi; theta; psi], 
%   desired_state.omega = [phidot; thetadot; psidot]
%   desired_state.acc = [xdotdot; ydotdot; zdotdot];
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   rot: will be stored as desired_state.rot = [phi; theta; psi], 
%
%   omega: will be stored as desired_state.omega = [phidot; thetadot; psidot]
%
%************  ATTITUDE PLANNER ************************

% Write code here
g = params.gravity;

acc_x_des = desired_state.acc(1);
acc_y_des = desired_state.acc(2);

psi_des   = desired_state.rot(3);
phi_des   = (sin(psi_des) * acc_x_des - cos(psi_des) * acc_y_des) / g; 
theta_des = (cos(psi_des) * acc_x_des + sin(psi_des) * acc_y_des) / g; 

rot = [phi_des; theta_des; psi_des];

R = [cos(psi_des), sin(psi_des); -sin(psi_des),cos(psi_des)];
A = [acc_x_des * desired_state.omega(3); acc_y_des * desired_state.omega(3)];
w = R * A / g;
omega = [w(1); w(2); desired_state.omega(3)];




end

