function [F, acc] = position_controller(current_state, desired_state, params, question)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.pos = [x; y; z], 
%   current_state.vel = [x_dot; y_dot; z_dot],
%   current_state.rot = [phi; theta; psi], 
%   current_state.omega = [phidot; thetadot; psidot]
%   current_state.rpm = [w1; w2; w3; w4];
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
%   question: Question number
%
% Output parameters
%
%   F: u1 or thrust
%
%   acc: will be stored as desired_state.acc = [xdotdot; ydotdot; zdotdot]
%
%************  POSITION CONTROLLER ************************

% Example PD gains
% Q2, trying another gain:
% Kp1 = 17;
% Q3, trying another gain:
% Kd3 = 30;

Kp1 = 13;
Kd1 = 6.6;

Kp2 = 17;
Kd2 = 6.6;

Kp3 = 20;
Kd3 = 9;

% Write code here

% Different gain for each dimension
Kp = [Kp1; Kp2; Kp3];
Kd = [Kd1; Kd2; Kd3];

% Position errors and Velocity errors
ep = current_state.pos - desired_state.pos;
ev = current_state.vel - desired_state.vel;

% Don't need this, we will just take F(3)
% b3 = [0; 0; 1];

F = params.mass * (params.gravity + desired_state.acc - Kp .* ep - Kd .* ev);
F = F(3);

% Actual acceleration 
acc = desired_state.acc - Kp .* ep - Kd .* ev;

end
