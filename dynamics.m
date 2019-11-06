function [state_dot] = dynamics(params, state, F, M, rpm_motor_dot)
% Input parameters
% 
%   state: current state, will be using ode45 to update
%
%   F, M: actual force and moment from motor model
%
%   rpm_motor_dot: actual change in RPM from motor model
% 
%   params: Quadcopter parameters
%
%   question: Question number
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

% Write code here

% Just for reference:
% States:
% state(1)  = waypoints(1,1); %x
% state(2)  = waypoints(2,1); %y
% state(3)  = waypoints(3,1); %z
% state(4)  =  0;             %xdot
% state(5)  =  0;             %ydot
% state(6)  =  0;             %zdot
% state(7) =   0;             %phi
% state(8) =   0;             %theta
% state(9) =   waypoints(4,1);%psi
% state(10) =  0;             %phidot 
% state(11) =  0;             %thetadot
% state(12) =  0;             %psidot
% state(13:16) =  0;          %rpm

% Params:
% 'mass',                   0.770, ...
% 'gravity',                9.80665, ...
% 'arm_length',           0.1103, ...
% 'motor_spread_angle',     0.925, ...
% 'thrust_coefficient',     8.07e-9, ...
% 'moment_scale',           1.3719e-10, ...
% 'motor_constant',        36.5, ...
% 'rpm_min',             3000, ...
% 'rpm_max',            20000, ...
% 'inertia',                diag([0.0033 0.0033 0.005]),...
% 'COM_vertical_offset',     0.05);

% Setup parameters
mass = params.mass;
g = params.gravity;
I = params.inertia;
phi   = state(7);
theta = state(8);
psi   = state(9);
state_dot = zeros(16, 1); 

% % Non-linear model:
% Rz = [1, 0, 0; 0, cos(phi), -sin(phi); 0, sin(phi), cos(phi)];
% Ry = [cos(theta), 0, sin(theta); 0, 1, 0; -sin(theta), 0,cos(theta)];
% Rx = [cos(psi), -sin(psi), 0; sin(psi), cos(psi), 0; 0,0,1];
% Reb = Rz * Ry * Rx;
% Fb = [0; 0; F];
% Fe = Reb * Fb - [0; 0; mass * g];
% acc = Fe / mass;
% x_dotdot = acc(1);
% y_dotdot = acc(2);
% z_dotdot = acc(3);

% From slide 4 page 7 (linearization):
x_dotdot = g * (theta * cos(psi) + phi * sin(psi));
y_dotdot = g * (theta * sin(psi) - phi * cos(psi));
z_dotdot = F / mass - g;
if (state(3) == 0) && (F < mass * g)
    F = mass * g;
end
z_dotdot = F / mass - g;

% M = I * alpha
alpha = inv(I) * M;


% Setup state_dot:
state_dot(1)     = state(4); % x_dot
state_dot(2)     = state(5); % y_dot
state_dot(3)     = state(6); % z_dot
state_dot(4)     = x_dotdot;
state_dot(5)     = y_dotdot;  
state_dot(6)     = z_dotdot;  
state_dot(7)     = state(10); % phi_dot
state_dot(8)     = state(11); % theta_dot
state_dot(9)     = state(12); % psi_dot
state_dot(10)    = alpha(1);  % phi_dot_dot
state_dot(11)    = alpha(2);  % theta_dot_dot
state_dot(12)    = alpha(3);  % psi_dot_dot
state_dot(13:16) = rpm_motor_dot;

end