function [M] = attitude_controller(state,desired_state,params,question)

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
%   M: u2 or moment [M1; M2; M3]
%
%************  ATTITUDE CONTROLLER ************************

% Example PD gains
Kpphi = 190;
Kdphi = 30;

Kptheta = 198;
Kdtheta = 30;

Kppsi = 80;
Kdpsi = 17.88;

Kp = [Kpphi; Kptheta; Kppsi];
Kd = [Kdphi; Kdtheta; Kdpsi];


% Write code here
eR = state.rot - desired_state.rot;
eW = state.omega - transpose(state.rot) * desired_state.rot * desired_state.omega;
M = params.inertia * (-Kp .* eR - Kd .* eW);

% In the A B C, the x' y' z' and phi theta psi states are flipped


if (question == 6.2) || (question == 6.3) || (question == 6.5)
    
    I = params.inertia;
    m = params.mass;
    g = params.gravity;
    psidot = state.omega(3);
    thetadot = state.omega(2);  
    phidot = state.omega(1);
    phi = state.rot(1);
    theta = state.rot(2);
    psi = state.rot(3);
    
    A = [0 0 0 0 0 0 1 0 0 0 0 0;...
         0 0 0 0 0 0 0 1 0 0 0 0;...
         0 0 0 0 0 0 0 0 1 0 0 0;...
         
         0 0 0 0 0 0 0 0 0 1 0 0;...
         0 0 0 0 0 0 0 0 0 0 1 0;...
         0 0 0 0 0 0 0 0 0 0 0 1;...
         0 0 0 g * sin(psi) g * cos(psi) 0 0 0 0 0 0 0;...
         0 0 0 -g * cos(psi) g * sin(psi) 0 0 0 0 0 0 0;...
         0 0 0 0 0 0 0 0 0 0 0 0;...
         
         0 0 0 (I(5) - I(9)) * psidot.^2/I(1) 0 0 0 0 0 0 (I(5) - I(9)) * psidot / I(1) 0;...
         0 0 0 0 (I(1) - I(5)) * psidot.^2/I(5) 0 0 0 0 -(I(1)-I(5))* psidot/I(5) 0 0;...
         0 0 0 0 0 0 0 0 0 0 0 0];

B = [0 0 0 0;...
    0 0 0 0;...
    0 0 0 0;...
    
    0 0 0 0;...
    0 0 0 0;...
    0 0 0 0;...
    
    0 0 0 0;...
    0 0 0 0;...
    1/m 0 0 0;...
    
    0 1/I(1) 0 0;...
    0 0 1/I(5) 0;...
    0 0 0 1/I(9)];

C = [1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0;...
     0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0];
 
Q = diag([500, 100, 300, 100, 100, 100, 1, 1, 1, 1, 1, 1]);

R = diag([80,40,70,20]);

K = lqr(A, B, Q, R);

v = -inv(C * inv(A - B * K) * B) * [desired_state.pos;desired_state.rot(3)];
u = v - K * [state.pos; state.rot;state.vel;state.omega] + [m * g; 0; 0; 0];
M = u(2:4);




end



end

