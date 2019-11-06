function [F, acc] = position_controller(state, desired_state, params, question)

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

if (question == 6.2) || (question == 6.3) || (question == 6.5)
    
    I = params.inertia;
    m = params.mass;
    g = params.gravity;
    phi_dot = state.omega(1);
    theta_dot = state.omega(2);  
    psi_dot = state.omega(3);
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
         0 0 0 (I(5) - I(9)) * psi_dot.^2/I(1) 0 0 0 0 0 0 (I(5) - I(9)) * psi_dot / I(1) 0;...
         0 0 0 0 (I(1) - I(5)) * psi_dot.^2/I(5) 0 0 0 0 -(I(1)-I(5))* psi_dot/I(5) 0 0;...
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

    Q = zeros(1, 12);
    R = zeros(1, 4);
    
    if question == 6.2
        Q = diag([500, 200, 200, 100, 100, 100, 1, 1, 1, 1, 1, 1]);
        R = diag([80, 30, 75, 15]);
    elseif question == 6.3
        Q = diag([10, 30, 1000, 10, 10, 10, 10, 10, 20, 1, 1, 1]);
        R = diag([3, 5, 40, 5]);
    elseif question == 6.5
        Q = diag([10, 10, 500, 10, 10, 100, 30, 30, 20, 1, 1, 1]);
        R = diag([10, 10, 50, 20]);
    end

    K = lqr(A, B, Q, R);

    v = -inv(C * inv(A - B * K) * B) * [desired_state.pos; desired_state.rot(3)];
    u = v - K * [state.pos; state.rot; state.vel; state.omega] + [m * g; 0; 0; 0];

    F = u(1);
    acc = desired_state.acc;

else
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
    ep = state.pos - desired_state.pos;
    ev = state.vel - desired_state.vel;

    % Don't need this, we will just take F(3)
    % b3 = [0; 0; 1];

    F = params.mass * (params.gravity + desired_state.acc - Kp .* ep - Kd .* ev);
    F = F(3);

    % Actual acceleration 
    acc = desired_state.acc - Kp .* ep - Kd .* ev;
end
end
