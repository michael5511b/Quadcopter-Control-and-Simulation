function [waypoints, waypoint_times] = lookup_waypoints(question)
%
% Input parameters
%
%   question: which question of the project we are on 
%      Possible arguments for question: 2, 3, 5, 6.2, 6.3, 6.5, 7, 9, 10
%
% Output parameters
%
%   waypoints: of the form [x; y; z; yaw]
% 
%   waypoint_times: [1 x n] vector of times where n is the number of waypoings, 
%   represents the seconds you should be at each respective waypoint
%
%************ LOOKUP WAYPOINTS ************************

% Write code here

% Universal Time-step
time_step = 0.005;

if question == 2
    waypoints = [0,   0.1, 0.2, 0.3;... 
    			 0,   0,   0,   0;... 
    			 0.5, 0.5, 0.5, 0.5;... 
    			 0,   0,   0,   0];

    waypoint_times = [0, 2, 4, 6];

elseif question == 3
    duration = 10;
    numOfPoints = duration / time_step;
    waypoint_times = linspace(0, 10, numOfPoints);
    
    % Symbolic fuction
    syms acc(t) vel(t) z(t)
    
    % Ramp shaped profile for acceleration
    acc(t) = piecewise(t < 0, 0, (t >= 0 & t < 1), 0.5, (t >= 1 & t < 2), 0, (t >= 2 & t < 3), -0.5, (t >= 3 & t < 4), 0, (t >= 4 & t < 5), -0.5, (t >= 5 & t < 6), 0, (t >= 6 & t < 7), 0.5, (t >= 7), 0) ;
    %acc(t) = piecewise(t < 0, 0, (t >= 0 & t < 2), 0.125, (t >= 2 & t < 4), 0, (t >= 4 & t < 6), -0.125, t >= 6, 0);
    % Velocity is the integral of Acceleration
    vel(t) = int(acc(t), 0, t);
    % Position is the integral of Velocity
    z(t) = int(vel(t), 0, t);
    
    waypts_x = zeros(1, numOfPoints);
    waypts_y = zeros(1, numOfPoints);
    waypts_z = z(waypoint_times);
    waypts_theta = zeros(1, numOfPoints);
    waypts_vel = vel(waypoint_times);
    waypts_acc = acc(waypoint_times);
    
    
    waypoints = [waypts_x; waypts_y; waypts_z; waypts_theta; waypts_vel; waypts_acc];
    
elseif question == 5
    idle_t = 0;
    takeoff_t = 0;
    hover_t = 0;
    track_t = 10;
    land_t = 0; 
    numOfPoints = track_t / time_step;
    
    %(1) 
    % waypoint_theta = zeros(1, numOfPoints);
    
    %(2)
    waypoints_theta = [zeros(1, 400), deg2rad(15) * ones(1, 1600)];
    
    waypoints_x = zeros(1, numOfPoints);
    waypoints_y = zeros(1, numOfPoints);
    % z: 1m to 1.1m
    waypoint_z = [zeros(1, 400), ones(1, 1600) * 0.1];
    trajectory = [waypoints_x; waypoints_y; waypoint_z; waypoints_theta];  
    
    % state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t)
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t);
elseif question == 6.2
    waypoints = [0,   0.1, 0.2, 0.3;... 
    			 0,   0,   0,   0;... 
    			 0.5, 0.5, 0.5, 0.5;... 
    			 0,   0,   0,   0];

    waypoint_times = [0, 2, 4, 6];
    
elseif question == 6.3
    duration = 10;
    numOfPoints = duration / time_step;
    waypoint_times = linspace(0, 10, numOfPoints);
    
    % Symbolic fuction
    syms acc(t) vel(t) z(t)
    
    % Ramp shaped profile for acceleration
    % acc(t) = piecewise(t < 0, 0, (t >= 0 & t < 2), 0.125, (t >= 2 & t < 4), 0, (t >= 4 & t < 6), -0.125, t >= 6, 0);
    acc(t) = piecewise(t < 0, 0, (t >= 0 & t < 1), 0.5, (t >= 1 & t < 2), 0, (t >= 2 & t < 3), -0.5, (t >= 3 & t < 4), 0, (t >= 4 & t < 5), -0.5, (t >= 5 & t < 6), 0, (t >= 6 & t < 7), 0.5, (t >= 7), 0) ;
    % Velocity is the integral of Acceleration
    vel(t) = int(acc(t), 0, t);
    % Position is the integral of Velocity
    z(t) = int(vel(t), 0, t);
    
    waypts_x = zeros(1, numOfPoints);
    waypts_y = zeros(1, numOfPoints);
    waypts_z = z(waypoint_times);
    waypts_theta = zeros(1, numOfPoints);
    waypts_vel = vel(waypoint_times);
    waypts_acc = acc(waypoint_times);
    
    
    waypoints = [waypts_x; waypts_y; waypts_z; waypts_theta; waypts_vel; waypts_acc];
    
elseif question == 6.5
    idle_t = 0;
    takeoff_t = 0;
    hover_t = 0;
    track_t = 10;
    land_t = 0; 
    numOfPoints = track_t / time_step;
    
    %(1) 
    % waypoint_theta = zeros(1, numOfPoints);
    
    %(2)
    waypoints_theta = [zeros(1, 400), deg2rad(15) * ones(1, 1600)];
    
    waypoints_x = zeros(1, numOfPoints);
    waypoints_y = zeros(1, numOfPoints);
    % z: 1m to 1.1m
    waypoint_z = [zeros(1, 400), ones(1, 1600) * 0.1];
    trajectory = [waypoints_x; waypoints_y; waypoint_z; waypoints_theta];  
    
    % state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t)
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t);
    
elseif question == 7
    idle_t = 0;
    takeoff_t = 1;
    hover_t = 2;
    track_t = 5;
    land_t = 2; 

    numOfPoints = track_t / time_step;
    
    waypoints_x = zeros(1, numOfPoints);
    waypoints_y = zeros(1, numOfPoints);
    waypoints_z = linspace(1, 10, numOfPoints);
    waypoints_theta = zeros(1, numOfPoints);
    
    trajectory = [waypoints_x; waypoints_y; waypoints_z; waypoints_theta];
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t);
    
elseif question == 8
    r1 = 2; 
    r2 = 1;
    
    idle_t = 0;
    takeoff_t = 0.5;
    hover_t = 0;
    track_t = 9.42;
    land_t = 2; 
    numOfPoints = track_t / time_step;
    
    t = linspace(0,track_t,numOfPoints);
    waypoints_x = r1 * cos(t - pi/2);
    waypoints_y = r2 * sin(t - pi/2) + 1;
    waypoints_z = ones(1, numOfPoints) * 1;
    waypoints_theta = zeros(1, numOfPoints);
    trajectory = [waypoints_x; waypoints_y; waypoints_z; waypoints_theta];   
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t,land_t);
    
    x_dot = linspace(0, 1, numOfPoints);
    waypoints_x_vel = zeros(1, size(waypoints, 2));
    waypoints_y_vel = zeros(1, size(waypoints, 2));
    waypoints_x_vel(1, (takeoff_t + hover_t) / time_step + 1 : (takeoff_t + hover_t + track_t) / time_step) = x_dot;
    waypoints = [waypoints; waypoints_x_vel; waypoints_y_vel];
    
elseif question == 8.2
    r1 = 2; 
    r2 = 1;
    
    idle_t = 0;
    takeoff_t = 0.5;
    hover_t = 0;
    track_t = 10;
    land_t = 2; 
    numOfPoints = track_t / time_step;

    step = 0.005;
    t1 = 0 : step : 2.5;
    t2 = 2.5 : step : 5;
    t3 = 5 : step : 7.5;
    t4 = 7.5 : step : 10;

    spl = spline([0 2.5],[0 0 1 1]);

    quad_1 = ppval(spl, t1);
    quad_2 = 2 - flip(quad_1(:, 2 : end));
    quad_3 = flip(quad_2(:, 2 : end));
    quad_4 = flip(quad_1(:, 2 : end));
    
    y1 = [quad_1, quad_2];
    y2 = [quad_3, quad_4];
    x1 = sqrt((1 - (y1 - 1) .^ 2 / r2) * r1 ^ 2);
    x2 = -sqrt((1 - (y2 - 1) .^ 2 / r2) * r1 ^ 2);

    waypoints_x =[x1, x2];
    waypoints_y = [y1, y2];
    waypoints_z = ones(1, numOfPoints) * 1;
    waypoints_theta = zeros(1, numOfPoints);
    
    x_dot = gradient(waypoints_x) / step;
    y_dot = gradient(waypoints_y) / step;

    trajectory = [waypoints_x;waypoints_y;waypoints_z;waypoints_theta];
 
    X_DOT = [x_dot,x_dot,x_dot,x_dot];
    Y_DOT = [y_dot,y_dot,y_dot,y_dot];
    trajectory = [trajectory, trajectory, trajectory, trajectory];
    track_t = track_t * 4;
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t);

    waypoints_x_vel = zeros(1, size(waypoints,2));
    waypoints_y_vel = zeros(1, size(waypoints,2));
    waypoints_x_vel(1, (takeoff_t + hover_t) / time_step + 1 : (takeoff_t + hover_t + track_t) / time_step) = X_DOT;
    waypoints_y_vel(1, (takeoff_t + hover_t) / time_step + 1 : (takeoff_t + hover_t + track_t) / time_step) = Y_DOT;
    waypoints = [waypoints; waypoints_x_vel; waypoints_y_vel];
    
end

end
