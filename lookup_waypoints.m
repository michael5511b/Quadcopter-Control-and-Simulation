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
    waypoint_theta = [zeros(1, 400), deg2rad(15) * ones(1, 1600)];
    
    waypoint_x = zeros(1, numOfPoints);
    waypoint_y = zeros(1, numOfPoints);
    % z: 1m to 1.1m
    waypoint_z = [zeros(1, 400), ones(1, 1600) * 0.1];
    trajectory = [waypoint_x; waypoint_y; waypoint_z; waypoint_theta];  
    
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
    waypoint_theta = [zeros(1, 400), deg2rad(15) * ones(1, 1600)];
    
    waypoint_x = zeros(1, numOfPoints);
    waypoint_y = zeros(1, numOfPoints);
    % z: 1m to 1.1m
    waypoint_z = [zeros(1, 400), ones(1, 1600) * 0.1];
    trajectory = [waypoint_x; waypoint_y; waypoint_z; waypoint_theta];  
    
    % state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t)
    [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t);
    
elseif question == 7
    
end

end
