function [waypoints, waypoint_times] = state_machine(trajectory, time_step, track_t, idle_t, takeoff_t, hover_t, land_t)

    %% Variable Setup
    numOfPoints_idle = idle_t / time_step;
    numOfPoints_takeoff = takeoff_t / time_step;
    numOfPoints_hover = hover_t / time_step;
    numOfPoints_land = land_t / time_step;

    %% Idle (Not used for now)
    waypoints_x = zeros(1, numOfPoints_idle);
    waypoints_y = zeros(1, numOfPoints_idle);
    waypoints_z = zeros(1, numOfPoints_idle);
    waypoints_theta = zeros(1, numOfPoints_idle);
    
    waypoints_idle = [waypoints_x; waypoints_y; waypoints_z; waypoints_theta];
    
    %% Takeoff
    % Use the first z coordinate as hover height
    hover_z = trajectory(3, 1);
    
    waypoints_x = zeros(1, numOfPoints_takeoff);
    waypoints_y = zeros(1, numOfPoints_takeoff);
    waypoints_z = linspace(0, hover_z, numOfPoints_takeoff);
    waypoints_theta = zeros(1, numOfPoints_takeoff);
    
    waypoints_takeoff = [waypoints_x; waypoints_y; waypoints_z; waypoints_theta];

    
    %% Hover
    % hover takes place after take off and before landing
    waypoints_after_takeoff = generate_hover_waypoints(trajectory(4, 1), trajectory(3, 1), numOfPoints_hover);
    waypoints_before_land = generate_hover_waypoints(trajectory(4, end), trajectory(3, end), numOfPoints_hover);


    %% Land 
    hover_z = trajectory(3,end);
    waypoints_x = zeros(1, numOfPoints_land);
    waypoints_y = zeros(1, numOfPoints_land);
    waypoints_z = linspace(hover_z, 0, numOfPoints_land);
    waypoints_theta = linspace(trajectory(4, end), 0, numOfPoints_land);
    
    waypoints_land = [waypoints_x; waypoints_y; waypoints_z; waypoints_theta];



    %% Combine
    
    total_time = takeoff_t + hover_t + track_t + hover_t + land_t;
    total_point_nums = total_time / time_step;
    waypoint_times = linspace(0,total_time,total_point_nums);
    
    waypoints = [waypoints_takeoff, waypoints_after_takeoff, trajectory, waypoints_before_land, waypoints_land];

end