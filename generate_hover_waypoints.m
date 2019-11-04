function waypoints_hover = generate_hover_waypoints(theta, hover_z, numOfPoints)
    
    waypoints_x = zeros(1, numOfPoints);
    waypoints_y = zeros(1, numOfPoints);
    waypoints_z = hover_z * ones(1, numOfPoints);
    waypoints_theta = theta * ones(1, numOfPoints);
    
    waypoints_hover = [waypoints_x, waypoints_y, waypoints_z, waypoints_theta];
    
end