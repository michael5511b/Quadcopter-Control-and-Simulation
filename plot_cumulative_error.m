function [] = plot_cumulative_error(state, state_des,time_vector)
    %% Get individual plots from state

    pos = state(1:3,:);
    vel = state(4:6,:);
    rpy = state(7:9,:);
    ang_vel = state(10:12,:);
    acc = state(13:15,:);

    pos_des = state_des(1:3,:);
    vel_des = state_des(4:6,:);
    rpy_des = state_des(7:9,:);
    ang_vel_des = state_des(10:12,:);
    acc_des = state_des(13:15,:);

    %% Get error from desired and actual

    error_pos = pos-pos_des;
    error_vel = vel-vel_des;
    error_rpy = rpy-rpy_des;
    error_ang_vel = ang_vel-ang_vel_des;
    error_acc = acc - acc_des;
    
    for i = 1:size(error_pos,2)-1
        error_pos(:,i + 1) = error_pos(:,i + 1) + error_pos(:,i);
        error_vel(:,i + 1) = error_vel(:,i + 1) + error_vel(:,i);
        error_rpy(:,i + 1) = error_rpy(:,i + 1) + error_rpy(:,i);
        error_ang_vel(:,i + 1) = error_ang_vel(:,i + 1) + error_ang_vel(:,i);
        error_acc(:,i + 1) = error_acc(:,i + 1) + error_acc(:,i);
    end
    % No error in rpm, assume perfect motor speed control

    %% Plot error

    % Plot position error
    labels = {'x [m]', 'y [m]', 'z [m]'};
    title_name = {'Error in x','Error in y','Error in z'};
    str = 'Plot of Cumulative Error';
    figure('Name',str);

    for i = 1:3
        subplot(5, 3, i)
        plot(time_vector,error_pos(i,:),'r');
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
        title(title_name{i})
    end

    % Plot orientation error

    labels = {'\phi', '\theta', '\psi'};
    title_name = {'Error in \phi','Error in \theta','Error in \psi'};

    for i = 1:3
        subplot(5, 3, i+3)
        plot(time_vector,error_rpy(i,:),'r');
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
        title(title_name{i})
    end

    % Plot velocity error

    labels = {'vx [m/s]', 'vy [m/s]', 'vz [m/s]'};
    title_name = {'Error in v_x','Error in v_y','Error in v_z'};

    for i = 1:3
        subplot(5, 3, i+6)
        plot(time_vector,error_vel(i,:),'r');
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
        title(title_name{i})
    end

    % Plot angular velocity error

    labels = {'\omega x [rad/s]', '\omega y [rad/s]', '\omega z [rad/s]'};
    title_name = {'Error in \omega_x','Error in \omega_y','Error in \omega_z'};

    for i = 1:3
        subplot(5, 3, i+9)
        plot(time_vector,error_ang_vel(i,:),'r');

        grid on
        xlabel('time [s]')
        ylabel(labels{i})
        title(title_name{i})
    end

    labels = {'ax [m/s^2]', 'ay [m/s^2]', 'az [m/s^2]'};
    title_name = {'Error in a_x','Error in a_y','Error in a_z'};

    for i = 1:3
        subplot(5, 3, i+12)
        plot(time_vector,error_acc(i,:),'r');
        grid on
        xlabel('time [s]')
        ylabel(labels{i})
        title(title_name{i})
    end
end