function [F_motor,M_motor,rpm_motor_dot] = motor_model(F,M,motor_rpm,params)

% Input parameters
% 
%   F,M: required force and moment
%
%   motor_rpm: current motor RPM
%
%   params: Quadcopter parameters
%
% Output parameters
%
%   F_motor: Actual thrust generated by Quadcopter's Motors
%
%   M_motor: Actual Moment generated by the Quadcopter's Motors
%
%   rpm_dot: Derivative of the RPM
%
%************ MOTOR MODEL ************************

% Setup matrix with thrust and moments
f = [F;M];




% Setup the equations of motion matrix
ct = params.thrust_coefficient;
cq = params.moment_scale;
d = params.arm_length;
mat = [ct, ct, ct, ct; 0, d*ct, 0, -d*ct; -d*ct, 0, d*ct, 0;-cq, cq, -cq, cq];


% This can run WAY faster if I figure out the inverse representation of
% these coefficients on paper and implement it, not using the inv()
% function every iteration!
mat_inv = [1 / (4 * ct), 0, -1 / (2 * ct * d), -1 / (4 * cq);
           1 / (4 * ct), 1 / (2 * ct * d), 0, 1 / (4 * cq);
           1 / (4 * ct), 0, 1 / (2 * ct * d), -1 / (4 * cq);
           1 / (4 * ct), -1 / (2 * ct * d), 0, 1 / (4 * cq)] ;
% rpm_sqr = inv(mat) * f;
rpm_sqr = mat_inv * f;
rpm_des = sqrt(rpm_sqr);

% Watch out for rpms larger or smaller than the limit!
for i = 1:4
    if rpm_des(i) > params.rpm_max
        rpm_sqr(i) = params.rpm_max ^ 2;
    elseif rpm_des(i) < params.rpm_min
        rpm_sqr(i) = params.rpm_min ^ 2;
    end
end
   
% Calculate actual F and M
f_actual = mat * motor_rpm .^ 2;
F_motor = f_actual(1, :);
M_motor = f_actual(2 : 4, :);

% The motors are a first-ordered controlled subsystem
% Thus we can get w_dot!
km = params.motor_constant;
rpm_motor_dot = km * (rpm_des - motor_rpm);

end
