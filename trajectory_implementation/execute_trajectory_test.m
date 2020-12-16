% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
motion_control_state_derivative_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','trajectory_implementation','motion_control_state_derivative_fn',filesep);
motion_control_state_derivative_fn_library_path = strcat(root_path,motion_control_state_derivative_fn_path_relative_to_matlab_ws); addpath(motion_control_state_derivative_fn_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/trajectory_implementation/motion_control_state_derivative_fn')

clear;
clc;
close all;

%% Ovidius robot spaecifications 
dq_max(1) = 10; %[rad/s]
dq_max(2) = 20; %[rad/s]
dq_max(3) = 20; %[rad/s]
ddq_max(1) = 100; %[rad/s2]
ddq_max(2) = 200; %[rad/s2]
ddq_max(3) = 200; %[rad/s2]
nDoF = 3;

%% Define a C-space trajectory using a 5th order polynomial interpolation
% as specified @ : https://www.mathworks.com/help/robotics/ref/quinticpolytraj.html

% wpts must be nxp array
way_pts = [1.5708 0.84 0.15 -0.7854 -1.5708 ;...    %q1
           1.5708 1.35 0.58  0.7854  1.5708 ;...   %q2
           1.5708 2.14 2.45  1.5708  0 ];...       %q3
% tpts must be px1 vector        
time_pts = [0 1 2 3 4]';

% sampling times is a mx1 vector
time_samples = time_pts(1):0.01:time_pts(length(time_pts));

% Set angular velocity boundaries as a nxp array
dq_bound = zeros(nDoF,length(time_pts));
dq_bound_set =  dq_max'; 
for i_cnt=1:length(time_pts)
    dq_bound(:,i_cnt) = dq_bound_set;
end

% Set angular acceleration boundaries as a nxp array
ddq_bound = zeros(nDoF,length(time_pts));
ddq_bound_set =  ddq_max'; 
for i_cnt=1:length(time_pts)
    ddq_bound(:,i_cnt) = ddq_bound_set;
end

% Trajectory:
[q_d, dq_d, ddq_d, pp] = quinticpolytraj(way_pts, time_pts, time_samples);

%% Solve ode for 1st pair of points at the same time step(just for example)
tspan = [time_pts(1) time_pts(2)];
%tspan = time_pts(1):0.1:time_pts(2);
dt = tspan(length(tspan)) - tspan(1);
epsilon_previous = 0;
x0(1:3) = q_d(:,1);
x0(4:6) = dq_d(:,1);
x_d =  vertcat(q_d(:,101), dq_d(:,101));
ode_derivative_Fcn = @(t,x) PID_computed_torque_control_3DoF_MMD(x_d,ddq_d(:,101),dt,x);
% options = odeset('OutputFcn',@(t,x,flag) TorqueOutputFcn_PID_computed_torque_control_3DoF_MMD(t,x,flag,x_d,ddq_d(:,1)) );
[t,x] = ode113(ode_derivative_Fcn, tspan, x0);

[~,real_torque] = PID_computed_torque_control_3DoF_MMD(x_d,ddq_d(:,101),dt,x(1,:)')