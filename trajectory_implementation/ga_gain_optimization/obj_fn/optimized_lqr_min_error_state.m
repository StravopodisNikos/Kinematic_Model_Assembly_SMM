function [star] = optimized_lqr_min_error_state(s_dyn,way_pts,time_pts,x)
% Trajectory Implementation on Isotropic Extracted anatomies
% Anatomy optimization was performed in:
% ga_call_anatomy_optimization_isotropy_investigation.m

% In this file the lqr matrices are ga optimized w.r.t:
% Achieving minimal position error for all joints at each task point
% specified
% * Modified on Monday 28/12 -> max_torque_norm is minimized  

% *  WORKS ONLY FOR 3 DOF STRUCTURES
% ** LQR CONTROL IMPLEMENTATION

% This file was developed from:
% optimized_structure_anatomy_trajectory_implementation_custom2.m

% Input: 1. structure string definition 7x1
%        2. structure assembly parameters 4x3 array
%        3. isotropic_anatomy_extracted
%        4. way_pts must be nxp array (n:DoF, p:number of trajectory points)
%        5. time_pts must be px1 array
%        6. chromosome x -> builds the s_lqr struct with Q,R,N matrices

% Output: 1. Position Error Minimization Index

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 
% tic;

rng default % reset the random input

%% Add paths to all functions:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
screws_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','screws',filesep);
screws_library_path = strcat(root_path,screws_path_relative_to_matlab_ws); addpath(screws_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')

util_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','util',filesep);
util_library_path = strcat(root_path,util_path_relative_to_matlab_ws); addpath(util_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')

screw_dynamics_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_dynamics',filesep);
screw_dynamics_library_path = strcat(root_path,screw_dynamics_path_relative_to_matlab_ws); addpath(screw_dynamics_library_path);
% addpath('/home/nikos/matlab_ws/screw_dynamics')

ga_building_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','building_functions',filesep);
ga_building_functions_library_path = strcat(root_path,ga_building_functions_path_relative_to_matlab_ws); addpath(ga_building_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')

ga_synthetic_joints_tfs_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','synthetic_joints_tfs',filesep);
ga_synthetic_joints_tfs_library_path = strcat(root_path,ga_synthetic_joints_tfs_path_relative_to_matlab_ws); addpath(ga_synthetic_joints_tfs_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')

ga_calculateFunctions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','calculateFunctions',filesep);
ga_calculateFunctions_library_path = strcat(root_path,ga_calculateFunctions_path_relative_to_matlab_ws); addpath(ga_calculateFunctions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')

ga_objective_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_objective_functions',filesep);
ga_objective_functions_library_path = strcat(root_path,ga_objective_functions_path_relative_to_matlab_ws); addpath(ga_objective_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions')

ga_optimization_call_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_optimization_call_functions',filesep);
ga_optimization_call_functions_library_path = strcat(root_path,ga_optimization_call_functions_path_relative_to_matlab_ws); addpath(ga_optimization_call_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions')

subroutines_executed_in_objective_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_objective_functions','subroutines_executed_in_objective_fn',filesep);
subroutines_executed_in_objective_fn_library_path = strcat(root_path,subroutines_executed_in_objective_fn_path_relative_to_matlab_ws); addpath(subroutines_executed_in_objective_fn_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions/subroutines_executed_in_objective_fn')

% Add libraries relative to matlab_ws folder
motion_control_state_derivative_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','trajectory_implementation','motion_control_state_derivative_fn',filesep);
motion_control_state_derivative_fn_library_path = strcat(root_path,motion_control_state_derivative_fn_path_relative_to_matlab_ws); addpath(motion_control_state_derivative_fn_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/trajectory_implementation/motion_control_state_derivative_fn')

%% Form s_lqr from chromosome x
q_eps  = [x(1) x(2) x(3)];    % penaltizes high integral error
q_e    = [x(4) x(5) x(6)];    % penaltizes position error
q_de   = [x(7) x(8) x(9)];          % penaltizes velocity error
r_pen1   = x(10);         % only testing
r_pen2   = x(10);     
r_pen3   = x(10);      
field1 = 'Qpen';        value1 = diag(horzcat(q_eps,q_e,q_de)); % 9x9
field2 = 'Rpen';        value2 = diag([r_pen1 r_pen2 r_pen3]);  % 3x3    
field3 = 'Nnonlin';     value3 = zeros(9,3);
s_lqr  = struct(field1,value1,field2,value2,field3,value3);


%% Ovidius robot spaecifications 
dq_max(1) = 10;             %[rad/s]
dq_max(2) = 20;             %[rad/s]
dq_max(3) = 20;             %[rad/s]
ddq_max(1) = 100;           %[rad/s2]
ddq_max(2) = 200;           %[rad/s2]
ddq_max(3) = 200;           %[rad/s2]
torque_limit = [80 45 45]'; % [N*m]
nDoF = 3;

% DEFINE TASK->in main ga call file!

% sampling times is a mx1 vector
dt = 1;
time_samples = time_pts(1):dt:time_pts(length(time_pts));

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

%% EXTRACT TRAJECTORY -> THIS IS REFERENCE FOR EXECUTION COMMANDS
% desired(reference) arrays dimension is: ndofxtask_points (3x5)
[q_d, dq_d, ddq_d, ~] = quinticpolytraj(way_pts, time_pts, time_samples);

%% EXECUTION - USING ODE
ode_calc_points    = 10;
for task_point_cnt=2:size(q_d,2)
    % set new execution time
%     tspan = [time_samples(task_point_cnt-1) time_samples(task_point_cnt)];
    tspan_discrete_ode = linspace(time_samples(task_point_cnt-1),time_samples(task_point_cnt),ode_calc_points);
    
    % set new initial state
    if task_point_cnt == 2
        epsilon0= zeros(3,1);
        x0(1:3) = q_d(:,task_point_cnt-1);
        x0(4:6) = dq_d(:,task_point_cnt-1);
    else
        epsilon0= last_epsilon';
        x0(1:3) = last_x(1:3);
        x0(4:6) = last_x(4:6);        
    end
    
    % set new desired state
    x_d     =  vertcat(q_d(:,task_point_cnt), dq_d(:,task_point_cnt));
    
    % build the error state
    epsilon = epsilon0;
    e       = x_d(1:3) - x0(1:3)';
    de      = x_d(4:6) - x0(4:6)';
    e_state0 = vertcat(epsilon,e,de);
    
    % solve the ode
    tic;
    ode_opts = odeset('InitialStep',1e-3,'MaxStep',1e-2,'RelTol',1e-5,'AbsTol',1e-7);
    ode_derivative_Fcn = @(t,e_state) LQR_computed_torque_control_3DoF_MMD(e_state,s_lqr);
    [t,e_state] = ode45(ode_derivative_Fcn, tspan_discrete_ode, e_state0,ode_opts);
    disp('finished ode');
    toc;
    
    % go back to current state (error_state = desired - current | state)
    % and calculate delta qdot norm
    for ode_cnt=1:size(t,1)
        e_c(ode_cnt,:) = e_state(ode_cnt,1:3);
        x_c(ode_cnt,:) = x_d - e_state(ode_cnt,4:9)';
        delta_qdot = abs(abs(dq_max) - abs(x_c(ode_cnt,4:6)));
        norm_delta_qdot(ode_cnt) = norm(delta_qdot);
    end
    
     % calculate torque for the [t,x] trajectory segment calculated by ODE
    Vold = calculatePotentialEnergyMatrix_anat_3dof(x0(1:3),s_dyn.par1,s_dyn.par4,s_dyn.par5);
    norm_delta_torque  = zeros(1,size(t,1));
    epsilon_norm_new = zeros(1,size(t,1));
    for ode_cnt=1:size(t,1)
        [torque(:,ode_cnt),epsilon_norm_new(ode_cnt),Vold] = postODEoutput_LQR_computed_torque_control2_3DoF_MMD(e_state(ode_cnt,:)',x_c(ode_cnt,:)',ddq_d(:,task_point_cnt),dt,s_dyn,s_lqr,Vold);
        delta_torque = abs( abs(torque_limit) - abs(torque(:,ode_cnt)) );
        norm_delta_torque(ode_cnt) = norm(delta_torque);
    end
    
    % save last x to pass as initial state for next step
    last_epsilon = e_c(size(t,1),:);
    last_x       = x_c(size(t,1),:);
    
    % sum delta torques/qdots/epsilons
    norm_delta_torque_final(task_point_cnt) =  sum(norm_delta_torque);
    norm_delta_qdot_final(task_point_cnt) =  sum(norm_delta_qdot);
    epsilon_norm_final(task_point_cnt) = sum(epsilon_norm_new);  
end



% % INDEX1: Mimimize delta norms (average)
% norm_delta_torque_final_star = sum(norm_delta_torque_final)/size(q_d,2);
% norm_delta_qdot_final_star   = sum(norm_delta_qdot_final)/size(q_d,2);
% 
% star = 0.5 * norm_delta_torque_final_star + 0.5 *norm_delta_qdot_final_star;

% % INDEX2: Mimimize epsilon norms (average)
epsilon_norm_final_star = sum(epsilon_norm_final)/size(q_d,2);
star = epsilon_norm_final_star;
% toc;
end