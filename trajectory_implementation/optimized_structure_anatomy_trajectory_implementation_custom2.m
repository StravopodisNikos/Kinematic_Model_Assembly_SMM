function optimized_structure_anatomy_trajectory_implementation_custom2(structure,assembly_parameters,isotropic_anatomy_extracted,way_pts,time_pts,s_lqr,plot_title)
% Trajectory Implementation on Isotropic Extracted anatomies
% Anatomy optimization was performed in:
% ga_call_anatomy_optimization_isotropy_investigation.m

% *  WORKS ONLY FOR 3 DOF STRUCTURES
% ** LQR CONTROL IMPLEMENTATION

% This file was developed from: optimized_structure_anatomy_trajectory_implementation_test.m 

% Input: 1. structure string definition 7x1
%        2. structure assembly parameters 4x3 array
%        3. isotropic_anatomy_extracted
%        4. way_pts must be nxp array (n:DoF, p:number of trajectory points)
%        5. time_pts must be px1 array
%        6. s_lqr struct with Q,R,N matrices

% Output: 1. ONLY PLOTS TRAJECTORY

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

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

% Add libraries relative to matlab_ws folder
ode_output_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','trajectory_implementation','ode_output_fn',filesep);
ode_output_fn_library_path = strcat(root_path,ode_output_fn_path_relative_to_matlab_ws);  addpath(ode_output_fn_library_path);

% clear;
% clc;
% close all;

%% MASS BALANCED STRUCTURE & ISOTROPIC ANATOMY ARE PASSED AS ARGUMENTS!

% fix anatomy vector for the given structure(it is needed because in ga a fixed number of pseudos is optimized!)
isotropic_anatomy_structure_dependent = calculate_transformed_anatomy_vector(structure,'3dof',isotropic_anatomy_extracted);

%% BUILD REFERENCE ANATOMY
[xi_ai_ref,xi_pj_ref,~,~,~,~,~,~] = structure_assembly_3dof(structure,assembly_parameters);

% BUILD ISOTROPIC ANATOMY
[~,xi_ai_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_no_graph(structure,assembly_parameters,isotropic_anatomy_structure_dependent,xi_pj_ref);

% EXTRACT INERTIAS
[g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);

% CALCULATE Metalink Inertia Matrix|Body frame
[M_b_link_as2] = calculateMetalinkInertiaMatrixBody(g_s_link_as_anat,M_s_link_as_anat);

% DEFINE PARAMETERS TO BE PASSED TO compute_Mij_429_3DoF INSIDE ODE
% DERIVATIVE
for i_cnt=1:size(xi_ai_ref,2)
    g_sli_anat(:,:,i_cnt) = eye(4);
    g_sli_anat(1:3,4,i_cnt) = g_s_link_as_anat(:,1,i_cnt); % par4->tranform matrix of the CoMi of eac link @ ZERO q
end
Pi_for_old(:,:,1) = eye(4); Pi_for_old(:,:,2) = eye(4);    % par3->passive exponentials(it may look dummy but because fn was written for fixed structure and variable anatomy the reference structure twists were recomputed for each anatomy inside the function! poor programming...)
%par1->xi_ai_anat
%par5->M_b_link_as2
% par2+6 are calculated inside ODE DERIVATIVE!
field1 = 'par1'; value1 = xi_ai_anat;
field2 = 'par2'; value2 = zeros(4, 4, size(xi_ai_ref,2)); % just preallocation for expai
field3 = 'par3'; value3 = Pi_for_old;
field4 = 'par4'; value4 = g_sli_anat;
field5 = 'par5'; value5 = M_b_link_as2;
field6 = 'par6'; value6 = zeros(3,1); % just preallocation for dq
s_for_compute_Mij_429_3DoF = struct(field1,value1,field2,value2,field3,value3,field4,value4,field5,value5,field6,value6);

%% Ovidius robot spaecifications 
dq_max(1) = 10; %[rad/s]
dq_max(2) = 20; %[rad/s]
dq_max(3) = 20; %[rad/s]
ddq_max(1) = 100; %[rad/s2]
ddq_max(2) = 200; %[rad/s2]
ddq_max(3) = 200; %[rad/s2]
nDoF = 3;

%% DEFINE TASK
% Define a C-space trajectory using a 5th order polynomial interpolation
% as specified @ : https://www.mathworks.com/help/robotics/ref/quinticpolytraj.html

% % wpts must be nxp array
% way_pts = [1.5708 -0.84 0.15 -0.7854 1.5708  ;...    %q1
%            1.5708 1.35 -0.58  0.7854 -1.5708  ;...    %q2
%            1.5708 2.14 -2.45  1.5708 -1.2456 ];...    %q3
           
% way_pts = [1.5708 0.84  ;...    %q1
%            1.5708 1.35  ;...    %q2
%            1.5708 2.14 ];...    %q3
% tpts must be px1 vector        
% time_pts = [0 1 2 3 4]';

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
ode_calc_points    = 100;
tx_torque_er_old = zeros(1,13);
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
%         delta_qdot = abs(abs(dq_max) - abs(x_c(ode_cnt,4:6)));
%         norm_delta_qdot(ode_cnt) = norm(delta_qdot);
    end
    
    % calculate torque for the [t,x] trajectory segment calculated by ODE
    Vold = calculatePotentialEnergyMatrix_anat_3dof(x0(1:3),xi_ai_anat,g_sli_anat,M_b_link_as2);
    torque      = zeros(size(t,1),3);
    error       = zeros(size(t,1),3);
    epsilon     = zeros(size(t,1),3);
    for ode_cnt=1:size(t,1)
        [torque(ode_cnt,:),Vold,error(ode_cnt,:),epsilon(ode_cnt,:)] = postODEoutput_LQR_computed_torque_control_3DoF_MMD(e_state(ode_cnt,:)',x_c(ode_cnt,:)',ddq_d(:,task_point_cnt),dt,s_for_compute_Mij_429_3DoF,s_lqr,Vold);
    end
    
%     plot(t,error(:,1)) % pos_error of joint1 : only for visual tuning of pid gains 
    
    % save last x to pass as initial state for next step
    last_epsilon = e_c(size(t,1),:);
    last_x       = x_c(size(t,1),:);
    
    % Build final(t,x,torque,epsilon) trajectory details
    tx_torque_er_new = horzcat(t,x_c,torque,error);
    final_tx_torque_er = vertcat(tx_torque_er_old,tx_torque_er_new);
    tx_torque_er_old = final_tx_torque_er;
end
final_tx_torque_er = final_tx_torque_er(2:size(final_tx_torque_er,1),:); % final trajectory extracted

%% PLOTS
% t : final_tx_torque(:,1)
% q1: final_tx_torque(:,1)
% Plot1-> q(t), Plot2-> dq(t), Plot3-> torque(t)
% figure;
% subplot(4,1,1); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,2),final_tx_torque_er(:,1),final_tx_torque_er(:,3),final_tx_torque_er(:,1),final_tx_torque_er(:,4));  title(plot_title); xlabel('Time [s]'); ylabel('Joint Position [rad]');   lgd = legend('q1','q2','q3'); 
% subplot(4,1,2); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,5),final_tx_torque_er(:,1),final_tx_torque_er(:,6),final_tx_torque_er(:,1),final_tx_torque_er(:,7));  xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]'); lgd = legend('dq1','dq2','dq3'); 
% subplot(4,1,3); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,8),final_tx_torque_er(:,1),final_tx_torque_er(:,9),final_tx_torque_er(:,1),final_tx_torque_er(:,10)); xlabel('Time [s]'); ylabel('Joint Torque [N*m]');     lgd = legend('T1','T2','T3');
% subplot(4,1,4); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,11),final_tx_torque_er(:,1),final_tx_torque_er(:,12),final_tx_torque_er(:,1),final_tx_torque_er(:,13)); xlabel('Time [s]'); ylabel('Position Tracking Error [rad]');     lgd = legend('E1','E2','E3');

figure; % Position Comparison
subplot(3,1,1); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,2),time_samples,q_d(1,:)); title('Joint 1 Actual vs Desired Position'); xlabel('Time [s]'); ylabel('Joint Position [rad]'); lgd = legend('q1','qd1');
subplot(3,1,2); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,3),time_samples,q_d(2,:)); title('Joint 2 Actual vs Desired Position'); xlabel('Time [s]'); ylabel('Joint Position [rad]'); lgd = legend('q2','qd2');
subplot(3,1,3); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,4),time_samples,q_d(3,:)); title('Joint 3 Actual vs Desired Position'); xlabel('Time [s]'); ylabel('Joint Position [rad]'); lgd = legend('q3','qd3');

figure; % Velocity Comparison
subplot(3,1,1); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,5),time_samples,dq_d(1,:)); title('Joint 1 Actual vs Desired Velocity'); xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]'); lgd = legend('dq1','dqd1');
subplot(3,1,2); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,6),time_samples,dq_d(2,:)); title('Joint 2 Actual vs Desired Velocity'); xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]'); lgd = legend('dq2','dqd2');
subplot(3,1,3); plot(final_tx_torque_er(:,1),final_tx_torque_er(:,7),time_samples,dq_d(3,:)); title('Joint 3 Actual vs Desired Velocity'); xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]'); lgd = legend('dq3','dqd3');

end