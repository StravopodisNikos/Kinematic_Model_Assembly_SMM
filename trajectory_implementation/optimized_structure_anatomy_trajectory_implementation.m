% Trajectory Implementation on Isotropic Extracted anatomies
% Anatomy optimization was performed in:
% ga_call_anatomy_optimization_isotropy_investigation.m

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

%% Add paths to all functions:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
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

clear;
clc;
close all;

%% DEFINE MASS BALANCED STRUCTURE & ISOTROPIC ANATOMY 
% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name = 's1_ga_test_mult_4_11_20';
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3

structure(1,:) = fixed_active_string_notation;
structure(2,:) = passive_under_string_notation;
structure(3,:) = no_passive_string_notation;
structure(4,:) = fixed_active_string_notation;
structure(5,:) = passive_back_string_notation;
structure(6,:) = passive_back_string_notation;
structure(7,:) = fixed_active_string_notation;
assembly_parameters(1,:) = [-0.0121    0.0175   -0.8370]';                  % syn2 bcause 31
assembly_parameters(2,:) = [-0.0256   -0.0190    1.2980]';                  % syn3 because 31
assembly_parameters(3,:) = [-0.0408    0.0092    0.4821]';                % syn4 because 21
assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
assembly_parameters(4,2) = 0.5987;                   % 1st dxl assembly pitch parameter
assembly_parameters(4,3) = 0.6594;                   % 2nd dxl assembly pitch parameter

% Anatomies are extracted from files in:
% Kinematic_Model_Assembly_SMM/investigate_dynamic_kinematic_isotropy/optimized-structures-anatomies-matfiles/MBS_structures/4_11/
%                                        θp2(x)
isotropic_anatomy_extracted = [ 9.0000   2.0000   9.0000    3.0000 ];  %14.6095   0.0027
isotropic_anatomy_structure_dependent = calculate_transformed_anatomy_vector(structure,'3dof',isotropic_anatomy_extracted);

% BUILD REFERENCE ANATOMY
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters);

% BUILD ISOTROPIC ANATOMY
[gst_anat,xi_ai_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_no_graph(structure,assembly_parameters,isotropic_anatomy_extracted,xi_pj_ref);

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

%% DEFINE TASK
% Define a C-space trajectory using a 5th order polynomial interpolation
% as specified @ : https://www.mathworks.com/help/robotics/ref/quinticpolytraj.html

% wpts must be nxp array
way_pts = [1.5708 0.84 0.15 -0.7854 -1.5708  ;...    %q1
           1.5708 1.35 0.58  0.7854  1.5708  ;...    %q2
           1.5708 2.14 2.45  1.5708 -1.2456 ];...    %q3
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

%% EXTRACT TRAJECTORY -> THIS IS REFERENCE FOR EXECUTION COMMANDS
% desired(reference) arrays dimension is: ndofxtask_points (3x5)
[q_d, dq_d, ddq_d, pp] = quinticpolytraj(way_pts, time_pts, time_samples);

%% EXECUTION - USING ODE
for task_point_cnt=2:length(q_d,2)
    tspan = [time_pts(task_point_cnt-1) time_pts(task_point_cnt)];
    %dt = tspan(length(tspan)) - tspan(1);
    dt = 0.01;
    x0(1:3) = q_d(:,task_point_cnt-1);
    x0(4:6) = dq_d(:,task_point_cnt-1);
    x_d     =  vertcat(q_d(:,task_point_cnt), dq_d(:,task_point_cnt));
    ode_derivative_Fcn = @(t,x) PID_computed_torque_control_3DoF_MMD(x_d,ddq_d(:,task_point_cnt),x,dt,s_for_compute_Mij_429_3DoF);
    % options = odeset('OutputFcn',@(t,x,flag) TorqueOutputFcn_PID_computed_torque_control_3DoF_MMD(t,x,flag,x_d,ddq_d(:,1)) );
    [t(:,task_point_cnt),x(:,:,task_point_cnt)] = ode113(ode_derivative_Fcn, tspan, x0);
end