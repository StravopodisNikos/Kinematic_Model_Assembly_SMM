% Here C-space evaluation of the optimized anatomies extracted from generate_matfiles.m in folder:
% /home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/investigate_dynamic_kinematic_isotropy/optimized-structure-anatomies-matfiles/MBS_structures/4_11
% Each matfile loads the variables:  'ga_structure_name','ga_structure','ga_assembly_parameters','s{i}_ga_test_mult_4_11_20_opt_anat')
clear;
clc;
close all;
%% Add paths to all matfiles:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
matfiles_4_11_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','investigate_dynamic_kinematic_isotropy','optimized-structure-anatomies-matfiles','MBS_structures','4_11',filesep);
matfiles_4_11_library_path = strcat(root_path,matfiles_4_11_path_relative_to_matlab_ws); addpath(matfiles_4_11_path_relative_to_matlab_ws);

screws_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','screws',filesep);
screws_library_path = strcat(root_path,screws_path_relative_to_matlab_ws); addpath(screws_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
util_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','util',filesep);
util_library_path = strcat(root_path,util_path_relative_to_matlab_ws); addpath(util_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
screw_dynamics_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_dynamics',filesep);
screw_dynamics_library_path = strcat(root_path,screw_dynamics_path_relative_to_matlab_ws); addpath(screw_dynamics_library_path);
% addpath('/home/nikos/matlab_ws/screw_dynamics')
Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM',filesep);
Kinematic_Model_Assembly_SMM_library_path = strcat(root_path,Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws); addpath(Kinematic_Model_Assembly_SMM_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM')
building_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','building_functions',filesep);
building_functions_library_path = strcat(root_path,building_functions_path_relative_to_matlab_ws); addpath(building_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
synthetic_joints_tfs_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','synthetic_joints_tfs',filesep);
synthetic_joints_tfs_library_path = strcat(root_path,synthetic_joints_tfs_path_relative_to_matlab_ws); addpath(synthetic_joints_tfs_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
calculateFunctions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','calculateFunctions',filesep);
calculateFunctions_library_path = strcat(root_path,calculateFunctions_path_relative_to_matlab_ws); addpath(calculateFunctions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
 
%% I. LOAD STRUCTURE-ANATOMY DATA
load('s1_4_11_optimized_structure_anatomies.mat');

%% II. VISUALIZE STRUCTURE-ANATOMY

%% III. BUILD STRUCTURE-ANATOMY @ reference configution
% III.1 BUILD STRUCTURE @ REFERENCE ANTOMY
[xi_ai_struct_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,~,~,wrong_string_structure] = structure_assembly_3dof(ga_structure,ga_assembly_parameters);

anatomy_index = 1; % change anatomy index!
optimal_anatomy = s1_ga_test_mult_4_11_20_opt_anat(anatomy_index,:); % change s{i} index!

optimal_anatomy_structure_dependent = calculate_transformed_anatomy_vector(ga_structure,'3dof',optimal_anatomy);
[~,xi_ai_struct_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_no_graph(ga_structure,ga_assembly_parameters,optimal_anatomy_structure_dependent,xi_pj_ref);
[g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);
[M_b_link_as_anat] = calculateMetalinkInertiaMatrixBody(g_s_link_as_anat,M_s_link_as_anat);

%% IV. RUN C-SPACE INVESTIGATION
step_angle = [0 0.1 0.1];
theta_lim  = [pi (4*pi)/6 pi];
PM_string = 'dci-all';
[PM_Cspace,Th2,Th3] = configuration_space_theta23loops(step_angle,theta_lim,PM_string,xi_ai_struct_anat,g_s_link_as_anat,M_b_link_as_anat);

%% v. PLOTS
figure; surf(Th2,Th3,PM_Cspace')


