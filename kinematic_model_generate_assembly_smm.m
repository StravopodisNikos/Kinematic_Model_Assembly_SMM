% This code generates the assembly of the sructure defined by the
% optimization parameters and saves the reference anatomy/structure twists
% and joint & link frames tfs. It calculates in matlab the tf's induced by
% xacro file assembly developed in:
% /PhD/projects/Parametric_Simulation_Model_SMM
% It is the MATLAB evaluation of the robot visualization tools used to
% assembly modular structure!

% HOW TO EXECUTE
% 1. Set the parameters extracted from optimization:
% Section "Define smm structure string" <--> test_structure_string_definition.yaml
% the synthetic tfs are given inside the switch statement in
% "add_synthetic_joint_tf" <-->  tf_list_for_conditioned_assembly.yaml and
% assembly_parameters_for_ovidius_robot.yaml
% 2. Urdf file is built from xacro file, having set the corresponding
% parameters in the yaml file
% >> nikos@syros-b14-nikos-ubuntu:~/PhD/projects/Parametric_Simulation_Model_SMM/xacros$ xacro conditioned_parameterized_SMM_assembly.xacro > generated_urdf_from_xacros_here/conditioned_parameterized_SMM_assembly.urdf
% 3. structure in l.49-55 is set by user
% 4. Run code

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
ga_objective_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_objective_functions',filesep);
ga_objective_functions_library_path = strcat(root_path,ga_objective_functions_path_relative_to_matlab_ws); addpath(ga_objective_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions')
subroutines_executed_in_objective_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_objective_functions','subroutines_executed_in_objective_fn',filesep);
subroutines_executed_in_objective_fn_library_path = strcat(root_path,subroutines_executed_in_objective_fn_path_relative_to_matlab_ws); addpath(subroutines_executed_in_objective_fn_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions/subroutines_executed_in_objective_fn')
clear;
close all;

%% Only visualization for code evaluation
% Ref structure robot - All data must be calculated for reference structure
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/test_3dof_ref_anat.urdf'; % only for test!
% robotURDFfile = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions/ga_optimization_results/urdf_files_ref_anatomy_of_evaluated_structures/s3_ga_test_mult_4_11_20.urdf'; % only for structure evaluation!
[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);

robotURDFfile_test_anat = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/test_3dof_test_anat.urdf'; % only for test!
% robotURDFfile_test_anat = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions/ga_optimization_results/urdf_files_ref_anatomy_of_evaluated_structures/s3_ga_test_mult_4_11_20.urdf'; % only for structure evaluation!
[TestRobot,TestFig,TestConfig,~] = ImportRobotRefAnatomyModel(robotURDFfile_test_anat);

%% Define smm structure string (in optimization it is ga generated!)
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3

structure(1,:) = fixed_active_string_notation;
structure(2,:) = passive_under_string_notation;
structure(3,:) = no_passive_string_notation;
structure(4,:) = fixed_active_string_notation;
structure(5,:) = passive_back_string_notation;
structure(6,:) = no_passive_string_notation;
structure(7,:) = fixed_active_string_notation;

% NEXT ARE ONLY FOR CODE VERIFICATION: WILL BE IMPLEMENTED IN SEPARATE
% FUNCTIONS!
% ================= GA PARAMETERS =================
%  I.1 Assembly sequence definition -> given in l.49-55
%  I.2 Assembly parameters ONLY FOR THE STRUCTURE TESTED, must manually
%  give the same values given the asembly sequence and the values @
%  assembly_parameters_for_ovidius_robot_for_ga.yaml
assembly_parameters(1,:) = [-0.0226,0.0211,0.3009]';                  % syn2 bcause 31
assembly_parameters(2,:) = [0.0244,-0.0153,1.4347]';                  % syn3 because 31
assembly_parameters(3,:) = [-0.0138,0.0137,0.0331]';                % syn4 because 21
assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
assembly_parameters(4,2) = 0.9486;                   % 1st dxl assembly pitch parameter
assembly_parameters(4,3) = 1.1225;                   % 2nd dxl assembly pitch parameter
% ================= GA PARAMETERS =================
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters);
figure(RefFig); xi1_graph = drawtwist(xi_ai_ref(:,1)); hold on; xi2_graph = drawtwist(xi_ai_ref(:,2)); hold on; xi3_graph = drawtwist(xi_ai_ref(:,3)); hold on; 
figure(RefFig); xip1_graph = drawtwist(xi_pj_ref(:,1)); hold on; xip2_graph = drawtwist(xi_pj_ref(:,2)); hold on;% xip3_graph = drawtwist(xi_pj_ref(:,3)); hold on; 

% 1.POE FORWARD KINEMATICS(works fine)
% SET configuration and anatomy of assembled structure => must agree with
% xacro file that built urdf
qa = [0 0 0]'; qp_xacro_user_given = [0.7854 0 -1.5708 0]';
qp_structure_dependent = calculate_transformed_anatomy_vector(structure,'3dof',qp_xacro_user_given);

figure(TestFig); show(TestRobot,qa); hold on;
%[TestFig] = visualize_robot_urdf(robotURDFfile,qa);
[g_ai,g_pj,Jsp,Pi,gst] = calculateForwardKinematicsPOE(structure,'3dof',xi_ai_ref,xi_pj_ref,qa,qp_structure_dependent,g_ai_ref,g_pj_ref,gst0);
figure(TestFig); drawframe(g_ai(:,:,1),0.15); hold on; drawframe(g_ai(:,:,2),0.15); hold on; drawframe(g_ai(:,:,3),0.15); drawframe(gst,0.15); hold on; hold on; %xi_a2_graph = drawtwist(Jsp(:,2)); hold on; xi_a3_graph = drawtwist(Jsp(:,3)); hold on;
figure(TestFig); drawframe(g_pj(:,:,1),0.15); hold on; drawframe(g_pj(:,:,2),0.15); hold on;  %drawframe(g_pj(:,:,3),0.15); hold on; drawframe(g_pj(:,:,4),0.15); hold on;
figure(TestFig); xi1_graph = drawtwist(Jsp(:,1)); hold on; xi2_graph = drawtwist(Jsp(:,2)); hold on; xi3_graph = drawtwist(Jsp(:,3)); hold on; 

% Only used to save data for mpampis ikp
% var_saved = save_var_for_ikp_mpampis(structure,xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,qp_structure_dependent,'ikp_test_structure.mat');

% Build the Jacobians
% [Jsp, Jbd, gst1] = calculateJacobians_for_assembled_structure('3dof',xi_ai_ref,qa, Pi, gst0);
GlobalKinematicIsotropicIndex = calculateKinematicIsotropicIndex_3DoF(xi_ai_ref, Pi, gst0);

% 2.EXTRACT INERTIAS FOR GIVEN STRUCTURE @ REFERENCE ANATOMY
% Evaluate calculated Link Inertias(works fine)
[g_s_link_as,M_s_link_as] = calculateCoMmetalinks(M_s_com_k_i,g_s_com_k_i);
% figure(RefFig); scatter3(g_s_link_as(1,1,1), g_s_link_as(2,1,1), g_s_link_as(3,1,1),500,'p','filled'); hold on;
% figure(RefFig); scatter3(g_s_link_as(1,1,2), g_s_link_as(2,1,2), g_s_link_as(3,1,2),500,'p','filled'); hold on;
% figure(RefFig); scatter3(g_s_link_as(1,1,3), g_s_link_as(2,1,3), g_s_link_as(3,1,3),500,'p','filled'); hold on;

% 2.1 EXTRACT INERTIAS FOR GIVEN STRUCTURE & ANATOMY(Here the ga fn are checked-parameters must agree!)
[gst_anat,xi_ai_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy(structure,assembly_parameters,qp_structure_dependent,xi_pj_ref,TestFig);
[g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);
figure(TestFig); scatter3(g_s_link_as_anat(1,1,1), g_s_link_as_anat(2,1,1), g_s_link_as_anat(3,1,1),500,'p','filled'); hold on;
figure(TestFig); scatter3(g_s_link_as_anat(1,1,2), g_s_link_as_anat(2,1,2), g_s_link_as_anat(3,1,2),500,'p','filled'); hold on;
figure(TestFig); scatter3(g_s_link_as_anat(1,1,3), g_s_link_as_anat(2,1,3), g_s_link_as_anat(3,1,3),500,'p','filled'); hold on;
figure(TestFig); xi_a2_graph_anat = drawtwist(xi_ai_anat(:,2)); hold on; xi_a3_graph_anat = drawtwist(xi_ai_anat(:,3)); hold on;
% 3.Check mass balancing function
[MBS] = calculateMBS(structure,assembly_parameters,xi_ai_ref,xi_pj_ref,qp_structure_dependent,TestFig);

% 4.Now that metalinks COM were found the Metalink Inertia Matrix|Body frame
% is found
[M_b_link_as1] = calculateMetalinkInertiaMatrixBody(g_s_link_as,M_s_link_as);
[M_b_link_as2] = calculateMetalinkInertiaMatrixBody(g_s_link_as_anat,M_s_link_as_anat); % this is the good one
% 5.CONSTRUCT LINK BODY JACOBIANS ANG GENERALIZED INERTIA MATRIX
[J_b_sli1] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi, g_s_link_as );
[J_b_sli2] = calculateCoM_BodyJacobians_for_anat(xi_ai_anat, qa, g_s_link_as_anat );  % this is the good one
[M_b1] = calculateGIM(J_b_sli1,M_b_link_as1); %
[M_b2] = calculateGIM(J_b_sli2,M_b_link_as2);  % this is the good one
M_b_matlab = massMatrix(TestRobot,qa);
% here check old files(screw_dynamics folder) for mass-coriolis
for i_cnt=1:3
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),qa(i_cnt));
    g_sli_anat(:,:,i_cnt) = eye(4);
    g_sli_anat(1:3,4,i_cnt) = g_s_link_as_anat(:,1,i_cnt);
end
Pi_for_old(:,:,1) = eye(4);
Pi_for_old(:,:,2) = eye(4);
qdot = [1 1 1]';
[M_POE,~,C_POE] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai, Pi_for_old, g_sli_anat, M_b_link_as2, [1 1 1]');
C_b2 = C_POE * qdot;
C_matlab = velocityProduct(TestRobot,qa,qdot);

% 5. Check lamda conditioning index fn
 LmdCI = calculateGlobalLamdaConditionIndex_3DoF(xi_ai_anat,xi_ai_ref,g_s_link_as_anat,M_b_link_as2, Pi, gst0);