% Calls ga optimization for lqr control on trajectory implementation
% ========================================================================
% FUNCTIONS USED:
% 1. obj fn                   : optimized_lqr_min_error_state
% 2. ode for state calculation: LQR_computed_torque_control_3DoF_MMD
% 3. output fn for minimizaion: postODEoutput_LQR_computed_torque_control2_3DoF_MMD
% FUNCTIONS FOR RESULT EVALUATION:
% 1. evaluate_trajectory_implementation_custom -> optimized_structure_anatomy_trajectory_implementation_custom2
% ========================================================================

% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
obj_fn_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','trajectory_implementation','ga_gain_optimization','obj_fn',filesep);
obj_fn_library_path = strcat(root_path,obj_fn_path_relative_to_matlab_ws); addpath(obj_fn_library_path);

clear;
clc;
close all;

%% SET DESIRED TRAJECTORY PROPERTIES
% C-space task points
way_pts = [1.5708 -0.54 0.15  -0.7854  0.7854  ;...    %q1
           1.5708 1.05 -0.58   0.7854  0       ;...    %q2
           1.5708 2.14 -2.45  -1.5708  -0.5   ];...    %q3
% time steps
t_start = 0; t_finish = 10;  % [sec] 
time_pts = linspace(t_start,t_finish,size(way_pts,2))';    

%% LOAD STRUCTURES
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3
%% SET RANDOM-STRUCTURE 0
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name0 = 's0_3dof_random';
ga_structure0(1,:) = fixed_active_string_notation;
ga_structure0(2,:) = passive_under_string_notation;
ga_structure0(3,:) = passive_under_string_notation;
ga_structure0(4,:) = fixed_active_string_notation;
ga_structure0(5,:) = passive_back_string_notation;
ga_structure0(6,:) = passive_under_string_notation;
ga_structure0(7,:) = fixed_active_string_notation;
ga_assembly_parameters0(1,:) = [-0.005    -0.0175   0.7854]';                  % syn2 bcause 31
ga_assembly_parameters0(2,:) = [-0.04     0.0250    -1.5708]';                  % syn3 because 31
ga_assembly_parameters0(3,:) = [0.0508    0.0092    1.5708]';                % syn4 because 21
ga_assembly_parameters0(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters0(4,2) = 1.5708;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters0(4,3) = -1.57084;                   % 2nd dxl assembly pitch parameter
%% RANDOM ANATOMY SELECTED                          
s0_rand_s_rand_anat(1,:) =  [ 7.0000   5.0000   6.0000    10.0000 ];  % for [rad]: deg2rad(15) * (floor(ci)-1) + (-1.5708)

%% SET GA-OPTIMIZED-STRUCTURE 1
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name1 = 's1_ga_test_mult_4_11_20';
ga_structure1(1,:) = fixed_active_string_notation;
ga_structure1(2,:) = passive_under_string_notation;
ga_structure1(3,:) = no_passive_string_notation;
ga_structure1(4,:) = fixed_active_string_notation;
ga_structure1(5,:) = passive_back_string_notation;
ga_structure1(6,:) = passive_back_string_notation;
ga_structure1(7,:) = fixed_active_string_notation;
ga_assembly_parameters1(1,:) = [-0.0121    0.0175   -0.8370]';                  % syn2 bcause 31
ga_assembly_parameters1(2,:) = [-0.0256   -0.0190    1.2980]';                  % syn3 because 31
ga_assembly_parameters1(3,:) = [-0.0408    0.0092    0.4821]';                % syn4 because 21
ga_assembly_parameters1(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters1(4,2) = 0.5987;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters1(4,3) = 0.6594;                   % 2nd dxl assembly pitch parameter
%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s1_ga_test_mult_4_11_20
%                                                 θp2(x)
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
s1_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 9.0000   2.0000   9.0000    3.0000 ];  %14.6095   0.0027 
s1_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 9.0000   7.0000   4.0000    2.0000 ];  %14.3670   0.0038 
s1_ga_test_mult_4_11_20_opt_anat(3,:) =  [ 9.0000   9.0000   10.0000   3.0000 ];  %14.8638   0.0026 
s1_ga_test_mult_4_11_20_opt_anat(4,:) =  [ 10.0000  2.0000   11.0000   6.0000];  %15.9332   0.0020
s1_ga_test_mult_4_11_20_opt_anat(5,:) =  [ 10.0000  7.0000   12.0000   5.0000];  %15.1380   0.0020
s1_ga_test_mult_4_11_20_opt_anat(6,:) =  [ 10.0000  4.0000   2.0000    2.0000];  %14.1808   0.0040

%% Build the evaluated assembled structure:
%% ==========GIVE STRUCTURE-ASSEMBLY-ANATOMY==========
structure                    = ga_structure0;
assembly_parameters          = ga_assembly_parameters0;
isotropic_anatomy_extracted  = s0_rand_s_rand_anat;
%% ===================================================
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


%% Call ga 
generations = 250;
population = 25;
tolerance = 1e-03;
stall_limit = 15;
options = optimoptions('ga','Generations',generations,'PopulationSize',population,'Display','iter','FunctionTolerance',tolerance,'StallGenLimit',stall_limit,'UseParallel', true, 'UseVectorized', false);
FitnessFunction1 = @(x)optimized_lqr_min_error_state(s_for_compute_Mij_429_3DoF,way_pts,time_pts,x); 

nvars1 = 10;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
IntCon1 = [];
LB1 = 0.00001 * ones(1,10);
UB1 = 100000  * ones(1,10);
tic
[X,Fval,Exitflag,Output] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options)
toc