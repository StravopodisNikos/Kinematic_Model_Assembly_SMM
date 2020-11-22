% GA Optimization Call File
% Performs Anatomy optimization for ga-optimized 3dof smm structures
% 1. Global kinematic isotropy index(tool-space kinematic isotropy)
% 2. Global Lamda Condition Index(tool-space dynamic isotropy)

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

% Chromosome
% x1-4 = pseudojoint angles as integers

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

clc;
clear;
close all;

%% SET GA-OPTIMIZED-STRUCTURE
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name = 's3_ga_test_mult_4_11_20';
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3

ga_structure(1,:) = fixed_active_string_notation;
ga_structure(2,:) = passive_under_string_notation;
ga_structure(3,:) = no_passive_string_notation;
ga_structure(4,:) = fixed_active_string_notation;
ga_structure(5,:) = passive_back_string_notation;
ga_structure(6,:) = no_passive_string_notation;
ga_structure(7,:) = fixed_active_string_notation;
ga_assembly_parameters(1,:) = [-0.0226,0.0211,0.3009]';                  % syn2 bcause 31
ga_assembly_parameters(2,:) = [0.0244,-0.0153,1.4347]';                  % syn3 because 31
ga_assembly_parameters(3,:) = [-0.0138,0.0137,0.0331]';                % syn4 because 21
ga_assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters(4,2) = 0.9486;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters(4,3) = 1.1225;                   % 2nd dxl assembly pitch parameter

%%

%% 1. Global Lamda Condition Index(tool-space dynamic isotropy) 
FitnessFunction1 = @(x)obj_fn_kinematic_isotropy_anatomy_optimization(x,ga_structure,ga_assembly_parameters);
nvars1 = 4;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
IntCon1 = [1,2,3,4];

%     x1   x2   x3     x4 
LB1 = [1    1    1   1 ];
UB1 = [13   13   13  13];  % for max ci=13 
Bound = [LB1; UB1];
% Option settings
generations = 15;
population = 15;
tolerance = 1e-03;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 5;
options = optimoptions('ga','Generations',generations,'PopulationSize',population,'Display','iter','FunctionTolerance',tolerance,'StallGenLimit',stall_limit,'UseParallel', true, 'UseVectorized', false);

tic
[X1,Fval1,Exitflag1,Output1] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options);
toc
save('ga_anatomy_optimization_kin_isotropy_22_11_20.mat','generations','population','tolerance','stall_limit','X1','Fval1','Exitflag1','Output1','ga_structure_name');

%% 2. Global kinematic isotropy 
FitnessFunction1 = @(x)obj_fn_dynamic_isotropy_anatomy_optimization(x,ga_structure,ga_assembly_parameters);
nvars1 = 4;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
IntCon1 = [1,2,3,4];

%     x1   x2   x3     x4 
LB1 = [1    1    1   1 ];
UB1 = [13   13   13  13];  % for max ci=13 
Bound = [LB1; UB1];
% Option settings
generations = 15;
population = 15;
tolerance = 1e-03;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 5;
options = optimoptions('ga','Generations',generations,'PopulationSize',population,'Display','iter','FunctionTolerance',tolerance,'StallGenLimit',stall_limit,'UseParallel', true, 'UseVectorized', false);

tic
[X2,Fval2,Exitflag2,Output2] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options);
toc
save('ga_anatomy_optimization_dyn_isotropy_22_11_20.mat','generations','population','tolerance','stall_limit','X2','Fval2','Exitflag2','Output2','ga_structure_name');
