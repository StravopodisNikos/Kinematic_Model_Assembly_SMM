% GA MultiObjective Optimization Call File
% Performs Structure optimization by scoring the min+max Mass Balancing Score of all 
% structure anatomies

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

% Chromosome
% x1 = 1st meta link's 2nd pseudo assembly type     -> integer
% x2,3 = 2nd meta link, 2 pseudos assembly type     -> integers
% x4-6 = 2nd pseudo of 1st metalink parameters      -> real syn2-4
% x7-9 = 1st pseudo of 2nd metalink parameters      -> real syn3-5
% x10-12 = 2nd pseudo of 2nd metalink parameters    -> real syn2-4
% x13 = 1st dxl assembly pitch parameter            -> real
% x14 = 2nd dxl assembly pitch parameter            -> real

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

ga_custom_gamultiobj_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_custom_gamultiobj_functions',filesep);
ga_custom_gamultiobj_functions_library_path = strcat(root_path,ga_custom_gamultiobj_functions_path_relative_to_matlab_ws); addpath(ga_custom_gamultiobj_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions')


clc;
clear;
close all;

%% Here call ga's
generations = 300;
population = 250;
tolerance = 1e-03;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 15;

%% Multiobjective with min+max MBS
FitnessFunctions = @(x)[obj_fn_mass_balancing_structure_optimization_minMBS(x),obj_fn_mass_balancing_structure_optimization_maxMBS(x)];
nvars1 = 14;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
% No IntCon here, it is declared in each custom function
%      x1   x2   x3     x4      x5      x6     x7      x8        x9     x10      x11      x12     x13     x14 
LB1 = [1    2    1   -0.075  -0.050  -1.5708 -0.055  -0.050  -1.5708  -0.075   -0.050   -1.5708    0       0];
UB3 = [3    3    3    0.050   0.050   1.5708  0.025   0.050   1.5708   0.050    0.050    1.5708    1       1];
Bound = [LB1; UB3];

% % Call gamultiob
options = optimoptions('gamultiobj','CreationFcn', @mixed_int_pop_gamult,...
                                    'MutationFcn', @mixed_int_mutation_gamult,...
                                    'CrossoverFcn',@mixed_int_crossover_gamult,...
                                    'Generations',generations,'PopulationSize',population,'PopInitRange',Bound,'Display','iter','CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
tic;
[X_mult,Fval_mult,exitflag_mult,output_mult] = gamultiobj(FitnessFunctions,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
toc;
save('ga_test_mult_25_11_20.mat','generations','population','tolerance','stall_limit','X_mult','Fval_mult','exitflag_mult','output_mult')