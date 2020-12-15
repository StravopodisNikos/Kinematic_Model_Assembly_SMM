% GA Optimization Call File
% Performs Anatomy optimization for ga-optimized 3dof smm structures
% 1. Global kinematic isotropy index(tool-space kinematic isotropy)
% 2. Global Lamda Condition Index(tool-space dynamic isotropy)

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

% Current string-assembly parameters for structures extracted at 4.11

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

ga_custom_gamultiobj_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_custom_gamultiobj_functions',filesep);
ga_custom_gamultiobj_functions_library_path = strcat(root_path,ga_custom_gamultiobj_functions_path_relative_to_matlab_ws); addpath(ga_custom_gamultiobj_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions')
 
clc;
clear;
close all;

%% SET GA-OPTIMIZED-STRUCTURE 1
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name = 's1_ga_test_mult_4_11_20';
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3

ga_structure(1,:) = fixed_active_string_notation;
ga_structure(2,:) = passive_under_string_notation;
ga_structure(3,:) = no_passive_string_notation;
ga_structure(4,:) = fixed_active_string_notation;
ga_structure(5,:) = passive_back_string_notation;
ga_structure(6,:) = passive_back_string_notation;
ga_structure(7,:) = fixed_active_string_notation;
ga_assembly_parameters(1,:) = [-0.0121    0.0175   -0.8370]';                  % syn2 bcause 31
ga_assembly_parameters(2,:) = [-0.0256   -0.0190    1.2980]';                  % syn3 because 31
ga_assembly_parameters(3,:) = [-0.0408    0.0092    0.4821]';                % syn4 because 21
ga_assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters(4,2) = 0.5987;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters(4,3) = 0.6594;                   % 2nd dxl assembly pitch parameter

%%

%% MULTIOBJECTIVE OPTIMIZATION-1:TOOL-SPACE-2:C-SPACE
% 1. Global Lamda Condition Index(tool-space dynamic isotropy)
% 2. Global Dynamic Manipulability Index(config-space dynamic manipulability)

FitnessFunctions = @(x)[obj_fn_dynamic_isotropy_anatomy_optimization_LamdaIndex(x,ga_structure,ga_assembly_parameters),...
                        obj_fn_dynamic_isotropy_anatomy_optimization_DMEIndex(x,ga_structure,ga_assembly_parameters)];
% Settings
nvars1 = 4;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
% IntCon1 = [1,2,3,4]; % No IntCon here, it is declared in each custom function

%     x1   x2   x3     x4 
LB1 = [1    1    1   1 ];
UB1 = [13.9999999   13.9999999   13.9999999  13.9999999];  % for step_angle = deg2rad(15); 
Bound = [LB1; UB1];
% Option settings
generations = 100;
population = 150;
tolerance = 1e-04;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 15;
%options = optimoptions('gamultiobj','CreationFcn', @mixed_int_pop_gamult,...
%                                    'MutationFcn', @mixed_int_mutation_gamult,...
%                                    'CrossoverFcn',@mixed_int_crossover_gamult,...
%                                    'Generations',generations,'PopulationSize',population,'PopInitRange',Bound,'Display','iter','CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','diagnose','MigrationInterval',5,'CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
tic;
[X_mult1,Fval_mult1,exitflag_mult1,output_mult1] = gamultiobj(FitnessFunctions,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],options);
toc;
save('s1_4_11_anatomy_optimization_dyn_isotropy_15_12_20.mat','generations','population','tolerance','stall_limit','X_mult1','Fval_mult1','exitflag_mult1','output_mult1','ga_structure_name');

%% SET GA-OPTIMIZED-STRUCTURE 2
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name = 's2_ga_test_mult_4_11_20';
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3

ga_structure(1,:) = fixed_active_string_notation;
ga_structure(2,:) = passive_under_string_notation;
ga_structure(3,:) = passive_back_string_notation;
ga_structure(4,:) = fixed_active_string_notation;
ga_structure(5,:) = passive_back_string_notation;
ga_structure(6,:) = no_passive_string_notation;
ga_structure(7,:) = fixed_active_string_notation;
ga_assembly_parameters(1,:) = [0.0269   -0.0422   -0.6583]';                  % syn2 bcause 31
ga_assembly_parameters(2,:) = [0.0064   -0.0195   -1.3607]';                  % syn3 because 31
ga_assembly_parameters(3,:) = [0.0242    0.0070   -0.4861]';                % syn4 because 21
ga_assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters(4,2) = 1.2028;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters(4,3) = 1.1134;                   % 2nd dxl assembly pitch parameter

%% MULTIOBJECTIVE OPTIMIZATION-1:TOOL-SPACE-2:C-SPACE
% 1. Global Lamda Condition Index(tool-space dynamic isotropy)
% 2. Global Dynamic Manipulability Index(config-space dynamic manipulability)

FitnessFunctions = @(x)[obj_fn_dynamic_isotropy_anatomy_optimization_LamdaIndex(x,ga_structure,ga_assembly_parameters),...
                        obj_fn_dynamic_isotropy_anatomy_optimization_DMEIndex(x,ga_structure,ga_assembly_parameters)];
% Settings
nvars1 = 4;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
% IntCon1 = [1,2,3,4]; % No IntCon here, it is declared in each custom function

%     x1   x2   x3     x4 
LB1 = [1    1    1   1 ];
UB1 = [13.9999999   13.9999999   13.9999999  13.9999999];  % for step_angle = deg2rad(15); 
Bound = [LB1; UB1];
% Option settings
generations = 100;
population = 150;
tolerance = 1e-04;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 15;
% options = optimoptions('gamultiobj','CreationFcn', @mixed_int_pop_gamult,...
%                                     'MutationFcn', @mixed_int_mutation_gamult,...
%                                     'CrossoverFcn',@mixed_int_crossover_gamult,...
%                                     'Generations',generations,'PopulationSize',population,'PopInitRange',Bound,'Display','iter','CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','diagnose','MigrationInterval',5,'CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
tic;
[X_mult2,Fval_mult2,exitflag_mult2,output_mult2] = gamultiobj(FitnessFunctions,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],options);
toc;
save('s2_4_11_anatomy_optimization_dyn_isotropy_15_12_20.mat','generations','population','tolerance','stall_limit','X_mult2','Fval_mult2','exitflag_mult2','output_mult2','ga_structure_name');


%% SET GA-OPTIMIZED-STRUCTURE 3
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

%% MULTIOBJECTIVE OPTIMIZATION-1:TOOL-SPACE-2:C-SPACE
% 1. Global Lamda Condition Index(tool-space dynamic isotropy)
% 2. Global Dynamic Manipulability Index(config-space dynamic manipulability)

FitnessFunctions = @(x)[obj_fn_dynamic_isotropy_anatomy_optimization_LamdaIndex(x,ga_structure,ga_assembly_parameters),...
                        obj_fn_dynamic_isotropy_anatomy_optimization_DMEIndex(x,ga_structure,ga_assembly_parameters)];
% Settings
nvars1 = 4;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
% IntCon1 = [1,2,3,4]; % No IntCon here, it is declared in each custom function

%     x1   x2   x3     x4 
LB1 = [1    1    1   1 ];
UB1 = [13.9999999   13.9999999   13.9999999  13.9999999];  % for step_angle = deg2rad(15); 
Bound = [LB1; UB1];
% Option settings
generations = 100;
population = 150;
tolerance = 1e-04;
crossover_fraction = 0.95; % added to optimize mixed integer ga run
elite_count = 0.15*population; % added to optimize mixed integer ga run
stall_limit = 15;
% options = optimoptions('gamultiobj','CreationFcn', @mixed_int_pop_gamult,...
%                                     'MutationFcn', @mixed_int_mutation_gamult,...
%                                     'CrossoverFcn',@mixed_int_crossover_gamult,...
%                                     'Generations',generations,'PopulationSize',population,'PopInitRange',Bound,'Display','iter','CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','diagnose','MigrationInterval',5,'CrossoverFraction',crossover_fraction,'StallGenLimit',stall_limit,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
tic;
[X_mult3,Fval_mult3,exitflag_mult3,output_mult3] = gamultiobj(FitnessFunctions,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],options);
toc;
save('s3_4_11_anatomy_optimization_dyn_isotropy_15_12_20.mat','generations','population','tolerance','stall_limit','X_mult3','Fval_mult3','exitflag_mult3','output_mult3','ga_structure_name');
