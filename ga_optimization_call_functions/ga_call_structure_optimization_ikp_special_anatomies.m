% x: structure definition-parameterization chromosome
% Chromosome x is built considering the smm structure building principles
% defined and the parameters given in the corresponding xacro file used for
% visualization (/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/conditioned_parameterized_SMM_assembly_6dof_for_ga2.xacro)

% x1     = 1st meta link, only 2nd pseudo assembly type
% x2,3   = 2nd meta link, 2 pseudos assembly type
% x4,5   = 3rd meta link, 2 pseudos assembly type
% x6,7   = 4th meta link, 2 pseudos assembly type
% x8,9   = 5th meta link, 2 pseudos assembly type
% x10-12 = 2nd pseudo of 1st metalink parameters
% x13-15 = 1st pseudo of 2nd metalink parameters
% x16-18 = 2nd pseudo of 2nd metalink parameters
% x19-21 = 1st pseudo of 3rd metalink parameters
% x22-24 = 2nd pseudo of 3rd metalink parameters
% x25-27 = 1st pseudo of 4th metalink parameters
% x28-30 = 2nd pseudo of 4th metalink parameters
% x31-33 = 1st pseudo of 5th metalink parameters
% x34-36 = 2nd pseudo of 5th metalink parameters
% x37    = 1st dxl assembly pitch parameter (2nd active joint)
% x38    = 2nd dxl assembly pitch parameter (3rd active joint)
% x39    = 3rd dxl assembly pitch parameter (4th active joint)
% x40    = 4th dxl assembly pitch parameter (5th active joint)
% x41    = 5th dxl assembly pitch parameter (6th active joint)

% degrees: 'degree2'            or  'degree4'
%              ||                       ||
%              \/                       \/
% criteria:'parallelism3'            'parallel'
%          'orth_intersect3'       'intersecting'
%                             'perpendicular_intersecting'

%% Add paths to all functions:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws/'));
root_path = strcat(root_path(1),'matlab_ws/');

% Add libraries relative to matlab_ws folder
ga_building_functions_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','building_functions',filesep);
ga_building_functions_library_path = strcat(root_path,ga_building_functions_path_relative_to_matlab_ws); addpath(ga_building_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')

ga_synthetic_joints_tfs_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','synthetic_joints_tfs',filesep);
ga_synthetic_joints_tfs_library_path = strcat(root_path,ga_synthetic_joints_tfs_path_relative_to_matlab_ws); addpath(ga_synthetic_joints_tfs_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')

ga_calculateFunctions_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','calculateFunctions',filesep);
ga_calculateFunctions_library_path = strcat(root_path,ga_calculateFunctions_path_relative_to_matlab_ws); addpath(ga_calculateFunctions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')

ga_objective_functions_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','ga_objective_functions',filesep);
ga_objective_functions_library_path = strcat(root_path,ga_objective_functions_path_relative_to_matlab_ws); addpath(ga_objective_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions')

ga_optimization_call_functions_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','ga_optimization_call_functions',filesep);
ga_optimization_call_functions_library_path = strcat(root_path,ga_optimization_call_functions_path_relative_to_matlab_ws); addpath(ga_optimization_call_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions')

%% START
clc;
clear;
close all;

%% Here call ga's
generations = 750;
population = 500;
tolerance = 1e-04;
stall_limit = 50;
%% 1.Metric with Total special anatomies
degree = 'degree4';
criterion = 'parallel';
FitnessFunction1 = @(x)obj_fn_analytic_ikp_for_6dof_smm(x,degree,criterion); 

nvars1 = 41;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
IntCon1 = [1,2,3,4,5,6,7,8,9,37,38,39,40,41];

LB1_1_9   = [1    2    1    2    1    2    1    2    1 ]; % for pseudo existence-type
LB1_10_12 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_13_15 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_16_18 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_19_21 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_22_24 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_25_27 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_28_30 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_31_33 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_34_36 = [-0.075  -0.050  -1.5708  ];                  % for synthetic tfs
LB1_37_41 = [0   0   0   0   0];                          % for dxl pitch angle
LB1 = horzcat(LB1_1_9,LB1_10_12,LB1_13_15,LB1_16_18,LB1_19_21,LB1_22_24,LB1_25_27,LB1_28_30,LB1_31_33,LB1_34_36,LB1_37_41);

UB1_1_9   = [3    3    3    3    3    3    3    3    3 ]; % for pseudo existence-type
UB1_10_12 = [0.050   0.050   1.5708 ];                  % for synthetic tfs
UB1_13_15 = [0.050   0.050   1.5708 ];                  % for synthetic tfs
UB1_16_18 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_19_21 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_22_24 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_25_27 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_28_30 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_31_33 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_34_36 = [0.050   0.050   1.5708];                  % for synthetic tfs
UB1_37_41 = [1   1   1   1   1];                          % for dxl pitch angle
UB1 = horzcat(UB1_1_9,UB1_10_12,UB1_13_15,UB1_16_18,UB1_19_21,UB1_22_24,UB1_25_27,UB1_28_30,UB1_31_33,UB1_34_36,UB1_37_41);
%% Call ga 
options = optimoptions('ga','Generations',generations,'PopulationSize',population,'Display','iter','FunctionTolerance',tolerance,'StallGenLimit',stall_limit,'UseParallel', true, 'UseVectorized', false);

tic
[X,Fval,Exitflag,Output] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options);
toc
save('ga_test_special_anatomies_ikp_5_11_20.mat','generations','population','tolerance','stall_limit','X','Fval','Exitflag','Output','degree','criterion');
