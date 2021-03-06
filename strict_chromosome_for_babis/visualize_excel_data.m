%% VISUALIZATION OF TSP RESULTS
% Input: Book1.xlsx excel data matrix
% Output: Each excel row is trasnsformed to robot structure
% written 8-3-21
clear; clc; close all;

% 1. Give input in matlab array
opt_sec(1,:) = [10 7 1 8  3 9  5  4  6 2  2 3 2 20 4 3 15 1 1 2  6 3 0 1 10 5  13  9];
opt_sec(2,:) = [9  8 2 10 7 3  6  4  5 1  3 3 1 6  3 3 12 3 2 15 3 3 0 0  4 14 10  1];
opt_sec(3,:) = [5  8 9 7  1 2  3  10 4 6  3 3 2 18 2 3 9  4 3 14 9 2 1 0 11 10  6  2];
opt_sec(4,:) = [10 1 9 2  5 4  6  7  8 3  2 2 1 4  3 1 18 9 1 2  2 1 1 1  4 3   4  11];
opt_sec(5,:) = [9  4 5 2  7 8  10 1  3 6  2 3 2 11 2 3 5  9 1 4  1 3 1 1  7 13 13  15];
opt_sec(6,:) = [7  1 2 8  5 10 6  3  4 9  3 2 1 17 5 1 11 1 1 11 9 2 1 1  2  4 10  2];

% 2. Extract 2 sequences
for i=1:6    
    seq_tsp(i,:)    = opt_sec(i,1:10);
    seq_str_an(i,:) = opt_sec(i,11:28);
end

% 3. Call decode_babis_chromosome -> STRUCTURE+ASSEMBLY PARAMS+ANATOMY
for i=1:6   
    [chromosome_for_urdf(i,:),anatomy(i,:)] = decode_babis_chromosome(seq_str_an(i,:));
end
% 4. THE RESULTS WILL BE PASSED IN XACRO FILE FOR VISUALIZATION(NOT IMPLEMENTED HERE) 
% Xacro file:
% /home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/conditioned_parameterized_SMM_assembly_for_ga2.xacro
% Xacro command:
% xacro ~/PhD/projects/Parametric_Simulation_Model_SMM/xacros/conditioned_parameterized_SMM_assembly_for_ga2.xacro > ~/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec1.urdf
% Yaml files:
% 1. test_structure_string_definition_for_ga.yaml
% 2. assembly_parameters_for_ovidius_robot_for_ga.yaml

% 5. Visualize constructed urdfs + TSP points
config = [0 0 0]';

opt_sec_1_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec1.urdf'; 
[opt_sec_1_figure] = visualize_robot_urdf(opt_sec_1_urdf,config);
PointsXYZ_1 = fetchpoints(seq_tsp(1,:),opt_sec_1_figure);
opt_sec_2_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec2.urdf'; 
[opt_sec_2_figure] = visualize_robot_urdf(opt_sec_2_urdf,config);
PointsXYZ_2 = fetchpoints(seq_tsp(2,:),opt_sec_2_figure);
opt_sec_3_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec3.urdf'; 
[opt_sec_3_figure] = visualize_robot_urdf(opt_sec_3_urdf,config);
PointsXYZ_3 = fetchpoints(seq_tsp(3,:),opt_sec_3_figure);
opt_sec_4_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec4.urdf'; 
[opt_sec_4_figure] = visualize_robot_urdf(opt_sec_4_urdf,config);
PointsXYZ_4 = fetchpoints(seq_tsp(4,:),opt_sec_4_figure);
opt_sec_5_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec5.urdf'; 
[opt_sec_5_figure] = visualize_robot_urdf(opt_sec_5_urdf,config);
PointsXYZ_5 = fetchpoints(seq_tsp(5,:),opt_sec_5_figure);
opt_sec_6_urdf = '/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/strict_chromosome_for_babis/visualized_results/Book1_opt_sec6.urdf'; 
[opt_sec_6_figure] = visualize_robot_urdf(opt_sec_6_urdf,config);
PointsXYZ_6 = fetchpoints(seq_tsp(6,:),opt_sec_6_figure);