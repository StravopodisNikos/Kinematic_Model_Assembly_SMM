% This code generates the assembly of the sructure extracted from
% optimization code run from file:
% ga_call_structure_optimization_3DoF_minmaxDCI_MBS_richness.m
% Will be used to visualize and evaluate optimal results

% How to use:
% 1. First check mat file extracted form ga run
% 2. Build the urdf file from the xacro file: 
% /Parametric_Simulation_Model_SMM/xacros/conditioned_parameterized_SMM_assembly_for_ga.xacro
% and the corresponding yaml files:
% /Parametric_Simulation_Model_SMM/yamls/test/assembly_parameters_for_ovidius_robot_for_ga.yaml
% /Parametric_Simulation_Model_SMM/yamls/test/test_structure_string_definition_for_ga.yaml
% 3. Specify the correct path to urdf file in l.25

% Include libraries
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
clear;
close all;

%% Only visualization for code evaluation
% Ref structure robot - All data must be calculated for reference structure
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/best_structure_test_27_10.urdf';

[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);
