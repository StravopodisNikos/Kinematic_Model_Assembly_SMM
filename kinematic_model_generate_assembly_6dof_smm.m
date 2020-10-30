% This code...

% HOW TO EXECUTE


% Include libraries
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions/subroutines_executed_in_objective_fn')

clear;
close all;

%% Only visualization for code evaluation
% Ref structure robot - All data must be calculated for reference structure
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/test_6dof.urdf';

[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);
