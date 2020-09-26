% Kinematic Model Assembly of SMM

addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')
addpath('/home/nikos/matlab_ws/smm_dynamics')

% Only for test: Get robot urdf
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/conditioned_parameterized_SMM_assembly.urdf';

[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);

[RefRobot_links,CoM_RefRobot_links,gsli0,gsbj0,M0b_CoM,M0s_CoM] = robot_links_subtree_new(RefRobot,RefConfig,NumDoF);

% 