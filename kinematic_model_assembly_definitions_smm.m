% Kinematic Model Assembly Definitions of SMM
% Tfs of structural blocks and assembly parameters used to build the Ovidius SMM are generated
% Are used in kinematic_model_generate_assembly_smm.m that follows the procedure executed in
% structure_assembly_GA.m - Modifications are made with regard to OVIDIUS
% manipulator characteristics

clear;
close all;

% Include libraries
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

% Only for test: Get robot urdf
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/conditioned_parameterized_SMM_assembly.urdf';

[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);

[RefRobot_links,CoM_RefRobot_links,gsli0,gsbj0,M0b_CoM,M0s_CoM] = robot_links_subtree_new(RefRobot,RefConfig,NumDoF);

%% Create SB of assembly parts

% 1. baselink
baselink_URDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/baselink.urdf';
[baselink_Robot,baselink_Fig,baselink_Config,baselink_NumDoF] = ImportRobotRefAnatomyModel(baselink_URDFfile);

base_link = char(baselink_Robot.BodyNames(1));
g_s_a1_0 = getTransform(baselink_Robot,baselink_Config,base_link);
p_a1_0 = g_s_a1_0(1:3,4);
w_a1_0 = [0 0 1]';
xi_a1_0 = createtwist(w_a1_0,p_a1_0); % ξa1

% 2. Pseudojoint Module
pseudo_module_URDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/pseudo_module.urdf';
[pseudo_module_Robot,pseudo_module_Fig,pseudo_module_Config,pseudo_module_NumDoF] = ImportRobotRefAnatomyModel(pseudo_module_URDFfile);

[pseudo_module_links,CoM_pseudo_module_links,g_k_lj_0,g_k_bj_0,M0b_CoM,M0s_CoM] = robot_links_subtree_new(pseudo_module_Robot,pseudo_module_Config,pseudo_module_NumDoF);

wp_j_0 = [1 0 0]';
PseudoConnectorja = char(pseudo_module_Robot.BodyNames(2));
PseudoConnectorjb = char(pseudo_module_Robot.BodyNames(3));
g_s_pj_0 = getTransform(pseudo_module_Robot, pseudo_module_Config, PseudoConnectorjb); % pseudo tf with respect to {S}
g_k_pj_0 = getTransform(pseudo_module_Robot, pseudo_module_Config,PseudoConnectorjb, PseudoConnectorja);  % pseudo moving tf with respect to pseudo static frame {ki} (from)ja->(into)jb
% g_k_lj_0 = [];% the tf from CoM of pseudo module to the {k_j} frame (origin) of the module

% 3. Pseudo moving -> Pseudo static
PseudoConnector_b_1 = char(RefRobot.BodyNames(3));
PseudoConnector_a_2 = char(RefRobot.BodyNames(4));
PseudoConnector_b_2 = char(RefRobot.BodyNames(5));
g_s_p_b_1_0 = getTransform(RefRobot, RefConfig, PseudoConnector_b_1);
g_s_p_a_2_0 = getTransform(RefRobot, RefConfig, PseudoConnector_a_2); % pseudo tf with respect to {S}
g_s_p_b_2_0 = getTransform(RefRobot, RefConfig, PseudoConnector_b_2); % pseudo tf with respect to {S}
g_p1_p2  = getTransform(RefRobot, RefConfig,PseudoConnector_a_2, PseudoConnector_b_1);  % pseudo_static2 tf with respect to pseudo_moving1 frame
g_p_b_2_p_a_2_0 = getTransform(RefRobot, RefConfig,PseudoConnector_b_2, PseudoConnector_a_2);  % pseudo moving tf with respect to pseudo static frame {ki} (from)ja->(into)jb

%% Synthetic joints tfs with
syn1_rpy = [0 0 0];
syn1_xyz = [0 0 0.048]';
syn1_tform = eul2tform(syn1_rpy);
syn1_tform(1:3,4) = syn1_xyz;

% Evaluation of fwk g_s_pj_0 - g_s_a1_0 * syn1_tform * g_k_pj_0 == 0