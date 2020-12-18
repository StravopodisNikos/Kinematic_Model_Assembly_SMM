clear;
clc;
close all;

% GENERATES TRAJECTORY IMPLEMENTATION PLOTS FOR EVALUATED:
% 1. MASS BALANCED STRUCTURES -> generated from 4_11
% 2. ISOTROPIC ANATOMIES      -> for dyn+kin mat files are saved in laptop:
% /home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/investigate_dynamic_kinematic_isotropy/optimized-structure-anatomies-matfiles/MBS_structures/4_11/pc-lef_mat_files
% 3. EXTRACTED STRUCTURES-ANATOMIES ARE PASSED AS URDF FILES ONLY - SINCE
% MATLAB DEFAULT ALGORITHMS ARE IMPLEMENTED

%% DEFINE ROBOT URDF FILES DESTINATION
rand_robot = 'C:\Users\admin-b14\Documents\matlab_ws\Kinematic_Model_Assembly_SMM\trajectory_implementation\urdf_files_structures_anatomies_here\s0_rand_s_rand_anat.urdf';
s3_ga_test_mult_4_11_20_opt_anat1_robot = 'C:\Users\admin-b14\Documents\matlab_ws\Kinematic_Model_Assembly_SMM\trajectory_implementation\urdf_files_structures_anatomies_here\s3_ga_test_mult_4_11_20_opt_anat1.urdf';

motion_type = "ComputedTorqueControl";
way_pts = [1.5708 -0.84 0.15 -0.7854 1.5708  ;...     %q1
           1.5708 1.35 -0.58  0.7854 -1.5708  ;...    %q2
           1.5708 2.14 -2.45  1.5708 -1.2456 ];...    %q3
           
field1 = 'way_pts'; value1 = way_pts;
n_seg_points = 5;
field2 = 'time'; value2 = n_seg_points*size(way_pts,2); % just preallocation for expai
field3 = 'traj'; value3 = 'trapveltraj';
s_traj = struct(field1,value1,field2,value2,field3,value3);
plot_title = 'first attempt';

%% Execute trajectory simulation
optimized_structure_anatomy_trajectory_implementation_matlab(rand_robot,motion_type,s_traj,plot_title)
optimized_structure_anatomy_trajectory_implementation_matlab(s3_ga_test_mult_4_11_20_opt_anat1_robot,motion_type,s_traj,plot_title)
