% Here C-space evaluation of the optimized anatomies extracted from generate_matfiles.m in folder:
% /home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/investigate_dynamic_kinematic_isotropy/optimized-structure-anatomies-matfiles/MBS_structures/4_11
% Each matfile loads the variables:  'ga_structure_name','ga_structure','ga_assembly_parameters','s{i}_ga_test_mult_4_11_20_opt_anat')
clear;
clc;
close all;
%% Add paths to all matfiles:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
matfiles_4_11_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','investigate_dynamic_kinematic_isotropy','optimized-structure-anatomies-matfiles','MBS_structures','4_11',filesep);
matfiles_4_11_library_path = strcat(root_path,matfiles_4_11_path_relative_to_matlab_ws); addpath(matfiles_4_11_path_relative_to_matlab_ws);

%% STRUCTURE 1
load('s1_4_11_optimized_structure_anatomies.mat');



