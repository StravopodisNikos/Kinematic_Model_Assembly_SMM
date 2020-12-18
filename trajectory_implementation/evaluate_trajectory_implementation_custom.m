clear;
clc;
close all;

% GENERATES TRAJECTORY IMPLEMENTATION PLOTS FOR EVALUATED:
% 1. MASS BALANCED STRUCTURES -> generated from 4_11
% 2. ISOTROPIC ANATOMIES      -> for dyn+kin mat files are saved in laptop:
% /home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/investigate_dynamic_kinematic_isotropy/optimized-structure-anatomies-matfiles/MBS_structures/4_11/pc-lef_mat_files

%% SET TRAJECTORY PROPERTIES
% C-space task points
way_pts = [1.5708 -0.84 0.15 -0.7854 1.5708  ;...     %q1
           1.5708 1.35 -0.58  0.7854 -1.5708  ;...    %q2
           1.5708 2.14 -2.45  1.5708 -1.2456 ];...    %q3
% time steps           
time_pts = [0  5  10  15   20]';    
% PID Parametes
field1 = 'kp';        value1 = 300;
field2 = 'ki_factor'; value2 = 0.4; % for closed loop stability ki < kp * kd => ki = i_factor * (kp * kd), i_factor \in [0,1]
field3 = 'kd';        value3 = 10;
s_pid = struct(field1,value1,field2,value2,field3,value3);
%% LOAD STRUCTURES
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3
%% SET RANDOM-STRUCTURE 0
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name0 = 's0_3dof_random';
ga_structure0(1,:) = fixed_active_string_notation;
ga_structure0(2,:) = passive_under_string_notation;
ga_structure0(3,:) = passive_under_string_notation;
ga_structure0(4,:) = fixed_active_string_notation;
ga_structure0(5,:) = passive_back_string_notation;
ga_structure0(6,:) = passive_under_string_notation;
ga_structure0(7,:) = fixed_active_string_notation;
ga_assembly_parameters0(1,:) = [-0.005    -0.0175   0.7854]';                  % syn2 bcause 31
ga_assembly_parameters0(2,:) = [-0.04     0.0250    -1.5708]';                  % syn3 because 31
ga_assembly_parameters0(3,:) = [0.0508    0.0092    1.5708]';                % syn4 because 21
ga_assembly_parameters0(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters0(4,2) = 1.5708;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters0(4,3) = -1.57084;                   % 2nd dxl assembly pitch parameter
%% RANDOM ANATOMY SELECTED                          
s0_rand_s_rand_anat(1,:) =  [ 7.0000   5.0000   6.0000    10.0000 ];  % for [rad]: deg2rad(15) * (floor(ci)-1) + (-1.5708)

%% SET GA-OPTIMIZED-STRUCTURE 1
%% STRUCTURE-NAME GIVEN IN ga_optimization_results/{evaluate file}
ga_structure_name1 = 's1_ga_test_mult_4_11_20';
ga_structure1(1,:) = fixed_active_string_notation;
ga_structure1(2,:) = passive_under_string_notation;
ga_structure1(3,:) = no_passive_string_notation;
ga_structure1(4,:) = fixed_active_string_notation;
ga_structure1(5,:) = passive_back_string_notation;
ga_structure1(6,:) = passive_back_string_notation;
ga_structure1(7,:) = fixed_active_string_notation;
ga_assembly_parameters1(1,:) = [-0.0121    0.0175   -0.8370]';                  % syn2 bcause 31
ga_assembly_parameters1(2,:) = [-0.0256   -0.0190    1.2980]';                  % syn3 because 31
ga_assembly_parameters1(3,:) = [-0.0408    0.0092    0.4821]';                % syn4 because 21
ga_assembly_parameters1(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters1(4,2) = 0.5987;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters1(4,3) = 0.6594;                   % 2nd dxl assembly pitch parameter
%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s1_ga_test_mult_4_11_20
%                                                 θp2(x)
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
s1_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 9.0000   2.0000   9.0000    3.0000 ];  %14.6095   0.0027 
s1_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 9.0000   7.0000   4.0000    2.0000 ];  %14.3670   0.0038 
s1_ga_test_mult_4_11_20_opt_anat(3,:) =  [ 9.0000   9.0000   10.0000   3.0000 ];  %14.8638   0.0026 
s1_ga_test_mult_4_11_20_opt_anat(4,:) =  [ 10.0000  2.0000   11.0000   6.0000];  %15.9332   0.0020
s1_ga_test_mult_4_11_20_opt_anat(5,:) =  [ 10.0000  7.0000   12.0000   5.0000];  %15.1380   0.0020
s1_ga_test_mult_4_11_20_opt_anat(6,:) =  [ 10.0000  4.0000   2.0000    2.0000];  %14.1808   0.0040

%% SET GA-OPTIMIZED-STRUCTURE 2
ga_structure_name2 = 's2_ga_test_mult_4_11_20';
ga_structure2(1,:) = fixed_active_string_notation;
ga_structure2(2,:) = passive_under_string_notation;
ga_structure2(3,:) = passive_back_string_notation;
ga_structure2(4,:) = fixed_active_string_notation;
ga_structure2(5,:) = passive_back_string_notation;
ga_structure2(6,:) = no_passive_string_notation;
ga_structure2(7,:) = fixed_active_string_notation;
ga_assembly_parameters2(1,:) = [0.0269   -0.0422   -0.6583]';                  % syn2 bcause 31
ga_assembly_parameters2(2,:) = [0.0064   -0.0195   -1.3607]';                  % syn3 because 31
ga_assembly_parameters2(3,:) = [0.0242    0.0070   -0.4861]';                % syn4 because 21
ga_assembly_parameters2(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters2(4,2) = 1.2028;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters2(4,3) = 1.1134;                   % 2nd dxl assembly pitch parameter

%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s2_ga_test_mult_4_11_20
%                                                                         θp4(x)
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
s2_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 7.0000   12.0000    3.0000    7.0000 ];  %13.2597    0.0014
s2_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 7.0000   12.0000    4.0000    9.0000 ];  %13.3935    0.0014 

%% SET GA-OPTIMIZED-STRUCTURE 3
ga_structure_name3 = 's3_ga_test_mult_4_11_20';
ga_structure3(1,:) = fixed_active_string_notation;
ga_structure3(2,:) = passive_under_string_notation;
ga_structure3(3,:) = no_passive_string_notation;
ga_structure3(4,:) = fixed_active_string_notation;
ga_structure3(5,:) = passive_back_string_notation;
ga_structure3(6,:) = no_passive_string_notation;
ga_structure3(7,:) = fixed_active_string_notation;
ga_assembly_parameters3(1,:) = [-0.0226,0.0211,0.3009]';                  % syn2 bcause 31
ga_assembly_parameters3(2,:) = [0.0244,-0.0153,1.4347]';                  % syn3 because 31
ga_assembly_parameters3(3,:) = [-0.0138,0.0137,0.0331]';                % syn4 because 21
ga_assembly_parameters3(4,1) = 0;                       % dummy zero since 1st active joint is fixed
ga_assembly_parameters3(4,2) = 0.9486;                   % 1st dxl assembly pitch parameter
ga_assembly_parameters3(4,3) = 1.1225;                   % 2nd dxl assembly pitch parameter

%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s3_ga_test_mult_4_11_20
%                                                    θp2(x)              θp4(x)
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
s3_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 8.0000    8.0000    5.0000    7.0000 ];  %11.6569    0.0014
s3_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 9.0000    8.0000    4.0000   10.0000 ];  %11.7682    0.0013
s3_ga_test_mult_4_11_20_opt_anat(3,:) =  [ 10.0000   11.0000   4.0000   12.0000 ];  %11.8152    0.0013
s3_ga_test_mult_4_11_20_opt_anat(4,:) =  [ 10.0000    5.0000    5.0000    4.0000];  %12.4256    0.0012

%% EXECUTE TRAJECTORY FOR EACH ANATOMY with CUSTOM PID MOTION CONTROL!
optimized_structure_anatomy_trajectory_implementation_custom(ga_structure0,ga_assembly_parameters0,s0_rand_s_rand_anat(1,:),way_pts,time_pts,s_pid,'PID[100-0.4-25] MOTION CONTROL - 5th ORDER TRAJECTORY[task points:5-dt:0.1] ');

optimized_structure_anatomy_trajectory_implementation_custom(ga_structure3,ga_assembly_parameters3,s3_ga_test_mult_4_11_20_opt_anat(1,:),way_pts,time_pts,s_pid,'PID[100-0.4-25] MOTION CONTROL - 5th ORDER TRAJECTORY[task points:5-dt:0.1] ');
