% Here matfile with optimization results extracted from:

%% MATFILE1
% STRUCTURES: gamultiobj optimization in file:
% ga_call_structure_optimization_minmaxMBS_richness_3DoF in pc-grafeio
% ANATOMIES:- gamultiobj optimization in file:
% ga_call_anatomy_optimization_isotropy_investigation.m in pc-lef

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

%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s1_ga_test_mult_4_11_20
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
%                                                 θp2(x)
s1_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 9.0000   2.0000   9.0000    3.0000 ];  %14.6095   0.0027 
s1_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 9.0000   7.0000   4.0000    2.0000 ];  %14.3670   0.0038 
s1_ga_test_mult_4_11_20_opt_anat(3,:) =  [ 9.0000   9.0000   10.0000   3.0000 ];  %14.8638   0.0026 
s1_ga_test_mult_4_11_20_opt_anat(4,:) =  [ 10.0000   2.0000   11.0000   6.0000];  %15.9332   0.0020
s1_ga_test_mult_4_11_20_opt_anat(5,:) =  [ 10.0000   7.0000   12.0000   5.0000];  %15.1380   0.0020
s1_ga_test_mult_4_11_20_opt_anat(6,:) =  [ 10.0000   4.0000   2.0000    2.0000];  %14.1808   0.0040

save('s1_4_11_optimized_structure_anatomies.mat','ga_structure_name','ga_structure','ga_assembly_parameters','s1_ga_test_mult_4_11_20_opt_anat');

%% MATFILE2
% STRUCTURES: gamultiobj optimization in file:
% ga_call_structure_optimization_minmaxMBS_richness_3DoF in pc-grafeio
% ANATOMIES:- gamultiobj optimization in file:
% ga_call_anatomy_optimization_isotropy_investigation.m in pc-lef

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

%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s1_ga_test_mult_4_11_20
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
%                                                                         θp4(x)
s2_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 7.0000   12.0000    3.0000    7.0000 ];  %13.2597    0.0014
s2_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 7.0000   12.0000    4.0000    9.0000 ];  %13.3935    0.0014 

save('s2_4_11_optimized_structure_anatomies.mat','ga_structure_name','ga_structure','ga_assembly_parameters','s2_ga_test_mult_4_11_20_opt_anat');

%% MATFILE2
% STRUCTURES: gamultiobj optimization in file:
% ga_call_structure_optimization_minmaxMBS_richness_3DoF in pc-grafeio
% ANATOMIES:- gamultiobj optimization in file:
% ga_call_anatomy_optimization_isotropy_investigation.m in pc-lef

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

%% OPTIMAL ANATOMIES EXTRACTED FOR STRUCTURE s1_ga_test_mult_4_11_20
% optimal_anatomies extracted from 25,26_11_executed_in_pc_lef
%                                                    θp2(x)              θp4(x)
s3_ga_test_mult_4_11_20_opt_anat(1,:) =  [ 8.0000    8.0000    5.0000    7.0000 ];  %11.6569    0.0014
s3_ga_test_mult_4_11_20_opt_anat(2,:) =  [ 9.0000    8.0000    4.0000   10.0000 ];  %11.7682    0.0013
s3_ga_test_mult_4_11_20_opt_anat(3,:) =  [ 10.0000   11.0000   4.0000   12.0000 ];  %11.8152    0.0013
s3_ga_test_mult_4_11_20_opt_anat(4,:) =  [ 10.0000    5.0000    5.0000    4.0000];  %12.4256    0.0012

save('s3_4_11_optimized_structure_anatomies.mat','ga_structure_name','ga_structure','ga_assembly_parameters','s3_ga_test_mult_4_11_20_opt_anat');
