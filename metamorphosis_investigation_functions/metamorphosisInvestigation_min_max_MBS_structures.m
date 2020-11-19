% this function visualizes the metamorphosis effect on the MBS variation of
% the optimized structure extracted by ga for min/max MBS.
% The structures investigated here are extracted from file:
% Kinematic_Model_Assembly/ga_optimization_call_functions/ga_optimization_results/evaluate_ga_test_mult_4_11_20.txt

% GOAL: 1. investigate which sets of pseudoangles OR the variation of which
% pseudo angles play key role on minimization of MBS for the structure 2.
% many sets of pseudoangles that lead to min MBS

clear;
clc;

% Methodology:
% 1. extract structure and build it
% 2. exhaustive calculation of MBS for all anatomies@reference
% configuration
% 3. visualization
% 4. connected components

%% 1. Define structure: s1_ga_test_mult_4_11_20 and build it
%% Define smm structure string produced by ga
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3
% structure sting
structure(1,:) = fixed_active_string_notation;  % default-constant
structure(2,:) = passive_under_string_notation; % default-constant
structure(3,:) = no_passive_string_notation;
structure(4,:) = fixed_active_string_notation;  % default-constant
structure(5,:) = passive_back_string_notation;
structure(6,:) = passive_back_string_notation;
structure(7,:) = fixed_active_string_notation;  % default-constant
% assembly parameters
assembly_parameters(1,:) = [-0.0121,    0.0175,   -0.8370]';                  % syn2 bcause 31
assembly_parameters(2,:) = [-0.0256,   -0.0190,    1.2980]';                  % syn3 because 31
assembly_parameters(3,:) = [-0.0408,    0.0092,    0.4821]';                % syn4 because 21
assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
assembly_parameters(4,2) = 0.5987;                   % 1st dxl assembly pitch parameter
assembly_parameters(4,3) = 0.6594;                   % 2nd dxl assembly pitch parameter

[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters);

%% 2. Exhaustive calculation of anatomies & MBS
[all_pseudo_sets,total_number_of_sets] = calculateExhaustiveAnatomies_for_metamorphosis_investigation(xi_pj_ref);
%% Evaluate each anatomy
for anatomy_cnt=1:total_number_of_sets
    
        evaluated_anatomy = all_pseudo_sets(anatomy_cnt,:); % pick the row of array
        
        MBS_i(anatomy_cnt) = calculateMBS_no_graph(structure,assembly_parameters,xi_ai_ref,xi_pj_ref,evaluated_anatomy);
        
end
all_MBS_vector = MBS_i';

%% 3. reshape? scatter3?