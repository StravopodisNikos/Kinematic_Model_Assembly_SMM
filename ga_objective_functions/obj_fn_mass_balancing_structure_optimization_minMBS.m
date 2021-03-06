function MB_star = obj_fn_mass_balancing_structure_optimization_minMBS(x)
% x: structure definition-parameterization chromosome
% Chromosome x is uilt considering the smm structure building principles
% defined and the parameters given in the corresponding xacro file used for
% visualization
% x1 = 1st meta link's 2nd pseudo assembly type
% x2,3 = 2nd meta link, 2 pseudos assembly type
% x4-6 = 2nd pseudo of 1st metalink parameters
% x7-9 = 1st pseudo of 2nd metalink parameters
% x10-12 = 2nd pseudo of 2nd metalink parameters
% x13 = 1st dxl assembly pitch parameter
% x14 = 2nd dxl assembly pitch parameter

addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions/subroutines_executed_in_objective_fn')

terminate_ga_obj_fn = false;    % used to control ga

%% I.  Assign structure parameters values
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
passive_under_string_notation = '21';

%  I.1 Assembly sequence definition
% Since no stribg values are produced in ga, matching must be first made
% for x1,x2,x3
for ga_i_cnt=1:3
    if floor(x(ga_i_cnt)) == 1
        x_s(ga_i_cnt,:) = 'x9';
    elseif floor(x(ga_i_cnt)) == 2
        x_s(ga_i_cnt,:) = '21';
    elseif floor(x(ga_i_cnt)) == 3
        x_s(ga_i_cnt,:) = '31';
    end
end
% and for x13,x14
for ga_i_cnt=13:14
    if floor(x(ga_i_cnt)) == 0
        x_r(ga_i_cnt) = 0;
    elseif floor(x(ga_i_cnt)) == 1
        x_r(ga_i_cnt) = 1.5708;
    end
end

structure(1,:) = fixed_active_string_notation;      % ALWAYS ACTIVE
structure(2,:) = passive_under_string_notation;     % FIXED ASSEMBLY FOR 1st pseudo       
structure(3,:) = x_s(1,:);
structure(4,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(5,:) = x_s(2,:);
structure(6,:) = x_s(3,:);
structure(7,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly

%  I.2 Assembly parameters
assembly_parameters(1,:) = x(4:6);                  % 2nd pseudo of 1st metalink parameters
assembly_parameters(2,:) = x(7:9);                  % 1st pseudo of 2nd metalink parameters
assembly_parameters(3,:) = x(10:12);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
assembly_parameters(4,2) = x_r(13);                   % 1st dxl assembly pitch parameter
assembly_parameters(4,3) = x_r(14);                   % 2nd dxl assembly pitch parameter    

%% II. Structure assembly
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters);

if wrong_string_structure
    MB_star = 10000000;
    terminate_ga_obj_fn = true;
end

if terminate_ga_obj_fn==false
    
% II.1 FWD KINEMATICS @ Reference Anatomy and configuration
% nDoF = size(xi_ai_ref,2);
% nPseudo = size(xi_pj_ref,2); 

% qa = zeros(nDoF,1);     % Reference Configuration
% qp = zeros(nPseudo,1);  % Reference Anatomy
% [~,~,~,Pi_i1_ref,~] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);

%% III. Compute CoMi and Ms,Mb of metamorphic links @ Reference Anatomy
%[g_s_link_as_ref,M_s_link_as_ref] = calculateCoMmetalinks(M_s_com_k_i,g_s_com_k_i);
%[M_b_link_as_ref] = calculateMetalinkInertiaMatrixBody(g_s_link_as_ref,M_s_link_as_ref);

%% IV. MBS  @ Reference Anatomy
% MBS_ref = calculateMBS_no_graph(structure,assembly_parameters,xi_ai_ref,xi_pj_ref,qp);

%% V. Anatomy Richness - Here starts anatomy exhaustive calculation
[min_MBS_anat] = calculateExhaustiveAnatomies_for_MBS(structure,assembly_parameters,'min',xi_ai_ref,xi_pj_ref);

%% V. Final structure score
MB_star = min_MBS_anat;
end

end