function MB_star = obj_fn_mass_balancing_structure_optimization_maxDCIoffdiag_SMM(x)
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

terminate_ga_obj_fn = false;    % used to control ga

%% I.  Assign structure parameters values
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
passive_under_string_notation = '21';
%  I.1 Assembly sequence definition
structure(1,:) = fixed_active_string_notation;      % ALWAYS ACTIVE
structure(2,:) = passive_under_string_notation;     % FIXED ASSEMBLY FOR 1st pseudo       
structure(3,:) = x(1);
structure(4,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(5,:) = x(2);
structure(6,:) = x(3);
structure(7,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
%  I.2 Assembly parameters
assembly_parameters(1,:) = x(4:6);                  % 2nd pseudo of 1st metalink parameters
assembly_parameters(2,:) = x(7:9);                  % 1st pseudo of 2nd metalink parameters
assembly_parameters(3,:) = x(10:12);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(4,1) = 0;                       % dummy zero since 1st active joint is fixed
assembly_parameters(4,2) = x(13);                   % 1st dxl assembly pitch parameter
assembly_parameters(4,3) = x(14);                   % 2nd dxl assembly pitch parameter    

%% II. Structure assembly
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters);

if wrong_string_structure
    MB_star = 10000000;
    terminate_ga_obj_fn = true;
end

if terminate_ga_obj_fn==false
    
% II.1 FWD KINEMATICS @ Reference Anatomy and configuration
% nDoF = size(xi_ai_ref,2);
nPseudo = size(xi_pj_ref,2); 

% qa = zeros(nDoF,1);     % Reference Configuration
qp = zeros(nPseudo,1);  % Reference Anatomy
%[~,~,~,Pi_i1_ref,~] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);

%% III. Compute CoMi and Ms,Mb of metamorphic links @ Reference Anatomy
[g_s_link_as_ref,M_s_link_as_ref] = calculateCoMmetalinks(M_s_com_k_i,g_s_com_k_i);
[M_b_link_as_ref] = calculateMetalinkInertiaMatrixBody(g_s_link_as_ref,M_s_link_as_ref);

%% IV. MBS  @ Reference Anatomy
MBS_ref = calculateMBS_no_graph(structure,xi_ai_ref,xi_pj_ref,g_s_link_as_ref,g_ai_ref,g_pj_ref,gst0,M_s_link_as_ref,qp);

%% V. GDCI  @ Reference Anatomy
DCI_star_ref = calculateGlobalDCI_3DoF('max','offdiag',xi_ai_ref,Pi_i1,g_s_link_as_ref,M_b_link_as_ref);

%% VI. Anatomy Richness - Here starts anatomy exhaustive calculation
[F_i_rich] = calculateExhaustiveAnatomies_for_MBS(MBS_ref,xi_ai_ref,xi_pj_ref,g_s_link_as_ref,g_ai_ref,g_pj_ref,gst0,M_s_link_as_ref);

%% V. Final structure score
MB_star = DCI_star_ref/F_i_rich;

end

end