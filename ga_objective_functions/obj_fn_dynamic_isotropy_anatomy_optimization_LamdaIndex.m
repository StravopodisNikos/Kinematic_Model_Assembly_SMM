function LmdCI_star = obj_fn_dynamic_isotropy_anatomy_optimization_LamdaIndex(x,extracted_structure,extracted_assembly_parameters)
% This function optimizes the anatomy of a smm manipulator structure w.r.t kinematic
% isotropy condition. the structure MUST previously be extracted.

%% x: structure definition-parameterization chromosome
% Chromosome x is built considering the smm structure building principles
% defined and the parameters given in the corresponding xacro file used for
% visualization

% x1-4 = since 3dof, 4 pseudos are max
ci = x(1:4); 

%% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

ga_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','ga_objective_functions','subroutines_executed_in_objective_fn',filesep);
ga_library_path = strcat(root_path,ga_path_relative_to_matlab_ws); addpath(ga_library_path);

terminate_ga_obj_fn = false;    % used to control ga

%% I. Determine tpi pseudojoint angles
step_angle = deg2rad(15);
tpi0 = -1.5708; tp = zeros(4,1);
for k=1:4
    tp(k) = step_angle * (floor(ci(k))-1) + tpi0;
end

%% II. Structure assembly
[xi_ai_struct_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,~,~,wrong_string_structure] = structure_assembly_3dof(extracted_structure,extracted_assembly_parameters);

if wrong_string_structure
    LmdCI_star = 10000000;
    terminate_ga_obj_fn = true;
end

if terminate_ga_obj_fn==false
    %% III. Calculate pseudo exponentials of ga anatomy @ zero configuration
    qp_structure_dependent = calculate_transformed_anatomy_vector(extracted_structure,'3dof',tp);
    [~,~,~,Pi,~] = calculateForwardKinematicsPOE(extracted_structure,'3dof',xi_ai_struct_ref,xi_pj_ref,zeros(3,1),qp_structure_dependent,g_ai_ref,g_pj_ref,gst0);  
    
    %% IV. Calculate new twists and inertia matrices of ga anatomy @ zero configuration
    [~,xi_ai_struct_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_no_graph(extracted_structure,extracted_assembly_parameters,qp_structure_dependent,xi_pj_ref);
    [g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);
    [M_b_link_as_anat] = calculateMetalinkInertiaMatrixBody(g_s_link_as_anat,M_s_link_as_anat);
    
    %% V. Calculate index at C-Space of ga anatomy 
    LmdCI = calculateGlobalLamdaConditionIndex_3DoF(xi_ai_struct_anat,xi_ai_struct_ref,g_s_link_as_anat,M_b_link_as_anat, Pi, gst0);
    
    LmdCI_star = LmdCI;
end

end