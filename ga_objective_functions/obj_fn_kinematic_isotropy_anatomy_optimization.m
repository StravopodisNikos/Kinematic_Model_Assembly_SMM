function MB_star = obj_fn_kinematic_isotropy_anatomy_optimization(x,extracted_structure,extracted_assembly_parameters)
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

%% Determine tpi pseudojoint angles
step_angle = deg2rad(45);
tpi0 = -1.5708;
for k=1:4
    tp(k) = step_angle * (floor(ci(k))-1) + tpi0;
end

%% II. Structure assembly
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(extracted_structure,extracted_assembly_parameters);

if wrong_string_structure
    MB_star = 10000000;
    terminate_ga_obj_fn = true;
end

if terminate_ga_obj_fn==false
    

end

end