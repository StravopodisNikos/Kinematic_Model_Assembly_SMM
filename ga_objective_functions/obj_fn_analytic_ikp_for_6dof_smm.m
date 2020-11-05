function analyticIKP_anatomy_richness = obj_fn_analytic_ikp_for_6dof_smm(x,degree,criterion)
% x: structure definition-parameterization chromosome
% Chromosome x is uilt considering the smm structure building principles
% defined and the parameters given in the corresponding xacro file used for
% visualization
% x1     = 1st meta link, only 2nd pseudo assembly type
% x2,3   = 2nd meta link, 2 pseudos assembly type
% x4,5   = 3rd meta link, 2 pseudos assembly type
% x6,7   = 4th meta link, 2 pseudos assembly type
% x8,9   = 5th meta link, 2 pseudos assembly type
% x10-12 = 2nd pseudo of 1st metalink parameters
% x13-15 = 1st pseudo of 2nd metalink parameters
% x16-18 = 2nd pseudo of 2nd metalink parameters
% x19-21 = 1st pseudo of 3rd metalink parameters
% x22-24 = 2nd pseudo of 3rd metalink parameters
% x25-27 = 1st pseudo of 4th metalink parameters
% x28-30 = 2nd pseudo of 4th metalink parameters
% x31-33 = 1st pseudo of 5th metalink parameters
% x34-36 = 2nd pseudo of 5th metalink parameters
% x37    = 1st dxl assembly pitch parameter (2nd active joint)
% x38    = 2nd dxl assembly pitch parameter (3rd active joint)
% x39    = 3rd dxl assembly pitch parameter (4th active joint)
% x40    = 4th dxl assembly pitch parameter (5th active joint)
% x41    = 5th dxl assembly pitch parameter (6th active joint)

% degrees: 'degree2'            or  'degree4'
%              ||                       ||
%              \/                       \/
% criteria:'parallelism3'            'parallel'
%          'orth_intersect3'       'intersecting'
%                             'perpendicular_intersecting'

% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws/'));
root_path = strcat(root_path(1),'matlab_ws/');

% Add libraries relative to matlab_ws folder
ga_path_relative_to_matlab_ws = fullfile('Kinematic_Model_Assembly_SMM','ga_objective_functions','subroutines_executed_in_objective_fn',filesep);
ga_library_path = strcat(root_path,ga_path_relative_to_matlab_ws); addpath(ga_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions/subroutines_executed_in_objective_fn') % works only for pc-grafeio

%% I.  Assign structure parameters values
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
passive_under_string_notation = '21';

%  I.1 Assembly sequence definition
% Since no stribg values are produced in ga, matching must be first made
% for x1-9
for ga_i_cnt=1:9
    if floor(x(ga_i_cnt)) == 1
        x_s(ga_i_cnt,:) = 'x9';
    elseif floor(x(ga_i_cnt)) == 2
        x_s(ga_i_cnt,:) = '21';
    elseif floor(x(ga_i_cnt)) == 3
        x_s(ga_i_cnt,:) = '31';
    end
end
% and for x37,x41
for ga_i_cnt=37:41
    if floor(x(ga_i_cnt)) == 0
        x_r(ga_i_cnt) = 0;
    elseif floor(x(ga_i_cnt)) == 1
        x_r(ga_i_cnt) = 1.5708;
    end
end

% Now build structure as in file:
% /home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/yamls/test/test_structure_string_definition_6dof_for_ga.yaml
structure(1,:) = fixed_active_string_notation;      % ALWAYS ACTIVE
structure(2,:) = passive_under_string_notation;     % FIXED ASSEMBLY FOR 1st pseudo       
structure(3,:) = x_s(1,:);
structure(4,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(5,:) = x_s(2,:);
structure(6,:) = x_s(3,:);
structure(7,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(8,:) = x_s(4,:);
structure(9,:) = x_s(5,:);
structure(10,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(11,:) = x_s(6,:);
structure(12,:) = x_s(7,:);
structure(13,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly
structure(14,:) = x_s(8,:);
structure(15,:) = x_s(9,:);
structure(16,:) = fixed_active_string_notation;      % ALWAYS ACTIVE but now a pitch angle is also given for Dxl assembly

%  I.2 Assembly parameters as in file: /home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/yamls/test/assembly_parameters_for_ovidius_robot_6dof_for_ga.yaml
assembly_parameters(1,:) = x(10:12);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(2,:) = x(13:15);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(3,:) = x(16:18);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(4,:) = x(19:21);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(5,:) = x(22:24);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(6,:) = x(25:27);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(7,:) = x(28:30);                % 2nd pseudo of 2nd metalink parameters
assembly_parameters(8,:) = x(31:33);                  % 2nd pseudo of 1st metalink parameters
assembly_parameters(9,:) = x(34:36);                  % 1st pseudo of 2nd metalink parameters

assembly_parameters(10,1) = 0;                      % dummy zero since 1st active joint is fixed
assembly_parameters(10,2) = x_r(37);                % 1st dxl assembly pitch parameter
assembly_parameters(10,3) = x_r(38);                % 2nd dxl assembly pitch parameter    
assembly_parameters(11,1) = x_r(39);                % 3rd dxl assembly pitch parameter
assembly_parameters(11,2) = x_r(40);                % 4th dxl assembly pitch parameter 
assembly_parameters(11,3) = x_r(41);                % 5th dxl assembly pitch parameter

%% II. Structure assembly
[xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,~,~,~] = structure_assembly_6dof(structure,assembly_parameters);

%% III. Anatomy Richness - Here starts anatomy exhaustive calculation for the assembled structure
[total_special_anatomies] = calculateExhaustiveAnatomies_for_analytic_ikp(structure,degree,criterion,xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0);

%% V. Final structure score
if total_special_anatomies == 0
    analyticIKP_anatomy_richness = 10000000;
else
    analyticIKP_anatomy_richness = 1/total_special_anatomies;
end

end
