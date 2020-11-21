function DCI_index = calculateGlobalDCIindex_3DoF(diag_select,min_max_select,xi_ai_ref,g_s_link_as_ref,M_b_link_as_ref)
% This matlab code was copied from moul2 PC on Th. 22.10.20

% Function that returns min||max Global(DCI) in each active C-space

wrong_selection = false;

switch diag_select 
    case 'offdiag'
        wx = [0 0 0 1 1 1]; % for off diagonal elements
    case 'diag'
        wx = [1 1 1 0 0 0]; % for diagonal elements
    otherwise
        wrong_selection = true;
end

%% Ovidius robot properties
active_angle_limit(1) = 2.8; % [rad]
active_angle_limit(2) = 2; % [rad]
active_angle_limit(3) = 3.4; % [rad]
step_a2 = 0.20;
step_a3 = 0.20;
%% I.a. Definition of Configuration Space of first 3DoF
Cspace_count = 0;

for ta2=-active_angle_limit(2):step_a2:active_angle_limit(2)
        for ta3=-active_angle_limit(3):step_a3:active_angle_limit(3)
            
            % Vector for current Cspace point
            qa = [0.1 ta2 ta3];
            Cspace_count = Cspace_count + 1;

            %% I.b. In this configuration(qa1.qa2,qa3) compute GIM
            
            % calculateCoM_BodyJacobians_for_anat is better beacuse works
            % for all anatomies, but first the g_s_link_as(CoM frame of
            % each link @ desired anatomy) has to be specified. Here we are
            % still working on reference, so all is ok(for methodology check: kinematic_model_generate_assembly_smm.m)
            [J_b_sli] = calculateCoM_BodyJacobians_for_anat(xi_ai_ref, qa, g_s_link_as_ref );
             
            [M_b] = calculateGIM(J_b_sli,M_b_link_as_ref);
            
            %%  I.c Compute DCI value
            
            [DCI(Cspace_count)] = calculateDynamicConditioningIndexDimLess_3DoF(M_b,wx);
        end
end

switch min_max_select 
    case 'min'
        DCI_index = min(DCI);
    case 'max'
        DCI_index = max(DCI);
end

if wrong_selection == true
    DCI_index = 0;
    warning('WRONG SELECTION INPUT! DCI SET TO 0! MUST ABORT')
end

end