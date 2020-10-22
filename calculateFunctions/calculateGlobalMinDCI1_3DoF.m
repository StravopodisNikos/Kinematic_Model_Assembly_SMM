function DCI_star = calculateGlobalMinDCI1_3DoF(xi_ai_ref,Pi_i1,g_s_link_as_ref,M_b_link_as_ref)
% This matlab code was copied from moul2 PC on Th. 22.10.20

% Function that returns min(DCI) in each active C-space

wx = [0 0 0 1 1 1]; % for off diagonal elements

%% Ovidius robot properties
active_angle_limit(1) = 2; % [rad]
active_angle_limit(2) = 2; % [rad]
active_angle_limit(3) = 2; % [rad]

%% I.a. Definition of Configuration Space of first 3DoF
Cspace_count = 0;

for ta2=-active_angle_limit(2):step_a2:active_angle_limit(2)
        for ta3=-active_angle_limit(3):step_a3:active_angle_limit(3)
            
            % Vector for current Cspace point
            qa = [0.1 ta2 ta3];
            Cspace_count = Cspace_count + 1;

            %% I.b. In this configuration(qa1.qa2,qa3) compute GIM
            
            [J_b_sli,~] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi_i1, g_s_link_as_ref );
             
            [M_b] = calculateGIM(J_b_sli,M_b_link_as_ref);
            
            %%  I.c Compute DCI value
            
            [DCI(Cspace_count)] = calculateDynamicConditioningIndexDimLess_3DoF(M_b,wx);
        end
end

DCI_star = min(DCI);
end