function [PM_Cspace,Th2,Th3] = configuration_space_theta23loops(step_angle,theta_lim,PM_string,xi_ai_anat,g_s_link_as_anat,M_b_link_as_anat)
% Input: 1,2. 3x1 vectors of step/limit angle of active joints
%          3. string that indicates the performance measure to calculate, yet available: 1. 'dci-off' 2. 'dci-diag' 3. 'dci-all' 
%          4. active joint twists for optimized structure+anatomy!

% Pre-allocate Memory
theta2_vec = linspace(-theta_lim(2),theta_lim(2),((2*theta_lim(2))/step_angle(2))+1);
theta3_vec = linspace(-theta_lim(3),theta_lim(3),((2*theta_lim(3))/step_angle(3))+1);
array_size2 = size( theta2_vec,2 );
array_size3 = size( theta3_vec,2 );

PM_Cspace = zeros(array_size2,array_size3);
[Th2,Th3] = meshgrid(theta2_vec,theta3_vec);

% Loops
ta2_cnt = 0;
for ta2=-theta_lim(2):step_angle(2):theta_lim(2)
    ta2_cnt = ta2_cnt +1;
        ta3_cnt = 0;
        for ta3=-theta_lim(3):step_angle(3):theta_lim(3)
            ta3_cnt = ta3_cnt + 1;
            
            qa = [0.1 ta2 ta3];
            
            [J_b_sli] = calculateCoM_BodyJacobians_for_anat(xi_ai_anat, qa, g_s_link_as_anat );
             
            [M_b] = calculateGIM(J_b_sli,M_b_link_as_anat);
            
            %%  I.c Compute DCI value
            switch PM_string
                case 'dci-off'
                    wx = [0 0 0 1 1 1]; % for off diagonal elements
                    [PM_Cspace(ta2_cnt,ta3_cnt)] = calculateDynamicConditioningIndexDimLess_3DoF(M_b,wx);
                case 'dci-diag'
                    wx = [1 1 1 0 0 0]; % for diagonal elements
                    [PM_Cspace(ta2_cnt,ta3_cnt)] = calculateDynamicConditioningIndexDimLess_3DoF(M_b,wx);
               case 'dci-all'
                    wx = [1 1 1 1 1 1]; % for diagonal elements
                    [PM_Cspace(ta2_cnt,ta3_cnt)] = calculateDynamicConditioningIndexDimLess_3DoF(M_b,wx);
                case 'DME'
                    
            end
        end
end

end