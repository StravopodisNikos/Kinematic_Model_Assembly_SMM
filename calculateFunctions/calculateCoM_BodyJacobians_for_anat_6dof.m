function [J_b_sli] = calculateCoM_BodyJacobians_for_anat_6dof(xi_ai_anat,qa, g_s_link_as )
% Follows calculateCoM_BodyJacobians BUT NOW each CoM of the links is
% pre-calculated under construction and NOT by the fwd kinematics. So no
% pseudo exponentials product must be integrated in the calculation!

% Also the active anatomy twists are different wrt to reference anatomy!

% Built together with function calculateCoM_ki_s_structure_anatomy.m

nDoF = size(xi_ai_anat,2);           % determine total number of active joints by number of columns
% g_s_li(:,:,1) = eye(4);

% preallocate memory
for i_cnt=1:nDoF
    J_b_sli(:,:,i_cnt) = zeros(6,nDoF);
    g_sli_anat(:,:,i_cnt) = eye(4);
    g_sli_anat(1:3,4,i_cnt) = g_s_link_as(:,1,i_cnt);
end

% Compute only active exponentials
for i_cnt=1:nDoF
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),qa(i_cnt));
end

% Based on equations for Jbsli @ p.168 Murray
for i_cnt=1:nDoF
    if i_cnt==1
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*g_sli_anat(:,:,1)))*xi_ai_anat(:,1);
        
    elseif i_cnt==2
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*g_sli_anat(:,:,2)))*xi_ai_anat(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*g_sli_anat(:,:,2)))*xi_ai_anat(:,2);
        
    elseif i_cnt==3
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*g_sli_anat(:,:,3)))*xi_ai_anat(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*exp_ai(:,:,3)*g_sli_anat(:,:,3)))*xi_ai_anat(:,2); 
        J_b_sli(:,3,i_cnt) = inv(ad(exp_ai(:,:,3)*g_sli_anat(:,:,3)))*xi_ai_anat(:,3); 
        
    elseif i_cnt==4
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*g_sli_anat(:,:,4)))*xi_ai_anat(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*g_sli_anat(:,:,4)))*xi_ai_anat(:,2); 
        J_b_sli(:,3,i_cnt) = inv(ad(exp_ai(:,:,3)*exp_ai(:,:,4)*g_sli_anat(:,:,4)))*xi_ai_anat(:,3);
        J_b_sli(:,4,i_cnt) = inv(ad(exp_ai(:,:,4)*g_sli_anat(:,:,4)))*xi_ai_anat(:,4);
 
    elseif i_cnt==5
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*g_sli_anat(:,:,5)))*xi_ai_anat(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*g_sli_anat(:,:,5)))*xi_ai_anat(:,2); 
        J_b_sli(:,3,i_cnt) = inv(ad(exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*g_sli_anat(:,:,5)))*xi_ai_anat(:,3);
        J_b_sli(:,4,i_cnt) = inv(ad(exp_ai(:,:,4)*exp_ai(:,:,5)*g_sli_anat(:,:,5)))*xi_ai_anat(:,4);
        J_b_sli(:,5,i_cnt) = inv(ad(exp_ai(:,:,5)*g_sli_anat(:,:,5)))*xi_ai_anat(:,5);
        
    elseif i_cnt==6
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,2); 
        J_b_sli(:,3,i_cnt) = inv(ad(exp_ai(:,:,3)*exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,3);
        J_b_sli(:,4,i_cnt) = inv(ad(exp_ai(:,:,4)*exp_ai(:,:,5)*exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,4);
        J_b_sli(:,5,i_cnt) = inv(ad(exp_ai(:,:,5)*exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,5);        
        J_b_sli(:,6,i_cnt) = inv(ad(exp_ai(:,:,6)*g_sli_anat(:,:,6)))*xi_ai_anat(:,6);
        
    else
         warning('[SMM CALCULATE BODY JACOBIANS]: IMPLEMENTED ONLY FOR 6 DoF STRUCTURE')
    end
end

end