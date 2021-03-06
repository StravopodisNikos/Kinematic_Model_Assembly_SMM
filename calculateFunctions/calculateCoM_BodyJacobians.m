function [J_b_sli] = calculateCoM_BodyJacobians(xi_ai_ref,qa,Pi_i1, g_s_link_as )
% Follows the methodology used in calculateForwardKinematicsPOE.m:
% Structure string is evaluateed in order to extract the equations
% Assumes that for desired structre @ ref anatomy:
% 1. the active/pseudo twists are extracted
% 2. the intermidiate pseudo exponential product induced by each metalink
% is extracted
% 3. the CoM and inertia matrices of assembled metalinks are extracted

nDoF = size(xi_ai_ref,2);           % determine total number of active joints by number of columns
% g_s_li(:,:,1) = eye(4);

% preallocate memory
for i_cnt=1:nDoF
    J_b_sli(:,:,i_cnt) = zeros(6,nDoF);
    g_sli_ref(:,:,i_cnt) = eye(4);
    g_sli_ref(1:3,4,i_cnt) = g_s_link_as(:,1,i_cnt);
end

% Compute only active exponentials
for i_cnt=1:nDoF
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_ref(:,i_cnt),qa(i_cnt));
end

% Based on equations for Jbsli @ p.168 Murray
for i_cnt=1:nDoF
    if i_cnt==1
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*Pi_i1(:,:,i_cnt)*g_sli_ref(:,:,1)))*xi_ai_ref(:,1);
        
%         g_s_li(:,:,i_cnt) = exp_ai(:,:,1)*Pi_i1(:,:,i_cnt)*g_sli_ref(:,:,1);
    elseif i_cnt==2
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*Pi_i1(:,:,i_cnt-1)*exp_ai(:,:,2)*Pi_i1(:,:,i_cnt)*g_sli_ref(:,:,2)))*xi_ai_ref(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*Pi_i1(:,:,i_cnt)*g_sli_ref(:,:,2)))*xi_ai_ref(:,2);
        
%         g_s_li(:,:,i_cnt) = exp_ai(:,:,1)*Pi_i1(:,:,i_cnt-1)*exp_ai(:,:,2)*Pi_i1(:,:,i_cnt)*g_sli_ref(:,:,2);
    elseif i_cnt==3
        J_b_sli(:,1,i_cnt) = inv(ad(exp_ai(:,:,1)*Pi_i1(:,:,i_cnt-2)*exp_ai(:,:,2)*Pi_i1(:,:,i_cnt-1)*exp_ai(:,:,3)*g_sli_ref(:,:,3)))*xi_ai_ref(:,1);
        J_b_sli(:,2,i_cnt) = inv(ad(exp_ai(:,:,2)*Pi_i1(:,:,i_cnt-1)*exp_ai(:,:,3)*g_sli_ref(:,:,3)))*xi_ai_ref(:,2); 
        J_b_sli(:,3,i_cnt) = inv(ad(exp_ai(:,:,3)*g_sli_ref(:,:,3)))*xi_ai_ref(:,3); 
        
%         g_s_li(:,:,i_cnt) = exp_ai(:,:,1)*Pi_i1(:,:,i_cnt-2)*exp_ai(:,:,2)*Pi_i1(:,:,i_cnt-1)*exp_ai(:,:,3)*g_sli_ref(:,:,3);
    else
         warning('[SMM CALCULATE BODY JACOBIANS]: IMPLEMENTED ONLY FOR 3 DoF STRUCTURE')
    end
end

end