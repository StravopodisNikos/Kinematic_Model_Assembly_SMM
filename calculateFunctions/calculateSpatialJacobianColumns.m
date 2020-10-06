function [Jsp_i_column] = calculateSpatialJacobianColumns(xi_ai_ref,g_for_sp,i_cnt)
% Computes the Jacobians of the SMM

if i_cnt==1
    Jsp_i_column = xi_ai_ref(:,i_cnt);
else
    Jsp_i_column = ad(g_for_sp)*xi_ai_ref(:,i_cnt);
end

end