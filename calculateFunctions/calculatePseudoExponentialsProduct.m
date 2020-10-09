function [P_i_i1] = calculatePseudoExponentialsProduct(A_i,A_i1,xi_ai_ref_i1, qa_i1)
% This function is valid only when the Forward Kinematics for the desired
% structure is obtained

P_i_i1 = A_i * inv(A_i1) * twistexp(xi_ai_ref_i1,-qa_i1);
end