function [N,V] = calculate_gravity_matrix_numer_anat(dof_string, q, xi_ai_anat,g_sli_anat,M_b_link, Vold, delta_q)
% WORKS ONLY FOR DESIRED STRUCTURE+ANATOMY! TWISTS+CoM TF's are given for 
% Numerically calculates the gradient of Gravity matrix @ each C-space point. 

switch dof_string
    case '3dof'
        n_Dof = 3;
    case '6dof'
        n_Dof = 6;
end

V = calculatePotentialEnergyMatrix_anat_3dof(q,xi_ai_anat,g_sli_anat,M_b_link);

DeltaV = zeros(n_Dof,1);
N = zeros(n_Dof,1);

for i = 1:n_Dof
    DeltaV(i) = (V(i) - Vold(i) );
    if (delta_q(i) == 0)
        delta_q(i) = 0.001;
    end
    N(i) = DeltaV(i) / delta_q(i);

end

end