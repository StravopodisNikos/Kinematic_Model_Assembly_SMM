function M = MassMatrixMatlab(e_state,s_desired,meta_params_struct,dt,robot)
q_d   = s_desired.q_d;
dq_d  = s_desired.dq_d;

x_d  = [q_d; dq_d];

z = x_d - e_state;

% matlab
% M1 = massMatrix(robot,z(1:3));

% custom
xi_ai_anat   = meta_params_struct.par1;
exp_ai       = meta_params_struct.par2;  % this must be calculated for actual q=x(1:3,1)
for i_cnt=1:3
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),z(i_cnt));
    exp_ai_d(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),q_d(i_cnt));
end
Pi_for_old   = meta_params_struct.par3;
g_sli_anat   = meta_params_struct.par4;
M_b_link_as2 = meta_params_struct.par5;
qdot         = meta_params_struct.par6; % this must be calculated for actual q=dq = x(4:6,1)
qdot = z(4:6,1);
% Calculate Mass+Coriolis Matrix
[M1,~,~] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai, Pi_for_old, g_sli_anat, M_b_link_as2, qdot);     % current
% % [Md,~,Cd] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai_d, Pi_for_old, g_sli_anat, M_b_link_as2, dq_d);   % desired

% Calculate Gravity matrix
delta_q   = qdot * dt;          % approximation for Delta C-Space of each joint
delta_q_d = dq_d * dt;

q_old   = z(1:3) - delta_q;       % approximation of previous state angular posistion
q_old_d = q_d    - delta_q_d;

% % Vold   = calculatePotentialEnergyMatrix_anat_3dof(q_old,xi_ai_anat,g_sli_anat,M_b_link_as2);
% % Vold_d = calculatePotentialEnergyMatrix_anat_3dof(q_old_d,xi_ai_anat,g_sli_anat,M_b_link_as2);
% % 
% % [G,~]  = calculate_gravity_matrix_numer_anat('3dof', z(1:3)', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold, delta_q);
% % [Gd,~] = calculate_gravity_matrix_numer_anat('3dof', q_d', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold_d, delta_q_d);

O = zeros(3);
M = [M1 O; O M1];
end