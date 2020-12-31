function [torque,V,pos_error] = postODEoutput_PD_inverse_dynamics_control_3DoF_MMD(e_state,s_desired,s_pd,meta_params_struct,dt,Vold_d)


% Form input:
q_d   = s_desired.q_d;
dq_d  = s_desired.dq_d;
ddq_d = s_desired.ddq_d;
x_d  = [q_d; dq_d];

I = eye(3);
kp = s_pd.kp;
kd = s_pd.kd;
Kp = I .* kp;
Kd = I .* kd;

z = x_d - e_state;

e  = e_state(1:3);
de = e_state(4:6);

% In order to compute [B}, the input vector u must be determined. Vector u
% is the output vector of the stabilizing linear control and the nonlinear
% compensation and decoupling part of the designed controller.. Here is
% where the dynamic model of the robot is introduced! The assumption is
% that our dynamic model is "exact".
% Important notes(28-12-20)
% * We want to show, that for mass balanced robot, nonlinear terms are
% excluded and this assumption is valid, leading to a "good" controller.
% ** I must describe in detail what "good" means.

%% Compute robot dynamics (M,n)
% To calculate M,C fn: compute_Mij_429_3DoF inside 'screw_dynamics' is
% executed. Required input(the fields of the passed struct):
% 1. The active twists of the structure/anatomy @ ZERO q
% 2. the active exponentials @ REAL q computed at this step
% 3. the passive exponential are passed as identity matrices(no need to transform the twists, they are given for the specified anatomy)
% 4. the tranform matrix of the CoMi of eac link @ ZERO q
% 5. the Metalink Inertia Matrix|Body frame 
% 6. the REAL q_dot computed at this step
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
[Md,~,Cd] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai_d, Pi_for_old, g_sli_anat, M_b_link_as2, dq_d);   % desired
% Calculate Gravity matrix
delta_q_d = dq_d * dt;
% q_old = z(1:3) - delta_q;       % approximation of previous state angular posistion
% Vold = calculatePotentialEnergyMatrix_anat_3dof(q_old,xi_ai_anat,g_sli_anat,M_b_link_as2);
[Gd,V] = calculate_gravity_matrix_numer_anat('3dof', q_d', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold_d, delta_q_d);

% matlab
% % M = massMatrix(robot,z(1:3));
% % Cz= velocityProduct(robot,z(1:3),z(4:6)); C = Cz * pinv(z(4:6));
% % G = gravityTorque(robot,z(1:3));
% % Md = massMatrix(robot,q_d);
% % Czd= velocityProduct(robot,q_d,dq_d); Cd = Czd * pinv(dq_d);
% % Gd = gravityTorque(robot,q_d);

% calculate torque command
u = Kp *e + Kd * de + Md * ddq_d + Cd * dq_d + Gd;  


%% Calculate torque and position error
torque    = u';
pos_error = e(1:3)';
end