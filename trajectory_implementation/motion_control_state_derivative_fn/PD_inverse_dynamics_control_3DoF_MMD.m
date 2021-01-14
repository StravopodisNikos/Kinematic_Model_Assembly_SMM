function [de_state] = PD_inverse_dynamics_control_3DoF_MMD(t,e_state,s_desired,s_pd,meta_params_struct,dt,robot)
% Executes Inverse Dynamics Control in State Space Model. Main idea is
% presented in par.8.5.2 Sicilliano-Robotics Modelling Planning and Control
% State space analysis is in detail presented in my notes(based on fig.8.22)

% INPUT: 1. s_desired: struct with desired variables values {q_d, dq_d, ddq_d}
%        2. s_pd : struct with PD gain matrices

% OUTPUT: 1. state derivative
%          2. system ss matrices
% ========================================================================
% FUNCTIONS FOR RESULT EVALUATION:
% 1. evaluate_trajectory_implementation_custom -> optimized_structure_anatomy_trajectory_implementation_custom3
% ========================================================================
%% Form input:
tic;

q_d   = s_desired.q_d;
dq_d  = s_desired.dq_d;
ddq_d = s_desired.ddq_d;
x_d  = [q_d; dq_d];
% dx_d = [dq_d; ddq_d];

O = zeros(3);
I = eye(3);
kp = s_pd.kp;
kd = s_pd.kd;
Kp = I .* kp;
Kd = I .* kd;

z = x_d - e_state;
%% Set error state for PD controller(out)
% e_state = x_d - z;

e  = e_state(1:3);
de = e_state(4:6);

% System to control:
%     ---------
% u-> | ROBOT |-> y_out = z(state)
%     ---------
% System state: z = [   q
%                      dq   ]
% Now, input vector u is not that of a simple PD controller! The inverse
% dynamics must be counterbalanced 

% IN Controller(INV-DYN):
%     --------------
% y-> | CONTROLLER |-> u   | * y must not be mixed with y_out!!! It is the
%     --------------           output of the PD controller!
% OUT Controller(PD):
%     --------------
% z-> | CONTROLLER |-> y   | * z is the output of the total system, and
%                              first is passed at the PD controller
%     --------------           

% Calculate error state tracking vector of the PD controller
% r = ddq_d + Kp * q_d + Kd * dq_d;
% y = -Kp * z(1:3) -Kd * z(4:6) + r;

% y = ddq_d + Kd * de + Kp * e; % same as above

% In order to compute the input vector u, the robot dynamics must be determined. Vector u
% is the output vector of the stabilizing linear control and the nonlinear
% compensation and decoupling part of the designed controller.. Here is
% where the dynamic model of the robot is introduced! The assumption is
% that our dynamic model is "exact".
% Important notes(28-12-20)
% * We want to show, that for mass balanced robot, nonlinear terms are
% excluded and this assumption is valid, leading to a "good" controller.
% ** I must describe in detail what "good" means.

%% Compute robot dynamics (Mc,ng) for current and desired states -> RESIDUAL DYNAMICS
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
[M,~,C] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai, Pi_for_old, g_sli_anat, M_b_link_as2, qdot);     % current
[Md,~,Cd] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai_d, Pi_for_old, g_sli_anat, M_b_link_as2, dq_d);   % desired

% Calculate Gravity matrix
delta_q   = qdot * dt;          % approximation for Delta C-Space of each joint
delta_q_d = dq_d * dt;

q_old   = z(1:3) - delta_q;       % approximation of previous state angular posistion
q_old_d = q_d    - delta_q_d;

Vold   = calculatePotentialEnergyMatrix_anat_3dof(q_old,xi_ai_anat,g_sli_anat,M_b_link_as2);
Vold_d = calculatePotentialEnergyMatrix_anat_3dof(q_old_d,xi_ai_anat,g_sli_anat,M_b_link_as2);

[G,~]  = calculate_gravity_matrix_numer_anat('3dof', z(1:3)', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold, delta_q);
[Gd,~] = calculate_gravity_matrix_numer_anat('3dof', q_d', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold_d, delta_q_d);

% matlab 
% % M = massMatrix(robot,z(1:3));
% % Cz= velocityProduct(robot,z(1:3),z(4:6)); C = Cz * pinv(z(4:6));
% % G = gravityTorque(robot,z(1:3));
% % Md = massMatrix(robot,q_d);
% % Czd= velocityProduct(robot,q_d,dq_d); Cd = Czd * pinv(dq_d);
% % Gd = gravityTorque(robot,q_d);


h = (Md - M) * ddq_d + (Cd - C) * dq_d + (Gd - G);      % p.271(282) Kelly-Davilla-Loria Advanced Textbook in Control


%% State space
% % u = Kp *e + Kd * de + Md * ddq_d + Cd * dq_d + Gd;
% % display('A21 exec time:'); tic; A21 = -inv(M)*Kp; toc;
% % display('A22 exec time:'); tic; A22 = -inv(M)*(Kd+C); toc;
% % 
% % A = [O I; A21 A22];
% % display('B* exec time:'); tic; B_star = -inv(M)*h; toc;
% % display('B2 exec time:'); tic; B2 = B_star * pinv(u); toc;
% % B = [O; B2];
% % Co = ctrb(A,B);
% % unco = length(A) - rank(Co)
% % de_state = A * e_state + B * u;
%% Form ode
% dde = inv(M) * (-Kp*e -Kd*de -C*de -h);
dde = (-Kp*e -Kd*de -C*de -h);
de  = M * de;
de_state = [de ; dde];
t
% dot_z = A * z + B * u;
%disp('PD_inverse_dynamics_control_3DoF_MMD exec time:'); toc; % avg~0.1sec
end