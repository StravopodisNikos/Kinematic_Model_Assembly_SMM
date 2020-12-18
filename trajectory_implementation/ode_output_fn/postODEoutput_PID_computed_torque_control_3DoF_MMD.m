function [torque,V] = postODEoutput_PID_computed_torque_control_3DoF_MMD(x,x_d,ddq_d,dt,meta_params_struct,s_pid,Vold)
% This function is executed for each [x] state received from ode113 @
% specified t. No persistent variables here. Simple (re)calculation.

% last sync with pc-lef 18-12-20 18:43

% STATES:
% q_d = 3x1 vector for desired final joint position
% dq_d = 3x1 vector for desired final joint velocity
% ddq_d = 3x1 vector for desired final joint acceleration
% x = [  q         x_d = [ q_d
%        dq  ]             dq_d  ]

epsilon  =  (x_d(1:3)-x(1:3))*dt; % 3x1

e(1:3,1) = epsilon ;                % epsilon
e(4:6,1) = x_d(1:3) - x(1:3) ;      % e
e(7:9,1) = x_d(4:6) - x(4:6) ;      % de

% For state vector x = [eps e de] 9x1 vector
% d(x)/dt = A * x + B * w
I = eye(3);
O = zeros(3);
kp = s_pid.kp;
kd = s_pid.kd;
i_factor = s_pid.ki_factor;
ki = i_factor * (kp*kd);
Kp = I .* kp;
Kd = I .* kd;
Ki = I .* ki;

%% Torque output must be extracted using outputFcn!
% Decoupled controller unit
% u = -Kp*e -Ki*epsilon - Kd*de
u = -Kp*e(4:6,1) -Ki*e(1:3,1) - Kd*e(7:9,1); % 3x1 controller output vector

% Torque Law calculation-Here Non-Linear Robot Dynamics are introduced
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
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),x(i_cnt));
end
Pi_for_old   = meta_params_struct.par3;
g_sli_anat   = meta_params_struct.par4;
M_b_link_as2 = meta_params_struct.par5;
qdot         = meta_params_struct.par6; % this must be calculated for actual q=dq = x(4:6,1)
qdot = x(4:6,1);
% Calculate Mass+Coriolis Matrix
[M,~,C] = compute_Mij_429_3DoF(xi_ai_anat, exp_ai, Pi_for_old, g_sli_anat, M_b_link_as2, qdot);
% Calculate Gravity matrix
delta_q = x(4:6) * dt;          % approximation for Delta C-Space of each joint
[G,V] = calculate_gravity_matrix_numer_anat('3dof', x(1:3)', xi_ai_anat, g_sli_anat, M_b_link_as2, Vold, delta_q);

N = C*x(4:6) + G;
% Calculate torque
torque = M*(ddq_d-u) + N;       % 3x1 robot's motor torque input vector [Nm]

torque = torque';
end