function [dot_x,torque] = PID_computed_torque_control_3DoF_MMD(x_d,ddq_d,x,dt,meta_params_struct)
% Follows the procedure pesented in p.197-199 in Robot Manipulator
% Control - Theory and Practice 2nd ed.

% M,C,G are calculated at each C-space
% q_d = 3x1 vector for desired final joint position
% dq_d = 3x1 vector for desired final joint velocity
% ddq_d = 3x1 vector for desired final joint acceleration
% x = [  q         x_d = [ q_d
%        dq  ]             dq_d  ]

epsilon  =  (x_d(1:3)-x(1:3))*dt; % 3x1
dot_x_d  = vertcat(x_d(4:6), ddq_d);                % 6x1

e(1:3,1) = epsilon ;                % epsilon
e(4:6,1) = x_d(1:3) - x(1:3) ;      % e
e(7:9,1) = x_d(4:6) - x(4:6) ;      % de

% For state vector x = [eps e de] 9x1 vector
% d(x)/dt = A * x + B * w
I = eye(3);
O = zeros(3);
kp = 1000;
kd = 200;
i_factor = 2; % must be >1
ki = (kp*kd)/i_factor;
Kp = I .* kp;
Kd = I .* kd;
Ki = I .* ki;
A = [O I O; O O I;-Ki -Kp -Kd]; 
B = [O;  O; I];
w = [10 10 10]';

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

torque = M*(ddq_d-u) + N;       % 3x1 robot's motor torque input vector

%% Linear System of differential equations to solve
dot_e = A*e + B*w;
dot_x = dot_x_d - dot_e(4:9);
end