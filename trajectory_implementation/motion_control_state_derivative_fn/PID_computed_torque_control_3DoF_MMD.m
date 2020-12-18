function [dot_x] = PID_computed_torque_control_3DoF_MMD(x_d,ddq_d,x,dt,s_pid)
% Follows the procedure pesented in p.197-199 in Robot Manipulator
% Control - Theory and Practice 2nd ed.

% last sync with pc-lef 18-12-20 18:46

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
kp = s_pid.kp;
kd = s_pid.kd;
i_factor = s_pid.ki_factor;
ki = i_factor * (kp*kd);
Kp = I .* kp;
Kd = I .* kd;
Ki = I .* ki;
A = [O I O; O O I;-Ki -Kp -Kd]; 
B = [O;  O; I];
w = [10 10 10]';

%% Linear System of differential equations to solve
dot_e = A*e + B*w;
dot_x = dot_x_d - dot_e(4:9);
end