function [dot_x] = LQR_computed_torque_control_3DoF_MMD(x_d,ddq_d,x,dt,s_lqr)
% Follows the procedure pesented in p.246-247 in Robot Manipulator
% Control - Theory and Practice 2nd ed.

% q_d = 3x1 vector for desired final joint position
% dq_d = 3x1 vector for desired final joint velocity
% ddq_d = 3x1 vector for desired final joint acceleration
% x = [  q         x_d = [ q_d
%        dq  ]             dq_d  ]

% Linear State Space System:
% d/dt  [  ε     = [  0     I33     0     *  [  ε         + [ 0     * u   
%          e          0      0     I33          e             0
%          de ]       0      0      0  ]        de  ]         I33 ]

% d(e)/dt        =         A              *   e           + B       *  u

epsilon  =  (x_d(1:3)-x(1:3))*dt; % 3x1
dot_x_d  = vertcat(x_d(4:6), ddq_d);                % 6x1

% Error State vector
e(1:3,1) = epsilon ;                % epsilon
e(4:6,1) = x_d(1:3) - x(1:3) ;      % e
e(7:9,1) = x_d(4:6) - x(4:6) ;      % de

% For state vector x = [eps e de] 9x1 vector
% d(x)/dt = A * x + B * w
I = eye(3);
O = zeros(3);

A = [O I O; O O I;O  O  O]; 
B = [O;  O; I];

% LQR Optimal Gain Matrix
Q = s_lqr.Qpen;
R = s_lqr.Rpen;
N = s_lqr.Nnonlin;

% Check prerequisites for Theorem4.6-1 p.245 [Robot Manipulator Control: Theory and Practice]
% Check controllability
% Co = ctrb(A,B);
% unco = length(A) - rank(Co);    % uncontrollable states
% Check observability
% sqrtQ = sqrtm(Q);
% obsv_matrix = obsv(A,sqrtQ)
% unobsv = length(A) - rank(obsv_matrix);    % unobservable states

K = lqr(A,B,Q,R,N);

%% Linear System of differential equations to solve
dot_e = (A - B*K) * e;
dot_x = dot_x_d - dot_e(4:9);
end