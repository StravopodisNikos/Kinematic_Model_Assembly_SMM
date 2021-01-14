function [dot_e] = LQR_computed_torque_control_3DoF_MMD(e,s_lqr)
% Follows the procedure pesented in p.246-247 in Robot Manipulator
% Control - Theory and Practice 2nd ed.

% Linear State Space System:
% d/dt  [  ε     = [  0     I33     0     *  [  ε         + [ 0     * u   
%          e          0      0     I33          e             0
%          de ]       0      0      0  ]        de  ]         I33 ]

% d(e)/dt        =         A              *   e           + B       *  u

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
end