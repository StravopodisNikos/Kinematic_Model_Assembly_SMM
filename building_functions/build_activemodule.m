function [xi_ai_0,g_s_m_i1] = build_activemodule(g_s_m_i)
% Adds an  active Dynamixel module in the serial metamorphic chain
% Input: 1. the previously last frame attached to serial chain

g_k_ai_0 =    [ 1.0000    0.0000         0    0.0000;...
                0.0000    1.0000   -0.0000    0.0320;...
                     0   -0.0000    1.0000   -0.0000;...
                     0         0         0    1.0000];

% Extract g_s_pj_0: the tf of pseudo moving with respect to spatial {S}
% frame
g_s_ai_0 = g_s_m_i * g_k_ai_0;

g_s_ai_0_loc = g_s_m_i * g_k_ai_0 * inv(g_s_m_i);

% Extract point in the pseudo twist axis
p_ai_0 = g_s_ai_0(1:3,4);

% Extract wmega of pseudo twist axis
%R_s_pj_0 = g_s_pj_0(1:3,1:3);
R_s_ai_0_loc = g_s_ai_0_loc(1:3,1:3);

% ERROR HERE!!! Doesn't recognise the local tf for active rotation axis!
[w_s_ai_0,th_s_ai_0] = rotparam(R_s_ai_0_loc); %rotparam(R_s_pj_0*inv(rotz(pi/2)*inv(rotx(pi/2)))) helped me understand i needed local tf

% Extract reference twist
xi_ai_0 = createtwist(w_s_ai_0,p_ai_0); % ξpj

% New last frame
g_s_m_i1 = g_s_ai_0;
end