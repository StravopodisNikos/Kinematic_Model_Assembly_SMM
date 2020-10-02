function [xi_pj_0,g_s_m_i1] = build_pseudomodule(g_s_m_i)
% Adds a pseudojoint module in the serial metamorphic chain
% Input: 1. the type of the synthetic joint(assembly type)
%        2. the previously last frame attached to serial chain
%        3. number of the pseudo attached

% g_k_pj_0 = loaded from mat file. is the default tf between static and moving frame
g_k_pj_0 =    [ 1.0000         0         0         0; ...
                     0   -0.0000   -1.0000    0.0250; ...
                     0    1.0000   -0.0000    0.1180; ...
                     0         0         0    1.0000];

% Extract g_s_pj_0: the tf of pseudo moving with respect to spatial {S}
% frame
g_s_pj_0 = g_s_m_i * g_k_pj_0;

g_s_pj_0_loc = g_s_m_i * g_k_pj_0 * inv(g_s_m_i);

% Extract point in the pseudo twist axis
p_pj_0 = g_s_pj_0(1:3,4);

% Extract wmega of pseudo twist axis
%R_s_pj_0 = g_s_pj_0(1:3,1:3);
R_s_pj_0_loc = g_s_pj_0_loc(1:3,1:3);

[w_s_pj_0,th_s_pj_0] = rotparam(R_s_pj_0_loc); %rotparam(R_s_pj_0*inv(rotz(pi/2)*inv(rotx(pi/2)))) helped me understand i needed local tf

% Extract reference twist
xi_pj_0 = createtwist(w_s_pj_0,p_pj_0); % ξpj

% New last frame
g_s_m_i1 = g_s_pj_0;
end