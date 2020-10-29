function [last_expo,Jsp_pseudo_column,g_s_m_i1] = build_pseudomodule_with_anat(g_s_m_i,xi_pi_j,qp_j,last_expo)
% Adds a pseudojoint module in the serial metamorphic chain for the
% specified anatomy assembled
% Input: 1. the previously last frame attached to serial chain
%        2. spatial passive twist and pseudo angle of the module assembled 

% g_k_pj_0 = loaded from mat file. is the default tf between static and moving frame
g_k_pj_0 =    [ 1.0000         0         0         0; ...
                     0   -0.0000   -1.0000    0.0250; ...
                     0    1.0000   -0.0000    0.1180; ...
                     0         0         0    1.0000];

Jsp_pseudo_column = ad(last_expo)*xi_pi_j;
  
g_s_pj_0 = g_s_m_i *g_k_pj_0;

g_s_pj = twistexp(Jsp_pseudo_column,qp_j)*g_s_pj_0;

% New output
g_s_m_i1 = g_s_pj;
last_expo = last_expo*twistexp(xi_pi_j,qp_j);
end