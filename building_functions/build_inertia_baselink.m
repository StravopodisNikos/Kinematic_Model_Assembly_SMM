function [g_s_com_k_i, M_s_com_k_i] = build_inertia_baselink(g_s2bodyframe)
% for the baselink of the serial chain it computes the com
% vector and the body inertia matrix with respect to the spatial frame

% Default matrices given here are extracted from
% kinematic_model_assembly_definitions_smm.m

% the com_k_i with respecto to pseudomodule body frame
g_com_k_i =  [ 1.0000         0         0    0.0021;...
                    0    1.0000         0    0.0037;...
                    0         0    1.0000    0.0158;...
                    0         0         0    1.0000 ];
% the pseudomodule inertia matrix with respect to body frame
M_com_k_i =    [ 0.5000         0         0         0         0         0;...
                      0    0.5000         0         0         0         0;...
                      0         0    0.5000         0         0         0;...
                      0         0         0    5.063e-004    2.773e-005   7.042e-007;...
                      0         0         0    2.773e-005    5.387e-004    1.237e-006;...
                      0         0         0    7.042e-007    1.237e-006    7.207e-004 ];                

% find pseudomodule com with respect to S frame
g_s_com_k_i = g_s2bodyframe * g_com_k_i;

% find pseudomodule inertia matrix with respect to S frame
M_s_com_k_i = ad(inv(g_s_com_k_i))' * M_com_k_i * ad(inv(g_s_com_k_i));

end