function [g_s_com_k_i, M_s_com_k_i] = build_inertia_active_static(g_s2bodyframe)
% for the active module(static part) of the serial chain it computes the com
% vector and the body inertia matrix with respect to the spatial frame

% Default matrices given here are extracted from
% kinematic_model_assembly_definitions_smm.m

% the com_k_i with respecto to synthetic frame
g_com_k_i =  [ 1.0000         0         0    0.0000;...
                    0    1.0000         0    0.0000;...
                    0         0    1.0000    -0.015;...
                    0         0         0    1.0000 ];
% the pseudomodule inertia matrix with respect to body frame
M_com_k_i =    [ 0.8920         0         0         0               0           0;...
                      0    0.8920         0         0               0           0;...
                      0         0    0.8920         0               0           0;...
                      0         0         0         0.001        -1.292e-11     3.624e-12;...
                      0         0         0         -1.292e-11    4.195e-04     -9.613e-06;...
                      0         0         0         3.624e-12    -9.613e-06     0.001 ];                

% find pseudomodule com with respect to S frame
g_s_com_k_i = g_s2bodyframe * g_com_k_i;

% find pseudomodule inertia matrix with respect to S frame
M_s_com_k_i = ad(inv(g_s_com_k_i))' * M_com_k_i * ad(inv(g_s_com_k_i));

end