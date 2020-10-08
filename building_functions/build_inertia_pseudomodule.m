function [g_s_com_k_i, M_s_com_k_i] = build_inertia_pseudomodule(g_s2bodyframe)
% for each pseudomodule added in the serial chain it computes the com
% vector and the body inertia matrix with respect to the spatial frame

% Default matrices given here are extracted from
% kinematic_model_assembly_definitions_smm.m

% the com_k_i with respecto to pseudomodule body frame(on pseudo axis when module is assembled!!!)
g_com_k_i =  [ 1.0000         0         0         0;...
                    0    1.0000         0         -0.03;...
                    0         0    1.0000         0.02;...
                    0         0         0    1.0000 ];
% the pseudomodule inertia matrix with respect to body frame
M_com_k_i =    [ 2.6487         0         0         0         0         0;...
                      0    2.6487         0         0         0         0;...
                      0         0    2.6487         0         0         0;...
                      0         0         0    0.0186    0.0000   -0.0000;...
                      0         0         0    0.0000    0.0186    0.0000;...
                      0         0         0   -0.0000    0.0000    0.0000 ];                

% find pseudomodule com with respect to S frame
g_s_com_k_i = g_s2bodyframe * g_com_k_i;

% find pseudomodule inertia matrix with respect to S frame
M_s_com_k_i = ad(inv(g_s_com_k_i))' * M_com_k_i * ad(inv(g_s_com_k_i));

end