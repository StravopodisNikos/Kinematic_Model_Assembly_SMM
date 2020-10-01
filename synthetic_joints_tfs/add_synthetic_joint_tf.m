function [synthetic_tform,g_s_m_i1,error_msg] = add_synthetic_joint_tf(synthetic_name,g_s_m_i)
% Input: Type of synthetic joint as string 
% Synthetic tforms are defined in tf_list_for_conditioned_assembly.yaml

% Next parameters are defined in assembly_parameters_for_ovidius_robot.yaml
% ==================================================
% -------> OPTIMIZATION ASSEMBLY PARAMETERS <-------
synthetic2_origin_x = -0.075 %var \in [-0.075 ~ 0.025]
synthetic2_origin_p = 1.5708 %var \in [-1.5708 ~ 1.5708]
synthetic3_origin_x = -0.055  %var \in [-0.055~-0.025]
synthetic3_origin_z = -0.05  %var \in [-0.05~0.05]
synthetic3_origin_p = 1.5708  %var \in [-1.5708 ~ 1.5708]
synthetic4_origin_x = 0 %var \in [-0.025 ~ 0.025]
synthetic4_origin_p = 1.5708 %var \in [-1.5708 ~ 1.5708]
synthetic5_origin_z = -0.015 %var \in [-0.015~0.025]
synthetic5_origin_p = 0 %var \in [-1.5708 ~ 1.5708]

% ==================================================

switch synthetic_name
    case 'synthetic1'   % base_link->pseudo_static_under
        syn1_rpy = [0 0 0];
        syn1_xyz = [0 0 0.048]';

        synthetic_tform = eul2tform(syn1_rpy);
        synthetic_tform(1:3,4) = syn1_xyz;

        % New global tf
        g_s_m_i1 = g_s_m_i * synthetic_tform;
        error_msg = false;
        
    case 'synthetic4'   % pseudo_moving->pseudo_static_under
        syn1_rpy = [0 synthetic4_origin_p 0];
        syn1_xyz = [synthetic4_origin_x 0.101 0]';

        synthetic_tform = eul2tform(syn1_rpy);
        synthetic_tform(1:3,4) = syn1_xyz;

        % New global tf
        g_s_m_i1 = g_s_m_i * synthetic_tform;
        error_msg = false;
        
    otherwise
        error_msg = true;
end

end