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

    case 'synthetic2'   % pseudo_moving->pseudo_static_under
        syn2_rpy = [0 synthetic2_origin_p 0];
        syn2_xyz = [synthetic2_origin_x 0.1444 0]';

        synthetic_Rot_tf = rotz(syn2_rpy(3))*roty(syn2_rpy(2))*rotx(syn2_rpy(1));
        [wmega_Rsyn, theta_Rsyn] = rotparam(synthetic_Rot_tf);
        exp_wmega_Rsyn = skewexp(wmega_Rsyn, theta_Rsyn);
        
        synthetic_tform = [exp_wmega_Rsyn syn2_xyz; 0 0 0 1];   % the local tf induced by synthetic joint
        
        % New global tf
        g_s_m_i1 = g_s_m_i * synthetic_tform;
        error_msg = false;
        
    case 'synthetic4'   % pseudo_moving->pseudo_static_under
        syn4_rpy = [-1.5708 synthetic4_origin_p 0];
        syn4_xyz = [synthetic4_origin_x 0.101 0]';

%         synthetic_tform = eul2tform(syn4_rpy);
%         synthetic_tform(1:3,4) = syn4_xyz;
        synthetic_Rot_tf = rotz(syn4_rpy(3))*roty(syn4_rpy(2))*rotx(syn4_rpy(1));
        [wmega_Rsyn, theta_Rsyn] = rotparam(synthetic_Rot_tf);
        exp_wmega_Rsyn = skewexp(wmega_Rsyn, theta_Rsyn);
        
        synthetic_tform = [exp_wmega_Rsyn syn4_xyz; 0 0 0 1];   % the local tf induced by synthetic joint
        
        % New global tf
        g_s_m_i1 = g_s_m_i * synthetic_tform;
        error_msg = false;
    case 'active_assembly'
        active2pseudo_origin_rpy= [0 1.5708 0];
        active2pseudo_origin_xyz= [0 0.1 0]'; 
                
        active2pseudo_Rot_tf = rotz(active2pseudo_origin_rpy(3))*roty(active2pseudo_origin_rpy(2))*rotx(active2pseudo_origin_rpy(1));
        [wmega_Ractive2pseudo, theta_Ractive2pseudo] = rotparam(active2pseudo_Rot_tf);
        exp_wmega_Ractive2pseudo = skewexp(wmega_Ractive2pseudo, theta_Ractive2pseudo);
        
        synthetic_tform = [exp_wmega_Ractive2pseudo active2pseudo_origin_xyz; 0 0 0 1];   % the local tf induced by active2pseudo assembly
        
        % New global tf
        g_s_m_i1 = g_s_m_i * synthetic_tform;
        error_msg = false;

    otherwise
        error_msg = true;
end

end