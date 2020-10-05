function [xi_0, g_0] = extract_ref_structure_anatomy_info(joint_type, xi, g_s_m_i1)
% Saves spatial active/passive twists and frames

switch joint_type
    case 'active'
        xi_0 = xi;
        g_0 = g_s_m_i1;
    case 'passive'
        xi_0 = xi;
        g_0 = g_s_m_i1;
    otherwise
end

end