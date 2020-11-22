function MBS = calculateMBS_no_graph(structure,assembly_parameters,xi_ai_ref,xi_pj_ref,qp)
% Calculates Mass Balancing Score at given anatomy
% The robot structure must be previously extracted

% Include libraries
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
geom3d_path_relative_to_matlab_ws = fullfile('matlab_ws','geom3d_library','geom3d',filesep); geom3d_library_path = strcat(root_path,geom3d_path_relative_to_matlab_ws); addpath(geom3d_library_path);

nDoF = size(xi_ai_ref,2);           % determine total number of active joints by number of columns

MBS  = 0;

%% Calculate CoM of links for specified anatomy @ reference configuration
% qa = zeros(nDoF,1);
% [~,~,Jsp,Pi,~] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);

% The first 2 fn are wrong for calculating the CoM of links of each new
% anatomy! Are left here only for remembering the procedure!
% [g_s_li] = calculate_CoM_fwd_kin(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_s_link_as);
% [~,g_s_li] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi, g_s_link_as );

% These 2 functions do the job:
[~,xi_ai_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_no_graph(structure,assembly_parameters,qp,xi_pj_ref);
[g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);

for current_link=nDoF:-1:1
    
    % Calculate CoM from last link to current
    mass_last2current = 0;
    sum_com_last2current = zeros(3,1);
    for com_iter=nDoF:-1:current_link
        mass_last2current = mass_last2current + M_s_link_as_anat(1,1,com_iter);
        sum_com_last2current  = sum_com_last2current  + M_s_link_as_anat(1,1,com_iter) .* g_s_link_as_anat(1:3,1,com_iter) ;
    end
    com_last2current = sum_com_last2current / mass_last2current;
    
    % Calculate euclidean distance from current active twist to calculated CoM
    [p, dir] = twistaxis(xi_ai_anat(:,current_link));
    
    twist_line = createLine3d(p(1), p(2), p(3), dir(1), dir(2), dir(3));
    
    d = distancePointLine3d(com_last2current', twist_line);

    % Total Sum
    MBS = MBS + d * mass_last2current;
end

end