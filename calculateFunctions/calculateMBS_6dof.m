function [MBS] = calculateMBS_6dof(structure,assembly_parameters,xi_ai_ref,xi_pj_ref,qp,point2figure)
% Calculates Mass Balancing Score at given anatomy
% The robot structure must be previously extracted

% Include libraries
addpath('/home/nikos/matlab_ws/geom3d_library/geom3d')

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
[~,xi_ai_anat,M_s_com_k_i_anat,g_s_com_k_i_anat] = calculateCoM_ki_s_structure_anatomy_6dof(structure,assembly_parameters,qp,xi_pj_ref,point2figure);
[g_s_link_as_anat,M_s_link_as_anat] = calculateCoMmetalinks(M_s_com_k_i_anat,g_s_com_k_i_anat);

figure(point2figure); scatter3(g_s_link_as_anat(1,1,1), g_s_link_as_anat(2,1,1), g_s_link_as_anat(3,1,1),500,'p','filled'); hold on;
figure(point2figure); scatter3(g_s_link_as_anat(1,1,2), g_s_link_as_anat(2,1,2), g_s_link_as_anat(3,1,2),500,'p','filled'); hold on;
figure(point2figure); scatter3(g_s_link_as_anat(1,1,3), g_s_link_as_anat(2,1,3), g_s_link_as_anat(3,1,3),500,'p','filled'); hold on;

for current_link=nDoF:-1:1
    
    % Calculate CoM from last link to current
    mass_last2current = 0;
    sum_com_last2current = zeros(3,1);
    for com_iter=nDoF:-1:current_link
        mass_last2current = mass_last2current + M_s_link_as_anat(1,1,com_iter);
        sum_com_last2current  = sum_com_last2current  + M_s_link_as_anat(1,1,com_iter) .* g_s_link_as_anat(1:3,1,com_iter) ;
    end
    com_last2current = sum_com_last2current / mass_last2current;
    figure(point2figure); scatter3(com_last2current(1), com_last2current(2), com_last2current(3),500,'p','filled'); hold on;
    
    % Calculate euclidean distance from current active twist to calculated CoM
    [p, dir] = twistaxis(xi_ai_anat(:,current_link));
    
    twist_line = createLine3d(p(1), p(2), p(3), dir(1), dir(2), dir(3));
    figure(point2figure); drawLine3d(twist_line); hold on;
    
    d = distancePointLine3d(com_last2current', twist_line);

    % Total Sum
    MBS = MBS + d * mass_last2current;
end

end