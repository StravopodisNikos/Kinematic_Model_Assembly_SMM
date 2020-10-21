function [MBS] = calculateMBS(structure,xi_ai_ref,xi_pj_ref,g_s_link_as,g_ai_ref,g_pj_ref,gst0,M_s_link_as,qp,point2figure)
% Calculates Mass Balancing Score at given anatomy
% The robot structure must be previously extracted

% Include libraries
addpath('/home/nikos/matlab_ws/geom3d_library/geom3d')

nDoF = size(xi_ai_ref,2);           % determine total number of active joints by number of columns
nPseudo = size(xi_pj_ref,2);

MBS  = 0;

% Calculate CoM of links for specified anatomy @ reference configuration
qa = zeros(nDoF,1);
%qp = zeros(nPseudo,1);
[~,~,Jsp,Pi,~] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);

%[g_s_li] = calculate_CoM_fwd_kin(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_s_link_as);
[~,g_s_li] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi, g_s_link_as );
figure(point2figure); scatter3(g_s_li(1,4,1), g_s_li(2,4,1), g_s_li(3,4,1),500,'p','filled'); hold on;
figure(point2figure); scatter3(g_s_li(1,4,2), g_s_li(2,4,2), g_s_li(3,4,2),500,'p','filled'); hold on;
figure(point2figure); scatter3(g_s_li(1,4,3), g_s_li(2,4,3), g_s_li(3,4,3),500,'p','filled'); hold on;

for current_link=nDoF:-1:1
    
    % Calculate CoM from last link to current
    mass_last2current = 0;
    sum_com_last2current = zeros(3,1);
    for com_iter=nDoF:-1:current_link
        mass_last2current = mass_last2current + M_s_link_as(1,1,com_iter);
        sum_com_last2current  = sum_com_last2current  + M_s_link_as(1,1,com_iter) .* g_s_li(1:3,4,com_iter) ;
    end
    com_last2current = sum_com_last2current / mass_last2current;
    figure(point2figure); scatter3(com_last2current(1), com_last2current(2), com_last2current(3),500,'p','filled'); hold on;
    
    % Calculate euclidean distance from current active twist to calculated CoM
    [p, dir] = twistaxis(Jsp(:,current_link));
    
    twist_line = createLine3d(p(1), p(2), p(3), dir(1), dir(2), dir(3));
    figure(point2figure); drawLine3d(twist_line); hold on;
    
    d = distancePointLine3d(com_last2current', twist_line);

    % Total Sum
    MBS = MBS + d * mass_last2current;
end

end