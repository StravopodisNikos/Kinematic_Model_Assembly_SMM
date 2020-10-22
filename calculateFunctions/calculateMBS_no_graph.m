function MBS = calculateMBS_no_graph(structure,xi_ai_ref,xi_pj_ref,g_s_link_as,g_ai_ref,g_pj_ref,gst0,M_s_link_as,qp)
% Calculates Mass Balancing Score at given anatomy
% The robot structure must be previously extracted

% Include libraries
addpath('/home/nikos/matlab_ws/geom3d_library/geom3d')

nDoF = size(xi_ai_ref,2);           % determine total number of active joints by number of columns

MBS  = 0;

% Calculate CoM of links for specified anatomy @ reference configuration
qa = zeros(nDoF,1);
[~,~,Jsp,Pi,~] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);

%[g_s_li] = calculate_CoM_fwd_kin(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_s_link_as);
[~,g_s_li] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi, g_s_link_as );

for current_link=nDoF:-1:1
    
    % Calculate CoM from last link to current
    mass_last2current = 0;
    sum_com_last2current = zeros(3,1);
    for com_iter=nDoF:-1:current_link
        mass_last2current = mass_last2current + M_s_link_as(1,1,com_iter);
        sum_com_last2current  = sum_com_last2current  + M_s_link_as(1,1,com_iter) .* g_s_li(1:3,4,com_iter) ;
    end
    com_last2current = sum_com_last2current / mass_last2current;
    
    % Calculate euclidean distance from current active twist to calculated CoM
    [p, dir] = twistaxis(Jsp(:,current_link));
    
    twist_line = createLine3d(p(1), p(2), p(3), dir(1), dir(2), dir(3));
    
    d = distancePointLine3d(com_last2current', twist_line);

    % Total Sum
    MBS = MBS + d * mass_last2current;
end

end