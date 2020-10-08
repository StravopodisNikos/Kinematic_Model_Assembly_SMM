function [g_s_link_as,M_s_link_as] = calculateCoMmetalinks(M_s_com_k_i,g_s_com_k_i)
% Given the 4D matrices extracted for each link it calculates w.r.t SPATIAL frame each:
% 1. metalink CoM
% 2. metalink Inertia Matrix

%% Allocate memory for inertia properties of links constructed
% We know that nDoF=3!
nDoF=size(M_s_com_k_i,3);
for link_count=1:nDoF
    M_s_link_as(:,:,link_count) = zeros(6);   % Final Link Inertia Matrices of robot w.r.t. {S} frame
    g_s_link_as(:,:,link_count) = zeros(3,1);   % Final CoM spatial tf of each link
    weighted_g_s_link_as(:,:,link_count) = zeros(3,1); % only com vector
end

% Metalink Inertia Matrix w.r.t. spatial frame!
for link_count=1:size(M_s_com_k_i,3)
    for bodies_cnt =1:size(M_s_com_k_i,4)
        M_s_link_as(:,:,link_count) = M_s_link_as(:,:,link_count) + M_s_com_k_i(:,:,link_count,bodies_cnt);   % Final Link Inertia Matrices of robot w.r.t. {S} frame
    end
end

% Com
for link_count=1:size(M_s_com_k_i,3)
    mass_of_metalink(link_count) = M_s_link_as(1,1,link_count);

    for bodies_cnt =1:size(M_s_com_k_i,4)
        mass_of_body_of_metalink(bodies_cnt) = M_s_com_k_i(1,1,link_count,bodies_cnt);
        
        weighted_g_s_link_as(:,1,link_count) = weighted_g_s_link_as(:,1,link_count) +  mass_of_body_of_metalink(bodies_cnt) .* g_s_com_k_i(1:3,4,link_count,bodies_cnt);
    end
    
    g_s_link_as(:,1,link_count) = weighted_g_s_link_as(:,1,link_count) / mass_of_metalink(link_count) ;
end

end