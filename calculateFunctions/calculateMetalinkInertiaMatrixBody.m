function [M_b_metalink] = calculateMetalinkInertiaMatrixBody(g_s_link_as,M_s_metalink)

nDoF=size(M_s_metalink,3);

for i_cnt=1:nDoF
    g_s_li_0(:,:,i_cnt) = eye(4);
    g_s_li_0(1:3,4,i_cnt) = g_s_link_as(:,1,i_cnt);
end

for i_cnt=1:nDoF
    M_b_metalink(:,:,i_cnt) = ad(g_s_li_0(:,:,i_cnt))' * M_s_metalink(:,:,i_cnt) * ad(g_s_li_0(:,:,i_cnt));
end

end