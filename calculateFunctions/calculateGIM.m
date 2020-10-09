function [M_b] = calculateGIM(J_b_sli,M_b_link_as)
%calculates GIM defined as of eq.4.19 p.168 Murray

nDoF=size(M_b_link_as,3);

M_b = zeros(nDoF);

for i_cnt=1:nDoF
    M_b = M_b + J_b_sli(:,:,i_cnt)' * M_b_link_as(:,:,i_cnt) * J_b_sli(:,:,i_cnt);
end

end