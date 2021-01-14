function V = calculatePotentialEnergyMatrix_anat_3dof(q,xi_ai_anat,g_sli_anat,M_b_link)
% twists,CoM tf's must be given for desired STRUCTURE & ANATOMY!

g = -9.8067; % [m/s^2]

for i_cnt=1:3
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_anat(:,i_cnt),q(i_cnt));
end

mi(1) = M_b_link(1,1,1);    % mass of metalink1
mi(2) = M_b_link(1,1,2);    % mass of metalink2
mi(3) = M_b_link(1,1,3);    % mass of metalink3

gsli(:,:,1) = exp_ai(:,:,1)*g_sli_anat(:,:,1);                              % fwkin of CoMi for metalink1
gsli(:,:,2) = exp_ai(:,:,1)*exp_ai(:,:,2)*g_sli_anat(:,:,2);                % fwkin of CoMi for metalink2
gsli(:,:,3) = exp_ai(:,:,1)*exp_ai(:,:,2)*exp_ai(:,:,3)*g_sli_anat(:,:,3);  % fwkin of CoMi for metalink3

for i = 1:3
    V(i) = mi(i)*g*abs(gsli(3,4,i));
end

end