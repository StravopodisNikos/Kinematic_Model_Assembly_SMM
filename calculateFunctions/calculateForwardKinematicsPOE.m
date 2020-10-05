function [g_ai,g_pj] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref)

nDoF = size(xi_ai_ref,2);           % determine total number of active joints by number of columns
nPseudo = size(xi_pj_ref,2);        % determine total number of passive joints by number of columns
nAssemblyParts = size(structure,1);

% Compute exponentials
for i_cnt=1:nDoF
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_ref(:,i_cnt),qa(i_cnt));
end
for j_cnt=1:nPseudo
    exp_pj(:,:,j_cnt) = twistexp(xi_pj_ref(:,j_cnt),qp(j_cnt));
end

% Obtain POE equation by structure string
i_cnt = 0;
j_cnt = 0;
interm_cnt = 1;
exp_pj_interm(:,:,1) = eye(4);
g_pj(:,:,1) = eye(4);
g_ai(:,:,1) = eye(4);

for assembly_part_cnt=1:nAssemblyParts
    switch structure(assembly_part_cnt,:)
        case 'x0' % add active exponential
            i_cnt = i_cnt +1;
            
            g_ai(:,:,i_cnt) = g_ai(:,:,i_cnt) * exp_pj_interm(:,:,interm_cnt) * exp_ai(:,:,i_cnt) * g_ai_ref(:,:,i_cnt);
            
            interm_cnt = 0;             
        case 'x9' % nothing here
            exp_pj_interm(:,:,interm_cnt) = exp_pj_interm(:,:,interm_cnt) * eye(4);
            
        case ('21' | '31') % add passive exponential
            j_cnt = j_cnt + 1;
            interm_cnt = interm_cnt +1;
            
            exp_pj_interm(:,:,interm_cnt) = exp_pj_interm(:,:,interm_cnt) * exp_pj(:,:,j_cnt);
            
            g_pj(:,:,j_cnt) = g_pj(:,:,j_cnt) * exp_pj(:,:,i_cnt) * g_pj_ref(:,:,j_cnt);
    end
end

end