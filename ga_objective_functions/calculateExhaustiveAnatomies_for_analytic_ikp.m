function [special_anatomies_of_structure] = calculateExhaustiveAnatomies_for_analytic_ikp(structure,degree,criterion,xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0)
% Calculates all possible anatomies given the total number of pseudos and
% the known pseudo step angle. returns total number of anatomies that
% satisfy the conditions for analytic IKP solution of 6R manipulators

% Thanks to Jos (10584) for allcomb.m code.
% Downloaded from MAthWorks File Exchange @ : https://www.mathworks.com/matlabcentral/fileexchange/10064-allcomb-varargin

pseudo_step_angle = deg2rad(45);
pseudo_limit_angle = pi/2;
nPseudo = size(xi_pj_ref,2);

%% Build anatomies in a (n x nPseudo) array, where n is the product of the elements of the pseudojoint angle  vectors(Cartesian Product)
for pseudo_cnt=1:nPseudo
    pseudo_angle_cnt = 0; % for each pseudojoint, the counter must count all possible angles, so before for must initialize to 0
    for pseudo_angle = -pseudo_limit_angle:pseudo_step_angle:pseudo_limit_angle
            pseudo_angle_cnt = pseudo_angle_cnt +1;
            % Give all possible values to each pseudo
            qp(pseudo_angle_cnt,pseudo_cnt) = pseudo_angle;  
    end
end

% following cases exist for 6 dof manipulators only
if nPseudo == 5
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5));  % each row of the array is a possible anatomy
elseif nPseudo == 6
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5),qp(:,6)); 
elseif nPseudo == 7
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5),qp(:,6),qp(:,7)); 
elseif nPseudo == 8
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5),qp(:,6),qp(:,7),qp(:,8)); 
elseif nPseudo == 9
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5),qp(:,6),qp(:,7),qp(:,7),qp(:,9)); 
elseif nPseudo == 10
    all_possible_anatomies = allcomb(qp(:,1),qp(:,2),qp(:,3),qp(:,4),qp(:,5),qp(:,6),qp(:,7),qp(:,7),qp(:,9),qp(:,10));  
end

total_possible_anatomies = size(all_possible_anatomies,1);  
special_anatomies_of_structure = 0;

%% Evaluate each anatomy
for anatomy_cnt=1:total_possible_anatomies
    
        evaluated_anatomy = all_possible_anatomies(anatomy_cnt,:);  % pick the row of array
        qa = zeros(6,1);                                            % reference configuration
        
        [~,~,xi_ai_anat,~,gst] = calculateForwardKinematicsPOE(structure,'6dof',xi_ai_ref,xi_pj_ref,qa,evaluated_anatomy,g_ai_ref,g_pj_ref,gst0);
        
        % for analytic degree & criterion specified by user
        switch degree
            case 'degree2'
                [criterion_satisfied] = find_special_anatomies_degree2(xi_ai_anat,criterion,TOL);
            case 'degree4'
                [criterion_satisfied] = find_special_anatomies_degree4(xi_ai_anat,criterion,TOL);
            otherwise
                warning('[calculateExhaustiveAnatomies_for_analytic_ikp]: Characteristic polynomial degree for analytic ikp solution specified NOT valid')

        end
        
        if criterion_satisfied
            special_anatomies_of_structure = special_anatomies_of_structure + 1;
        end
end

end