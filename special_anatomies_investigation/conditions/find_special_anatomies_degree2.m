function [criterion_satisfied] = find_special_anatomies_degree2(xi_ai_anat,criterion,TOL)
% Conditions for 6R manipulators as defined in:
% Nielsen-Roth/Kinematic Analysis of Robotic Mechanisms(1999)/p.1156
%
% The characteristic polynomial is reduced to degree 2 when:
% 1. the manipulator has three parallel revolute axes & 1 additional pair
%    of axes that are either parallel OR intersect(at most 4 configurations)
% 2. the manipulator has three pairs of parallel revolute axes(in theory 16
%    configurations but in practice at most 8 at this time)
% CRITERION 2 IN NOT ANALYTIC AND WILL NOT BE CONSIDERED!
% 3. the manipulator has three intersecting perpendicular axes and one
%    additional pair of orthogonally intersecting axes

% Here each criterion is investigated for a 6 DoF SMM structure

% Input : 1. the active twists @ reference configuration for the desired
% structure/anatomy 2. desired criterion 3. tolerance

% Output is ONLY a 1x1 logical value! => Thisfunction can be used to
% evaluate a specified structure+anatomy (exhaustive anatomy evaluation)

% Add libraries relative to matlab_ws folder
geom3d_path_relative_to_matlab_ws = fullfile('geom3d_library','geom3d',filesep); geom3d_library_path = strcat(root_path,geom3d_path_relative_to_matlab_ws); addpath(geom3d_library_path);

% calculate DoF
nDoF = size(xi_ai_anat,2);

% create lines from active twist
for i_cnt=1:nDoF
    [p(:,i_cnt), dir(:,i_cnt)] = twistaxis(xi_ai_anat(:,i_cnt));
    twist_line(i_cnt,:) = createLine3d(p(1,i_cnt), p(2,i_cnt), p(3,i_cnt), dir(1,i_cnt), dir(2,i_cnt), dir(3,i_cnt));
end

% 1. Check criterion for 3 consecutive axes
special_twists_cnt = 0;
possible_3_combinations = 4;
default_twists_indexes = [1,2,3,4,5,6]';

%criterion_satisfied = false(possible_3_combinations,1);
criterion_satisfied = false;

for comb_cnt=1:possible_3_combinations
    
    [criterion_3_axes_satisfied(comb_cnt)] = check_criterion_3axes(xi_ai_anat(:,comb_cnt),xi_ai_anat(:,comb_cnt+1),xi_ai_anat(:,comb_cnt+2),criterion,TOL);
    
    % 2. Extract special twists if criterion is satisfied
    if criterion_3_axes_satisfied(comb_cnt)
        special_twists_cnt = special_twists_cnt + 1;                        % 1 set of special axes has been found
        
        special_twists(:,1,special_twists_cnt) = xi_ai_anat(:,comb_cnt);    % 1st special axis of the set
        special_twists(:,2,special_twists_cnt) = xi_ai_anat(:,comb_cnt+1);  % 2nd special axis of the set
        special_twists(:,3,special_twists_cnt) = xi_ai_anat(:,comb_cnt+2);  % 3rd special axis of the set
        special_twists_indexes(:,special_twists_cnt) = [comb_cnt, comb_cnt+1, comb_cnt+2]';     % save the 3 special indexes
        
        % determine index of non special twists
        remaining_twists = nDoF-3;
        non_special_twists_indexes1 = default_twists_indexes(1:special_twists_indexes(1)-1);  
        non_special_twists_indexes2 = default_twists_indexes(special_twists_indexes(3)+1:6);
        non_special_twists_indexes = vertcat(non_special_twists_indexes1,non_special_twists_indexes2);
   
        % extract the 3 non-special twists
        non_special_twists(:,1,special_twists_cnt) = xi_ai_anat(:,non_special_twists_indexes(1));
        non_special_twists(:,2,special_twists_cnt) = xi_ai_anat(:,non_special_twists_indexes(2));
        non_special_twists(:,3,special_twists_cnt) = xi_ai_anat(:,non_special_twists_indexes(3));
        
        % 3. Check remaining non-special twists with their adjacent (in first twist matrix!)
        criterion_adjacent_axes_satisfied1(:,special_twists_cnt) = false(remaining_twists,1);
        criterion_adjacent_axes_satisfied2(:,special_twists_cnt) = false(remaining_twists,1);
        for remain_cnt = 1:remaining_twists
            
            non_special_twist_rechecked = non_special_twists(:,remain_cnt,special_twists_cnt);
            
            % determine the adjacent twists, picks k twist, checks if
            % k-1,k+1 twist indexes exist in initial twist matrix
            if (non_special_twists_indexes(remain_cnt) - 1) >= 1 
                adjacent_left_twist = xi_ai_anat(:,non_special_twists_indexes(remain_cnt) - 1);
                [criterion_adjacent_axes_satisfied1(remain_cnt,special_twists_cnt)] = check_criterion_adjacent_axes(non_special_twist_rechecked,adjacent_left_twist,criterion,TOL);
            end
            if (non_special_twists_indexes(remain_cnt) + 1) <= 6 
                adjacent_right_twist = xi_ai_anat(:,non_special_twists_indexes(remain_cnt) + 1);
                [criterion_adjacent_axes_satisfied2(remain_cnt,special_twists_cnt)] = check_criterion_adjacent_axes(non_special_twist_rechecked,adjacent_right_twist,criterion,TOL);               
            end
           
            if criterion_adjacent_axes_satisfied1(remain_cnt,special_twists_cnt) || criterion_adjacent_axes_satisfied2(remain_cnt,special_twists_cnt)
%                 criterion_satisfied(special_twists_cnt) = true;
                 criterion_satisfied = true;
            end
                  
        end
         
    end
      
end


end