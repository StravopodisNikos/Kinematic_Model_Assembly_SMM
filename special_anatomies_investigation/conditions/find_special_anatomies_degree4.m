function [criterion_satisfied] = find_special_anatomies_degree4(xi_ai_anat,criterion,TOL)
% Conditions for 6R manipulators as defined in:
% Nielsen-Roth/Kinematic Analysis of Robotic Mechanisms(1999)/p.1156
%
% The characteristic polynomial is reduced to degree 4 when:
% 1.a the manipulator has three parallel revolute axes
% 1.b the manipulator has three intersecting revolute axes
% 2.  the manipulator has three intersecting perpendicular pairs

% Here each criterion is investigated for a 6 DoF SMM structure

% Input : 1. the active twists @ reference configuration for the desired
% structure/anatomy 2. desired criterion 3. tolerance

% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);

% Add libraries relative to matlab_ws folder
geom3d_path_relative_to_matlab_ws = fullfile('matlab_ws','geom3d_library','geom3d',filesep); geom3d_library_path = strcat(root_path,geom3d_path_relative_to_matlab_ws); addpath(geom3d_library_path);

% calculate DoF
nDoF = size(xi_ai_anat,2); 

% create lines from active twist
for i_cnt=1:nDoF
    [p(:,i_cnt), dir(:,i_cnt)] = twistaxis(xi_ai_anat(:,i_cnt));
    twist_line(i_cnt,:) = createLine3d(p(1,i_cnt), p(2,i_cnt), p(3,i_cnt), dir(1,i_cnt), dir(2,i_cnt), dir(3,i_cnt));
end

cond_1_satisfied = false;
cond_2_satisfied = false;

switch criterion
    case 'parallel' %1.a
        if (isParallel3d(dir(:,1)', dir(:,2)', TOL) && isParallel3d(dir(:,2)', dir(:,3)', TOL)) && ~(isParallel3d(dir(:,3)', dir(:,4)', TOL)) && ~(isParallel3d(dir(:,4)', dir(:,5)', TOL)) && ~(isParallel3d(dir(:,5)', dir(:,6)', TOL)) 
            cond_1_satisfied = true;
        elseif ~(isParallel3d(dir(:,1)', dir(:,2)', TOL)) && ( isParallel3d(dir(:,2)', dir(:,3)', TOL) && isParallel3d(dir(:,3)', dir(:,4)', TOL) )  && ~(isParallel3d(dir(:,4)', dir(:,5)', TOL)) &&  ~(isParallel3d(dir(:,5)', dir(:,6)', TOL))
            cond_1_satisfied = true;
        elseif ~(isParallel3d(dir(:,1)', dir(:,2)', TOL)) && ~(isParallel3d(dir(:,2)', dir(:,3)', TOL)) && ( isParallel3d(dir(:,3)', dir(:,4)', TOL) && isParallel3d(dir(:,4)', dir(:,5)', TOL) ) &&  ~(isParallel3d(dir(:,5)', dir(:,6)', TOL))
            cond_1_satisfied = true;
        elseif ~(isParallel3d(dir(:,1)', dir(:,2)', TOL)) && ~(isParallel3d(dir(:,2)', dir(:,3)', TOL)) && ~(isParallel3d(dir(:,3)', dir(:,4)', TOL)) && ( isParallel3d(dir(:,4)', dir(:,5)', TOL) &&  isParallel3d(dir(:,5)', dir(:,6)', TOL) )
            cond_1_satisfied = true;
        else
            cond_1_satisfied = false;
        end
    case 'intersecting' %1.b
        if (isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL) && isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ~(isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL)) && ~(isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL)) && ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL)) 
            cond_1_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ( isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL) && isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL) )  && ~(isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL)) &&  ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL))
            cond_1_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ~(isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ( isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL) && isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL) ) &&  ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL))
            cond_1_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ~(isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ~(isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL)) && ( isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL) &&  isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL) )
            cond_1_satisfied = true;
        else
            cond_1_satisfied = false;
        end
    case 'perpendicular_intersecting'
        % first checks for intersecting
        if (isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL) && isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ~(isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL)) && ~(isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL)) && ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL)) 
            intersecting_1_satisfied = true;
            intersecting_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ( isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL) && isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL) )  && ~(isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL)) &&  ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL))
            intersecting_2_satisfied = true;
            intersecting_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ~(isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ( isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL) && isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL) ) &&  ~(isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL))
            intersecting_3_satisfied = true;
            intersecting_satisfied = true;
        elseif ~(isIntersecting3d(twist_line(1,:), twist_line(2,:), TOL)) && ~(isIntersecting3d(twist_line(2,:), twist_line(3,:), TOL)) && ~(isIntersecting3d(twist_line(3,:), twist_line(4,:), TOL)) && ( isIntersecting3d(twist_line(4,:), twist_line(5,:), TOL) &&  isIntersecting3d(twist_line(5,:), twist_line(6,:), TOL) )
            intersecting_4_satisfied = true;
            intersecting_satisfied = true;
        else
            intersecting_satisfied = false;
            cond_2_satisfied = false;
        end
        
        % secondly checks for orthogonality
        if intersecting_satisfied
            if intersecting_1_satisfied && ( (isPerpendicular3d(dir(:,1)', dir(:,2)', TOL) && isPerpendicular3d(dir(:,2)', dir(:,3)', TOL)) && ~(isPerpendicular3d(dir(:,3)', dir(:,4)', TOL)) && ~(isPerpendicular3d(dir(:,4)', dir(:,5)', TOL)) && ~(isPerpendicular3d(dir(:,5)', dir(:,6)', TOL)) )
                cond_2_satisfied = true;
            elseif intersecting_2_satisfied &&( ~(isPerpendicular3d(dir(:,1)', dir(:,2)', TOL)) && ( isPerpendicular3d(dir(:,2)', dir(:,3)', TOL) && isPerpendicular3d(dir(:,3)', dir(:,4)', TOL) )  && ~(isPerpendicular3d(dir(:,4)', dir(:,5)', TOL)) &&  ~(isPerpendicular3d(dir(:,5)', dir(:,6)', TOL)) )
                cond_2_satisfied = true;
            elseif intersecting_3_satisfied && ( ~(isPerpendicular3d(dir(:,1)', dir(:,2)', TOL)) && ~(isPerpendicular3d(dir(:,2)', dir(:,3)', TOL)) && ( isPerpendicular3d(dir(:,3)', dir(:,4)', TOL) && isPerpendicular3d(dir(:,4)', dir(:,5)', TOL) ) &&  ~(isPerpendicular3d(dir(:,5)', dir(:,6)', TOL)) )
                cond_2_satisfied = true;
            elseif intersecting_4_satisfied && ( ~(isPerpendicular3d(dir(:,1)', dir(:,2)', TOL)) && ~(isPerpendicular3d(dir(:,2)', dir(:,3)', TOL)) && ~(isPerpendicular3d(dir(:,3)', dir(:,4)', TOL)) && ( isPerpendicular3d(dir(:,4)', dir(:,5)', TOL) &&  isPerpendicular3d(dir(:,5)', dir(:,6)', TOL) ) )
                cond_2_satisfied = true;
            else
                cond_2_satisfied = false;
            end
        end
        
    otherwise
        warning('[find_special_anatomies_degree4_conditions]: Criterion specified NOT valid')
end

if cond_1_satisfied || cond_2_satisfied
    criterion_satisfied = true;
else
    criterion_satisfied = false;
end

end