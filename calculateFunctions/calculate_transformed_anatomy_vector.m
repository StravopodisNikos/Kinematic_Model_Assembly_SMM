function transformed_anatomy_vector = calculate_transformed_anatomy_vector(structure,nDoF_string,qp_xacro_user_given)
% checks the strings given only @ default pseudojoint indexes and
% transforms the anatomy vector given by user
no_passive_string_notation = 'x9';
default_pseudo_indexes_3dof = 4;
default_pseudo_indexes_vector_3dof = [2,3,5,6]';
default_pseudo_indexes_6dof = 10;
default_pseudo_indexes_vector_6dof = [2,3,5,6,8,9,11,12,14,15]';

new_j_cnt = 0;

switch nDoF_string
    case '3dof'
        for j_cnt=1:default_pseudo_indexes_3dof % rebuild strings
            switch structure(default_pseudo_indexes_vector_3dof(j_cnt),:)
                case no_passive_string_notation
                    % do nothing
                otherwise
                    new_j_cnt = new_j_cnt + 1;
                    transformed_anatomy_vector(new_j_cnt) = qp_xacro_user_given(j_cnt);
            end
        end
    case '6dof'
        for j_cnt=1:default_pseudo_indexes_6dof % rebuild strings
            switch structure(default_pseudo_indexes_vector_6dof(j_cnt),:)
                case no_passive_string_notation
                    % do nothing
                otherwise
                    new_j_cnt = new_j_cnt + 1;
                    transformed_anatomy_vector(new_j_cnt) = qp_xacro_user_given(j_cnt);
            end
        end
    otherwise
        warning('[calculate_transformed_anatomy_vector]: no valid DoF selected')
        transformed_anatomy_vector = 0; % return error
end

end