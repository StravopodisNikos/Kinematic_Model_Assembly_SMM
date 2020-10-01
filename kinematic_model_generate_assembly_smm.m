% Include libraries
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

clear;
close all;

%% Define smm structure string (in optimization it is ga generated!)
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';
passive_under_string_notation = '21';
passive_back_string_notation = '31';

structure(1,:) = fixed_active_string_notation;
structure(2,:) = passive_under_string_notation;
% structure(3,:) = '0';
% structure(4,:) = '0';
% structure(5,:) = '0';
% structure(6,:) = '0';
% structure(7,:) = '0';

%% Build structure using the rules specified
wrong_string_structure = false;                     % assumes initial string is correct
if ~(strcmp(structure(1,:),fixed_active_string_notation)) % if 1st string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 1st string element is not declared ACTIVE')
else
    %% Structure after 1st active joint is correct
    
    %% START - BUILD base_link
    
    [xi_a1_0] = build_base_link();

    %% END - BUILD base_link 
    
    
    %% START - Switch statement for 1st meta link follows-Always 2 sring elements are checked!
    % For 1st meta link 2 conditions exist:

    switch structure(2,:) % first switch for 1st element
        case no_passive_string_notation % 1st case is that no pseudo exists in 2nd string element
            % nested switch for 2nd element
             switch structure(3,:)
                 case passive_under_string_notation % case 2.1.1 ->  since 1st element is empty then MUST exist pseudo connected with under base
                 
                     [passive_under_tform] = add_passive_under_synthetic_joint_tf();
                     
                 otherwise
                    warning('[SMM STRUCTURE ASSEMBLY]: 3nd string element is not valid')                     
             end
             
        case passive_under_string_notation % 2nd case is that pseudo exists but only bolted in under base connectivity surface
            % nested switch for 2nd element
            switch structure(3,:)
                 case  no_passive_string_notation % case 2.2.1 -> this case leads to 2.1.1
                     
                 case passive_under_string_notation % case 2.2.2 ->
                     
                 case passive_back_string_notation % case 2.2.3 ->
                 
                otherwise
                    warning('[SMM STRUCTURE ASSEMBLY]: 3nd string element is not valid')                     
             end
        otherwise
            warning('[SMM STRUCTURE ASSEMBLY]: 2nd string element is not valid')
    end
    %% END - Switch statement for 1st meta link
            
end
    