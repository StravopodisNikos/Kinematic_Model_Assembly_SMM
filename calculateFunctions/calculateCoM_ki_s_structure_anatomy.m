function [gst_anat,xi_ai_anat,M_s_com_k_i,g_s_com_k_i] = calculateCoM_ki_s_structure_anatomy(structure,assembly_parameters,anatomy,xi_pj_ref,point2figure)
% Based on
% ga_ojective_function/subroutines_executed_in_obj_fn/structure_assembly_3dof.m
% Input: Assembly sequence parametes & Passive twists
% Calculates the CoM/Inertia matrix of each body w.r.t to spatial robot
% frame. Only inertia building functions are used here! Also the active twists of each anatomy are
% recalculated!
% Reference twists and joint frames are generated from the
% structure_assembly_3dof.m file ONLY for the reference anatomy! Here a
% user-specified anatomy is assembled

% It was constructed in 29.10.20 after errors found in utilizing the fwd
% kin mapping for CoM of links @ reference anatomy

%% Add paths to all functions:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);
% Add libraries relative to matlab_ws folder
screws_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','screws',filesep);
screws_library_path = strcat(root_path,screws_path_relative_to_matlab_ws); addpath(screws_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
util_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_kinematics_library','util',filesep);
util_library_path = strcat(root_path,util_path_relative_to_matlab_ws); addpath(util_library_path);
% addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
screw_dynamics_path_relative_to_matlab_ws = fullfile('matlab_ws','screw_dynamics',filesep);
screw_dynamics_library_path = strcat(root_path,screw_dynamics_path_relative_to_matlab_ws); addpath(screw_dynamics_library_path);
% addpath('/home/nikos/matlab_ws/screw_dynamics')
Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM',filesep);
Kinematic_Model_Assembly_SMM_library_path = strcat(root_path,Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws); addpath(Kinematic_Model_Assembly_SMM_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM')
building_functions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','building_functions',filesep);
building_functions_library_path = strcat(root_path,building_functions_path_relative_to_matlab_ws); addpath(building_functions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
synthetic_joints_tfs_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','synthetic_joints_tfs',filesep);
synthetic_joints_tfs_library_path = strcat(root_path,synthetic_joints_tfs_path_relative_to_matlab_ws); addpath(synthetic_joints_tfs_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
calculateFunctions_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM','calculateFunctions',filesep);
calculateFunctions_library_path = strcat(root_path,calculateFunctions_path_relative_to_matlab_ws); addpath(calculateFunctions_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
 

% Input  -> structure:  7x2 string array
% Output -> 
%% Active-Passive joints counters, number of bodies inside each link
i_cnt = 0;
j_cnt = 0;
i_bodies = 0;

last_expo = eye(4);
%% Build structure using the rules specified
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3
nDoF_string = '3dof';

wrong_string_structure = false;                     % assumes initial string is correct
if ~(strcmp(structure(1,:),fixed_active_string_notation)) % if 1st string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 1st string element is not declared ACTIVE')
else
    %% Structure after 1st active joint is correct
    i_cnt = i_cnt+1;
    i_bodies = i_bodies +1;
    %% START - BUILD base_link ineria - this is default
    [xi_ai_anat(:,i_cnt),g_s_m_i1_new] = build_base_link();
    figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
    [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_baselink(eye(4));
    %% END - BUILD base_link 
    
    %% START - Switch statement for 1st meta link follows-Always 2 sring elements are checked!
    % For 1st meta link, 2 conditions exist:
    switch structure(2,:) % first switch for 1st element         
        case passive_under_string_notation % 2nd case is that pseudo exists but only bolted in under base connectivity surface             
             j_cnt = j_cnt+1;
             pseudo_assembly_index = 0;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic1',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
             [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
             figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
             
             i_bodies = i_bodies +1; % increased counter for body count inside metalink
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
             
            % nested switch for 2nd element
            switch structure(3,:)
                 case  no_passive_string_notation % case 2.2.1 -> this case leads to 2.1.1
                     % nothing to add!
                        
                 case passive_under_string_notation % case 2.2.2 -> pseudo_moving->pseudo_static with syntetic 4   
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 1;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                     figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                 case passive_back_string_notation % case 2.2.3 ->  pseudo_moving->pseudo_static with syntetic 2            
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 1;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                     [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                     figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);                     
                otherwise
                    warning('[SMM STRUCTURE ASSEMBLY]: 3rd string element is not valid')                     
            end
             
        otherwise
            warning('[SMM STRUCTURE ASSEMBLY]: 2nd string element is not valid')
    end
    %% END - Switch statement for 1st meta link

    %% START - Rules check for 2nd meta link
    if ~(strcmp(structure(4,:),fixed_active_string_notation)) % if 1st string element is NOT active
        wrong_string_structure = true;
        warning('[SMM STRUCTURE ASSEMBLY]: 4st string element is not declared ACTIVE')
    else
        
         %% START - Add active DXL
         active_assembly_index = 2;
         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
         [xi_a2_anat,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);
         figure(point2figure); drawframe(g_s_m_i1_new,0.15); hold on;
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         
         i_cnt = i_cnt+1;
         xi_ai_anat(:,i_cnt) = xi_a2_anat;
         %% END - Add active DXL
        i_bodies = 0;   % the static part of Dynamixel is the last body of each metalink
         
        %% START - Switch statement for 2nd meta link follows-Always 2 sring elements are checked!
        i_bodies = i_bodies +1; % increased counter for body count inside metalink
        [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);         
        
        % For 2nd meta link, 3 conditions exist:
        switch structure(5,:) % first switch for 1st element
            case no_passive_string_notation % case 3.1           
                % nothing to add!
                switch structure(6,:)
                    case passive_under_string_notation  % case 3.1.1
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3; % was 2 for 26_10 tests
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    case passive_back_string_notation   % case 3.1.2                                
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3; % was 2 for 26_10 tests
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid') 
                end
                
            case passive_under_string_notation      
                j_cnt = j_cnt+1;
                pseudo_assembly_index = 2;
                [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                
                i_bodies = i_bodies +1; % increased counter for body count inside metalink
                [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                
                switch structure(6,:) % case 3.2
                    case no_passive_string_notation     % case 3.2.1
                         % nothing to add  -> this case leads to 3.1.1
                    case passive_under_string_notation  % case 3.2.2 
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         
                    case passive_back_string_notation   % case 3.2.3 
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);                         
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')
                end 
                
            case passive_back_string_notation                 
                 j_cnt = j_cnt+1;
                 pseudo_assembly_index = 2;
                 [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                 [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                 figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                 
                 i_bodies = i_bodies +1; % increased counter for body count inside metalink
                 [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                switch structure(6,:)
                    case no_passive_string_notation     % case 3.3.1
                        % nothing to add  -> this case leads to 3.1.2
                    case passive_under_string_notation  % case 3.3.2
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                    case passive_back_string_notation   % case 3.3.3
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [last_expo,xi_pj_anat(:,j_cnt),g_s_m_i1_new] = build_pseudomodule_with_anat(g_s_m_i1_new,xi_pj_ref(:,j_cnt),anatomy(j_cnt),last_expo);
                         figure(point2figure); drawframe(g_s_m_i1_new,0.15,'alternate_color'); hold on;
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')    
                end   
            otherwise
                warning('[SMM STRUCTURE ASSEMBLY]: 5th string element is not valid')
        end %% END - Switch statement for 2nd meta link    
    end  %% END - Rules check for 2nd meta link
    
    if ~(strcmp(structure(7,:),fixed_active_string_notation)) % if 7th string element is NOT active
        wrong_string_structure = true;
        warning('[SMM STRUCTURE ASSEMBLY]: 7th string element is not declared ACTIVE')
    else
         
         %% START - Add active DXL
         active_assembly_index = 3;
         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
         [xi_a3_anat,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);
         
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         
         i_cnt = i_cnt+1;
         xi_ai_anat(:,i_cnt) = xi_a3_anat;
         
         % Since this is last active twist, TOOL frame is added
         [gst_anat] = build_tool_frame(g_s_m_i1_new);
         
         %% END - Add active DXL
         i_bodies = 0;
         
         % Last link has only one body, the moving Dxl frame
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   
        
    end
end

end