function [xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_3dof(structure,assembly_parameters)
% Assembles smm structure for ovidius robot. Based on file:
% kinematic_model_generate_assembly_smm.m
% Generates active-passive twists and joints-tcp frame tfs of assembled
% robot

% Input  -> structure:  7x2 string array
% Output -> 
%% Active-Passive joints counters, number of bodies inside each link
i_cnt = 0;
j_cnt = 0;
i_bodies = 0;
%% Build structure using the rules specified
wrong_string_structure = false;                     % assumes initial string is correct
if ~(strcmp(structure(1,:),fixed_active_string_notation)) % if 1st string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 1st string element is not declared ACTIVE')
else
    %% Structure after 1st active joint is correct
    i_cnt = i_cnt+1;
    i_bodies = i_bodies +1;
    %% START - BUILD base_link
    [xi_a1_0,g_s_m_i1_new] = build_base_link();
    [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a1_0, g_s_m_i1_new);
    [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_baselink(eye(4));
    %% END - BUILD base_link 
    
    %% START - Switch statement for 1st meta link follows-Always 2 sring elements are checked!
    % For 1st meta link, 2 conditions exist:
    switch structure(2,:) % first switch for 1st element         
        case passive_under_string_notation % 2nd case is that pseudo exists but only bolted in under base connectivity surface             
             j_cnt = j_cnt+1;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic1',g_s_m_i1_new);
             [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
             [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
             
             i_bodies = i_bodies +1; % increased counter for body count inside metalink
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
             
            % nested switch for 2nd element
            switch structure(3,:)
                 case  no_passive_string_notation % case 2.2.1 -> this case leads to 2.1.1
                     % nothing to add!
                        
                 case passive_under_string_notation % case 2.2.2 -> pseudo_moving->pseudo_static with syntetic 4   
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 1;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                 case passive_back_string_notation % case 2.2.3 ->  pseudo_moving->pseudo_static with syntetic 2            
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 1;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
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
         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
         [xi_a2_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new,xi_pj_ref(:,j_cnt));
         
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         
         i_cnt = i_cnt+1;
         [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a2_0, g_s_m_i1_new);
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
                         pseudo_assembly_index = 2;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    case passive_back_string_notation   % case 3.1.2                                
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 2;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0,  g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid') 
                end
                
            case passive_under_string_notation      
                j_cnt = j_cnt+1;
                pseudo_assembly_index = 2;
                [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                
                i_bodies = i_bodies +1; % increased counter for body count inside metalink
                [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                
                switch structure(6,:) % case 3.2
                    case no_passive_string_notation     % case 3.2.1
                         % nothing to add  -> this case leads to 3.1.1
                    case passive_under_string_notation  % case 3.2.2 
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         
                    case passive_back_string_notation   % case 3.2.3 
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')
                end 
                
            case passive_back_string_notation                 
                 j_cnt = j_cnt+1;
                 pseudo_assembly_index = 2;
                 [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                 [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                 [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                 
                 i_bodies = i_bodies +1; % increased counter for body count inside metalink
                 [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                switch structure(6,:)
                    case no_passive_string_notation     % case 3.3.1
                        % nothing to add  -> this case leads to 3.1.2
                    case passive_under_string_notation  % case 3.3.2
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                    case passive_back_string_notation   % case 3.3.3
                         j_cnt = j_cnt+1;
                         pseudo_assembly_index = 3;
                         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
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
         [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga('active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
         [xi_a3_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new,xi_pj_ref(:,j_cnt));
         
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         
         i_cnt = i_cnt+1;
         [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a3_0, g_s_m_i1_new);
         
         % Since this is last active twist, TOOL frame is added
         [gst0] = build_tool_frame(g_s_m_i1_new);
         
         %% END - Add active DXL
         i_bodies = 0;
         
         % Last link has only one body, the moving Dxl frame
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   
        
    end
end


end