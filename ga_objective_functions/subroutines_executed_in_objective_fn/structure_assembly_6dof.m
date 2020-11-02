function [xi_ai_ref,xi_pj_ref,g_ai_ref,g_pj_ref,gst0,M_s_com_k_i,g_s_com_k_i,wrong_string_structure] = structure_assembly_6dof(structure,assembly_parameters,p2f)
% Assembles smm structure for ovidius robot. Based on file:
% structure_assembly_6dof.m
% Generates active-passive twists and joints-tcp frame tfs of assembled
% robot
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')


% Input  -> structure:  7x2 string array
% Output -> 
%% Active-Passive joints counters, number of bodies inside each link
i_cnt = 0;
j_cnt = 0;
i_bodies = 0;
%% Build structure using the rules specified
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';    % -> in ga Int Value:1
passive_under_string_notation = '21'; % -> in ga Int Value:2
passive_back_string_notation = '31';  % -> in ga Int Value:3
nDoF_string = '6dof';
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
             pseudo_assembly_index = 0;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic1',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                 case passive_back_string_notation % case 2.2.3 ->  pseudo_moving->pseudo_static with syntetic 2            
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 1;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
end

%% START - Rules check for 2nd meta link
if ~(strcmp(structure(4,:),fixed_active_string_notation)) % if 1st string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 4st string element is not declared ACTIVE')
else

     %% START - Add active DXL
     active_assembly_index = 2;
     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
     [xi_a2_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);

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
                     pseudo_assembly_index = 3; % was 2 for 26_10 tests
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                case passive_back_string_notation   % case 3.1.2                                
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 3; % was 2 for 26_10 tests
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
            [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

                case passive_back_string_notation   % case 3.2.3 
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 3;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                case passive_back_string_notation   % case 3.3.3
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 3;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
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
    
%% START - Rules check for 3rd meta link
if ~(strcmp(structure(7,:),fixed_active_string_notation)) % if 7th string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 7th string element is not declared ACTIVE')
else

     %% START - Add active DXL
     active_assembly_index = 3;
     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
     figure(p2f); drawframe(g_s_m_i1_new,0.15); hold on;
     [xi_a3_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);
     figure(p2f); drawframe(g_s_m_i1_new,0.15); hold on;
     
     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          

     i_cnt = i_cnt+1;
     [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a3_0, g_s_m_i1_new);
     figure(p2f); xi_a3_graph = drawtwist(xi_a3_0); hold on;
     %% END - Add active DXL
     i_bodies = 0;

     % Last link has only one body, the moving Dxl frame
     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   

     %% ADD 3RD METALINK PSEUDOS - START
     % For 3rd meta link, 3 conditions exist:
     switch structure(8,:) % first switch for 1st element
        case no_passive_string_notation % case 3.1           
            % nothing to add!
            switch structure(9,:)
                case passive_under_string_notation  % case 3.1.1
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5; 
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                case passive_back_string_notation   % case 3.1.2                                
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5; % was 2 for 26_10 tests
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0,  g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid') 
            end

        case passive_under_string_notation      
            j_cnt = j_cnt+1;
            pseudo_assembly_index = 4;
            [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
            [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
            [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

            i_bodies = i_bodies +1; % increased counter for body count inside metalink
            [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

            switch structure(9,:) % case 3.2
                case no_passive_string_notation     % case 3.2.1
                     % nothing to add  -> this case leads to 3.1.1
                case passive_under_string_notation  % case 3.2.2 
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

                case passive_back_string_notation   % case 3.2.3 
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);


                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')
            end 

        case passive_back_string_notation                 
             j_cnt = j_cnt+1;
             pseudo_assembly_index = 4;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
             [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
             [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

             i_bodies = i_bodies +1; % increased counter for body count inside metalink
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
            switch structure(9,:)
                case no_passive_string_notation     % case 3.3.1
                    % nothing to add  -> this case leads to 3.1.2
                case passive_under_string_notation  % case 3.3.2
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                case passive_back_string_notation   % case 3.3.3
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 5;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')    
            end   
        otherwise
            warning('[SMM STRUCTURE ASSEMBLY]: 5th string element is not valid')
     end
     %% ADD 3RD METALINK PSEUDOS - FINISH
end
 %% END - Rules check for 3rd meta link
     
%% START - Rules check for 4th meta link
if ~(strcmp(structure(10,:),fixed_active_string_notation)) % if 10th string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 10th string element is not declared ACTIVE')
else

     %% START - Add active DXL
     active_assembly_index = 4;
     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
     [xi_a4_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);

     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          

     i_cnt = i_cnt+1;
     [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a4_0, g_s_m_i1_new);
     figure(p2f); xi_a4_graph = drawtwist(xi_a4_0); hold on;
     %% END - Add active DXL
     i_bodies = 0;

     % Last link has only one body, the moving Dxl frame
     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   

     %% ADD 4th METALINK PSEUDOS - START
     % For 4th meta link, 3 conditions exist:
     switch structure(11,:) % first switch for 1st element
        case no_passive_string_notation % case 3.1           
            % nothing to add!
            switch structure(12,:)
                case passive_under_string_notation  % case 3.1.1
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7; 
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                case passive_back_string_notation   % case 3.1.2                                
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7; % was 2 for 26_10 tests
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0,  g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 12th string element is not valid') 
            end

        case passive_under_string_notation      
            j_cnt = j_cnt+1;
            pseudo_assembly_index = 6;
            [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
            [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
            [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

            i_bodies = i_bodies +1; % increased counter for body count inside metalink
            [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

            switch structure(12,:) % case 3.2
                case no_passive_string_notation     % case 3.2.1
                     % nothing to add  -> this case leads to 3.1.1
                case passive_under_string_notation  % case 3.2.2 
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

                case passive_back_string_notation   % case 3.2.3 
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);


                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 12th string element is not valid')
            end 

        case passive_back_string_notation                 
             j_cnt = j_cnt+1;
             pseudo_assembly_index = 6;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
             [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
             [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

             i_bodies = i_bodies +1; % increased counter for body count inside metalink
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
            switch structure(12,:)
                case no_passive_string_notation     % case 3.3.1
                    % nothing to add  -> this case leads to 3.1.2
                case passive_under_string_notation  % case 3.3.2
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                case passive_back_string_notation   % case 3.3.3
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 7;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 12th string element is not valid')    
            end   
        otherwise
            warning('[SMM STRUCTURE ASSEMBLY]: 11th string element is not valid')
     end
     %% ADD 4th METALINK PSEUDOS - FINISH
end
 %% END - Rules check for 4th meta link

%% START - Rules check for 5th meta link
if ~(strcmp(structure(13,:),fixed_active_string_notation)) % if 10th string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 13th string element is not declared ACTIVE')
else

     %% START - Add active DXL
     active_assembly_index = 5;
     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
     [xi_a5_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);

     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          

     i_cnt = i_cnt+1;
     [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a5_0, g_s_m_i1_new);
    figure(p2f); xi_a5_graph = drawtwist(xi_a5_0); hold on;
     %% END - Add active DXL
     i_bodies = 0;

     % Last link has only one body, the moving Dxl frame
     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   

     %% ADD 5th METALINK PSEUDOS - START
     % For 5th meta link, 3 conditions exist:
     switch structure(14,:) % first switch for 1st element
        case no_passive_string_notation       
            % nothing to add!
            switch structure(15,:)
                case passive_under_string_notation  
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9; 
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1;
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                case passive_back_string_notation                                  
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9; 
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0,  g_s_m_i1_new);

                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 15th string element is not valid') 
            end

        case passive_under_string_notation      
            j_cnt = j_cnt+1;
            pseudo_assembly_index = 8;
            [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic5',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
            [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
            [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

            i_bodies = i_bodies +1; 
            [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

            switch structure(15,:) 
                case no_passive_string_notation    
                     % nothing to add  -> this case leads to 
                case passive_under_string_notation  
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; 
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);

                case passive_back_string_notation   
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);


                     i_bodies = i_bodies +1; 
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 15th string element is not valid')
            end 

        case passive_back_string_notation                 
             j_cnt = j_cnt+1;
             pseudo_assembly_index = 8;
             [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic3',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
             [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
             [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

             i_bodies = i_bodies +1;
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
            switch structure(15,:)
                case no_passive_string_notation     
                    % nothing to add  -> this case leads to 
                case passive_under_string_notation  
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic4',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; 
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                case passive_back_string_notation   
                     j_cnt = j_cnt+1;
                     pseudo_assembly_index = 9;
                     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'synthetic2',g_s_m_i1_new,assembly_parameters,pseudo_assembly_index);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);

                     i_bodies = i_bodies +1; 
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                otherwise
                     warning('[SMM STRUCTURE ASSEMBLY]: 15th string element is not valid')    
            end   
        otherwise
            warning('[SMM STRUCTURE ASSEMBLY]: 14th string element is not valid')
     end
     %% ADD 5th METALINK PSEUDOS - FINISH
end
 %% END - Rules check for 5th meta link

 %% ADD final link - START
if ~(strcmp(structure(7,:),fixed_active_string_notation)) % if 7th string element is NOT active
    wrong_string_structure = true;
    warning('[SMM STRUCTURE ASSEMBLY]: 7th string element is not declared ACTIVE')
else

     %% START - Add active DXL
     active_assembly_index = 6;
     [~,g_s_m_i1_new] = add_synthetic_joint_tf_for_ga(nDoF_string,'active_assembly',g_s_m_i1_new,assembly_parameters,active_assembly_index);
     [xi_a6_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new);

     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          

     i_cnt = i_cnt+1;
     [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a6_0, g_s_m_i1_new);
    figure(p2f); xi_a6_graph = drawtwist(xi_a6_0); hold on;
     % Since this is last active twist, TOOL frame is added
     [gst0] = build_tool_frame(g_s_m_i1_new);

     %% END - Add active DXL
     i_bodies = 0;

     % Last link has only one body, the moving Dxl frame
     i_bodies = i_bodies +1; % increased counter for body count inside metalink
     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);   

end
 %% ADD final link - END
end