% This code generates the assembly of the sructure defined by the
% optimization parameters and saves the reference anatomy/structure twists
% and joint & link frames tfs. It calculates in matlab the tf's induced by
% xacro file assembly developed in:
% /PhD/projects/Parametric_Simulation_Model_SMM
% It is the MATLAB evaluation of the robot visualization tools used to
% assembly modular structure!

% HOW TO EXECUTE
% 1. Set the parameters extracted from optimization:
% Section "Define smm structure string" <--> test_structure_string_definition.yaml
% the synthetic tfs are given inside the switch statement in
% "add_synthetic_joint_tf" <-->  tf_list_for_conditioned_assembly.yaml and
% assembly_parameters_for_ovidius_robot.yaml
% 2. Urdf file is built from xacro file, having set the corresponding
% parameters in the yaml file
% >> nikos@syros-b14-nikos-ubuntu:~/PhD/projects/Parametric_Simulation_Model_SMM/xacros$ xacro conditioned_parameterized_SMM_assembly.xacro > generated_urdf_from_xacros_here/conditioned_parameterized_SMM_assembly.urdf
% 3. structure in l.41-47 is set by user
% 4. Run code

% Include libraries
addpath('/home/nikos/matlab_ws/screw_kinematics_library/screws')
addpath('/home/nikos/matlab_ws/screw_kinematics_library/util')
addpath('/home/nikos/matlab_ws/screw_dynamics')

addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
clear;
close all;

%% Only visualization for code evaluation
% Ref structure robot - All data must be calculated for reference structure
robotURDFfile = '/home/nikos/PhD/projects/Parametric_Simulation_Model_SMM/xacros/generated_urdf_from_xacros_here/conditioned_parameterized_SMM_assembly.urdf';

[RefRobot,RefFig,RefConfig,NumDoF] = ImportRobotRefAnatomyModel(robotURDFfile);

% LAST_ACTIVE = char(RefRobot.BodyNames(11));
% TOOL =  char(RefRobot.BodyNames(12));
% g_last_active_TOOL = getTransform(RefRobot, RefConfig,TOOL, LAST_ACTIVE);  % pseudo moving tf with respect to pseudo static frame {ki} (from)ja->(into)jb

%% Define smm structure string (in optimization it is ga generated!)
logical wrong_string_structure;
fixed_active_string_notation = 'x0';
no_passive_string_notation = 'x9';
passive_under_string_notation = '21';
passive_back_string_notation = '31';

structure(1,:) = fixed_active_string_notation;
structure(2,:) = passive_under_string_notation;
structure(3,:) = passive_back_string_notation;
structure(4,:) = fixed_active_string_notation;
structure(5,:) = passive_under_string_notation;
structure(6,:) = passive_back_string_notation;
structure(7,:) = fixed_active_string_notation;

% Each link Inertia Matrix is constructed as the Sum of the Links Bodies
% Inertia Marices specified for each metalink given the structure strink!
% For link k with i bodies: M_s_link_as(:,:,k) = Σ (M_s_com_k_i) |
% i=1:i_bodies. The i_bodies is specified inside the switch statement of
% each metalink!

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
    figure(RefFig); xi_graph = drawtwist(xi_a1_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
    [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a1_0, g_s_m_i1_new);
    
    [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_baselink(eye(4));
    figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
    %% END - BUILD base_link 
    
    %% START - Switch statement for 1st meta link follows-Always 2 sring elements are checked!
    % For 1st meta link, 2 conditions exist:
    switch structure(2,:) % first switch for 1st element
        case no_passive_string_notation % 1st case is that no pseudo exists in 2nd string element
            % nested switch for 2nd element
             switch structure(3,:)
                 case passive_under_string_notation % case 2.1.1 ->  since 1st element is empty then MUST exist pseudo connected with under base         
                     j_cnt = j_cnt+1;   
                     [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic1',g_s_m_i1_new);
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                     figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                 otherwise
                    warning('[SMM STRUCTURE ASSEMBLY]: 3nd string element is not valid')                     
             end
             
        case passive_under_string_notation % 2nd case is that pseudo exists but only bolted in under base connectivity surface             
             j_cnt = j_cnt+1;
             [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic1',g_s_m_i1_new);
             figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
             [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
             figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
             [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
             
             i_bodies = i_bodies +1; % increased counter for body count inside metalink
             [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
             figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
             
            % nested switch for 2nd element
            switch structure(3,:)
                 case  no_passive_string_notation % case 2.2.1 -> this case leads to 2.1.1
                     % nothing to add!
                        
                 case passive_under_string_notation % case 2.2.2 -> pseudo_moving->pseudo_static with syntetic 4   
                     j_cnt = j_cnt+1;
                     [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic4',g_s_m_i1_new);
                     figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;                     
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                     figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                 case passive_back_string_notation % case 2.2.3 ->  pseudo_moving->pseudo_static with syntetic 2            
                     j_cnt = j_cnt+1;
                     [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic2',g_s_m_i1_new);
                     figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
                     [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                     figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                     [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                     
                     i_bodies = i_bodies +1; % increased counter for body count inside metalink
                     [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);                     
                     figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
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
         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('active_assembly',g_s_m_i1_new);
         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
         [xi_a2_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new,xi_pj_ref(:,j_cnt));
         
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
         
         i_cnt = i_cnt+1;
         figure(RefFig); xi_graph = drawtwist(xi_a2_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
         [xi_ai_ref(:,i_cnt),g_ai_ref(:,:,i_cnt)] = extract_ref_structure_anatomy_info('active', xi_a2_0, g_s_m_i1_new);
         %% END - Add active DXL
        i_bodies = 0;   % the static part of Dynamixel is the last body of each metalink
         
        %% START - Switch statement for 2nd meta link follows-Always 2 sring elements are checked!
        i_bodies = i_bodies +1; % increased counter for body count inside metalink
        [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_moving(g_s_m_i1_new);         
        figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
        
        % For 2nd meta link, 3 conditions exist:
        switch structure(5,:) % first switch for 1st element
            case no_passive_string_notation % case 3.1           
                % nothing to add!
                switch structure(6,:)
                    case passive_under_string_notation  % case 3.1.1
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic5',g_s_m_i1_new);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;  
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                    case passive_back_string_notation   % case 3.1.2                                
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic3',g_s_m_i1_new);
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;  
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0,  g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid') 
                end
                
            case passive_under_string_notation      
                j_cnt = j_cnt+1;
                [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic5',g_s_m_i1_new);
                [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                
                i_bodies = i_bodies +1; % increased counter for body count inside metalink
                [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                
                switch structure(6,:) % case 3.2
                    case no_passive_string_notation     % case 3.2.1
                         % nothing to add  -> this case leads to 3.1.1
                    case passive_under_string_notation  % case 3.2.2 
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic4',g_s_m_i1_new);
                         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;                     
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                         
                    case passive_back_string_notation   % case 3.2.3 
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic2',g_s_m_i1_new);
                         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on; 
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                    otherwise
                         warning('[SMM STRUCTURE ASSEMBLY]: 6th string element is not valid')
                end 
                
            case passive_back_string_notation                 
                 j_cnt = j_cnt+1;
                 [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic3',g_s_m_i1_new);
                 [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                 figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;                  
                 [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                 
                 i_bodies = i_bodies +1; % increased counter for body count inside metalink
                 [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new);
                 figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                switch structure(6,:)
                    case no_passive_string_notation     % case 3.3.1
                        % nothing to add  -> this case leads to 3.1.2
                    case passive_under_string_notation  % case 3.3.2
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic4',g_s_m_i1_new);
                         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;                     
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
                    case passive_back_string_notation   % case 3.3.3
                         j_cnt = j_cnt+1;
                         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('synthetic2',g_s_m_i1_new);
                         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
                         [xi_pj_0,g_s_m_i1_new] = build_pseudomodule(g_s_m_i1_new);
                         figure(RefFig); xi_graph = drawtwist(xi_pj_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
                         [xi_pj_ref(:,j_cnt),g_pj_ref(:,:,j_cnt)] = extract_ref_structure_anatomy_info('passive', xi_pj_0, g_s_m_i1_new);
                         
                         i_bodies = i_bodies +1; % increased counter for body count inside metalink
                         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_pseudomodule(g_s_m_i1_new); 
                         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
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
         [synthetic_tform,g_s_m_i1_new] = add_synthetic_joint_tf('active_assembly',g_s_m_i1_new);
         figure(RefFig); drawframe(g_s_m_i1_new,0.15); hold on;
         [xi_a3_0,g_s_m_i1_new] = build_activemodule(g_s_m_i1_new,xi_pj_ref(:,j_cnt));
         
         i_bodies = i_bodies +1; % increased counter for body count inside metalink
         [g_s_com_k_i(:,:,i_cnt,i_bodies), M_s_com_k_i(:,:,i_cnt,i_bodies)] = build_inertia_active_static(g_s_m_i1_new);          
         figure(RefFig); drawframe( g_s_com_k_i(:,:,i_cnt,i_bodies),0.15,'alternate_color'); hold on;
         
         i_cnt = i_cnt+1;
         figure(RefFig); xi_graph = drawtwist(xi_a3_0); hold on; drawframe(g_s_m_i1_new,0.15); hold on;
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

% NEXT ARE ONLY FOR CODE VERIFICATION: WILL BE IMPLEMENTED IN SEPARATE
% FUNCTIONS!

% 1.POE FORWARD KINEMATICS(works fine)
% SET configuration and anatomy of assembled structure => must agree with
% xacro file that built urdf
qa = [0 1 1]'; qp = [0 0 0 0]';
TestFig = figure; show(RefRobot,qa); hold on;
%[TestFig] = visualize_robot_urdf(robotURDFfile,qa);
[g_ai,g_pj,Jsp,Pi,gst] = calculateForwardKinematicsPOE(structure,xi_ai_ref,xi_pj_ref,qa,qp,g_ai_ref,g_pj_ref,gst0);
figure(TestFig); drawframe(g_ai(:,:,1),0.15); hold on; drawframe(g_ai(:,:,2),0.15); hold on; drawframe(g_ai(:,:,3),0.15); drawframe(gst,0.15); hold on; hold on; xi_a2_graph = drawtwist(Jsp(:,2)); hold on; xi_a3_graph = drawtwist(Jsp(:,3)); hold on;
figure(TestFig); drawframe(g_pj(:,:,1),0.15); hold on; drawframe(g_pj(:,:,2),0.15); % hold on;  drawframe(g_pj(:,:,3),0.15); hold on; drawframe(g_pj(:,:,4),0.15); hold on;
% save('ikp_test_for_babis.mat','structure','qp','xi_ai_ref','xi_pj_ref','g_ai_ref','g_pj_ref','gst0')

% 2.EXTRACT INERTIAS FOR GIVEN STRUCTURE @ REFERENCE ANATOMY
% Evaluate calculated Link Inertias(works fine)
[g_s_link_as,M_s_link_as] = calculateCoMmetalinks(M_s_com_k_i,g_s_com_k_i);
figure(RefFig); scatter3(g_s_link_as(1,1,1), g_s_link_as(2,1,1), g_s_link_as(3,1,1),'p','filled'); hold on;
figure(RefFig); scatter3(g_s_link_as(1,1,2), g_s_link_as(2,1,2), g_s_link_as(3,1,2),'p','filled'); hold on;
figure(RefFig); scatter3(g_s_link_as(1,1,3), g_s_link_as(2,1,3), g_s_link_as(3,1,3),'p','filled'); hold on;

% Now that metalinks COM were found the Metalink Inertia Matrix|Body frame
% is found
[M_b_link_as] = calculateMetalinkInertiaMatrixBody(g_s_link_as,M_s_link_as);
% 3.CONSTRUCT LINK BODY JACOBIANS ANG GENERALIZED INERTIA MATRIX
[J_b_sli] = calculateCoM_BodyJacobians(xi_ai_ref, qa, Pi, g_s_link_as );
[M_b] = calculateGIM(J_b_sli,M_b_link_as); %
M_b_matlab = massMatrix(RefRobot,qa);

