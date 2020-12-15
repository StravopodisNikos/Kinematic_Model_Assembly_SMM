function GDME = calculateGlobalDME_Index_3DoF(xi_ai_struct_anat,xi_ai_struct_ref,g_s_link_as_anat,M_b_link_as_anat, Pi, gst0)
% Function that returns Dynamic Manipulability Index of manipulator in
% Conf-space. Used to study dynamic isotropy of anatomies

% xi_ai_struct_anat -> active twists are recalculated for each anatomy!
% This is done because inertia of the links must be recomputed for each new
% anatomy and not computed by the POE!
% xi_ai_struct_ref -> these are the REFERENCE ANATOMY twists of the
% assembled structure. Can be used for POE formula calculations with Pi's!

%% Ovidius robot properties
active_angle_limit(1) = 2.8; % [rad]
active_angle_limit(2) = 2; % [rad]
active_angle_limit(3) = 3.4; % [rad]
step_a2 = 0.20;
step_a3 = 0.20;
%% I.a. Definition of Configuration Space of first 3DoF
Cspace_count = 0;

for ta2=-active_angle_limit(2):step_a2:active_angle_limit(2)
        for ta3=-active_angle_limit(3):step_a3:active_angle_limit(3)
            
            % Vector for current Cspace point
            qa = [0.1 ta2 ta3];
            Cspace_count = Cspace_count + 1;

            %% I.b. In this configuration(qa1.qa2,qa3) compute GIM
            % calculateCoM_BodyJacobians_for_anat is better beacuse works
            % for all anatomies, but first the g_s_link_as(CoM frame of
            % each link @ desired anatomy) has to be specified. Here we are
            % still working on reference, so all is ok(for methodology check: kinematic_model_generate_assembly_smm.m)
            [J_b_sli] = calculateCoM_BodyJacobians_for_anat(xi_ai_struct_anat, qa, g_s_link_as_anat );
             
            [M_b] = calculateGIM(J_b_sli,M_b_link_as_anat);
            
            %%  I.c Compute Body Jacobian Matrix @ current configuration
            [~, Jbd,~] = calculateJacobians_for_assembled_structure('3dof', xi_ai_struct_ref, qa, Pi, gst0) ;
            
            Jtool = calculateToolJacobian_3dof(Jbd,[0 0 0]');
            %%  I.d Compute DME
            DME(Cspace_count) = abs(det(Jtool)/det(M_b));

         end
end

% Since LmdCI has an upper bound  of one. The global index is the max LmdCI
% calculated'
GDME = 1 / max(DME);
end