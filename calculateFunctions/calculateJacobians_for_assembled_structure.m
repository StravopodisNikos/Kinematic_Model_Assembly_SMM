function [metJs, metJb, gst] = calculateJacobians_for_assembled_structure(nDoF_string,xi_ai_ref,qa, Pi, gst0) 
% Calculates Js = Jspatial and Jb = Jbody
% of 3 OR 6 d.o.f. serial modular metamorphic manipulator 
% This function is used for structure synthesis purposes, so the number of
% pseudojoints of each metalink is not predetermined. The pseudo
% exponential Pi MUST have been computed with the reference
% structure+anatomy twists for the first time, when structure is assembled.
%% Caution!
% * There exists at least one pseudo between wach active joint!
% ** gst is not passed here for code evaluation
% *** must be the twists of the assembled structure for the REFERENCE
% ANATOMY!
%% 

% Compute active exponentials
nDoF = size(xi_ai_ref,2);
for i_cnt=1:nDoF
    exp_ai(:,:,i_cnt) = twistexp(xi_ai_ref(:,i_cnt),qa(i_cnt));
end

metJs = zeros(6,3);
metJb = zeros(6,3);
switch nDoF_string
    case '6dof'
        % Jspatial calculation (Murray p.116)
        metJs(:,1) = xi_ai_ref(:,1);
        metJs(:,2) = ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai_ref(:,2);
        metJs(:,3) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai_ref(:,3);
        metJs(:,4) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3))*xi_ai_ref(:,4);
        metJs(:,5) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4))*xi_ai_ref(:,5);
        metJs(:,6) = ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5))*xi_ai_ref(:,6);

        % Jbody calculation (Murray p.117)
        metJb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0))*xi_ai_ref(:,1);
        metJb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0))*xi_ai_ref(:,2);
        metJb(:,3) = ad(inv(exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0))*xi_ai_ref(:,3);
        metJb(:,4) = ad(inv(exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0))*xi_ai_ref(:,4);
        metJb(:,5) = ad(inv(exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0))*xi_ai_ref(:,5);
        metJb(:,6) = ad(inv(exp_ai(:,:,6)*gst0))*xi_ai_ref(:,6);
        
        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*Pi(:,:,3)*exp_ai(:,:,4)*Pi(:,:,4)*exp_ai(:,:,5)*Pi(:,:,5)*exp_ai(:,:,6)*gst0;
    case '3dof'
        % Jspatial calculation (Murray p.116)
        metJs(:,1)=xi_ai_ref(:,1);
        metJs(:,2)=ad(exp_ai(:,:,1)*Pi(:,:,1))*xi_ai_ref(:,2);
        metJs(:,3)=ad(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2))*xi_ai_ref(:,3);

        % Jbody calculation (Murray p.117)
        metJb(:,1) = ad(inv(exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai_ref(:,1);
        metJb(:,2) = ad(inv(exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0))*xi_ai_ref(:,2);
        metJb(:,3) = ad(inv(exp_ai(:,:,3)*gst0))*xi_ai_ref(:,3);

        gst = exp_ai(:,:,1)*Pi(:,:,1)*exp_ai(:,:,2)*Pi(:,:,2)*exp_ai(:,:,3)*gst0;
    otherwise
        warning('[calculateJacobians_for_assembled_structure]: DOF string NOT valid!')
end


% Gia epalh8eush xrhsimopoioume th sxesh (3.56)/Murray
% Js = (Adg)*Jb
% Adg = ad(gst);
% Jepal = Adg*metJb;
% Error = Jepal - metJs; % Eimaste swstoi
end