function [DCI] = calculateDynamicConditioningIndexDimLess_3DoF(M,wx)
% This matlab code was copied from moul2 PC on Th. 22.10.20

% Calculates Dynamic Conditioning Index for SMM 3 DoF structure
% A measure that quantifies how far is a given confguration from dynamic
% isotropy. Mass matrix M was previously generated for given structure,
% anatomy and configuration

%% DCI is dimension and configuration dependent!!!

% M: Generalized Inertia Matrix of Manipulator @ calculated configration
% wx:6x1 vextor: [w11 w22 w33 w12 w13 w23]

nDoF = size(M,2);

%% Sigma is a scalar defined such that the norm of difference matrix is minimum for fixed q
% sigma = (1/nDoF)*trace(M); % dimension-dependent definition
sigma = trace(M^2)/trace(M); % dimensionless definition
%% Get Difference Matrix between the generalized inertia matrix and its nearest isotropic matrix
% D = M - sigma.*eye(nDoF); % dimension-dependent definition
D = (1/sigma) .* (M - sigma.*eye(nDoF)); % dimensionless definition

% mask = triu(true(size(D)));
d1 = diag(D); %3
d2 = D(1,2:3)'; %2
d3 = D(2,3)'; %1
d = vertcat(d1,d2,d3); % extrac n*(n+1)/2 vector

% Define Weighting matrix depending on the user-desired relations between decoupling and isotropy
% and the dimensional inhomogeneities of inertia matrix

W = diag(wx);

DCI = 0.5*d'*W*d;

end