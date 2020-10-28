% GA MultiObjective Optimization Call File
% Performs Structure optimization by scoring the Global DCI of the reference anatomy of each 
% structure generated, and the richness w.r.t.Mass Balancing Score of all 
% structure anatomies

% Author: Nikolaos Stravopodis

% Research on Optimal Trajectory Implementation of SMM 

% Chromosome
% x1 = 1st meta link's 2nd pseudo assembly type     -> integer
% x2,3 = 2nd meta link, 2 pseudos assembly type     -> integers
% x4-6 = 2nd pseudo of 1st metalink parameters      -> real syn2-4
% x7-9 = 1st pseudo of 2nd metalink parameters      -> real syn3-5
% x10-12 = 2nd pseudo of 2nd metalink parameters    -> real syn2-4
% x13 = 1st dxl assembly pitch parameter            -> real
% x14 = 2nd dxl assembly pitch parameter            -> real

% Add paths to all functions
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/building_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/synthetic_joints_tfs')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/calculateFunctions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_objective_functions')
addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM/ga_optimization_call_functions')
clc;
clear;
close all;

%% Here call ga's
generations = 500;
population = 150;
tolerance = 1e-12;
stall_limit = 35;
%% 1.Metric with min DCI
FitnessFunction1 = @(x)obj_fn_mass_balancing_structure_optimization_minDCIoffdiag_SMM(x); 

nvars1 = 14;
A1 = []; b1 = [];
Aeq1 = []; beq1 = [];
IntCon1 = [1,2,3,13,14];
%      x1   x2   x3     x4      x5      x6     x7      x8        x9     x10      x11      x12     x13     x14 
LB1 = [1    2    1   -0.075  -0.050  -1.5708 -0.055  -0.050  -1.5708  -0.075   -0.050   -1.5708    0       0];
UB1 = [3    3    3   -0.035  -0.020  -0.7854 -0.025  -0.020  -0.7854  -0.035   -0.020   -0.7854    1       1];
LB2 = [1    2    1   -0.035  -0.020  -0.7854 -0.025  -0.020  -0.7854  -0.035   -0.020   -0.7854    0       0];
UB2 = [3    3    3    0.035    0.020  0.7854  0.025   0.020   0.7854   0.035    0.020    0.7854    1       1];
LB3 = [1    2    1    0.035    0.020  0.7854  0.025   0.020   0.7854   0.035    0.020    0.7854    0       0];
UB3 = [3    3    3    0.050   0.050   1.5708  0.025   0.050   1.5708   0.050    0.050    1.5708    1       1];
%% Call ga 
options = optimoptions('ga','Generations',generations,'PopulationSize',population,'Display','iter','FunctionTolerance',tolerance,'StallGenLimit',stall_limit,'UseParallel', true, 'UseVectorized', false);

tic
[X,Fval,Exitflag,Output] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],IntCon1,options);
toc
save('ga_test_27_10_20.mat','generations','population','tolerance','stall_limit','X','Fval','Exitflag','Output')

tic
[x1,fval1,exitflag1,output1] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],IntCon1,options);
toc
save('ga_test1_27_10_20.mat','generations','population','tolerance','stall_limit','x1','fval1','exitflag1','output1')

tic
[x2,fval2,exitflag2,output2] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB2,UB2,[],IntCon1,options);
toc
save('ga_test2_27_10_20.mat','generations','population','tolerance','stall_limit','x2','fval2','exitflag2','output2')

tic
[x3,fval3,exitflag3,output3] = ga(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB3,UB3,[],IntCon1,options);
toc
save('ga_test3_27_10_20.mat','generations','population','tolerance','stall_limit','x3','fval3','exitflag3','output3')
%% 2.Metric with max DCI

%% 3.Multiobjective with min+max DCI
% FitnessFunctions = @(x)[obj_fn_mass_balancing_structure_optimization_minDCIoffdiag_SMM(x),obj_fn_mass_balancing_structure_optimization_maxDCIoffdiag_SMM(x)];
% nvars1 = 14;
% Aeq1 = [0 0 0 0 0 0 0 0 0 0 0 0 0 1 1 1]; beq1 = 1;
% A1 = []; b1 = [];
% 
% %      Rx1      Ry1      Rz1    Px1  Py1  Pz1   Rx2      Ry2      Rz2     Px2  Py2  Pz2  w1  w2  w3
% LB1 = [0.05     -1.5708  -1.5708  -1.5708 0    0    0.08  -1.5708  -1.5708  -1.5708 0    0    0.03  0.1  0.1  0.1];
% 
% 
% % % Call gamultiob
% options = optimoptions('gamultiobj','Generations',generations,'PopulationSize',population,'Display','iter','StallGenLimit',50,'FunctionTolerance',tolerance,'UseParallel', true, 'UseVectorized', false);
% tic;
% [x1,fval1,exitflag1,output1] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB1,[],options);
% toc;
% tic;
% [x2,fval2,exitflag2,output2] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB2,UB2,[],options);
% toc;
% tic;
% [x3,fval3,exitflag3,output3] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB3,UB3,[],options);
% toc;
% tic;
% [x4,fval4,exitflag4,output4] = gamultiobj(FitnessFunction1,nvars1,A1,b1,Aeq1,beq1,LB1,UB3,[],options);
% toc;