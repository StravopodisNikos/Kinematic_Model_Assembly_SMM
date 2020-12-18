function optimized_structure_anatomy_trajectory_implementation_matlab(robot_urdf_file,motion_type,s_traj,plot_title)
% Trajectory Implementation on Isotropic Extracted anatomies
% Anatomy optimization was performed in:
% ga_call_anatomy_optimization_isotropy_investigation.m

% *  WORKS ONLY FOR 3 DOF STRUCTURES
% ** CALLS MATLAB DESIGNED MOTION CONTROL ALGORITHMS
% ***FOLLOWS EXAMPLE @ https://www.mathworks.com/help/robotics/ug/simulate-joint-space-trajectory-tracking.html

% This file was developed from: optimized_structure_anatomy_trajectory_implementation_test.m 

% Input: 1. optimized structure/anatomy urdf
%        2. motion type (1 of 3 matlab available)
%        3. struct for trajectory:
%                3.1 field1->way_pts: must be nxp array (n:DoF, p:number of trajectory points)
%                3.2 field2->time:    time_pts must be px1 array/num_samples(single int variable)
%                3.3 field3->traj:    trajectory implementation algorihm
%        4. plot names

%% Add paths to all functions:
% Obtain matlab_ws folder path on the pc
current_path = cd; % pc-grafeio
root_path = string(split(current_path,'matlab_ws'));
root_path = root_path(1);
% Add libraries relative to matlab_ws folder
Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws = fullfile('matlab_ws','Kinematic_Model_Assembly_SMM',filesep);
Kinematic_Model_Assembly_SMM_library_path = strcat(root_path,Kinematic_Model_Assembly_SMM_path_relative_to_matlab_ws); addpath(Kinematic_Model_Assembly_SMM_library_path);
% addpath('/home/nikos/matlab_ws/Kinematic_Model_Assembly_SMM')


%% load robot params
[robot,~] = visualize_robot_urdf(robot_urdf_file,[0 0 0]');

% robot = loadrobot(robot);

gen_traj_algorithm = s_traj.traj;

switch gen_traj_algorithm
    case 'trapveltraj'
        [q_d, dq_d, ddq_d, time_samples, ~] = trapveltraj( s_traj.way_pts, s_traj.time);
    otherwise
        warning('[optimized_structure_anatomy_trajectory_implementation_matlab] ONLY trapveltraj AVAILABLE')
end

ty_old = zeros(1,7);
for task_point_cnt=2:size(q_d,2)
    % set new execution time
    tspan = [time_samples(task_point_cnt-1) time_samples(task_point_cnt)];
    
    % set initial state
    if task_point_cnt == 2
        x0(1:3) = q_d(:,task_point_cnt-1);
        x0(4:6) = dq_d(:,task_point_cnt-1);
    else
        x0(1:3) = last_y_sim(1:3);
        x0(4:6) = last_y_sim(4:6);        
    end
    
    % set new desired state
    x_d     =  vertcat(q_d(:,task_point_cnt), dq_d(:,task_point_cnt), ddq_d(:,task_point_cnt));
    
    % model behaviour with joint space control
    TorqueMotion = jointSpaceMotionModel("RigidBodyTree",robot,"MotionType",motion_type);
    updateErrorDynamicsFromStep(TorqueMotion,0.2,0.1);
    
    qDes = x_d;
    
    % simulate using ODE solver
    [t_sim,y_sim] = ode45(@(t,y)derivative(TorqueMotion,y,qDes),tspan,x0);

    % save last x to pass as initial state for next step
    last_y_sim = y_sim(size(t_sim,1),:);
    
    % Build final(t,x,torque) trajectory details
    ty_new   = horzcat(t_sim,y_sim);
    final_ty = vertcat(ty_old,ty_new);
    ty_old   = final_ty;
end

figure;
subplot(2,1,1); plot(ty_old(:,1),ty_old(:,2),ty_old(:,1),ty_old(:,3),ty_old(:,1),ty_old(:,4));  title(plot_title); xlabel('Time [s]'); ylabel('Joint Position [rad]');   lgd = legend('q1','q2','q3'); 
subplot(2,1,2); plot(ty_old(:,1),ty_old(:,5),ty_old(:,1),ty_old(:,6),ty_old(:,1),ty_old(:,7));  xlabel('Time [s]'); ylabel('Joint Velocity [rad/s]'); lgd = legend('dq1','dq2','dq3'); 

end