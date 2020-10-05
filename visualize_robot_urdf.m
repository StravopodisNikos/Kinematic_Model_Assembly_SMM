% loads stl file to robot
% robot urdf is built from /structure_synthesis/S0110110.xacro
function [robot_figure] = visualize_robot_urdf(robotURDFfile,config)

[robot] = importrobot(robotURDFfile);  
robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.80665];
robot_figure = figure;
%% Show reference anatomy robot
figure(robot_figure);
%close all;
show(robot,config,'PreservePlot',false);
hold on;
axis auto;
box on;

robot.DataFormat = 'column';
robot.Gravity = [0 0 -9.80665];
showdetails(robot);
end