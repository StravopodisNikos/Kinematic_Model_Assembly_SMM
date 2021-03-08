function [chromosome_for_urdf,anatomy] = decode_babis_chromosome(seq_only_robot_anat_params)

% default discretization
dxl_bolt_step = 0.005;

x_4_7_10 = -0.05:dxl_bolt_step:0.05;    % quantized assembly params
x_5_8_11 = -0.025:dxl_bolt_step:0.025;
x_6_9_12 = [-1.5708 0 1.5708];

% build new chromosome
chromosome_for_urdf(1) = seq_only_robot_anat_params(1);  % structure definition
chromosome_for_urdf(2) = seq_only_robot_anat_params(2);
chromosome_for_urdf(3) = seq_only_robot_anat_params(3);

chromosome_for_urdf(4) = x_4_7_10(4);   % assembly parameters - set1
chromosome_for_urdf(5) = x_5_8_11(5);
chromosome_for_urdf(6) = x_6_9_12(6);

chromosome_for_urdf(7) = x_4_7_10(4);   % assembly parameters - set2
chromosome_for_urdf(8) = x_5_8_11(5);
chromosome_for_urdf(9) = x_6_9_12(6);

chromosome_for_urdf(10) = x_4_7_10(4);   % assembly parameters - set3
chromosome_for_urdf(11) = x_5_8_11(5);
chromosome_for_urdf(12) = x_6_9_12(6);

chromosome_for_urdf(13) = seq_only_robot_anat_params(13);
chromosome_for_urdf(14) = seq_only_robot_anat_params(14);

% anatomy(1) = 
end