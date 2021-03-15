function [chromosome_for_urdf,anatomy] = decode_babis_chromosome(seq_structure_anat)

% Decode results from Book1.xlsx [8-3-21]
% Book1: seq = [seq_tsp[1:10] + seq_structure_anat[11:28] + fitness_score [29] ]
% seq_structure_anat = K-AB excel columns
% K-M : STRUCTURE BLOCKS -> 1-3  
% N-X : ASSEMBLY PARAMS  -> 4-14 
% Y-AB: ANATOMY          -> 15-18

% default discretization
dxl_bolt_step = 0.005;
x_4_7_10 = -0.05:dxl_bolt_step:0.05;    % quantized assembly params
x_5_8_11 = -0.025:dxl_bolt_step:0.025;
x_6_9_12 = [-1.5708 0 1.5708];
x_13_14  = [0 1.5708];
tpi      = linspace(-1.5708,1.5708,15);

% build new chromosome
chromosome_for_urdf(1) = seq_structure_anat(1);  % structure definition
chromosome_for_urdf(2) = seq_structure_anat(2);
chromosome_for_urdf(3) = seq_structure_anat(3);

chromosome_for_urdf(4) = x_4_7_10(seq_structure_anat(4));   % assembly parameters - set1
chromosome_for_urdf(5) = x_5_8_11(seq_structure_anat(5));
chromosome_for_urdf(6) = x_6_9_12(seq_structure_anat(6));

chromosome_for_urdf(7) = x_4_7_10(seq_structure_anat(7));   % assembly parameters - set2
chromosome_for_urdf(8) = x_5_8_11(seq_structure_anat(8));
chromosome_for_urdf(9) = x_6_9_12(seq_structure_anat(9));

chromosome_for_urdf(10) = x_4_7_10(seq_structure_anat(10));   % assembly parameters - set3
chromosome_for_urdf(11) = x_5_8_11(seq_structure_anat(11));
chromosome_for_urdf(12) = x_6_9_12(seq_structure_anat(12));

chromosome_for_urdf(13) = x_13_14(seq_structure_anat(13)+1);
chromosome_for_urdf(14) = x_13_14(seq_structure_anat(14)+1);

anatomy(1) =  tpi(seq_structure_anat(15));
anatomy(2) =  tpi(seq_structure_anat(16));
anatomy(3) =  tpi(seq_structure_anat(17));
anatomy(4) =  tpi(seq_structure_anat(18));
end